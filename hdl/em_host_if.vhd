-- Host interface of the detector: UART commands in, frames out. VHDL-93.
-- Runs on the detector's own clock, so it keeps talking to the host while the
-- FPGA under test (and with it the forwarded clock) is being reconfigured.
--
-- Commands (one byte): 'S' start, 'P' stop, 'C' clear counters, 'R' resume
-- after a configuration error, 'D' dump counters, 'I' info, '?' status.
-- Every command but 'D', 'I' and '?' is answered with an ACK frame.
-- Frames: see em_pkg. Payloads (big-endian):
--   SOFT/CONFIG : addr, sweep(4), then the replica masks of both samples,
--                 a1 b1 c1 a2 b2 c2, each LANES bits in nbytes(LANES) bytes
--   SWEEP       : addr, sweep(4)          (index of the sweep just completed)
--   COUNTER     : addr, soft(2), config(1)
--   STATUS      : flags(1) = run, halted, clock alive, overflow (bits 0..3)
--   INFO        : version(1), lanes(2), addr_width(1), num_addr(4), settle(1)
--   ACK         : the command byte
--   CLOCK       : 1 = clock back, 0 = clock lost
--   CLEARED, DUMPEND : empty
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.em_pkg.all;

entity em_host_if is
  generic (
    CLK_HZ     : positive;
    BAUD       : positive;
    LANES      : positive;
    ADDR_WIDTH : positive;
    NUM_ADDR   : positive;
    SETTLE     : positive;
    ALIVE_US   : positive := 1000    -- no heartbeat for this long: clock lost
  );
  port (
    clk        : in  std_logic;
    rst        : in  std_logic;
    rxd        : in  std_logic;
    txd        : out std_logic;
    -- control towards the scanner (synchronised on the other side)
    run        : out std_logic;
    clear_tgl  : out std_logic;
    resume_tgl : out std_logic;
    -- status from the scanner, raw (synchronised here)
    halted_a   : in  std_logic;
    overflow_a : in  std_logic;
    heartbeat_a: in  std_logic;
    -- events from the scanner (FIFO read side, this clock)
    ev_rd      : out std_logic;
    ev_data    : in  std_logic_vector(8 + ADDR_WIDTH + 32 + 6 * LANES - 1 downto 0);
    ev_empty   : in  std_logic;
    -- counters (RAM port B, this clock)
    cnt_addr   : out std_logic_vector(ADDR_WIDTH - 1 downto 0);
    cnt_dout   : in  std_logic_vector(SOFT_CNT_W + CFG_CNT_W - 1 downto 0);
    -- for LEDs
    clock_ok   : out std_logic
  );
end em_host_if;

architecture rtl of em_host_if is
  constant AB : positive := nbytes(ADDR_WIDTH);
  constant LB : positive := nbytes(LANES);
  constant EV_PAYLOAD : positive := AB + 4 + 6 * LB;
  constant MAXP : positive := EV_PAYLOAD + 13;     -- comfortably above every payload
  -- field positions in an event word: type, addr, sweep, six masks
  constant EV_TOP : natural := 8 + ADDR_WIDTH + 32 + 6 * LANES - 1;
  constant A_HI : natural := EV_TOP - 8;
  constant S_HI : natural := A_HI - ADDR_WIDTH;
  constant M_HI : natural := S_HI - 32;
  type bytes_t is array (0 to MAXP - 1) of std_logic_vector(7 downto 0);
  signal pay : bytes_t := (others => (others => '0'));
  signal pay_len : natural range 0 to MAXP := 0;
  signal pay_type : std_logic_vector(7 downto 0) := (others => '0');

  type tx_state_t is (T_IDLE, T_SYNC, T_TYPE, T_LEN, T_PAY, T_SUM, T_WAIT);
  signal tx_state, tx_after : tx_state_t := T_IDLE;
  signal tx_idx : natural range 0 to MAXP := 0;
  signal tx_sum : std_logic_vector(7 downto 0) := (others => '0');
  signal u_data : std_logic_vector(7 downto 0) := (others => '0');
  signal u_start, u_busy : std_logic := '0';

  signal rx_data : std_logic_vector(7 downto 0);
  signal rx_valid : std_logic;

  signal run_i, clear_i, resume_i : std_logic := '0';
  signal pend_ack, pend_info, pend_status, pend_clock : std_logic := '0';
  signal ack_byte : std_logic_vector(7 downto 0) := (others => '0');
  signal dumping, dump_end, dump_wait : std_logic := '0';
  signal dump_addr : unsigned(ADDR_WIDTH - 1 downto 0) := (others => '0');
  signal ev_pop : std_logic := '0';

  signal halt_s1, halt_s2, ovf_s1, ovf_s2, hb_s1, hb_s2, hb_last : std_logic := '0';
  constant ALIVE_CYCLES : positive := (CLK_HZ / 1000000) * ALIVE_US;
  signal alive_cnt : natural range 0 to ALIVE_CYCLES := 0;
  signal alive, alive_last : std_logic := '0';

  -- big-endian bytes of a vector, into pay(at .. at + n - 1)
  procedure put(signal p : inout bytes_t; at : natural; n : positive;
                v : std_logic_vector) is
    variable w : std_logic_vector(8 * n - 1 downto 0);
  begin
    w := (others => '0');
    w(v'length - 1 downto 0) := v;
    for i in 0 to n - 1 loop
      p(at + i) <= w(8 * (n - i) - 1 downto 8 * (n - i - 1));
    end loop;
  end procedure;
begin
  run <= run_i;
  clear_tgl <= clear_i;
  resume_tgl <= resume_i;
  cnt_addr <= std_logic_vector(dump_addr);
  clock_ok <= alive;
  ev_rd <= ev_pop;

  u_rx : entity work.em_uart_rx
    generic map (CLK_HZ => CLK_HZ, BAUD => BAUD)
    port map (clk => clk, rst => rst, rxd => rxd, data => rx_data, valid => rx_valid);
  u_tx : entity work.em_uart_tx
    generic map (CLK_HZ => CLK_HZ, BAUD => BAUD)
    port map (clk => clk, rst => rst, data => u_data, start => u_start, busy => u_busy, txd => txd);

  -- status bits from the scanner's clock domain, and the clock monitor
  process(clk)
  begin
    if clk'event and clk = '1' then
      halt_s1 <= halted_a;    halt_s2 <= halt_s1;
      ovf_s1 <= overflow_a;   ovf_s2 <= ovf_s1;
      hb_s1 <= heartbeat_a;   hb_s2 <= hb_s1;
      hb_last <= hb_s2;
      if rst = '1' then
        alive_cnt <= 0;
        alive <= '0';
      elsif hb_s2 /= hb_last then
        alive_cnt <= 0;
        alive <= '1';
      elsif alive_cnt = ALIVE_CYCLES then
        alive <= '0';
      else
        alive_cnt <= alive_cnt + 1;
      end if;
    end if;
  end process;

  process(clk)
    variable ev_t : std_logic_vector(7 downto 0);
  begin
    if clk'event and clk = '1' then
      u_start <= '0';
      ev_pop <= '0';
      if rst = '1' then
        run_i <= '0';
        pend_ack <= '0'; pend_info <= '0'; pend_status <= '0'; pend_clock <= '0';
        dumping <= '0'; dump_end <= '0'; dump_wait <= '0';
        tx_state <= T_IDLE;
        alive_last <= '0';
      else
        -- commands
        if rx_valid = '1' then
          if rx_data = CMD_START then
            run_i <= '1'; pend_ack <= '1'; ack_byte <= rx_data;
          elsif rx_data = CMD_STOP then
            run_i <= '0'; pend_ack <= '1'; ack_byte <= rx_data;
          elsif rx_data = CMD_CLEAR then
            clear_i <= not clear_i; pend_ack <= '1'; ack_byte <= rx_data;
          elsif rx_data = CMD_RESUME then
            resume_i <= not resume_i; pend_ack <= '1'; ack_byte <= rx_data;
          elsif rx_data = CMD_DUMP then
            dumping <= '1'; dump_addr <= (others => '0'); dump_wait <= '1';
          elsif rx_data = CMD_INFO then
            pend_info <= '1';
          elsif rx_data = CMD_STATUS then
            pend_status <= '1';
          end if;
        end if;
        if alive /= alive_last then
          alive_last <= alive;
          pend_clock <= '1';
        end if;

        -- frames, one at a time
        case tx_state is
          when T_IDLE =>
            if pend_ack = '1' then
              pend_ack <= '0';
              pay_type <= FT_ACK; pay_len <= 1; pay(0) <= ack_byte;
              tx_state <= T_SYNC;
            elsif pend_info = '1' then
              pend_info <= '0';
              pay_type <= FT_INFO; pay_len <= 9;
              put(pay, 0, 1, std_logic_vector(to_unsigned(VERSION, 8)));
              put(pay, 1, 2, std_logic_vector(to_unsigned(LANES, 16)));
              put(pay, 3, 1, std_logic_vector(to_unsigned(ADDR_WIDTH, 8)));
              put(pay, 4, 4, std_logic_vector(to_unsigned(NUM_ADDR, 32)));
              put(pay, 8, 1, std_logic_vector(to_unsigned(SETTLE, 8)));
              tx_state <= T_SYNC;
            elsif pend_status = '1' then
              pend_status <= '0';
              pay_type <= FT_STATUS; pay_len <= 1;
              pay(0) <= "0000" & ovf_s2 & alive & halt_s2 & run_i;
              tx_state <= T_SYNC;
            elsif pend_clock = '1' then
              pend_clock <= '0';
              pay_type <= FT_CLOCK; pay_len <= 1;
              pay(0) <= "0000000" & alive;
              tx_state <= T_SYNC;
            elsif dumping = '1' then
              if dump_wait = '1' then
                dump_wait <= '0';                       -- RAM output is one cycle late
              elsif dump_end = '1' then
                dump_end <= '0'; dumping <= '0';
                pay_type <= FT_DUMPEND; pay_len <= 0;
                tx_state <= T_SYNC;
              else
                pay_type <= FT_COUNTER; pay_len <= AB + 3;
                put(pay, 0, AB, std_logic_vector(dump_addr));
                put(pay, AB, 2, cnt_dout(SOFT_CNT_W + CFG_CNT_W - 1 downto CFG_CNT_W));
                put(pay, AB + 2, 1, cnt_dout(CFG_CNT_W - 1 downto 0));
                if dump_addr = NUM_ADDR - 1 then
                  dump_end <= '1';
                else
                  dump_addr <= dump_addr + 1;
                  dump_wait <= '1';
                end if;
                tx_state <= T_SYNC;
              end if;
            elsif ev_empty = '0' then
              ev_pop <= '1';
              ev_t := ev_data(EV_TOP downto EV_TOP - 7);
              pay_type <= ev_t;
              put(pay, 0, AB, ev_data(A_HI downto A_HI - ADDR_WIDTH + 1));
              put(pay, AB, 4, ev_data(S_HI downto S_HI - 31));
              if ev_t = FT_SOFT or ev_t = FT_CONFIG then
                pay_len <= EV_PAYLOAD;
                for m in 0 to 5 loop
                  put(pay, AB + 4 + m * LB, LB,
                      ev_data(M_HI - m * LANES downto M_HI - m * LANES - LANES + 1));
                end loop;
              elsif ev_t = FT_SWEEP then
                pay_len <= AB + 4;
              else
                pay_len <= 0;                           -- CLEARED
              end if;
              tx_state <= T_SYNC;
            end if;

          -- byte sequencer: each byte waits for the UART to take it
          when T_SYNC =>
            u_data <= FRAME_SYNC; u_start <= '1';
            tx_after <= T_TYPE; tx_state <= T_WAIT;
          when T_TYPE =>
            u_data <= pay_type; u_start <= '1';
            tx_sum <= pay_type;
            tx_after <= T_LEN; tx_state <= T_WAIT;
          when T_LEN =>
            u_data <= std_logic_vector(to_unsigned(pay_len, 8)); u_start <= '1';
            tx_sum <= tx_sum xor std_logic_vector(to_unsigned(pay_len, 8));
            tx_idx <= 0;
            if pay_len = 0 then
              tx_after <= T_SUM;
            else
              tx_after <= T_PAY;
            end if;
            tx_state <= T_WAIT;
          when T_PAY =>
            u_data <= pay(tx_idx); u_start <= '1';
            tx_sum <= tx_sum xor pay(tx_idx);
            if tx_idx = pay_len - 1 then
              tx_after <= T_SUM;
            else
              tx_after <= T_PAY;
            end if;
            tx_idx <= tx_idx + 1;
            tx_state <= T_WAIT;
          when T_SUM =>
            u_data <= tx_sum; u_start <= '1';
            tx_after <= T_IDLE; tx_state <= T_WAIT;
          when T_WAIT =>
            -- u_start was raised last cycle; the UART is busy from now on,
            -- and ready for the next byte once busy falls again
            if u_start = '0' and u_busy = '0' then
              tx_state <= tx_after;
            end if;
        end case;
      end if;
    end if;
  end process;
end rtl;
