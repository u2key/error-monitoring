-- Host interface of the tester: UART commands in, frames out. VHDL-93.
--
-- Commands (first byte, then payload):
--   'S' start   'P' stop   'R' resume after a configuration error
--   'C' clear counters   'D' dump counters   'I' info   '?' status
--   'F' r2 r1 r0  design clock ratio R (read clocks per design clock, at least 2
--                 and more than GUARD)
--   'L' l         read latency
--   'A' line tap  delay tap of one input line (0xFF: of all). Lines: lane a, b, c,
--                 LANES each, then echo a, b, c
--   'M' m         0 sweep every address, 1 scan the list
--   'U' n1 n0 then n addresses   load the list (one address = fixed mode)
--   'T'           one training run at the current taps and latency
-- 'F', 'L', 'A', 'M', 'C', 'T' are refused with NACK while scanning. A list ('U')
-- is taken in any time, and its length applies at the next start.
-- Answers: ACK or NACK with the command byte, except 'D', 'I', '?', 'T'.
-- Frames: see em_pkg. Payloads (big-endian, addr in nbytes(ADDR_WIDTH) bytes):
--   SOFT/CONFIG : addr, sweep(4), masks a1 b1 c1 a2 b2 c2 (nbytes(LANES) each),
--                 echo(1) = bad echoes of read 1 (bits 5..3) and read 2 (2..0)
--   SWEEP       : addr, sweep(4)          (index of the sweep just completed)
--   COUNTER     : addr, soft(2), config(1)
--   STATUS      : flags(1) = run, halted, link ok, overflow, busy (bits 0..4),
--                 bad echo counts a(4) b(4) c(4)
--   INFO        : version(1), lanes(2), addr_width(1), num_addr(4), guard(1),
--                 wmax(2), taps(1), lmax(1), list depth(2), clk_hz(4)
--   TRAIN       : pass a, b, c (nbytes(LANES) each), pass echo(1) = c b a
--   LINK        : 1 = back, 0 = lost
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
    GUARD      : natural;
    WMAX       : positive;
    TAPS       : positive;
    TAP_BITS   : positive;
    LMAX       : positive;
    LIST_BITS  : positive
  );
  port (
    clk        : in  std_logic;
    rst        : in  std_logic;
    rxd        : in  std_logic;
    txd        : out std_logic;
    -- control towards the scanner
    run        : out std_logic;
    mode_list  : out std_logic;
    ratio      : out unsigned(23 downto 0);
    lat        : out unsigned(5 downto 0);
    list_len   : out unsigned(LIST_BITS downto 0);
    clear_p    : out std_logic;
    resume_p   : out std_logic;
    train_p    : out std_logic;
    taps_all   : out std_logic_vector((3 * LANES + 3) * TAP_BITS - 1 downto 0);
    -- list RAM, write side
    list_we    : out std_logic;
    list_waddr : out std_logic_vector(LIST_BITS - 1 downto 0);
    list_wdata : out std_logic_vector(ADDR_WIDTH - 1 downto 0);
    -- status from the scanner
    halted     : in  std_logic;
    busy       : in  std_logic;
    overflow   : in  std_logic;
    link_ok    : in  std_logic;
    echo_bad_a : in  unsigned(31 downto 0);
    echo_bad_b : in  unsigned(31 downto 0);
    echo_bad_c : in  unsigned(31 downto 0);
    -- events from the scanner (FIFO read side)
    ev_rd      : out std_logic;
    ev_data    : in  std_logic_vector(8 + ADDR_WIDTH + 32 + 6 * LANES + 6 - 1 downto 0);
    ev_empty   : in  std_logic;
    -- counters (RAM port B)
    cnt_addr   : out std_logic_vector(ADDR_WIDTH - 1 downto 0);
    cnt_dout   : in  std_logic_vector(SOFT_CNT_W + CFG_CNT_W - 1 downto 0)
  );
end em_host_if;

architecture rtl of em_host_if is
  constant AB : positive := nbytes(ADDR_WIDTH);
  constant LB : positive := nbytes(LANES);
  constant EV_PAYLOAD : positive := AB + 4 + 6 * LB + 1;
  constant MAXP : positive := EV_PAYLOAD + 20;
  -- field positions in an event word: type, addr, sweep, six masks, echoes
  constant EV_TOP : natural := 8 + ADDR_WIDTH + 32 + 6 * LANES + 6 - 1;
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

  -- command parser
  signal cmd : std_logic_vector(7 downto 0) := (others => '0');
  signal cmd_left : natural range 0 to 3 := 0;
  signal hold : std_logic_vector(23 downto 0) := (others => '0');
  signal list_left : unsigned(15 downto 0) := (others => '0');
  signal list_idx : unsigned(LIST_BITS downto 0) := (others => '0');
  signal list_byte : natural range 0 to AB := 0;
  signal list_word : std_logic_vector(8 * AB - 1 downto 0) := (others => '0');
  signal listing : std_logic := '0';

  signal run_i, mode_i : std_logic := '0';
  signal ratio_i : unsigned(23 downto 0) := to_unsigned(8, 24);
  signal lat_i : unsigned(5 downto 0) := to_unsigned(4, 6);
  signal len_i : unsigned(LIST_BITS downto 0) := (others => '0');
  signal taps_i : std_logic_vector((3 * LANES + 3) * TAP_BITS - 1 downto 0) := (others => '0');

  signal pend_ack, pend_nack, pend_info, pend_status, pend_link : std_logic := '0';
  signal ack_byte : std_logic_vector(7 downto 0) := (others => '0');
  signal dumping, dump_end, dump_wait : std_logic := '0';
  signal dump_addr : unsigned(ADDR_WIDTH - 1 downto 0) := (others => '0');
  signal ev_pop : std_logic := '0';
  signal link_last : std_logic := '1';

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
  mode_list <= mode_i;
  ratio <= ratio_i;
  lat <= lat_i;
  list_len <= len_i;
  taps_all <= taps_i;
  cnt_addr <= std_logic_vector(dump_addr);
  ev_rd <= ev_pop;
  list_waddr <= std_logic_vector(list_idx(LIST_BITS - 1 downto 0));
  list_wdata <= list_word(ADDR_WIDTH - 1 downto 0);

  u_rx : entity work.em_uart_rx
    generic map (CLK_HZ => CLK_HZ, BAUD => BAUD)
    port map (clk => clk, rst => rst, rxd => rxd, data => rx_data, valid => rx_valid);
  u_tx : entity work.em_uart_tx
    generic map (CLK_HZ => CLK_HZ, BAUD => BAUD)
    port map (clk => clk, rst => rst, data => u_data, start => u_start, busy => u_busy, txd => txd);

  process(clk)
    variable ev_t : std_logic_vector(7 downto 0);
    variable stopped : boolean;
    variable line, tp : natural;
    variable word : std_logic_vector(8 * AB - 1 downto 0);
    variable v24 : std_logic_vector(23 downto 0);
    variable v16 : std_logic_vector(15 downto 0);
  begin
    if clk'event and clk = '1' then
      u_start <= '0';
      ev_pop <= '0';
      clear_p <= '0';
      resume_p <= '0';
      train_p <= '0';
      list_we <= '0';
      stopped := run_i = '0' and busy = '0';
      if rst = '1' then
        run_i <= '0';
        cmd_left <= 0;
        listing <= '0';
        pend_ack <= '0'; pend_nack <= '0'; pend_info <= '0'; pend_status <= '0'; pend_link <= '0';
        dumping <= '0'; dump_end <= '0'; dump_wait <= '0';
        tx_state <= T_IDLE;
        link_last <= '1';
      else
        -- commands
        if rx_valid = '1' then
          if listing = '1' then
            -- list words, big-endian
            word := list_word(8 * AB - 9 downto 0) & rx_data;
            list_word <= word;
            if list_byte = AB - 1 then
              list_byte <= 0;
              list_we <= '1';
              list_idx <= list_idx + 1;
              if list_left = 1 then
                listing <= '0';
                pend_ack <= '1';
                ack_byte <= CMD_LIST;
              end if;
              list_left <= list_left - 1;
            else
              list_byte <= list_byte + 1;
            end if;
          elsif cmd_left /= 0 then
            v24 := hold(15 downto 0) & rx_data;
            v16 := v24(15 downto 0);
            hold <= v24;
            cmd_left <= cmd_left - 1;
            if cmd_left = 1 then
              if not stopped and cmd /= CMD_LIST then
                pend_nack <= '1'; ack_byte <= cmd;
              elsif cmd = CMD_RATIO then
                if unsigned(v24) >= 2 and unsigned(v24) > GUARD then
                  ratio_i <= unsigned(v24);
                  pend_ack <= '1'; ack_byte <= cmd;
                else
                  pend_nack <= '1'; ack_byte <= cmd;
                end if;
              elsif cmd = CMD_LAT then
                lat_i <= unsigned(rx_data(5 downto 0));
                pend_ack <= '1'; ack_byte <= cmd;
              elsif cmd = CMD_MODE then
                mode_i <= rx_data(0);
                pend_ack <= '1'; ack_byte <= cmd;
              elsif cmd = CMD_TAP then
                line := to_integer(unsigned(hold(7 downto 0)));
                tp := to_integer(unsigned(rx_data));
                if line = 255 and tp < TAPS then
                  for k in 0 to 3 * LANES + 2 loop
                    taps_i((k + 1) * TAP_BITS - 1 downto k * TAP_BITS) <=
                      std_logic_vector(to_unsigned(tp, TAP_BITS));
                  end loop;
                  pend_ack <= '1'; ack_byte <= cmd;
                elsif line < 3 * LANES + 3 and tp < TAPS then
                  taps_i((line + 1) * TAP_BITS - 1 downto line * TAP_BITS) <=
                    std_logic_vector(to_unsigned(tp, TAP_BITS));
                  pend_ack <= '1'; ack_byte <= cmd;
                else
                  pend_nack <= '1'; ack_byte <= cmd;
                end if;
              elsif cmd = CMD_LIST then
                -- count received: the words follow (refusal would desynchronise
                -- the stream, so a list sent while running is taken in and the
                -- new length applies at the next start)
                if unsigned(v16) = 0
                   or unsigned(v16) > 2**LIST_BITS then
                  pend_nack <= '1'; ack_byte <= cmd;
                else
                  listing <= '1';
                  list_left <= unsigned(v16);
                  len_i <= resize(unsigned(v16), LIST_BITS + 1);
                  list_idx <= (others => '1');      -- first write wraps to 0
                  list_byte <= 0;
                end if;
              end if;
            end if;
          elsif rx_data = CMD_START then
            run_i <= '1'; pend_ack <= '1'; ack_byte <= rx_data;
          elsif rx_data = CMD_STOP then
            run_i <= '0'; pend_ack <= '1'; ack_byte <= rx_data;
          elsif rx_data = CMD_RESUME then
            resume_p <= '1'; pend_ack <= '1'; ack_byte <= rx_data;
          elsif rx_data = CMD_CLEAR then
            if stopped then
              clear_p <= '1'; pend_ack <= '1';
            else
              pend_nack <= '1';
            end if;
            ack_byte <= rx_data;
          elsif rx_data = CMD_TRAIN then
            if stopped then
              train_p <= '1';
            else
              pend_nack <= '1'; ack_byte <= rx_data;
            end if;
          elsif rx_data = CMD_DUMP then
            dumping <= '1'; dump_addr <= (others => '0'); dump_wait <= '1';
          elsif rx_data = CMD_INFO then
            pend_info <= '1';
          elsif rx_data = CMD_STATUS then
            pend_status <= '1';
          elsif rx_data = CMD_RATIO then
            cmd <= rx_data; cmd_left <= 3;
          elsif rx_data = CMD_LAT or rx_data = CMD_MODE then
            cmd <= rx_data; cmd_left <= 1;
          elsif rx_data = CMD_TAP or rx_data = CMD_LIST then
            cmd <= rx_data; cmd_left <= 2;
          end if;
        end if;
        if link_ok /= link_last then
          link_last <= link_ok;
          pend_link <= '1';
        end if;

        -- frames, one at a time
        case tx_state is
          when T_IDLE =>
            if pend_nack = '1' then
              pend_nack <= '0';
              pay_type <= FT_NACK; pay_len <= 1; pay(0) <= ack_byte;
              tx_state <= T_SYNC;
            elsif pend_ack = '1' then
              pend_ack <= '0';
              pay_type <= FT_ACK; pay_len <= 1; pay(0) <= ack_byte;
              tx_state <= T_SYNC;
            elsif pend_info = '1' then
              pend_info <= '0';
              pay_type <= FT_INFO; pay_len <= 19;
              put(pay, 0, 1, std_logic_vector(to_unsigned(VERSION, 8)));
              put(pay, 1, 2, std_logic_vector(to_unsigned(LANES, 16)));
              put(pay, 3, 1, std_logic_vector(to_unsigned(ADDR_WIDTH, 8)));
              put(pay, 4, 4, std_logic_vector(to_unsigned(NUM_ADDR, 32)));
              put(pay, 8, 1, std_logic_vector(to_unsigned(GUARD, 8)));
              put(pay, 9, 2, std_logic_vector(to_unsigned(WMAX, 16)));
              put(pay, 11, 1, std_logic_vector(to_unsigned(TAPS, 8)));
              put(pay, 12, 1, std_logic_vector(to_unsigned(LMAX, 8)));
              put(pay, 13, 2, std_logic_vector(to_unsigned(2**LIST_BITS, 16)));
              put(pay, 15, 4, std_logic_vector(to_unsigned(CLK_HZ, 32)));
              tx_state <= T_SYNC;
            elsif pend_status = '1' then
              pend_status <= '0';
              pay_type <= FT_STATUS; pay_len <= 13;
              pay(0) <= "000" & busy & overflow & link_ok & halted & run_i;
              put(pay, 1, 4, std_logic_vector(echo_bad_a));
              put(pay, 5, 4, std_logic_vector(echo_bad_b));
              put(pay, 9, 4, std_logic_vector(echo_bad_c));
              tx_state <= T_SYNC;
            elsif pend_link = '1' then
              pend_link <= '0';
              pay_type <= FT_LINK; pay_len <= 1;
              pay(0) <= "0000000" & link_ok;
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
              if ev_t = FT_TRAIN then
                pay_len <= 3 * LB + 1;
                for m in 0 to 2 loop
                  put(pay, m * LB, LB, ev_data(M_HI - m * LANES downto M_HI - m * LANES - LANES + 1));
                end loop;
                put(pay, 3 * LB, 1, ev_data(2 downto 0));
              else
                put(pay, 0, AB, ev_data(A_HI downto A_HI - ADDR_WIDTH + 1));
                put(pay, AB, 4, ev_data(S_HI downto S_HI - 31));
                if ev_t = FT_SOFT or ev_t = FT_CONFIG then
                  pay_len <= EV_PAYLOAD;
                  for m in 0 to 5 loop
                    put(pay, AB + 4 + m * LB, LB,
                        ev_data(M_HI - m * LANES downto M_HI - m * LANES - LANES + 1));
                  end loop;
                  put(pay, AB + 4 + 6 * LB, 1, ev_data(5 downto 0));
                elsif ev_t = FT_SWEEP then
                  pay_len <= AB + 4;
                else
                  pay_len <= 0;                         -- CLEARED
                end if;
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
            if u_start = '0' and u_busy = '0' then
              tx_state <= tx_after;
            end if;
        end case;
      end if;
    end if;
  end process;
end rtl;
