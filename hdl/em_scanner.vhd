-- Address scanner and error classifier, clocked by the forwarded design clock.
-- VHDL-93.
--
-- Walks the addresses 0 .. NUM_ADDR-1 of tmrdff_readout. At each address it
-- waits SETTLE cycles for the address to reach the other FPGA and its word to
-- come back, then takes two samples on consecutive cycles of the design.
-- Per bit, a sample is in error when the three replicas are not all equal:
--
--   error in one sample only   -> flip-flop error: counted, reported, go on
--   error in both samples      -> configuration error: counted, reported, and
--                                  the scan halts until the host resumes it
--                                  (after reconfiguring the FPGA under test)
--
-- A flip-flop upset of a voting triplicated register is gone after one clock,
-- whereas a broken configuration keeps the replicas apart; two consecutive
-- cycles in error at the same place therefore point at the configuration.
--
-- Each address has a counter word (flip-flop errors, configuration errors) in
-- the dual-port RAM; every event also goes to the host through the FIFO with
-- the replica masks of both samples (which replica disagreed, per bit).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.em_pkg.all;

entity em_scanner is
  generic (
    LANES      : positive;
    ADDR_WIDTH : positive;
    NUM_ADDR   : positive;
    SETTLE     : positive := 8;      -- cycles from an address change to valid lanes
    SWEEP_LOG  : natural := 8        -- report a sweep count every 2**SWEEP_LOG sweeps
  );
  port (
    clk        : in  std_logic;       -- forwarded design clock
    rst        : in  std_logic;       -- synchronous to clk
    -- control from the host side, already synchronised to clk
    run        : in  std_logic;
    clear_tgl  : in  std_logic;       -- a change requests clearing the counters
    resume_tgl : in  std_logic;       -- a change leaves the configuration-error halt
    -- the FPGA under test
    rd_addr    : out std_logic_vector(ADDR_WIDTH - 1 downto 0);
    lane_a     : in  std_logic_vector(LANES - 1 downto 0);
    lane_b     : in  std_logic_vector(LANES - 1 downto 0);
    lane_c     : in  std_logic_vector(LANES - 1 downto 0);
    -- counters (RAM port A)
    cnt_we     : out std_logic;
    cnt_addr   : out std_logic_vector(ADDR_WIDTH - 1 downto 0);
    cnt_din    : out std_logic_vector(SOFT_CNT_W + CFG_CNT_W - 1 downto 0);
    cnt_dout   : in  std_logic_vector(SOFT_CNT_W + CFG_CNT_W - 1 downto 0);
    -- events to the host side
    ev_we      : out std_logic;
    ev_data    : out std_logic_vector(8 + ADDR_WIDTH + 32 + 6 * LANES - 1 downto 0);
    ev_full    : in  std_logic;
    -- status
    halted     : out std_logic;
    overflow   : out std_logic;       -- an event was lost because the FIFO was full
    heartbeat  : out std_logic        -- toggles every 8 cycles: the clock is alive
  );
end em_scanner;

architecture rtl of em_scanner is
  type state_t is (S_IDLE, S_CLEAR, S_SETTLE, S_SAMP1, S_SAMP2, S_CLASSIFY, S_HALT);
  signal state : state_t := S_IDLE;
  signal addr : unsigned(ADDR_WIDTH - 1 downto 0) := (others => '0');
  signal settle_cnt : natural range 0 to SETTLE := 0;
  signal sweep : unsigned(31 downto 0) := (others => '0');
  signal clear_seen, resume_seen : std_logic := '0';
  signal hb_cnt : unsigned(3 downto 0) := (others => '0');
  signal ovf : std_logic := '0';
  -- the RAM address is registered like cnt_we and cnt_din, so a write lands at
  -- the address of the cycle that requested it, not at the next one
  signal cnt_addr_r : unsigned(ADDR_WIDTH - 1 downto 0) := (others => '0');
  subtype lane_t is std_logic_vector(LANES - 1 downto 0);
  signal s1a, s1b, s1c, s2a, s2b, s2c : lane_t := (others => '0');

  function inc_sat(v : unsigned; n : natural) return unsigned is
    variable r : unsigned(v'length downto 0);
  begin
    r := ('0' & v) + n;
    if r(v'length) = '1' then
      return (v'range => '1');
    end if;
    return r(v'length - 1 downto 0);
  end function;
begin
  rd_addr <= std_logic_vector(addr);
  cnt_addr <= std_logic_vector(cnt_addr_r);
  halted <= '1' when state = S_HALT else '0';
  overflow <= ovf;
  heartbeat <= hb_cnt(3);

  process(clk)
    variable maj : lane_t;
    variable m1, m2, cfg, soft : lane_t;
    variable soft_cnt : unsigned(SOFT_CNT_W - 1 downto 0);
    variable cfg_cnt : unsigned(CFG_CNT_W - 1 downto 0);
    variable typ : std_logic_vector(7 downto 0);

    procedure emit(t : std_logic_vector(7 downto 0)) is
    begin
      if ev_full = '1' then
        ovf <= '1';
      else
        ev_we <= '1';
        ev_data <= t & std_logic_vector(addr) & std_logic_vector(sweep)
                   & s1a & s1b & s1c & s2a & s2b & s2c;
      end if;
    end procedure;

    procedure next_address is
    begin
      settle_cnt <= SETTLE;
      state <= S_SETTLE;
      if addr = NUM_ADDR - 1 then
        addr <= (others => '0');
        sweep <= sweep + 1;
      else
        addr <= addr + 1;
      end if;
    end procedure;
  begin
    if clk'event and clk = '1' then
      ev_we <= '0';
      cnt_we <= '0';
      cnt_addr_r <= addr;
      hb_cnt <= hb_cnt + 1;
      if rst = '1' then
        state <= S_IDLE;
        addr <= (others => '0');
        sweep <= (others => '0');
        ovf <= '0';
        clear_seen <= clear_tgl;
        resume_seen <= resume_tgl;
      elsif clear_tgl /= clear_seen and state /= S_CLEAR then
        -- clear the counters: one write per address, then report it
        clear_seen <= clear_tgl;
        addr <= (others => '0');
        sweep <= (others => '0');
        ovf <= '0';
        state <= S_CLEAR;
      else
        if state /= S_HALT and resume_tgl /= resume_seen then
          -- a resume while not halted means nothing; taking it in now keeps it
          -- from ending the next halt the moment it starts
          resume_seen <= resume_tgl;
        end if;
        case state is
          when S_IDLE =>
            addr <= (others => '0');
            if run = '1' then
              settle_cnt <= SETTLE;
              state <= S_SETTLE;
            end if;

          when S_CLEAR =>
            cnt_we <= '1';
            cnt_din <= (others => '0');
            if addr = NUM_ADDR - 1 then
              addr <= (others => '0');
              s1a <= (others => '0'); s1b <= (others => '0'); s1c <= (others => '0');
              s2a <= (others => '0'); s2b <= (others => '0'); s2c <= (others => '0');
              state <= S_IDLE;
            else
              addr <= addr + 1;
            end if;
            if addr = NUM_ADDR - 1 then
              -- reported with address 0 once the last word is written
              ev_we <= '1';
              ev_data <= FT_CLEARED & std_logic_vector(to_unsigned(0, ADDR_WIDTH))
                         & std_logic_vector(to_unsigned(0, 32))
                         & std_logic_vector(to_unsigned(0, 6 * LANES));
            end if;

          when S_SETTLE =>
            if run = '0' then
              state <= S_IDLE;
            elsif settle_cnt = 0 then
              state <= S_SAMP1;
            else
              settle_cnt <= settle_cnt - 1;
            end if;

          when S_SAMP1 =>
            maj := majority(lane_a, lane_b, lane_c);
            s1a <= lane_a xor maj;
            s1b <= lane_b xor maj;
            s1c <= lane_c xor maj;
            state <= S_SAMP2;

          when S_SAMP2 =>                     -- the very next cycle of the design
            maj := majority(lane_a, lane_b, lane_c);
            s2a <= lane_a xor maj;
            s2b <= lane_b xor maj;
            s2c <= lane_c xor maj;
            state <= S_CLASSIFY;

          when S_CLASSIFY =>
            m1 := s1a or s1b or s1c;
            m2 := s2a or s2b or s2c;
            cfg := m1 and m2;
            soft := (m1 or m2) and not cfg;
            soft_cnt := unsigned(cnt_dout(SOFT_CNT_W + CFG_CNT_W - 1 downto CFG_CNT_W));
            cfg_cnt := unsigned(cnt_dout(CFG_CNT_W - 1 downto 0));
            if cfg /= (cfg'range => '0') or soft /= (soft'range => '0') then
              soft_cnt := inc_sat(soft_cnt, popcount(soft));
              if cfg /= (cfg'range => '0') then
                cfg_cnt := inc_sat(cfg_cnt, 1);
              end if;
              cnt_we <= '1';
              cnt_din <= std_logic_vector(soft_cnt) & std_logic_vector(cfg_cnt);
            end if;
            if cfg /= (cfg'range => '0') then
              emit(FT_CONFIG);
              state <= S_HALT;
            else
              if soft /= (soft'range => '0') then
                emit(FT_SOFT);
              elsif addr = NUM_ADDR - 1 and ((sweep + 1) mod (2**SWEEP_LOG)) = 0 then
                -- every 2**SWEEP_LOG sweeps, so the host knows the scan is alive;
                -- carries the index of the sweep just completed
                emit(FT_SWEEP);
              end if;
              next_address;
            end if;

          when S_HALT =>                      -- wait for the host's resume
            if resume_tgl /= resume_seen then
              resume_seen <= resume_tgl;
              addr <= (others => '0');
              settle_cnt <= SETTLE;
              state <= S_SETTLE;
            end if;
        end case;
      end if;
    end if;
  end process;
end rtl;
