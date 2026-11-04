-- Tester, top level of the tester FPGA. VHDL-93.
--
-- read_clk runs everything here; give it the highest round frequency synthesis
-- allows. sample_clk is a multiple of it from the same PLL and samples the input
-- lines (read_clk itself if no finer timing adjustment is wanted): TAPS steps of
-- it should span at least one read_clk period.
--
-- The FPGA under test gets two clocks from here: dut_design_clk, read_clk divided
-- by the host-set ratio, for the design; and dut_read_clk, read_clk inverted, for
-- tmrdff_readout, so that it samples the addresses half a period after they change.
--
-- Board-independent: map these ports to pins in the board's project. With
-- LANES = 4 and ADDR_WIDTH = 18 the link is 12 lane inputs, 3 echo inputs, 54
-- address outputs and 2 clock outputs: 71 pins.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.em_pkg.all;

entity em_detector_top is
  generic (
    CLK_HZ     : positive := 50000000;   -- read_clk
    BAUD       : positive := 115200;
    LANES      : positive := 4;
    ADDR_WIDTH : positive := 18;
    NUM_ADDR   : positive := 171802;     -- ceil(TMRDFF width / LANES), at most 2**ADDR_WIDTH - 2
    GUARD      : natural := 1;           -- read cycles left unread after a design edge
    WMAX       : positive := 256;        -- largest window
    TAPS       : positive := 8;          -- input delay steps
    LMAX       : positive := 32;         -- largest read latency
    LIST_BITS  : positive := 12;         -- list depth 2**LIST_BITS
    SWEEP_LOG  : natural := 8;
    FIFO_LOG   : positive := 6;          -- event FIFO depth 2**FIFO_LOG
    TRAIN_RUN  : positive := 256;
    LINK_LOG   : positive := 16
  );
  port (
    read_clk      : in  std_logic;
    sample_clk    : in  std_logic;
    rst_n         : in  std_logic;       -- asynchronous, active low
    uart_rxd      : in  std_logic;
    uart_txd      : out std_logic;
    dut_design_clk: out std_logic;
    dut_read_clk  : out std_logic;
    dut_addr_a    : out std_logic_vector(ADDR_WIDTH - 1 downto 0);
    dut_addr_b    : out std_logic_vector(ADDR_WIDTH - 1 downto 0);
    dut_addr_c    : out std_logic_vector(ADDR_WIDTH - 1 downto 0);
    dut_lane_a    : in  std_logic_vector(LANES - 1 downto 0);
    dut_lane_b    : in  std_logic_vector(LANES - 1 downto 0);
    dut_lane_c    : in  std_logic_vector(LANES - 1 downto 0);
    dut_echo_a    : in  std_logic;
    dut_echo_b    : in  std_logic;
    dut_echo_c    : in  std_logic;
    led_run       : out std_logic;
    led_halt      : out std_logic;       -- configuration error, waiting for resume
    led_link      : out std_logic        -- the readout's echoes are right
  );
end em_detector_top;

architecture rtl of em_detector_top is
  constant EV_W : positive := 8 + ADDR_WIDTH + 32 + 6 * LANES + 6;
  constant CNT_W : positive := SOFT_CNT_W + CFG_CNT_W;
  constant TAP_BITS : positive := clog2(TAPS);
  constant LINES : positive := 3 * LANES + 3;

  signal rst_s1, rst : std_logic := '1';

  signal line_in, line_d, line_r : std_logic_vector(LINES - 1 downto 0);
  signal taps_all : std_logic_vector(LINES * TAP_BITS - 1 downto 0);

  signal run, mode_list, clear_p, resume_p, train_p : std_logic;
  signal ratio : unsigned(23 downto 0);
  signal lat : unsigned(5 downto 0);
  signal list_len : unsigned(LIST_BITS downto 0);
  signal list_we : std_logic;
  signal list_waddr, list_raddr : std_logic_vector(LIST_BITS - 1 downto 0);
  signal list_wdata, list_dout : std_logic_vector(ADDR_WIDTH - 1 downto 0);
  signal addr : std_logic_vector(ADDR_WIDTH - 1 downto 0);

  signal halted, busy, overflow, link_ok : std_logic;
  signal bad_a, bad_b, bad_c : unsigned(31 downto 0);

  signal ev_we, ev_full, ev_rd, ev_empty : std_logic;
  signal ev_wdata, ev_rdata : std_logic_vector(EV_W - 1 downto 0);

  signal cnt_we : std_logic;
  signal cnt_addr_a, cnt_addr_b : std_logic_vector(ADDR_WIDTH - 1 downto 0);
  signal cnt_din, cnt_dout_a, cnt_dout_b : std_logic_vector(CNT_W - 1 downto 0);
begin
  process(read_clk, rst_n)
  begin
    if rst_n = '0' then
      rst_s1 <= '1'; rst <= '1';
    elsif read_clk'event and read_clk = '1' then
      rst_s1 <= '0'; rst <= rst_s1;
    end if;
  end process;

  dut_read_clk <= not read_clk;
  dut_addr_a <= addr;
  dut_addr_b <= addr;
  dut_addr_c <= addr;

  -- every input line through its own delay, then into read_clk
  line_in <= dut_echo_c & dut_echo_b & dut_echo_a & dut_lane_c & dut_lane_b & dut_lane_a;
  g_lines : for i in 0 to LINES - 1 generate
    u_d : entity work.em_line_delay
      generic map (TAPS => TAPS, TAP_BITS => TAP_BITS)
      port map (sample_clk => sample_clk, d => line_in(i),
                tap => taps_all((i + 1) * TAP_BITS - 1 downto i * TAP_BITS), q => line_d(i));
  end generate;
  process(read_clk)
  begin
    if read_clk'event and read_clk = '1' then
      line_r <= line_d;
    end if;
  end process;

  u_scan : entity work.em_scanner
    generic map (LANES => LANES, ADDR_WIDTH => ADDR_WIDTH, NUM_ADDR => NUM_ADDR, GUARD => GUARD,
                 WMAX => WMAX, LMAX => LMAX, LIST_BITS => LIST_BITS, SWEEP_LOG => SWEEP_LOG,
                 TRAIN_RUN => TRAIN_RUN, LINK_LOG => LINK_LOG)
    port map (clk => read_clk, rst => rst,
              run => run, mode_list => mode_list, ratio => ratio, lat => lat, list_len => list_len,
              clear_p => clear_p, resume_p => resume_p, train_p => train_p,
              list_raddr => list_raddr, list_dout => list_dout,
              design_clk => dut_design_clk, addr_out => addr,
              lane_a => line_r(LANES - 1 downto 0),
              lane_b => line_r(2 * LANES - 1 downto LANES),
              lane_c => line_r(3 * LANES - 1 downto 2 * LANES),
              echo => line_r(3 * LANES + 2 downto 3 * LANES),
              cnt_we => cnt_we, cnt_addr => cnt_addr_a, cnt_din => cnt_din, cnt_dout => cnt_dout_a,
              ev_we => ev_we, ev_data => ev_wdata, ev_full => ev_full,
              halted => halted, busy => busy, overflow => overflow, link_ok => link_ok,
              echo_bad_a => bad_a, echo_bad_b => bad_b, echo_bad_c => bad_c);

  u_fifo : entity work.em_async_fifo
    generic map (DATA_W => EV_W, ADDR_W => FIFO_LOG)
    port map (wr_clk => read_clk, wr_rst => rst, wr_en => ev_we, wr_data => ev_wdata, full => ev_full,
              rd_clk => read_clk, rd_rst => rst, rd_en => ev_rd, rd_data => ev_rdata, empty => ev_empty);

  u_cnt : entity work.em_dpram
    generic map (DATA_W => CNT_W, ADDR_W => ADDR_WIDTH, DEPTH => NUM_ADDR)
    port map (clk_a => read_clk, we_a => cnt_we, addr_a => cnt_addr_a, din_a => cnt_din, dout_a => cnt_dout_a,
              clk_b => read_clk, addr_b => cnt_addr_b, dout_b => cnt_dout_b);

  u_list : entity work.em_dpram
    generic map (DATA_W => ADDR_WIDTH, ADDR_W => LIST_BITS, DEPTH => 2**LIST_BITS)
    port map (clk_a => read_clk, we_a => list_we, addr_a => list_waddr, din_a => list_wdata, dout_a => open,
              clk_b => read_clk, addr_b => list_raddr, dout_b => list_dout);

  u_host : entity work.em_host_if
    generic map (CLK_HZ => CLK_HZ, BAUD => BAUD, LANES => LANES, ADDR_WIDTH => ADDR_WIDTH,
                 NUM_ADDR => NUM_ADDR, GUARD => GUARD, WMAX => WMAX, TAPS => TAPS, TAP_BITS => TAP_BITS,
                 LMAX => LMAX, LIST_BITS => LIST_BITS)
    port map (clk => read_clk, rst => rst, rxd => uart_rxd, txd => uart_txd,
              run => run, mode_list => mode_list, ratio => ratio, lat => lat, list_len => list_len,
              clear_p => clear_p, resume_p => resume_p, train_p => train_p, taps_all => taps_all,
              list_we => list_we, list_waddr => list_waddr, list_wdata => list_wdata,
              halted => halted, busy => busy, overflow => overflow, link_ok => link_ok,
              echo_bad_a => bad_a, echo_bad_b => bad_b, echo_bad_c => bad_c,
              ev_rd => ev_rd, ev_data => ev_rdata, ev_empty => ev_empty,
              cnt_addr => cnt_addr_b, cnt_dout => cnt_dout_b);

  led_run <= run;
  led_halt <= halted;
  led_link <= link_ok;
end rtl;
