-- Error detector, top level of the detector FPGA. VHDL-93.
--
-- Two clock domains:
--   dut_clk  forwarded by tmrdff_readout from the FPGA under test. The lanes
--            are sampled and the scanner runs on it, so that two samples are
--            two consecutive cycles of the design.
--   sys_clk  the detector board's own oscillator: UART and host interface,
--            which must keep running while the other FPGA is reconfigured.
-- Events cross through a dual-clock FIFO, the counters live in a dual-clock
-- RAM, single control/status bits go through two-flop synchronisers.
--
-- Board-independent: map these ports to pins in the board's project. With
-- LANES = 16 and ADDR_WIDTH = 16 the link to the other FPGA is 48 lane inputs,
-- 16 address outputs and one clock input.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.em_pkg.all;

entity em_detector_top is
  generic (
    CLK_HZ          : positive := 50000000;   -- sys_clk
    BAUD            : positive := 115200;
    LANES           : positive := 16;
    ADDR_WIDTH      : positive := 16;
    NUM_ADDR        : positive := 42951;      -- ceil(TMRDFF width / LANES)
    SETTLE          : positive := 8;          -- see em_scanner
    SWEEP_LOG       : natural := 8;
    FIFO_LOG        : positive := 6;          -- event FIFO depth 2**FIFO_LOG
    ALIVE_US        : positive := 1000;       -- no forwarded clock this long: clock lost
    CAPTURE_FALLING : boolean := true         -- sample the lanes on the falling edge
  );
  port (
    sys_clk   : in  std_logic;
    rst_n     : in  std_logic;                -- asynchronous, active low
    uart_rxd  : in  std_logic;
    uart_txd  : out std_logic;
    dut_clk   : in  std_logic;
    dut_lane_a: in  std_logic_vector(LANES - 1 downto 0);
    dut_lane_b: in  std_logic_vector(LANES - 1 downto 0);
    dut_lane_c: in  std_logic_vector(LANES - 1 downto 0);
    dut_addr  : out std_logic_vector(ADDR_WIDTH - 1 downto 0);
    led_run   : out std_logic;
    led_halt  : out std_logic;                -- configuration error, waiting for resume
    led_clock : out std_logic                 -- the forwarded clock is running
  );
end em_detector_top;

architecture rtl of em_detector_top is
  constant EV_W : positive := 8 + ADDR_WIDTH + 32 + 6 * LANES;
  constant CNT_W : positive := SOFT_CNT_W + CFG_CNT_W;

  signal sys_rst_s1, sys_rst : std_logic := '1';
  signal d_rst_s1, d_rst : std_logic := '1';

  -- lanes: captured on the forwarded clock
  signal fa, fb, fc : std_logic_vector(LANES - 1 downto 0) := (others => '0');
  signal la, lb, lc : std_logic_vector(LANES - 1 downto 0) := (others => '0');

  signal run, clear_tgl, resume_tgl : std_logic;
  signal run_s1, run_s2, clr_s1, clr_s2, res_s1, res_s2 : std_logic := '0';
  signal halted, overflow, heartbeat, clock_ok : std_logic;

  signal ev_we, ev_full, ev_rd, ev_empty : std_logic;
  signal ev_wdata, ev_rdata : std_logic_vector(EV_W - 1 downto 0);

  signal cnt_we : std_logic;
  signal cnt_addr_a, cnt_addr_b : std_logic_vector(ADDR_WIDTH - 1 downto 0);
  signal cnt_din, cnt_dout_a, cnt_dout_b : std_logic_vector(CNT_W - 1 downto 0);
begin
  -- resets: asserted asynchronously, released in step with each clock
  process(sys_clk, rst_n)
  begin
    if rst_n = '0' then
      sys_rst_s1 <= '1'; sys_rst <= '1';
    elsif sys_clk'event and sys_clk = '1' then
      sys_rst_s1 <= '0'; sys_rst <= sys_rst_s1;
    end if;
  end process;
  process(dut_clk, rst_n)
  begin
    if rst_n = '0' then
      d_rst_s1 <= '1'; d_rst <= '1';
    elsif dut_clk'event and dut_clk = '1' then
      d_rst_s1 <= '0'; d_rst <= d_rst_s1;
    end if;
  end process;

  -- lane capture: on the falling edge, half a cycle away from the edge that
  -- launched them on the other board, then into the rising-edge domain
  g_fall : if CAPTURE_FALLING generate
    process(dut_clk)
    begin
      if dut_clk'event and dut_clk = '0' then
        fa <= dut_lane_a; fb <= dut_lane_b; fc <= dut_lane_c;
      end if;
    end process;
  end generate;
  g_rise : if not CAPTURE_FALLING generate
    process(dut_clk)
    begin
      if dut_clk'event and dut_clk = '1' then
        fa <= dut_lane_a; fb <= dut_lane_b; fc <= dut_lane_c;
      end if;
    end process;
  end generate;
  process(dut_clk)
  begin
    if dut_clk'event and dut_clk = '1' then
      la <= fa; lb <= fb; lc <= fc;
      -- control bits from the host side
      run_s1 <= run;        run_s2 <= run_s1;
      clr_s1 <= clear_tgl;  clr_s2 <= clr_s1;
      res_s1 <= resume_tgl; res_s2 <= res_s1;
    end if;
  end process;

  u_scan : entity work.em_scanner
    generic map (LANES => LANES, ADDR_WIDTH => ADDR_WIDTH, NUM_ADDR => NUM_ADDR,
                 SETTLE => SETTLE, SWEEP_LOG => SWEEP_LOG)
    port map (clk => dut_clk, rst => d_rst,
              run => run_s2, clear_tgl => clr_s2, resume_tgl => res_s2,
              rd_addr => dut_addr, lane_a => la, lane_b => lb, lane_c => lc,
              cnt_we => cnt_we, cnt_addr => cnt_addr_a, cnt_din => cnt_din, cnt_dout => cnt_dout_a,
              ev_we => ev_we, ev_data => ev_wdata, ev_full => ev_full,
              halted => halted, overflow => overflow, heartbeat => heartbeat);

  u_fifo : entity work.em_async_fifo
    generic map (DATA_W => EV_W, ADDR_W => FIFO_LOG)
    port map (wr_clk => dut_clk, wr_rst => d_rst, wr_en => ev_we, wr_data => ev_wdata, full => ev_full,
              rd_clk => sys_clk, rd_rst => sys_rst, rd_en => ev_rd, rd_data => ev_rdata, empty => ev_empty);

  u_cnt : entity work.em_dpram
    generic map (DATA_W => CNT_W, ADDR_W => ADDR_WIDTH)
    port map (clk_a => dut_clk, we_a => cnt_we, addr_a => cnt_addr_a, din_a => cnt_din, dout_a => cnt_dout_a,
              clk_b => sys_clk, addr_b => cnt_addr_b, dout_b => cnt_dout_b);

  u_host : entity work.em_host_if
    generic map (CLK_HZ => CLK_HZ, BAUD => BAUD, LANES => LANES, ADDR_WIDTH => ADDR_WIDTH,
                 NUM_ADDR => NUM_ADDR, SETTLE => SETTLE, ALIVE_US => ALIVE_US)
    port map (clk => sys_clk, rst => sys_rst, rxd => uart_rxd, txd => uart_txd,
              run => run, clear_tgl => clear_tgl, resume_tgl => resume_tgl,
              halted_a => halted, overflow_a => overflow, heartbeat_a => heartbeat,
              ev_rd => ev_rd, ev_data => ev_rdata, ev_empty => ev_empty,
              cnt_addr => cnt_addr_b, cnt_dout => cnt_dout_b, clock_ok => clock_ok);

  led_run <= run;
  led_halt <= halted;
  led_clock <= clock_ok;
end rtl;
