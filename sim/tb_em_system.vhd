-- System testbench: a triplicated design on "FPGA 1" with tmrdff_readout,
-- the detector on "FPGA 2", and the host as a UART bus-functional model.
-- VHDL-2008 (testbench code): external names reach into the design to inject
-- faults.
--
-- The design is sim/dut/areset.vhd converted with --tmr --light --probe
-- (37 TMRDFF bits); run_sim.sh passes HOLD_LO, the TMRDFF bit of HOLD(0), from
-- the converter's map. Checks:
--   1. INFO, CLEAR, START are answered
--   2. a one-cycle flip of one replica bit, injected at every offset across
--      an address visit, is reported as a flip-flop error at the right address
--      and lane or not seen at all -- never as a configuration error -- and is
--      seen at least once
--   3. a flip held in place is a configuration error: reported at the right
--      place, the scan halts; after "reconfiguration" (release) and RESUME it
--      scans again without errors
--   4. the counter dump agrees with the events received
--   5. stopping the forwarded clock reports CLOCK lost, restarting it back
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.em_pkg.all;

entity tb_em_system is
  generic (
    HOLD_LO : natural := 17;          -- TMRDFF bit of HOLD(0), from the map
    DEBUG   : boolean := false;       -- report every frame received
    RANDOM_FLIPS : positive := 1500
  );
end tb_em_system;

architecture tb of tb_em_system is
  constant WIDTH    : positive := 37;
  constant LANES    : positive := 8;
  constant ADDR_W   : positive := 3;
  constant NUM_ADDR : positive := 5;
  constant SETTLE   : positive := 8;
  constant CLK_HZ   : positive := 50000000;
  constant BAUD     : positive := 5000000;
  constant BIT_T    : time := 1 sec / BAUD;

  signal sys_clk : std_logic := '0';
  signal dclk : std_logic := '0';
  signal dclk_run : boolean := true;
  signal rst_n : std_logic := '0';
  signal done : boolean := false;

  -- FPGA 1
  signal rst_dut, enb, ta_rst : std_logic := '0';
  signal din : std_logic_vector(3 downto 0) := (others => '0');
  signal addr : std_logic_vector(1 downto 0) := (others => '0');
  signal tmr_a, tmr_b, tmr_c : std_logic_vector(WIDTH - 1 downto 0);
  signal lane_a, lane_b, lane_c : std_logic_vector(LANES - 1 downto 0);
  signal clk_fwd : std_logic;
  -- the board-to-board wires, with some delay
  signal w_lane_a, w_lane_b, w_lane_c : std_logic_vector(LANES - 1 downto 0);
  signal w_clk : std_logic;
  signal det_addr, w_addr : std_logic_vector(ADDR_W - 1 downto 0);

  -- host link
  signal h2d, d2h : std_logic := '1';

  -- frames received by the host model
  type bytes_t is array (0 to 31) of std_logic_vector(7 downto 0);
  type frames_t is array (0 to 1023) of bytes_t;
  type ints_t is array (0 to 1023) of natural;
  signal f_type : ints_t := (others => 0);
  signal f_len : ints_t := (others => 0);
  signal f_pay : frames_t;
  signal n_frames : natural := 0;
  signal bad_frames : natural := 0;
begin
  sys_clk <= not sys_clk after 10 ns when not done else '0';
  dclk <= not dclk after 25 ns when dclk_run and not done else dclk;

  -- FPGA 1: the triplicated design and its readout
  u_dut : entity work.areset
    port map (clk => dclk,
              rst_n_a => rst_dut, rst_n_b => rst_dut, rst_n_c => rst_dut,
              enb_a => enb, enb_b => enb, enb_c => enb,
              din_a => din, din_b => din, din_c => din,
              addr_a => addr, addr_b => addr, addr_c => addr,
              q_hold_a => open, q_hold_b => open, q_hold_c => open,
              q_mem_a => open, q_mem_b => open, q_mem_c => open,
              q_acc_a => open, q_acc_b => open, q_acc_c => open,
              q_tmp_a => open, q_tmp_b => open, q_tmp_c => open,
              q_k_a => open, q_k_b => open, q_k_c => open,
              q_r_a => open, q_r_b => open, q_r_c => open,
              tmrdff_a => tmr_a, tmrdff_b => tmr_b, tmrdff_c => tmr_c);
  u_ro : entity work.tmrdff_readout
    generic map (WIDTH => WIDTH, LANES => LANES, ADDR_WIDTH => ADDR_W)
    port map (clk => dclk, tmrdff_a => tmr_a, tmrdff_b => tmr_b, tmrdff_c => tmr_c,
              rd_addr => w_addr, lane_a => lane_a, lane_b => lane_b, lane_c => lane_c,
              clk_out => clk_fwd);

  w_lane_a <= lane_a after 4 ns;
  w_lane_b <= lane_b after 4 ns;
  w_lane_c <= lane_c after 4 ns;
  w_clk <= clk_fwd after 3 ns;
  w_addr <= det_addr after 4 ns;

  -- FPGA 2: the detector
  u_det : entity work.em_detector_top
    generic map (CLK_HZ => CLK_HZ, BAUD => BAUD, LANES => LANES, ADDR_WIDTH => ADDR_W,
                 NUM_ADDR => NUM_ADDR, SETTLE => SETTLE, SWEEP_LOG => 6, FIFO_LOG => 4,
                 ALIVE_US => 20)
    port map (sys_clk => sys_clk, rst_n => rst_n, uart_rxd => h2d, uart_txd => d2h,
              dut_clk => w_clk, dut_lane_a => w_lane_a, dut_lane_b => w_lane_b, dut_lane_c => w_lane_c,
              dut_addr => det_addr, led_run => open, led_halt => open, led_clock => open);

  -- the design runs with random inputs all along
  stim : process(dclk)
    variable lfsr : unsigned(15 downto 0) := x"ACE1";
  begin
    if rising_edge(dclk) then
      lfsr := lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
      enb <= lfsr(0) or lfsr(5);
      din <= std_logic_vector(lfsr(4 downto 1));
      addr <= std_logic_vector(lfsr(7 downto 6));
    end if;
  end process;

  -- host model, receive side: decode frames
  rx : process
    variable b : std_logic_vector(7 downto 0);
    variable typ, len : natural;
    variable sum : std_logic_vector(7 downto 0);
    variable pay : bytes_t;
    procedure getb(variable v : out std_logic_vector(7 downto 0)) is
    begin
      wait until d2h = '0';
      wait for BIT_T / 2;
      for i in 0 to 7 loop
        wait for BIT_T;
        v(i) := d2h;
      end loop;
      wait for BIT_T;                     -- stop bit
    end procedure;
  begin
    getb(b);
    if b = FRAME_SYNC then
      getb(b); typ := to_integer(unsigned(b)); sum := b;
      getb(b); len := to_integer(unsigned(b)); sum := sum xor b;
      for i in 0 to len - 1 loop
        getb(b); pay(i) := b; sum := sum xor b;
      end loop;
      getb(b);
      if b /= sum then
        bad_frames <= bad_frames + 1;
      else
        if DEBUG then
          report "frame type " & integer'image(typ) & " len " & integer'image(len)
                 & " b0 " & integer'image(to_integer(unsigned(pay(0))));
        end if;
        f_type(n_frames) <= typ;
        f_len(n_frames) <= len;
        f_pay(n_frames) <= pay;
        n_frames <= n_frames + 1;
      end if;
    else
      bad_frames <= bad_frames + 1;
    end if;
  end process;

  dbg : process
    alias ev_we is << signal .tb_em_system.u_det.ev_we : std_logic >>;
    alias ev_d is << signal .tb_em_system.u_det.ev_wdata : std_logic_vector(8 + ADDR_W + 32 + 6 * LANES - 1 downto 0) >>;
    alias ev_full is << signal .tb_em_system.u_det.ev_full : std_logic >>;
  begin
    wait until rising_edge(w_clk);
    if DEBUG and ev_we = '1' then
      report "FIFO write type " & integer'image(to_integer(unsigned(ev_d(ev_d'left downto ev_d'left - 7))))
             & " full=" & std_logic'image(ev_full);
    end if;
  end process;

  main : process
    alias hold is << signal .tb_em_system.u_dut.hold_a : std_logic_vector(3 downto 0) >>;
    alias ro_addr is << signal .tb_em_system.u_ro.addr_s2 : std_logic_vector(ADDR_W - 1 downto 0) >>;
    constant T_ADDR : natural := HOLD_LO / LANES;
    constant T_LANE : natural := HOLD_LO mod LANES;
    variable seen : natural := 0;
    variable errors : natural := 0;
    variable soft_events, soft_seen : natural := 0;
    variable a : natural;

    procedure send(c : std_logic_vector(7 downto 0)) is
    begin
      h2d <= '0'; wait for BIT_T;
      for i in 0 to 7 loop
        h2d <= c(i); wait for BIT_T;
      end loop;
      h2d <= '1'; wait for BIT_T;
    end procedure;

    procedure fail(msg : string) is
    begin
      report "CHECK FAILED: " & msg severity error;
      errors := errors + 1;
    end procedure;

    -- wait for the next frame of type t (skipping others); returns its index
    procedure expect(t : std_logic_vector(7 downto 0); variable idx : out natural;
                     limit : time := 200 us) is
      variable deadline : time;
    begin
      deadline := now + limit;
      loop
        while seen < n_frames loop
          seen := seen + 1;
          if f_type(seen - 1) = to_integer(unsigned(t)) then
            idx := seen - 1;
            return;
          end if;
        end loop;
        wait on n_frames for deadline - now;
        if now >= deadline then
          fail("no frame of type " & integer'image(to_integer(unsigned(t))));
          idx := 1023;
          return;
        end if;
      end loop;
    end procedure;

    -- frames of type t received from index from on
    impure function count(t : std_logic_vector(7 downto 0); from : natural) return natural is
      variable n : natural := 0;
    begin
      for i in from to n_frames - 1 loop
        if f_type(i) = to_integer(unsigned(t)) then
          n := n + 1;
        end if;
      end loop;
      return n;
    end function;

    -- an event's address and the masks of replica A in both samples
    -- (payload: addr(1) sweep(4) a1 b1 c1 a2 b2 c2, one byte each here)
    function u16(hi, lo : std_logic_vector(7 downto 0)) return natural is
      variable w : std_logic_vector(15 downto 0);
    begin
      w := hi & lo;
      return to_integer(unsigned(w));
    end function;

    impure function ev_addr(i : natural) return natural is
    begin
      return to_integer(unsigned(f_pay(i)(0)));
    end function;
    impure function ev_mask(i : natural; m : natural) return std_logic_vector is
    begin
      return f_pay(i)(5 + m);
    end function;

    procedure wait_sweeps(n : natural) is
      variable k : natural;
    begin
      for i in 1 to n loop
        wait until ro_addr = std_logic_vector(to_unsigned(NUM_ADDR - 1, ADDR_W));
        wait until ro_addr = std_logic_vector(to_unsigned(0, ADDR_W));
      end loop;
    end procedure;

    variable idx, from : natural;
    variable seed1 : positive := 11;
    variable seed2 : positive := 73;
    variable r : real;
    variable lane_bit : std_logic_vector(7 downto 0);
  begin
    lane_bit := (others => '0');
    lane_bit(T_LANE) := '1';
    rst_n <= '0'; rst_dut <= '0';
    wait for 200 ns;
    rst_n <= '1'; rst_dut <= '1';
    wait for 1 us;

    -- 1. INFO, CLEAR, START
    send(CMD_INFO);
    expect(FT_INFO, idx);
    if idx < 1023 then
      if u16(f_pay(idx)(1), f_pay(idx)(2)) /= LANES or
         to_integer(unsigned(f_pay(idx)(7))) /= NUM_ADDR then
        fail("INFO contents");
      end if;
    end if;
    send(CMD_CLEAR);
    expect(FT_ACK, idx);
    expect(FT_CLEARED, idx);
    send(CMD_START);
    expect(FT_ACK, idx);
    wait_sweeps(3);
    if count(FT_SOFT, 0) + count(FT_CONFIG, 0) /= 0 then
      fail("errors reported by a fault-free design");
    end if;

    -- 2. one-cycle flips, at every offset across a visit of the target address
    from := n_frames;
    for offset in 0 to SETTLE + 6 loop
      wait until ro_addr = std_logic_vector(to_unsigned(T_ADDR, ADDR_W));
      for i in 1 to offset loop
        wait until rising_edge(dclk);
      end loop;
      wait for 1 ns;                       -- just after the edge
      hold(0) <= force not hold(0);
      wait until rising_edge(dclk);
      wait for 1 ns;
      hold(0) <= release;
      wait_sweeps(2);
    end loop;
    wait for 100 us;                       -- let the last frames arrive
    for i in from to n_frames - 1 loop
      if f_type(i) = to_integer(unsigned(FT_CONFIG)) then
        fail("a one-cycle flip was taken for a configuration error");
      elsif f_type(i) = to_integer(unsigned(FT_SOFT)) then
        soft_events := soft_events + 1;
        if ev_addr(i) /= T_ADDR then
          fail("flip-flop error at address " & integer'image(ev_addr(i)));
        end if;
        -- replica A, sample 1 or sample 2, exactly the target lane
        if (ev_mask(i, 0) or ev_mask(i, 3)) /= lane_bit or
           (ev_mask(i, 1) or ev_mask(i, 2) or ev_mask(i, 4) or ev_mask(i, 5)) /= x"00" then
          fail("flip-flop error masks");
        end if;
      end if;
    end loop;
    report "one-cycle flips: " & integer'image(SETTLE + 7) & " injected, "
           & integer'image(soft_events) & " seen as flip-flop errors";
    if soft_events = 0 then
      fail("no one-cycle flip was seen");
    end if;

    -- 2b. the correction factor: one-cycle flips at random times are seen with
    -- probability 2 / (NUM_ADDR * (SETTLE + 4)), which host/em_host.py divides by
    from := n_frames;
    for k in 1 to RANDOM_FLIPS loop
      uniform(seed1, seed2, r);
      for i in 1 to 61 + integer(trunc(r * 120.0)) loop
        wait until rising_edge(dclk);
      end loop;
      wait for 1 ns;
      hold(0) <= force not hold(0);
      wait until rising_edge(dclk);
      wait for 1 ns;
      hold(0) <= release;
    end loop;
    wait for 100 us;
    report "random flips: " & integer'image(RANDOM_FLIPS) & " injected, "
           & integer'image(count(FT_SOFT, from)) & " seen, expected "
           & integer'image(RANDOM_FLIPS * 2 / (NUM_ADDR * (SETTLE + 4)));
    if abs (real(count(FT_SOFT, from)) - real(RANDOM_FLIPS) * 2.0 / real(NUM_ADDR * (SETTLE + 4)))
       > 4.0 * sqrt(real(RANDOM_FLIPS) * 2.0 / real(NUM_ADDR * (SETTLE + 4))) then
      fail("detection rate does not match the correction factor");
    end if;
    if count(FT_CONFIG, from) /= 0 then
      fail("a random one-cycle flip was taken for a configuration error");
    end if;

    -- 3. a flip that stays: configuration error, halt, resume
    from := n_frames;
    hold(0) <= force not hold(0);
    expect(FT_CONFIG, idx, 200 us);
    if idx < 1023 then
      if ev_addr(idx) /= T_ADDR or ev_mask(idx, 0) /= lane_bit or ev_mask(idx, 3) /= lane_bit then
        fail("configuration error place");
      end if;
    end if;
    send(CMD_STATUS);
    expect(FT_STATUS, idx);
    if idx < 1023 and f_pay(idx)(0)(1) /= '1' then
      fail("not halted after a configuration error");
    end if;
    wait for 5 us;
    if count(FT_CONFIG, from) /= 1 then
      fail("configuration error reported more than once while halted");
    end if;
    hold(0) <= release;                    -- the host reconfigures the FPGA
    send(CMD_RESUME);
    expect(FT_ACK, idx);
    from := n_frames;
    wait_sweeps(3);
    if count(FT_SOFT, from) + count(FT_CONFIG, from) /= 0 then
      fail("errors after resume");
    end if;

    -- 4. counter dump against the events. A held flip can show in one sample
    -- of a visit before it shows in both, so it may add flip-flop errors too:
    -- compare with every flip-flop error event received.
    send(CMD_STOP);
    expect(FT_ACK, idx);
    wait for 50 us;
    soft_events := count(FT_SOFT, 0);
    from := n_frames;
    send(CMD_DUMP);
    expect(FT_DUMPEND, idx, 100 us);
    for i in from to n_frames - 1 loop
      if f_type(i) = to_integer(unsigned(FT_COUNTER)) then
        a := ev_addr(i);
        if a = T_ADDR then
          if u16(f_pay(i)(1), f_pay(i)(2)) /= soft_events then
            fail("soft counter " & integer'image(u16(f_pay(i)(1), f_pay(i)(2)))
                 & " /= " & integer'image(soft_events));
          end if;
          if to_integer(unsigned(f_pay(i)(3))) /= 1 then
            fail("configuration counter");
          end if;
        elsif f_pay(i)(1) /= x"00" or f_pay(i)(2) /= x"00" or f_pay(i)(3) /= x"00" then
          fail("counter at address " & integer'image(a));
        end if;
      end if;
    end loop;
    if count(FT_COUNTER, from) /= NUM_ADDR then
      fail("dump has " & integer'image(count(FT_COUNTER, from)) & " counters");
    end if;

    -- 5. the forwarded clock stops and comes back
    send(CMD_START);
    expect(FT_ACK, idx);
    from := n_frames;
    dclk_run <= false;
    expect(FT_CLOCK, idx, 100 us);
    if idx < 1023 and f_pay(idx)(0) /= x"00" then
      fail("clock loss not reported");
    end if;
    dclk_run <= true;
    expect(FT_CLOCK, idx, 100 us);
    if idx < 1023 and f_pay(idx)(0) /= x"01" then
      fail("clock return not reported");
    end if;

    if bad_frames /= 0 then
      fail(integer'image(bad_frames) & " malformed frames");
    end if;
    if errors = 0 then
      report "TB PASS: " & integer'image(n_frames) & " frames";
    else
      report "TB FAIL: " & integer'image(errors) & " check(s) failed" severity error;
    end if;
    done <= true;
    wait;
  end process;
end tb;
