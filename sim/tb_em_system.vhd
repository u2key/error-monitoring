-- System testbench: a triplicated design with tmrdff_readout on "FPGA 1", the
-- tester on "FPGA 2", and the host as a UART bus-functional model. VHDL-2008
-- (testbench code): external names reach into the design to inject faults.
--
-- The design is sim/dut/areset.vhd converted with --tmr --light --probe (37
-- TMRDFF bits, 10 addresses at 4 lanes); run_sim.sh passes HOLD_LO, the TMRDFF
-- bit of HOLD(0), from the converter's map. Every lane and echo wire has its own
-- delay. Checks:
--   1. INFO; ratio commands (period measured, bad values and running refused)
--   2. before calibration the echoes reveal the misalignment
--   3. calibration finds a latency and per-line taps that pass; then a fault-free
--      scan shows no errors and no bad echoes
--   4. flips of one replica bit at random times, lasting to the next design edge
--      (as a voting flip-flop repairs them), are seen at the rate the visibility
--      formula says, at the right place, and never as configuration errors
--   5. a flip that stays is a configuration error: right place, scan halts;
--      released and resumed, the scan is clean again
--   6. the counter dump agrees with the events
--   7. list mode reads only the listed addresses, at the formula's rate; a list
--      of one is the fixed-address mode
--   8. a dead readout is reported as link lost, and back when it returns
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.em_pkg.all;

entity tb_em_system is
  generic (
    HOLD_LO : natural := 17;          -- TMRDFF bit of HOLD(0), from the map
    RANDOM_FLIPS : positive := 600
  );
end tb_em_system;

architecture tb of tb_em_system is
  constant WIDTH    : positive := 37;
  constant LANES    : positive := 4;
  constant ADDR_W   : positive := 4;
  constant NUM_ADDR : positive := 10;
  constant GUARD    : natural := 1;
  constant RATIO    : positive := 8;
  constant TAPS     : positive := 8;
  constant LMAX     : positive := 16;
  constant LINES    : positive := 3 * LANES + 3;
  constant CLK_HZ   : positive := 50000000;
  constant BAUD     : positive := 5000000;
  constant BIT_T    : time := 1 sec / BAUD;
  constant NO_FRAME : natural := 4095;

  -- a different delay on every wire, spanning more than half a read period
  function wire_delay(line : natural) return time is
  begin
    return 2 ns + ((line * 7) mod 13) * 1 ns;
  end function;

  signal read_clk, sample_clk : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal done : boolean := false;
  signal ro_alive : boolean := true;

  -- tester outputs and the wires to FPGA 1
  signal t_dclk, t_rclk, w_dclk, w_rclk : std_logic;
  signal t_addr_a, t_addr_b, t_addr_c, w_addr_a, w_addr_b, w_addr_c : std_logic_vector(ADDR_W - 1 downto 0);

  -- FPGA 1
  signal rst_dut, enb : std_logic := '0';
  signal din : std_logic_vector(3 downto 0) := (others => '0');
  signal addr : std_logic_vector(1 downto 0) := (others => '0');
  signal tmr_a, tmr_b, tmr_c : std_logic_vector(WIDTH - 1 downto 0);
  signal lane_a, lane_b, lane_c : std_logic_vector(LANES - 1 downto 0);
  signal echo_a, echo_b, echo_c : std_logic;

  -- the wires back, line by line in the tester's order
  signal lines_out, lines_w : std_logic_vector(LINES - 1 downto 0);

  -- host link
  signal h2d, d2h : std_logic := '1';

  -- frames received by the host model
  type bytes_t is array (0 to 31) of std_logic_vector(7 downto 0);
  type frames_t is array (0 to NO_FRAME) of bytes_t;
  type ints_t is array (0 to NO_FRAME) of natural;
  signal f_type : ints_t := (others => 0);
  signal f_pay : frames_t;
  signal n_frames : natural := 0;
  signal bad_frames : natural := 0;
begin
  read_clk <= not read_clk after 10 ns when not done else '0';
  sample_clk <= not sample_clk after 2.5 ns when not done else '0';

  -- the two clocks travel alike, so reads keep their phase to the design clock
  w_dclk <= transport t_dclk after 3 ns;
  w_rclk <= transport t_rclk after 3 ns when ro_alive else '0';
  w_addr_a <= transport t_addr_a after 3 ns;
  w_addr_b <= transport t_addr_b after 4 ns;
  w_addr_c <= transport t_addr_c after 5 ns;

  u_dut : entity work.areset
    port map (clk => w_dclk,
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
    port map (read_clk => w_rclk, tmrdff_a => tmr_a, tmrdff_b => tmr_b, tmrdff_c => tmr_c,
              addr_a => w_addr_a, addr_b => w_addr_b, addr_c => w_addr_c,
              lane_a => lane_a, lane_b => lane_b, lane_c => lane_c,
              echo_a => echo_a, echo_b => echo_b, echo_c => echo_c);

  lines_out <= echo_c & echo_b & echo_a & lane_c & lane_b & lane_a;
  g_wires : for i in 0 to LINES - 1 generate
    lines_w(i) <= transport lines_out(i) after wire_delay(i);
  end generate;

  u_det : entity work.em_detector_top
    generic map (CLK_HZ => CLK_HZ, BAUD => BAUD, LANES => LANES, ADDR_WIDTH => ADDR_W,
                 NUM_ADDR => NUM_ADDR, GUARD => GUARD, WMAX => 16, TAPS => TAPS, LMAX => LMAX,
                 LIST_BITS => 3, SWEEP_LOG => 8, FIFO_LOG => 6, TRAIN_RUN => 64, LINK_LOG => 8)
    port map (read_clk => read_clk, sample_clk => sample_clk, rst_n => rst_n,
              uart_rxd => h2d, uart_txd => d2h,
              dut_design_clk => t_dclk, dut_read_clk => t_rclk,
              dut_addr_a => t_addr_a, dut_addr_b => t_addr_b, dut_addr_c => t_addr_c,
              dut_lane_a => lines_w(LANES - 1 downto 0),
              dut_lane_b => lines_w(2 * LANES - 1 downto LANES),
              dut_lane_c => lines_w(3 * LANES - 1 downto 2 * LANES),
              dut_echo_a => lines_w(3 * LANES),
              dut_echo_b => lines_w(3 * LANES + 1),
              dut_echo_c => lines_w(3 * LANES + 2),
              led_run => open, led_halt => open, led_link => open);

  -- the design runs with random inputs all along
  stim : process(w_dclk)
    variable lfsr : unsigned(15 downto 0) := x"ACE1";
  begin
    if rising_edge(w_dclk) then
      lfsr := lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
      enb <= lfsr(0) or lfsr(5);
      din <= std_logic_vector(lfsr(4 downto 1));
      addr <= std_logic_vector(lfsr(7 downto 6));
    end if;
  end process;

  -- host model, receive side: decode frames
  rx : process
    variable b, sum : std_logic_vector(7 downto 0);
    variable typ, len : natural;
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
      elsif n_frames < NO_FRAME then
        f_type(n_frames) <= typ;
        f_pay(n_frames) <= pay;
        n_frames <= n_frames + 1;
      end if;
    else
      bad_frames <= bad_frames + 1;
    end if;
  end process;

  main : process
    alias hold is << signal .tb_em_system.u_dut.hold_a : std_logic_vector(3 downto 0) >>;
    constant T_ADDR : natural := HOLD_LO / LANES;
    constant T_LANE : natural := HOLD_LO mod LANES;
    constant W2_BIT : natural := HOLD_LO + 3;           -- HOLD(3): in another address
    constant T2_ADDR : natural := W2_BIT / LANES;
    variable seen : natural := 0;
    variable errors : natural := 0;
    variable idx, from, n : natural;
    variable seed1 : positive := 11;
    variable seed2 : positive := 73;
    variable r : real;
    variable lane_bit : std_logic_vector(7 downto 0);

    function u8(v : natural) return std_logic_vector is
    begin
      return std_logic_vector(to_unsigned(v, 8));
    end function;

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
    procedure expect(t : std_logic_vector(7 downto 0); variable i : out natural;
                     limit : time := 500 us) is
      variable deadline : time;
    begin
      deadline := now + limit;
      loop
        while seen < n_frames loop
          seen := seen + 1;
          if f_type(seen - 1) = to_integer(unsigned(t)) then
            i := seen - 1;
            return;
          end if;
        end loop;
        wait on n_frames for deadline - now;
        if now >= deadline then
          fail("no frame of type " & integer'image(to_integer(unsigned(t))));
          i := NO_FRAME;
          return;
        end if;
      end loop;
    end procedure;

    procedure acked(c : std_logic_vector(7 downto 0)) is
      variable i : natural;
    begin
      send(c);
      expect(FT_ACK, i);
    end procedure;

    impure function count(t : std_logic_vector(7 downto 0); first : natural) return natural is
      variable k : natural := 0;
    begin
      for i in first to n_frames - 1 loop
        if f_type(i) = to_integer(unsigned(t)) then
          k := k + 1;
        end if;
      end loop;
      return k;
    end function;

    function u16(hi, lo : std_logic_vector(7 downto 0)) return natural is
      variable w : std_logic_vector(15 downto 0);
    begin
      w := hi & lo;
      return to_integer(unsigned(w));
    end function;

    -- SOFT/CONFIG payload: addr(1) sweep(4) a1 b1 c1 a2 b2 c2 echo(1)
    impure function ev_addr(i : natural) return natural is
    begin
      return to_integer(unsigned(f_pay(i)(0)));
    end function;
    impure function ev_mask(i, m : natural) return std_logic_vector is
    begin
      return f_pay(i)(5 + m);
    end function;

    -- a flip of replica A's HOLD(b), from now to the next design clock edge
    procedure flip(b : natural) is
    begin
      hold(b) <= force not hold(b);
      wait until rising_edge(w_dclk);
      wait for 1 ns;
      hold(b) <= release;
    end procedure;

    -- k flips at random times, far enough apart for their frames to get out
    procedure flips(b, k : natural) is
    begin
      for i in 1 to k loop
        uniform(seed1, seed2, r);
        wait for 40 us + r * 20 us;
        flip(b);
      end loop;
      wait for 100 us;                    -- let the last frames arrive
    end procedure;

    -- seen flips against k * p, within 4 sigma
    procedure rate(name : string; got, k : natural; p : real) is
      constant m : real := real(k) * p;
    begin
      report name & ": " & integer'image(k) & " flips, " & integer'image(got) & " seen, expected "
             & integer'image(integer(m)) & " (P = " & real'image(p) & ")";
      if abs (real(got) - m) > 4.0 * sqrt(m * (1.0 - p)) then
        fail(name & ": detection rate does not match the formula");
      end if;
    end procedure;

    -- events from first on: none a configuration error, all at address a, lane
    -- as given, replica A only, no bad echoes
    procedure check_events(first, a : natural; lb : std_logic_vector(7 downto 0)) is
    begin
      for i in first to n_frames - 1 loop
        if f_type(i) = to_integer(unsigned(FT_CONFIG)) then
          fail("a flip was taken for a configuration error");
        elsif f_type(i) = to_integer(unsigned(FT_SOFT)) then
          if ev_addr(i) /= a then
            fail("flip-flop error at address " & integer'image(ev_addr(i)));
          end if;
          if (ev_mask(i, 0) or ev_mask(i, 3)) /= lb or
             (ev_mask(i, 1) or ev_mask(i, 2) or ev_mask(i, 4) or ev_mask(i, 5)) /= x"00" then
            fail("flip-flop error masks");
          end if;
          if f_pay(i)(11) /= x"00" then
            fail("bad echo marked on an event");
          end if;
        end if;
      end loop;
    end procedure;

    type pass_t is array (1 to LMAX, 0 to TAPS - 1) of std_logic_vector(LINES - 1 downto 0);
    variable pass : pass_t;
    type tap_t is array (0 to LINES - 1) of natural;
    variable best_tap : tap_t;
    variable best_l, best_score : natural := 0;
    variable score, run_len, top, top_at : natural;
    variable this_tap : tap_t;
    variable t0 : time;
  begin
    lane_bit := (others => '0');
    lane_bit(T_LANE) := '1';
    rst_n <= '0'; rst_dut <= '0';
    wait for 200 ns;
    rst_n <= '1'; rst_dut <= '1';
    wait for 1 us;

    -- 1. INFO and the ratio
    send(CMD_INFO);
    expect(FT_INFO, idx);
    if idx /= NO_FRAME then
      if u16(f_pay(idx)(1), f_pay(idx)(2)) /= LANES or to_integer(unsigned(f_pay(idx)(7))) /= NUM_ADDR
         or to_integer(unsigned(f_pay(idx)(8))) /= GUARD or to_integer(unsigned(f_pay(idx)(11))) /= TAPS
         or to_integer(unsigned(f_pay(idx)(12))) /= LMAX or u16(f_pay(idx)(13), f_pay(idx)(14)) /= 8 then
        fail("INFO contents");
      end if;
    end if;
    send(CMD_RATIO); send(x"00"); send(x"00"); send(x"05");
    expect(FT_ACK, idx);
    wait for 1 us;
    wait until rising_edge(w_dclk);
    t0 := now;
    wait until rising_edge(w_dclk);
    if now - t0 /= 100 ns then
      fail("design clock period with ratio 5: " & time'image(now - t0));
    end if;
    send(CMD_RATIO); send(x"00"); send(x"00"); send(x"01");
    expect(FT_NACK, idx);
    send(CMD_RATIO); send(x"00"); send(x"00"); send(u8(RATIO));
    expect(FT_ACK, idx);
    wait for 1 us;
    wait until rising_edge(w_dclk);
    t0 := now;
    wait until rising_edge(w_dclk);
    if now - t0 /= RATIO * 20 ns then
      fail("design clock period with ratio 8: " & time'image(now - t0));
    end if;

    -- 2. uncalibrated: a wrong latency shows in the echoes
    send(CMD_LAT); send(x"0E");
    expect(FT_ACK, idx);
    acked(CMD_START);
    send(CMD_RATIO); send(x"00"); send(x"00"); send(u8(RATIO));
    expect(FT_NACK, idx);
    wait for 40 us;
    send(CMD_STATUS);
    expect(FT_STATUS, idx);
    if idx /= NO_FRAME and f_pay(idx)(1) & f_pay(idx)(2) & f_pay(idx)(3) & f_pay(idx)(4) = x"00000000" then
      fail("no bad echoes at a wrong latency");
    end if;
    acked(CMD_STOP);
    wait for 50 us;
    seen := n_frames;

    -- 3. calibration: what em_host.py does
    for l in 1 to LMAX loop
      send(CMD_LAT); send(u8(l));
      expect(FT_ACK, idx);
      for t in 0 to TAPS - 1 loop
        send(CMD_TAP); send(x"FF"); send(u8(t));
        expect(FT_ACK, idx);
        send(CMD_TRAIN);
        expect(FT_TRAIN, idx);
        pass(l, t) := f_pay(idx)(3)(2 downto 0) & f_pay(idx)(2)(LANES - 1 downto 0)
                      & f_pay(idx)(1)(LANES - 1 downto 0) & f_pay(idx)(0)(LANES - 1 downto 0);
      end loop;
    end loop;
    for l in 1 to LMAX loop
      score := TAPS + 1;
      for line in 0 to LINES - 1 loop
        top := 0; top_at := 0; run_len := 0;
        for t in 0 to TAPS - 1 loop
          if pass(l, t)(line) = '1' then
            run_len := run_len + 1;
            if run_len > top then
              top := run_len;
              top_at := t;
            end if;
          else
            run_len := 0;
          end if;
        end loop;
        this_tap(line) := top_at - top / 2;
        if top = 0 then
          this_tap(line) := 0;
        end if;
        if top < score then
          score := top;
        end if;
      end loop;
      if score > best_score then
        best_score := score;
        best_l := l;
        best_tap := this_tap;
      end if;
    end loop;
    report "calibration: latency " & integer'image(best_l) & ", narrowest eye "
           & integer'image(best_score) & " taps";
    if best_score = 0 then
      fail("calibration found no latency that passes");
    else
      send(CMD_LAT); send(u8(best_l));
      expect(FT_ACK, idx);
      for line in 0 to LINES - 1 loop
        send(CMD_TAP); send(u8(line)); send(u8(best_tap(line)));
        expect(FT_ACK, idx);
      end loop;
      send(CMD_TRAIN);
      expect(FT_TRAIN, idx);
      if f_pay(idx)(0)(3 downto 0) /= x"F" or f_pay(idx)(1)(3 downto 0) /= x"F"
         or f_pay(idx)(2)(3 downto 0) /= x"F" or f_pay(idx)(3)(2 downto 0) /= "111" then
        fail("training does not pass at the calibrated settings");
      end if;
    end if;

    send(CMD_CLEAR);
    expect(FT_ACK, idx);
    expect(FT_CLEARED, idx);
    from := n_frames;
    acked(CMD_START);
    send(CMD_CLEAR);
    expect(FT_NACK, idx);
    wait for 40 us;
    if count(FT_SOFT, from) + count(FT_CONFIG, from) /= 0 then
      fail("errors reported by a fault-free design");
    end if;
    send(CMD_STATUS);
    expect(FT_STATUS, idx);
    if f_pay(idx)(0)(2) /= '1' or f_pay(idx)(1) & f_pay(idx)(2) & f_pay(idx)(3) & f_pay(idx)(4) /= x"00000000"
       or f_pay(idx)(5) & f_pay(idx)(6) & f_pay(idx)(7) & f_pay(idx)(8) /= x"00000000"
       or f_pay(idx)(9) & f_pay(idx)(10) & f_pay(idx)(11) & f_pay(idx)(12) /= x"00000000" then
      fail("bad echoes after calibration");
    end if;

    -- 4. random flips, in a sweep of 2 windows of 7: T_ADDR at phase GUARD + T_ADDR
    from := n_frames;
    seen := n_frames;
    flips(0, RANDOM_FLIPS);
    rate("sweep", count(FT_SOFT, from), RANDOM_FLIPS,
         (real(GUARD + T_ADDR) + 0.5) / real(RATIO) / 2.0);
    check_events(from, T_ADDR, lane_bit);

    -- 5. a flip that stays: configuration error, halt, resume
    from := n_frames;
    seen := n_frames;
    hold(0) <= force not hold(0);
    expect(FT_CONFIG, idx);
    if idx /= NO_FRAME then
      if ev_addr(idx) /= T_ADDR or ev_mask(idx, 0) /= lane_bit or ev_mask(idx, 3) /= lane_bit then
        fail("configuration error place");
      end if;
    end if;
    send(CMD_STATUS);
    expect(FT_STATUS, idx);
    if idx /= NO_FRAME and f_pay(idx)(0)(1) /= '1' then
      fail("not halted after a configuration error");
    end if;
    wait for 20 us;
    if count(FT_CONFIG, from) /= 1 then
      fail("configuration error reported more than once while halted");
    end if;
    hold(0) <= release;                    -- the host reconfigures the FPGA
    wait for 1 us;
    send(CMD_RESUME);
    expect(FT_ACK, idx);
    from := n_frames;
    wait for 40 us;
    if count(FT_SOFT, from) + count(FT_CONFIG, from) /= 0 then
      fail("errors after resume");
    end if;

    -- 6. counter dump against the events since the clear
    acked(CMD_STOP);
    wait for 100 us;
    n := 0;
    from := 0;
    for i in 0 to n_frames - 1 loop
      if f_type(i) = to_integer(unsigned(FT_CLEARED)) then
        from := i;
      end if;
    end loop;
    n := count(FT_SOFT, from);
    from := n_frames;
    seen := n_frames;
    send(CMD_DUMP);
    expect(FT_DUMPEND, idx, 1 ms);
    for i in from to n_frames - 1 loop
      if f_type(i) = to_integer(unsigned(FT_COUNTER)) then
        if ev_addr(i) = T_ADDR then
          if u16(f_pay(i)(1), f_pay(i)(2)) /= n then
            fail("soft counter " & integer'image(u16(f_pay(i)(1), f_pay(i)(2))) & " /= " & integer'image(n));
          end if;
          if to_integer(unsigned(f_pay(i)(3))) /= 1 then
            fail("configuration counter");
          end if;
        elsif f_pay(i)(1) /= x"00" or f_pay(i)(2) /= x"00" or f_pay(i)(3) /= x"00" then
          fail("counter at address " & integer'image(ev_addr(i)));
        end if;
      end if;
    end loop;
    if count(FT_COUNTER, from) /= NUM_ADDR then
      fail("dump has " & integer'image(count(FT_COUNTER, from)) & " counters");
    end if;

    -- 7. a list of three addresses, T_ADDR in the middle (not T2_ADDR)
    send(CMD_LIST); send(x"00"); send(x"03");
    send(u8(2)); send(u8(T_ADDR)); send(u8(7));
    expect(FT_ACK, idx);
    send(CMD_MODE); send(x"01");
    expect(FT_ACK, idx);
    from := n_frames;
    seen := n_frames;
    acked(CMD_START);
    flips(0, 200);
    rate("list", count(FT_SOFT, from), 200, (real(RATIO - 3 + 1) + 0.5) / real(RATIO));
    check_events(from, T_ADDR, lane_bit);
    from := n_frames;
    flips(3, 50);                          -- a bit the list does not read
    if count(FT_SOFT, from) /= 0 then
      fail("an address not on the list was reported");
    end if;
    -- a list of one: the fixed-address mode
    acked(CMD_STOP);
    wait for 50 us;
    send(CMD_LIST); send(x"00"); send(x"01");
    send(u8(T2_ADDR));
    expect(FT_ACK, idx);
    from := n_frames;
    seen := n_frames;
    acked(CMD_START);
    flips(3, 80);
    rate("fixed", count(FT_SOFT, from), 80, (real(RATIO - 1) + 0.5) / real(RATIO));
    lane_bit := (others => '0');
    lane_bit(W2_BIT mod LANES) := '1';
    check_events(from, T2_ADDR, lane_bit);
    acked(CMD_STOP);
    wait for 50 us;
    send(CMD_MODE); send(x"00");
    expect(FT_ACK, idx);

    -- 8. the readout dies and comes back
    acked(CMD_START);
    seen := n_frames;
    ro_alive <= false;
    expect(FT_LINK, idx, 200 us);
    if idx /= NO_FRAME and f_pay(idx)(0) /= x"00" then
      fail("link loss not reported");
    end if;
    ro_alive <= true;
    expect(FT_LINK, idx, 200 us);
    if idx /= NO_FRAME and f_pay(idx)(0) /= x"01" then
      fail("link return not reported");
    end if;
    acked(CMD_STOP);

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
