-- Tester core: design clock, windowed address scan, classification. VHDL-93.
-- Everything runs on read_clk, the tester's own clock.
--
-- The design clock is read_clk divided by R (host-set, at least 2). Each design
-- clock period reads a window of up to W = min(N, R - GUARD, WMAX) addresses, one
-- per read cycle, in the last W read cycles of the period. The next period reads
-- the same window at the same phases, and then the scan moves to the next window:
--
--     R = 8:  [0 1 .. 6] edge [0 1 .. 6] edge [7 8 .. 13] edge [7 8 .. 13] ...
--
-- So every address is read twice with exactly one design clock edge in between,
-- and nothing stands still waiting. Per bit, a read is in error when the three
-- replicas are not all equal:
--
--   error in one read only  -> flip-flop error: a voting triplicated flip-flop
--                              repairs an upset at the next design clock edge
--   error in both reads     -> configuration error: the scan halts until the
--                              host resumes it (after reconfiguring the FPGA)
--
-- Reads take L cycles to come back (L set by the host from training), so a tag
-- travels along with each address and the sample is matched to its tag there;
-- the first read's masks wait in a buffer for the second. echo_x must equal the
-- parity of the tagged address: a difference is counted per replica and marks
-- the event, and many of them over a stretch of reads means the link is lost.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.em_pkg.all;

entity em_scanner is
  generic (
    LANES      : positive;
    ADDR_WIDTH : positive;
    NUM_ADDR   : positive;
    GUARD      : natural := 1;       -- read cycles after a design edge left unread
    WMAX       : positive := 256;    -- largest window (first-read buffer depth)
    LMAX       : positive := 32;     -- largest latency
    LIST_BITS  : positive := 12;     -- list depth 2**LIST_BITS
    SWEEP_LOG  : natural := 8;       -- report the sweep count every 2**SWEEP_LOG sweeps
    TRAIN_RUN  : positive := 256;    -- reads compared per training run
    LINK_LOG   : positive := 16      -- link judged over 2**LINK_LOG reads (at least 2)
  );
  port (
    clk        : in  std_logic;
    rst        : in  std_logic;
    -- control from the host interface
    run        : in  std_logic;
    mode_list  : in  std_logic;
    ratio      : in  unsigned(23 downto 0);
    lat        : in  unsigned(5 downto 0);
    list_len   : in  unsigned(LIST_BITS downto 0);
    clear_p    : in  std_logic;
    resume_p   : in  std_logic;
    train_p    : in  std_logic;
    -- list RAM, read side
    list_raddr : out std_logic_vector(LIST_BITS - 1 downto 0);
    list_dout  : in  std_logic_vector(ADDR_WIDTH - 1 downto 0);
    -- to the FPGA under test
    design_clk : out std_logic;
    addr_out   : out std_logic_vector(ADDR_WIDTH - 1 downto 0);
    lane_a     : in  std_logic_vector(LANES - 1 downto 0);
    lane_b     : in  std_logic_vector(LANES - 1 downto 0);
    lane_c     : in  std_logic_vector(LANES - 1 downto 0);
    echo       : in  std_logic_vector(2 downto 0);   -- c, b, a
    -- counters (RAM port A)
    cnt_we     : out std_logic;
    cnt_addr   : out std_logic_vector(ADDR_WIDTH - 1 downto 0);
    cnt_din    : out std_logic_vector(SOFT_CNT_W + CFG_CNT_W - 1 downto 0);
    cnt_dout   : in  std_logic_vector(SOFT_CNT_W + CFG_CNT_W - 1 downto 0);
    -- events to the host interface
    ev_we      : out std_logic;
    ev_data    : out std_logic_vector(8 + ADDR_WIDTH + 32 + 6 * LANES + 6 - 1 downto 0);
    ev_full    : in  std_logic;
    -- status
    halted     : out std_logic;
    busy       : out std_logic;           -- scanning, clearing or training
    overflow   : out std_logic;           -- an event was lost
    link_ok    : out std_logic;
    echo_bad_a : out unsigned(31 downto 0);
    echo_bad_b : out unsigned(31 downto 0);
    echo_bad_c : out unsigned(31 downto 0)
  );
end em_scanner;

architecture rtl of em_scanner is
  constant EV_W : positive := 8 + ADDR_WIDTH + 32 + 6 * LANES + 6;
  constant IDX_BITS : positive := clog2(WMAX + 1);
  constant BUF_W : positive := 3 * LANES + 3;      -- masks a b c, echo-bad bits
  constant SWEEP_MASK : unsigned(31 downto 0) := to_unsigned(2**SWEEP_LOG - 1, 32);
  subtype lane_t is std_logic_vector(LANES - 1 downto 0);
  subtype addr_t is std_logic_vector(ADDR_WIDTH - 1 downto 0);

  type state_t is (S_IDLE, S_CLEAR, S_SCAN, S_DRAIN, S_HALT, S_TRAIN);
  signal state : state_t := S_IDLE;

  -- design clock: g is the phase of the read being generated now
  signal r_cur : unsigned(23 downto 0) := to_unsigned(2, 24);
  signal g : unsigned(23 downto 0) := (others => '0');
  signal dc1, dc2, dc3, dc4 : std_logic := '0';

  -- window
  signal w_full, w_cur : unsigned(23 downto 0) := to_unsigned(1, 24);
  signal base : unsigned(ADDR_WIDTH downto 0) := (others => '0');
  signal num : unsigned(ADDR_WIDTH downto 0) := to_unsigned(NUM_ADDR, ADDR_WIDTH + 1);
  signal pass : std_logic := '0';
  signal sweep : unsigned(31 downto 0) := (others => '0');

  -- stage 1: which read (and the list read); stage 2: the list word is out;
  -- then the address is launched with its tag
  signal s1_valid, s1_train, s1_pass, s1_exp, s1_list : std_logic := '0';
  signal s2_valid, s2_train, s2_pass, s2_exp, s2_list : std_logic := '0';
  signal s1_idx, s2_idx : unsigned(IDX_BITS - 1 downto 0) := (others => '0');
  signal s1_addr, s2_addr : addr_t := (others => '0');
  signal lat_i : natural range 1 to LMAX := 1;

  type tag_t is record
    valid : std_logic;
    train : std_logic;       -- a training read, not a scan read
    pass  : std_logic;
    exp   : std_logic;       -- training: the expected level
    idx   : unsigned(IDX_BITS - 1 downto 0);
    addr  : addr_t;
  end record;
  type tags_t is array (0 to LMAX) of tag_t;
  constant NO_TAG : tag_t := ('0', '0', '0', '0', (others => '0'), (others => '0'));
  signal tags : tags_t := (others => NO_TAG);

  -- first reads
  signal buf_we : std_logic := '0';
  signal buf_waddr, buf_raddr : std_logic_vector(IDX_BITS - 1 downto 0) := (others => '0');
  signal buf_din, buf_dout : std_logic_vector(BUF_W - 1 downto 0) := (others => '0');

  -- events and counters
  signal q_we, q_rd, q_full, q_empty : std_logic := '0';
  signal q_din, q_dout : std_logic_vector(EV_W - 1 downto 0) := (others => '0');
  type cons_t is (C_IDLE, C_READ, C_WAIT, C_WRITE);
  signal cons : cons_t := C_IDLE;
  signal q_word : std_logic_vector(EV_W - 1 downto 0) := (others => '0');
  signal clr_addr : unsigned(ADDR_WIDTH downto 0) := (others => '0');
  signal cons_addr : addr_t := (others => '0');
  signal ovf, ovf2 : std_logic := '0';

  -- training
  signal lfsr : std_logic_vector(6 downto 0) := "1011001";
  signal tr_left : natural range 0 to TRAIN_RUN + LMAX + 8 := 0;
  signal tr_pass : std_logic_vector(BUF_W - 1 downto 0) := (others => '1');
  signal tr_done : std_logic := '0';
  signal drain_cnt : natural range 0 to LMAX + 2 := 0;

  -- echo watch
  signal bad_a, bad_b, bad_c : unsigned(31 downto 0) := (others => '0');
  signal win_cnt, win_bad : unsigned(LINK_LOG - 1 downto 0) := (others => '0');
  signal link_i : std_logic := '1';

  function min3(a, b, c : unsigned(23 downto 0)) return unsigned is
    variable m : unsigned(23 downto 0) := a;
  begin
    if b < m then m := b; end if;
    if c < m then m := c; end if;
    return m;
  end function;

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
  design_clk <= dc4;
  addr_out <= tags(0).addr;
  halted <= '1' when state = S_HALT else '0';
  busy <= '0' when state = S_IDLE or state = S_HALT else '1';
  link_ok <= link_i;
  echo_bad_a <= bad_a;
  echo_bad_b <= bad_b;
  echo_bad_c <= bad_c;
  list_raddr <= std_logic_vector(resize(unsigned(s1_addr), LIST_BITS));
  overflow <= ovf or ovf2;
  lat_i <= to_integer(lat) when lat >= 1 and lat <= LMAX else
           1 when lat = 0 else LMAX;
  -- read address of the first-read buffer: the tag one stage before the sample,
  -- so its word is out of the RAM when that tag meets its sample
  buf_raddr <= std_logic_vector(tags(lat_i - 1).idx);

  u_buf : entity work.em_dpram
    generic map (DATA_W => BUF_W, ADDR_W => IDX_BITS, DEPTH => WMAX)
    port map (clk_a => clk, we_a => buf_we, addr_a => buf_waddr, din_a => buf_din, dout_a => open,
              clk_b => clk, addr_b => buf_raddr, dout_b => buf_dout);

  u_q : entity work.em_async_fifo
    generic map (DATA_W => EV_W, ADDR_W => 4)
    port map (wr_clk => clk, wr_rst => rst, wr_en => q_we, wr_data => q_din, full => q_full,
              rd_clk => clk, rd_rst => rst, rd_en => q_rd, rd_data => q_dout, empty => q_empty);

  -- the design clock runs all the time, so the design keeps going while the
  -- scan is stopped, halted or being set up
  process(clk)
  begin
    if clk'event and clk = '1' then
      if g >= r_cur - 1 then
        g <= (others => '0');
        if state = S_IDLE then
          r_cur <= ratio;         -- a new ratio takes effect between periods
        end if;
      else
        g <= g + 1;
      end if;
      if g < shift_right(r_cur, 1) then dc1 <= '1'; else dc1 <= '0'; end if;
      dc2 <= dc1; dc3 <= dc2; dc4 <= dc3;
    end if;
  end process;

  process(clk)
    variable t : tag_t;
    variable a_m, b_m, c_m, maj : lane_t;
    variable bad : std_logic_vector(2 downto 0);
    variable m1, m2, cfg, soft : lane_t;
    variable i : unsigned(23 downto 0);
    variable active : boolean;
    variable next_base : unsigned(ADDR_WIDTH downto 0);
    variable pushed : boolean;
    variable n : unsigned(23 downto 0);
    variable match : std_logic_vector(BUF_W - 1 downto 0);
  begin
    if clk'event and clk = '1' then
      buf_we <= '0';
      q_we <= '0';
      s1_valid <= '0';
      tr_done <= '0';
      pushed := false;

      -- launch the address with its tag; the tags then shift along
      s2_valid <= s1_valid;
      s2_train <= s1_train;
      s2_pass <= s1_pass;
      s2_exp <= s1_exp;
      s2_list <= s1_list;
      s2_idx <= s1_idx;
      s2_addr <= s1_addr;
      tags(0).valid <= s2_valid;
      tags(0).train <= s2_train;
      tags(0).pass <= s2_pass;
      tags(0).exp <= s2_exp;
      tags(0).idx <= s2_idx;
      if s2_valid = '1' then
        if s2_list = '1' then
          tags(0).addr <= list_dout;
        else
          tags(0).addr <= s2_addr;
        end if;
      end if;
      for k in 1 to LMAX loop
        tags(k) <= tags(k - 1);
      end loop;

      -- the read coming back now
      t := tags(lat_i);
      maj := majority(lane_a, lane_b, lane_c);
      a_m := lane_a xor maj;
      b_m := lane_b xor maj;
      c_m := lane_c xor maj;
      bad(0) := echo(0) xor parity(t.addr);
      bad(1) := echo(1) xor parity(t.addr);
      bad(2) := echo(2) xor parity(t.addr);

      if t.valid = '1' and t.train = '0' then
        if bad(0) = '1' then bad_a <= bad_a + 1; end if;
        if bad(1) = '1' then bad_b <= bad_b + 1; end if;
        if bad(2) = '1' then bad_c <= bad_c + 1; end if;
        -- over each 2**LINK_LOG reads: a quarter bad is a lost link (a dead
        -- readout's echoes still agree with half the addresses), none is a good one
        win_cnt <= win_cnt + 1;
        if win_cnt = (win_cnt'range => '1') then
          win_bad <= (others => '0');
          if win_bad(LINK_LOG - 1 downto LINK_LOG - 2) /= "00" then
            link_i <= '0';
          elsif win_bad = 0 and bad = "000" then
            link_i <= '1';
          end if;
        elsif bad /= "000" then
          win_bad <= win_bad + 1;
        end if;

        if t.pass = '0' then
          buf_we <= '1';
          buf_waddr <= std_logic_vector(t.idx);
          buf_din <= bad & c_m & b_m & a_m;
        else
          m1 := buf_dout(LANES - 1 downto 0) or buf_dout(2 * LANES - 1 downto LANES)
                or buf_dout(3 * LANES - 1 downto 2 * LANES);
          m2 := a_m or b_m or c_m;
          cfg := m1 and m2;
          soft := (m1 or m2) and not cfg;
          if cfg /= (cfg'range => '0') or soft /= (soft'range => '0') then
            if q_full = '1' then
              ovf <= '1';
            else
              q_we <= '1';
              pushed := true;
              if cfg /= (cfg'range => '0') then
                q_din(EV_W - 1 downto EV_W - 8) <= FT_CONFIG;
              else
                q_din(EV_W - 1 downto EV_W - 8) <= FT_SOFT;
              end if;
              q_din(EV_W - 9 downto 0) <=
                t.addr & std_logic_vector(sweep)
                & buf_dout(LANES - 1 downto 0)
                & buf_dout(2 * LANES - 1 downto LANES)
                & buf_dout(3 * LANES - 1 downto 2 * LANES)
                & a_m & b_m & c_m
                & buf_dout(3 * LANES + 2 downto 3 * LANES) & bad;
            end if;
          end if;
          if cfg /= (cfg'range => '0') and state = S_SCAN then
            drain_cnt <= LMAX + 2;
            state <= S_DRAIN;
          end if;
        end if;
      end if;

      if rst = '1' then
        state <= S_IDLE;
        ovf <= '0';
        bad_a <= (others => '0');
        bad_b <= (others => '0');
        bad_c <= (others => '0');
        win_cnt <= (others => '0');
        win_bad <= (others => '0');
        link_i <= '1';
        sweep <= (others => '0');
      else
        case state is
          when S_IDLE =>
            pass <= '0';
            base <= (others => '0');
            if clear_p = '1' then
              sweep <= (others => '0');
              ovf <= '0';
              bad_a <= (others => '0');
              bad_b <= (others => '0');
              bad_c <= (others => '0');
              state <= S_CLEAR;
            elsif train_p = '1' then
              tr_left <= TRAIN_RUN + LMAX + 8;
              tr_pass <= (others => '1');
              state <= S_TRAIN;
            elsif run = '1' and g >= r_cur - 1 and r_cur = ratio then
              if mode_list = '1' then
                num <= resize(list_len, ADDR_WIDTH + 1);
                n := resize(list_len, 24);
              else
                num <= to_unsigned(NUM_ADDR, ADDR_WIDTH + 1);
                n := to_unsigned(NUM_ADDR, 24);
              end if;
              i := min3(n, r_cur - GUARD, to_unsigned(WMAX, 24));
              if i = 0 then
                i := to_unsigned(1, 24);      -- a ratio no larger than GUARD
              end if;
              w_full <= i;
              w_cur <= i;
              if n /= 0 then
                state <= S_SCAN;
              end if;
            end if;

          when S_CLEAR =>                     -- see the consumer below
            if clr_addr = NUM_ADDR then
              state <= S_IDLE;
            end if;

          when S_SCAN =>
            if run = '0' then
              drain_cnt <= LMAX + 2;
              state <= S_DRAIN;
            else
              -- the last w_cur phases of the period read the window
              active := g >= r_cur - w_cur;
              if active then
                i := g - (r_cur - w_cur);
                s1_valid <= '1';
                s1_train <= '0';
                s1_list <= mode_list;
                s1_pass <= pass;
                s1_idx <= resize(i, IDX_BITS);
                s1_addr <= std_logic_vector(resize(base + resize(i, ADDR_WIDTH + 1), ADDR_WIDTH));
              end if;
              if g = r_cur - 1 then           -- the period ends
                if pass = '0' then
                  pass <= '1';
                else
                  pass <= '0';
                  next_base := base + resize(w_cur, ADDR_WIDTH + 1);
                  if next_base >= num then
                    next_base := (others => '0');
                    sweep <= sweep + 1;
                    if (sweep and SWEEP_MASK) = SWEEP_MASK and q_full = '0' and not pushed then
                      q_we <= '1';
                      q_din <= FT_SWEEP & std_logic_vector(to_unsigned(0, ADDR_WIDTH))
                               & std_logic_vector(sweep)
                               & std_logic_vector(to_unsigned(0, 6 * LANES + 6));
                    end if;
                  end if;
                  base <= next_base;
                  if num - next_base < resize(w_full, ADDR_WIDTH + 1) then
                    w_cur <= resize(num - next_base, 24);
                  else
                    w_cur <= w_full;
                  end if;
                end if;
              end if;
            end if;

          when S_DRAIN =>                     -- let the reads under way come back
            if drain_cnt = 0 then
              if run = '0' then
                state <= S_IDLE;
              else
                state <= S_HALT;
              end if;
            else
              drain_cnt <= drain_cnt - 1;
            end if;

          when S_HALT =>                      -- wait for the host's resume
            if resume_p = '1' then
              state <= S_IDLE;
            elsif run = '0' then
              state <= S_IDLE;
            end if;

          when S_TRAIN =>
            -- reads of the two training addresses in a pseudo-random order
            lfsr <= lfsr(5 downto 0) & (lfsr(6) xor lfsr(5));
            s1_valid <= '1';
            s1_train <= '1';
            s1_list <= '0';
            s1_exp <= lfsr(0);
            s1_addr <= (others => '1');
            if lfsr(0) = '0' then
              s1_addr(0) <= '0';
            end if;
            if tr_left = 0 then
              if q_full = '0' and not pushed then
                q_we <= '1';
                q_din <= FT_TRAIN & std_logic_vector(to_unsigned(0, ADDR_WIDTH))
                         & std_logic_vector(to_unsigned(0, 32))
                         & tr_pass(LANES - 1 downto 0)
                         & tr_pass(2 * LANES - 1 downto LANES)
                         & tr_pass(3 * LANES - 1 downto 2 * LANES)
                         & std_logic_vector(to_unsigned(0, 3 * LANES))
                         & "000" & tr_pass(3 * LANES + 2 downto 3 * LANES);
                state <= S_IDLE;
              end if;
            else
              tr_left <= tr_left - 1;
              if tr_left <= TRAIN_RUN and t.train = '1' and t.valid = '1' then
                for k in 0 to LANES - 1 loop
                  match(k) := not (lane_a(k) xor t.exp);
                  match(LANES + k) := not (lane_b(k) xor t.exp);
                  match(2 * LANES + k) := not (lane_c(k) xor t.exp);
                end loop;
                match(3 * LANES + 2 downto 3 * LANES) := not bad;
                tr_pass <= tr_pass and match;
              end if;
            end if;
        end case;
      end if;
    end if;
  end process;

  -- consumer: counts each error in its address' counter, then passes it on
  cnt_addr <= cons_addr;
  process(clk)
    variable soft_cnt : unsigned(SOFT_CNT_W - 1 downto 0);
    variable cfg_cnt : unsigned(CFG_CNT_W - 1 downto 0);
    variable m1, m2, cfg, soft : lane_t;
    constant M_HI : natural := 6 * LANES + 6 - 1;
    variable typ : std_logic_vector(7 downto 0);
  begin
    if clk'event and clk = '1' then
      q_rd <= '0';
      ev_we <= '0';
      cnt_we <= '0';
      if state /= S_CLEAR then
        clr_addr <= (others => '0');
      end if;
      if state = S_IDLE and clear_p = '1' then
        ovf2 <= '0';
      end if;
      if state = S_CLEAR then
        if clr_addr /= NUM_ADDR then
          cnt_we <= '1';
          cnt_din <= (others => '0');
          cons_addr <= std_logic_vector(resize(clr_addr, ADDR_WIDTH));
          clr_addr <= clr_addr + 1;
          if clr_addr = NUM_ADDR - 1 then
            ev_we <= '1';
            ev_data <= FT_CLEARED & std_logic_vector(to_unsigned(0, EV_W - 8));
          end if;
        end if;
      elsif rst = '0' then
        case cons is
          when C_IDLE =>
            if q_empty = '0' and q_rd = '0' then
              q_word <= q_dout;
              q_rd <= '1';
              cons <= C_READ;
              cons_addr <= q_dout(EV_W - 9 downto EV_W - 8 - ADDR_WIDTH);
            end if;
          when C_READ =>                      -- the RAM read takes a cycle
            cons <= C_WAIT;
          when C_WAIT =>
            cons <= C_WRITE;
          when C_WRITE =>
            typ := q_word(EV_W - 1 downto EV_W - 8);
            if typ = FT_SOFT or typ = FT_CONFIG then
              m1 := q_word(M_HI downto M_HI - LANES + 1)
                    or q_word(M_HI - LANES downto M_HI - 2 * LANES + 1)
                    or q_word(M_HI - 2 * LANES downto M_HI - 3 * LANES + 1);
              m2 := q_word(M_HI - 3 * LANES downto M_HI - 4 * LANES + 1)
                    or q_word(M_HI - 4 * LANES downto M_HI - 5 * LANES + 1)
                    or q_word(M_HI - 5 * LANES downto M_HI - 6 * LANES + 1);
              cfg := m1 and m2;
              soft := (m1 or m2) and not cfg;
              soft_cnt := unsigned(cnt_dout(SOFT_CNT_W + CFG_CNT_W - 1 downto CFG_CNT_W));
              cfg_cnt := unsigned(cnt_dout(CFG_CNT_W - 1 downto 0));
              soft_cnt := inc_sat(soft_cnt, popcount(soft));
              if cfg /= (cfg'range => '0') then
                cfg_cnt := inc_sat(cfg_cnt, 1);
              end if;
              cnt_we <= '1';
              cnt_din <= std_logic_vector(soft_cnt) & std_logic_vector(cfg_cnt);
            end if;
            if ev_full = '0' then
              ev_we <= '1';
              ev_data <= q_word;
            else
              ovf2 <= '1';
            end if;
            cons <= C_IDLE;
        end case;
      end if;
    end if;
  end process;
end rtl;
