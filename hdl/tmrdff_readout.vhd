-- Readout of TMRDFF_A/_B/_C on the FPGA under test. VHDL-93.
--
-- The converter's --probe option brings every triplicated flip-flop out as
-- TMRDFF_A/_B/_C (WIDTH bits each). The tester addresses them LANES bits at a
-- time: word k is bits k*LANES .. k*LANES+LANES-1 (bits past WIDTH read '0').
--
-- Each replica has its own address lines, registers and multiplexer, so a broken
-- configuration in one replica's readout shows as a disagreement instead of
-- silently steering all three to another word.
--
-- read_clk comes from the tester, half a period away from the edges that launch
-- the address, and is unrelated to the design's own clock except that the tester
-- derives both. Latency: address register, then word register.
--
-- echo_x is the parity of the address that word belongs to: the tester compares
-- it with what it asked for, which reveals timing drift and address-path errors
-- per replica. The two highest addresses return fixed patterns for training:
-- 2**ADDR_WIDTH - 1 reads all ones, 2**ADDR_WIDTH - 2 all zeros, so the number
-- of words must not exceed 2**ADDR_WIDTH - 2.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tmrdff_readout is
  generic (
    WIDTH      : positive;            -- bits per replica (TMRDFF_A'LENGTH)
    LANES      : positive := 4;       -- bits read per address and replica
    ADDR_WIDTH : positive := 18
  );
  port (
    read_clk : in  std_logic;
    tmrdff_a : in  std_logic_vector(WIDTH - 1 downto 0);
    tmrdff_b : in  std_logic_vector(WIDTH - 1 downto 0);
    tmrdff_c : in  std_logic_vector(WIDTH - 1 downto 0);
    addr_a   : in  std_logic_vector(ADDR_WIDTH - 1 downto 0);
    addr_b   : in  std_logic_vector(ADDR_WIDTH - 1 downto 0);
    addr_c   : in  std_logic_vector(ADDR_WIDTH - 1 downto 0);
    lane_a   : out std_logic_vector(LANES - 1 downto 0);
    lane_b   : out std_logic_vector(LANES - 1 downto 0);
    lane_c   : out std_logic_vector(LANES - 1 downto 0);
    echo_a   : out std_logic;
    echo_b   : out std_logic;
    echo_c   : out std_logic
  );
end tmrdff_readout;

architecture rtl of tmrdff_readout is
  subtype addr_t is std_logic_vector(ADDR_WIDTH - 1 downto 0);
  subtype word_t is std_logic_vector(LANES - 1 downto 0);
  signal ra, rb, rc : addr_t := (others => '0');
  attribute keep : boolean;
  attribute keep of ra : signal is true;
  attribute keep of rb : signal is true;
  attribute keep of rc : signal is true;

  function parity(v : addr_t) return std_logic is
    variable p : std_logic := '0';
  begin
    for i in v'range loop
      p := p xor v(i);
    end loop;
    return p;
  end function;

  function word(v : std_logic_vector; a : addr_t) return word_t is
    constant TOP : unsigned(ADDR_WIDTH - 1 downto 0) := (others => '1');
    variable w : word_t := (others => '0');
    variable idx : natural;
  begin
    if unsigned(a) = TOP then
      return (word_t'range => '1');
    elsif unsigned(a) = TOP - 1 then
      return (word_t'range => '0');
    end if;
    for i in 0 to LANES - 1 loop
      idx := to_integer(unsigned(a)) * LANES + i;
      if idx < WIDTH then
        w(i) := v(idx);
      end if;
    end loop;
    return w;
  end function;
begin
  process(read_clk)
  begin
    if read_clk'event and read_clk = '1' then
      ra <= addr_a;
      lane_a <= word(tmrdff_a, ra);
      echo_a <= parity(ra);
    end if;
  end process;

  process(read_clk)
  begin
    if read_clk'event and read_clk = '1' then
      rb <= addr_b;
      lane_b <= word(tmrdff_b, rb);
      echo_b <= parity(rb);
    end if;
  end process;

  process(read_clk)
  begin
    if read_clk'event and read_clk = '1' then
      rc <= addr_c;
      lane_c <= word(tmrdff_c, rc);
      echo_c <= parity(rc);
    end if;
  end process;
end rtl;
