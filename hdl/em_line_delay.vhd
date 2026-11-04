-- One input line through a delay selectable in sample-clock steps. VHDL-93.
-- sample_clk is the tester's fast clock (a multiple of read_clk from the same
-- PLL); the host picks the tap per line during training, which moves that
-- line's sampling instant to the middle of its data eye.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity em_line_delay is
  generic (
    TAPS     : positive;
    TAP_BITS : positive
  );
  port (
    sample_clk : in  std_logic;
    d          : in  std_logic;
    tap        : in  std_logic_vector(TAP_BITS - 1 downto 0);
    q          : out std_logic
  );
end em_line_delay;

architecture rtl of em_line_delay is
  signal sr : std_logic_vector(TAPS - 1 downto 0) := (others => '0');
begin
  process(sample_clk)
    variable t : natural;
  begin
    if sample_clk'event and sample_clk = '1' then
      sr <= sr(TAPS - 2 downto 0) & d;
      t := to_integer(unsigned(tap));
      if t < TAPS then
        q <= sr(t);
      else
        q <= sr(TAPS - 1);
      end if;
    end if;
  end process;
end rtl;
