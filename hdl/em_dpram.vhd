-- True dual-port RAM with one clock per port. VHDL-93.
-- Holds the per-address error counters: port A (the design's clock) reads and
-- updates them while scanning, port B (the host side) reads them for a dump.
-- Written as the dual-clock RAM template synthesis tools infer block RAM from.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity em_dpram is
  generic (
    DATA_W : positive;
    ADDR_W : positive
  );
  port (
    clk_a  : in  std_logic;
    we_a   : in  std_logic;
    addr_a : in  std_logic_vector(ADDR_W - 1 downto 0);
    din_a  : in  std_logic_vector(DATA_W - 1 downto 0);
    dout_a : out std_logic_vector(DATA_W - 1 downto 0);
    clk_b  : in  std_logic;
    addr_b : in  std_logic_vector(ADDR_W - 1 downto 0);
    dout_b : out std_logic_vector(DATA_W - 1 downto 0)
  );
end em_dpram;

architecture rtl of em_dpram is
  type mem_t is array (0 to 2**ADDR_W - 1) of std_logic_vector(DATA_W - 1 downto 0);
  shared variable mem : mem_t := (others => (others => '0'));
begin
  process(clk_a)
  begin
    if clk_a'event and clk_a = '1' then
      if we_a = '1' then
        mem(to_integer(unsigned(addr_a))) := din_a;
      end if;
      dout_a <= mem(to_integer(unsigned(addr_a)));
    end if;
  end process;

  process(clk_b)
  begin
    if clk_b'event and clk_b = '1' then
      dout_b <= mem(to_integer(unsigned(addr_b)));
    end if;
  end process;
end rtl;
