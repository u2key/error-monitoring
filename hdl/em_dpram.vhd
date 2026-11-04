-- Dual-port RAM: port A reads and writes, port B reads. VHDL-93.
-- Written as the template synthesis tools infer block RAM from. Addresses at or
-- beyond DEPTH read as the last word and do not write.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity em_dpram is
  generic (
    DATA_W : positive;
    ADDR_W : positive;
    DEPTH  : positive               -- words, at most 2**ADDR_W
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
  type mem_t is array (0 to DEPTH - 1) of std_logic_vector(DATA_W - 1 downto 0);
  shared variable mem : mem_t := (others => (others => '0'));
begin
  process(clk_a)
  begin
    if clk_a'event and clk_a = '1' then
      if to_integer(unsigned(addr_a)) < DEPTH then
        if we_a = '1' then
          mem(to_integer(unsigned(addr_a))) := din_a;
        end if;
        dout_a <= mem(to_integer(unsigned(addr_a)));
      else
        dout_a <= mem(DEPTH - 1);
      end if;
    end if;
  end process;

  process(clk_b)
  begin
    if clk_b'event and clk_b = '1' then
      if to_integer(unsigned(addr_b)) < DEPTH then
        dout_b <= mem(to_integer(unsigned(addr_b)));
      else
        dout_b <= mem(DEPTH - 1);
      end if;
    end if;
  end process;
end rtl;
