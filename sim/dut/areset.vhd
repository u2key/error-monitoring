-- Asynchronous reset, active low: a memory, a register reset to a constant,
-- a register without a reset value (must hold during reset), stateful and
-- stateless variables, all in one process.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
entity areset is
  generic (W : integer := 4);
  port (clk : in std_logic; rst_n : in std_logic; enb : in std_logic;
        din : in std_logic_vector(W-1 downto 0);
        addr : in std_logic_vector(1 downto 0);
        q_hold : out std_logic_vector(W-1 downto 0);
        q_mem : out std_logic_vector(W-1 downto 0);
        q_acc : out std_logic_vector(W-1 downto 0);
        q_tmp : out std_logic;
        q_k : out std_logic_vector(W-1 downto 0);
        q_r : out std_logic_vector(3 downto 0));
end areset;
architecture rtl of areset is
  constant K : std_logic_vector(W-1 downto 0) := std_logic_vector(to_unsigned(1, W));
  type mem_t is array (0 to 3) of std_logic_vector(W-1 downto 0);
  signal mem : mem_t;
  signal hold : std_logic_vector(W-1 downto 0) := (others => '0');
  signal kreg : std_logic_vector(W-1 downto 0);
  signal r : std_logic_vector(3 downto 0) := (others => '1');
begin
  process(clk, rst_n)
    variable acc : unsigned(W-1 downto 0);
    variable t : std_logic;
  begin
    if rst_n = '0' then
      mem <= (others => (others => '0'));
      kreg <= K;
      r <= "0101";
      acc := (others => '1');
      t := '0';
    elsif clk'event and clk = '1' then
      t := din(0) xor din(1);
      q_tmp <= t;
      if enb = '1' then
        mem(to_integer(unsigned(addr))) <= din;
        hold <= din;
        acc := acc + unsigned(din);
        kreg <= kreg(W-2 downto 0) & kreg(W-1);
        r <= din(3 downto 0);
      end if;
      q_acc <= std_logic_vector(acc);
    end if;
  end process;
  q_hold <= hold;
  q_mem <= mem(to_integer(unsigned(addr)));
  q_k <= kreg;
  q_r <= r;
end rtl;
