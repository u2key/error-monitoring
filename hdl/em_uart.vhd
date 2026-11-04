-- UART receiver and transmitter, 8N1. VHDL-93.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity em_uart_rx is
  generic (
    CLK_HZ : positive;
    BAUD   : positive
  );
  port (
    clk   : in  std_logic;
    rst   : in  std_logic;
    rxd   : in  std_logic;
    data  : out std_logic_vector(7 downto 0);
    valid : out std_logic                     -- one cycle per received byte
  );
end em_uart_rx;

architecture rtl of em_uart_rx is
  constant DIV : positive := CLK_HZ / BAUD;
  signal rx_s1, rx_s2 : std_logic := '1';
  signal busy : std_logic := '0';
  signal cnt : natural range 0 to DIV := 0;
  signal bitn : natural range 0 to 9 := 0;
  signal sh : std_logic_vector(7 downto 0) := (others => '0');
begin
  process(clk)
  begin
    if clk'event and clk = '1' then
      rx_s1 <= rxd;
      rx_s2 <= rx_s1;
      valid <= '0';
      if rst = '1' then
        busy <= '0';
      elsif busy = '0' then
        if rx_s2 = '0' then                  -- start bit edge
          busy <= '1';
          cnt <= DIV / 2;                     -- sample in the middle of each bit
          bitn <= 0;
        end if;
      elsif cnt /= 0 then
        cnt <= cnt - 1;
      else
        cnt <= DIV - 1;
        if bitn = 0 then
          if rx_s2 /= '0' then                -- glitch, not a start bit
            busy <= '0';
          end if;
          bitn <= 1;
        elsif bitn <= 8 then
          sh <= rx_s2 & sh(7 downto 1);
          bitn <= bitn + 1;
        else                                  -- stop bit
          busy <= '0';
          if rx_s2 = '1' then
            data <= sh;
            valid <= '1';
          end if;
        end if;
      end if;
    end if;
  end process;
end rtl;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity em_uart_tx is
  generic (
    CLK_HZ : positive;
    BAUD   : positive
  );
  port (
    clk   : in  std_logic;
    rst   : in  std_logic;
    data  : in  std_logic_vector(7 downto 0);
    start : in  std_logic;                    -- accepted when busy = '0'
    busy  : out std_logic;
    txd   : out std_logic
  );
end em_uart_tx;

architecture rtl of em_uart_tx is
  constant DIV : positive := CLK_HZ / BAUD;
  signal busy_i : std_logic := '0';
  signal cnt : natural range 0 to DIV := 0;
  signal bitn : natural range 0 to 10 := 0;
  signal sh : std_logic_vector(9 downto 0) := (others => '1');
begin
  busy <= busy_i;
  txd <= sh(0);
  process(clk)
  begin
    if clk'event and clk = '1' then
      if rst = '1' then
        busy_i <= '0';
        sh <= (others => '1');
      elsif busy_i = '0' then
        if start = '1' then
          sh <= '1' & data & '0';             -- stop, data (LSB first), start
          busy_i <= '1';
          cnt <= DIV - 1;
          bitn <= 0;
        end if;
      elsif cnt /= 0 then
        cnt <= cnt - 1;
      else
        cnt <= DIV - 1;
        if bitn = 9 then
          busy_i <= '0';
        else
          sh <= '1' & sh(9 downto 1);
          bitn <= bitn + 1;
        end if;
      end if;
    end if;
  end process;
end rtl;
