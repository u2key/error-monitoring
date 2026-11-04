-- Dual-clock FIFO with Gray-coded pointers. VHDL-93.
-- Carries the scanner's events from the design's clock to the host side.
-- Depth 2**ADDR_W; wr_en while full and rd_en while empty are ignored.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity em_async_fifo is
  generic (
    DATA_W : positive;
    ADDR_W : positive := 5
  );
  port (
    wr_clk  : in  std_logic;
    wr_rst  : in  std_logic;
    wr_en   : in  std_logic;
    wr_data : in  std_logic_vector(DATA_W - 1 downto 0);
    full    : out std_logic;
    rd_clk  : in  std_logic;
    rd_rst  : in  std_logic;
    rd_en   : in  std_logic;
    rd_data : out std_logic_vector(DATA_W - 1 downto 0);
    empty   : out std_logic
  );
end em_async_fifo;

architecture rtl of em_async_fifo is
  type mem_t is array (0 to 2**ADDR_W - 1) of std_logic_vector(DATA_W - 1 downto 0);
  signal mem : mem_t;

  subtype ptr_t is unsigned(ADDR_W downto 0);       -- one extra bit for full/empty
  signal wbin, wgray, rbin, rgray : ptr_t := (others => '0');
  signal rgray_w1, rgray_w2 : ptr_t := (others => '0');   -- read pointer in wr_clk
  signal wgray_r1, wgray_r2 : ptr_t := (others => '0');   -- write pointer in rd_clk
  signal full_i, empty_i : std_logic;

  function to_gray(b : ptr_t) return ptr_t is
  begin
    return b xor shift_right(b, 1);
  end function;
begin
  -- full: write pointer one lap ahead of the read pointer
  full_i <= '1' when wgray = (not rgray_w2(ADDR_W downto ADDR_W - 1)) & rgray_w2(ADDR_W - 2 downto 0)
            else '0';
  empty_i <= '1' when rgray = wgray_r2 else '0';
  full <= full_i;
  empty <= empty_i;
  rd_data <= mem(to_integer(rbin(ADDR_W - 1 downto 0)));

  process(wr_clk)
    variable nb : ptr_t;
  begin
    if wr_clk'event and wr_clk = '1' then
      if wr_rst = '1' then
        wbin <= (others => '0');
        wgray <= (others => '0');
        rgray_w1 <= (others => '0');
        rgray_w2 <= (others => '0');
      else
        rgray_w1 <= rgray;
        rgray_w2 <= rgray_w1;
        if wr_en = '1' and full_i = '0' then
          mem(to_integer(wbin(ADDR_W - 1 downto 0))) <= wr_data;
          nb := wbin + 1;
          wbin <= nb;
          wgray <= to_gray(nb);
        end if;
      end if;
    end if;
  end process;

  process(rd_clk)
    variable nb : ptr_t;
  begin
    if rd_clk'event and rd_clk = '1' then
      if rd_rst = '1' then
        rbin <= (others => '0');
        rgray <= (others => '0');
        wgray_r1 <= (others => '0');
        wgray_r2 <= (others => '0');
      else
        wgray_r1 <= wgray;
        wgray_r2 <= wgray_r1;
        if rd_en = '1' and empty_i = '0' then
          nb := rbin + 1;
          rbin <= nb;
          rgray <= to_gray(nb);
        end if;
      end if;
    end if;
  end process;
end rtl;
