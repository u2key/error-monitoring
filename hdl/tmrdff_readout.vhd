-- Readout of TMRDFF_A/_B/_C on the FPGA under test. VHDL-93.
--
-- The converter's --probe option brings every triplicated flip-flop out as
-- TMRDFF_A/_B/_C (WIDTH bits each). Far too many to send off chip at once, so
-- the detector addresses them LANES bits at a time: word k is bits
-- k*LANES .. k*LANES+LANES-1 (bits past WIDTH read as '0').
--
-- Every clock cycle the three replicas' words at the current address are
-- registered on the same edge, so lane_a/_b/_c always describe one and the
-- same cycle of the design -- a disagreement between them is in the design,
-- not in the readout. clk_out forwards the design clock: the detector samples
-- the lanes with it and runs its scanner on it, so it sees consecutive cycles.
--
-- rd_addr comes from the other FPGA, asynchronous to clk. It is synchronised
-- here; the detector holds an address for several cycles before it samples
-- (em_scanner SETTLE), which covers the synchroniser and this register.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tmrdff_readout is
  generic (
    WIDTH      : positive;            -- bits per replica (TMRDFF_A'LENGTH)
    LANES      : positive := 16;      -- bits read per address and replica
    ADDR_WIDTH : positive := 16       -- enough for ceil(WIDTH / LANES) words
  );
  port (
    clk      : in  std_logic;         -- the design's clock
    tmrdff_a : in  std_logic_vector(WIDTH - 1 downto 0);
    tmrdff_b : in  std_logic_vector(WIDTH - 1 downto 0);
    tmrdff_c : in  std_logic_vector(WIDTH - 1 downto 0);
    rd_addr  : in  std_logic_vector(ADDR_WIDTH - 1 downto 0);
    lane_a   : out std_logic_vector(LANES - 1 downto 0);
    lane_b   : out std_logic_vector(LANES - 1 downto 0);
    lane_c   : out std_logic_vector(LANES - 1 downto 0);
    clk_out  : out std_logic
  );
end tmrdff_readout;

architecture rtl of tmrdff_readout is
  signal addr_s1, addr_s2 : std_logic_vector(ADDR_WIDTH - 1 downto 0) := (others => '0');
  -- keep the synchroniser registers as registers
  attribute keep : boolean;
  attribute keep of addr_s1 : signal is true;
  attribute keep of addr_s2 : signal is true;
begin
  process(clk)
    variable base, idx : natural;
  begin
    if clk'event and clk = '1' then
      addr_s1 <= rd_addr;
      addr_s2 <= addr_s1;
      base := to_integer(unsigned(addr_s2)) * LANES;
      for i in 0 to LANES - 1 loop
        idx := base + i;
        if idx < WIDTH then
          lane_a(i) <= tmrdff_a(idx);
          lane_b(i) <= tmrdff_b(idx);
          lane_c(i) <= tmrdff_c(idx);
        else
          lane_a(i) <= '0';
          lane_b(i) <= '0';
          lane_c(i) <= '0';
        end if;
      end loop;
    end if;
  end process;

  clk_out <= clk;
end rtl;
