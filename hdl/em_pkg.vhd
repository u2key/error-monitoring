-- Shared constants and helpers of the error monitor. VHDL-93.
--
-- Frame format, detector -> host (UART, 8N1):
--   0xA5, type, length, payload (length bytes, big-endian fields), checksum
--   where checksum = type xor length xor every payload byte.
-- Command bytes, host -> detector: see em_host_if.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package em_pkg is
  constant FRAME_SYNC : std_logic_vector(7 downto 0) := x"A5";

  -- frame types
  constant FT_SOFT    : std_logic_vector(7 downto 0) := x"01";  -- flip-flop error
  constant FT_CONFIG  : std_logic_vector(7 downto 0) := x"02";  -- configuration error
  constant FT_SWEEP   : std_logic_vector(7 downto 0) := x"03";  -- sweeps completed
  constant FT_COUNTER : std_logic_vector(7 downto 0) := x"04";  -- one address' counters
  constant FT_STATUS  : std_logic_vector(7 downto 0) := x"05";
  constant FT_DUMPEND : std_logic_vector(7 downto 0) := x"06";
  constant FT_ACK     : std_logic_vector(7 downto 0) := x"07";
  constant FT_INFO    : std_logic_vector(7 downto 0) := x"08";
  constant FT_CLOCK   : std_logic_vector(7 downto 0) := x"09";  -- DUT clock lost / back
  constant FT_CLEARED : std_logic_vector(7 downto 0) := x"0A";

  -- command bytes
  constant CMD_START  : std_logic_vector(7 downto 0) := x"53";  -- 'S'
  constant CMD_STOP   : std_logic_vector(7 downto 0) := x"50";  -- 'P'
  constant CMD_CLEAR  : std_logic_vector(7 downto 0) := x"43";  -- 'C'
  constant CMD_RESUME : std_logic_vector(7 downto 0) := x"52";  -- 'R'
  constant CMD_DUMP   : std_logic_vector(7 downto 0) := x"44";  -- 'D'
  constant CMD_INFO   : std_logic_vector(7 downto 0) := x"49";  -- 'I'
  constant CMD_STATUS : std_logic_vector(7 downto 0) := x"3F";  -- '?'

  constant SOFT_CNT_W : positive := 16;   -- per-address flip-flop error counter
  constant CFG_CNT_W  : positive := 8;    -- per-address configuration error counter
  constant VERSION    : natural := 1;

  function clog2(n : positive) return natural;
  function nbytes(bits : positive) return positive;
  function popcount(v : std_logic_vector) return natural;
  function majority(a, b, c : std_logic_vector) return std_logic_vector;
end em_pkg;

package body em_pkg is
  function clog2(n : positive) return natural is
    variable r : natural := 0;
    variable v : natural := 1;
  begin
    while v < n loop
      v := v * 2;
      r := r + 1;
    end loop;
    return r;
  end function;

  function nbytes(bits : positive) return positive is
  begin
    return (bits + 7) / 8;
  end function;

  function popcount(v : std_logic_vector) return natural is
    variable n : natural := 0;
  begin
    for i in v'range loop
      if v(i) = '1' then
        n := n + 1;
      end if;
    end loop;
    return n;
  end function;

  -- the voter's expression, bit by bit
  function majority(a, b, c : std_logic_vector) return std_logic_vector is
  begin
    return (a and b) or (b and c) or (c and a);
  end function;
end em_pkg;
