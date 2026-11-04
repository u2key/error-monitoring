#!/bin/bash
# Build and run the system testbench.
#
#   sim/run_sim.sh            (GEN_TMR: the generate-tmr-vhdl checkout)
#
# The design under test, sim/dut/areset.vhd, is converted with
# --tmr --light --probe; HOLD_LO, the TMRDFF bit of its HOLD register, is read
# from the converter's map. Design files compile as VHDL-93, the testbench as
# VHDL-2008.
set -eu
S="$(cd "$(dirname "$0")" && pwd)"; R="$(dirname "$S")"
Q=${QUESTA:-/opt/questa/25.1std/questa_fse/bin}
GEN_TMR=${GEN_TMR:-/nfs/0/users/sekioka/generate-tmr-vhdl}
W="$S/work"; mkdir -p "$W"; cd "$W"

(cd "$GEN_TMR" && python3 main.py --tmr --light --probe "$S/dut/areset.vhd" "$W/areset_tmr.vhd") > gen.log 2>&1 \
  || { echo "conversion failed:"; tail -3 gen.log; exit 1; }
HOLD_LO=$(awk '$2 == "HOLD" { split($1, r, ":"); print (r[2] != "" ? r[2] : r[1]) }' ARESET.tmrdff.map)

rm -rf em; "$Q/vlib" em >/dev/null; : > compile.log
for f in em_pkg em_uart em_async_fifo em_dpram em_scanner em_host_if em_detector_top tmrdff_readout; do
  "$Q/vcom" -93 -work em -quiet "$R/hdl/$f.vhd" >> compile.log 2>&1 || { grep Error compile.log; exit 1; }
done
"$Q/vcom" -93 -work em -quiet areset_tmr.vhd >> compile.log 2>&1 || { grep Error compile.log; exit 1; }
"$Q/vcom" -2008 -work em -quiet "$S/tb_em_system.vhd" >> compile.log 2>&1 || { grep Error compile.log; exit 1; }
"$Q/vsim" -c -lib em tb_em_system -gHOLD_LO="$HOLD_LO" -do "run -all; quit -f" > sim.log 2>&1 || true
grep -E "one-cycle flips|random flips|CHECK FAILED|TB PASS|TB FAIL|\*\* (Error|Fatal)" sim.log | sed 's/^# //'
grep -q "TB PASS" sim.log
