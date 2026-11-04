#!/usr/bin/env python3
"""Run an error-monitoring test from the host PC.

    python3 em_host.py config.ini [--duration SECONDS] [--out DIR]

The host keeps the test time, reconfigures the FPGA under test when the
detector reports a configuration error, and writes the results as CSV files
in a new directory under [output] dir:

    events.csv     every error, one row per bit: time, kind (soft / config /
                   clock_loss), sweep, address, lane, bit, register, sample,
                   replicas that disagreed
    reconfig.csv   every reconfiguration: trigger, start, end, return code
    counters.csv   the detector's per-address counters, dumped at the end
                   (counters_history.csv: the periodic dumps, if enabled)
    per_bit.csv    errors per TMRDFF bit, from the events
    summary.csv    test time, active time, downtime, totals
"""
import argparse
import configparser
import csv
import datetime
import os
import shlex
import subprocess
import sys
import time

import em_protocol as P
from em_layout import Layout


class DetectorError(Exception):
    pass


class Monitor:
    def __init__(self, cfg, port, outdir, clock=time.monotonic, log=print):
        self.cfg = cfg
        self.port = port
        self.outdir = outdir
        self.clock = clock
        self.log = log
        self.reader = P.FrameReader()
        self.pending = []
        self.info = None
        mp = cfg.get("layout", "map", fallback="").strip()
        self.layout = Layout.from_map(mp) if mp else Layout()
        self.t0 = None
        self.downtime = 0.0
        self.sweeps = 0
        self.per_bit = {}            # bit -> [soft, config]
        self.totals = {"soft_events": 0, "soft_bits": 0, "config_errors": 0,
                       "clock_losses": 0, "reconfigs": 0, "reconfig_failures": 0}
        self.overflow = False
        self.clock_ok = True
        self.reconfiguring = False

    # ---------------------------------------------------------------- link
    def send(self, cmd):
        self.port.write(cmd)

    def poll(self, wait=0.05):
        n = getattr(self.port, "in_waiting", 0) or 1
        data = self.port.read(n)
        if data:
            self.pending += self.reader.feed(data)
        elif wait:
            time.sleep(0)            # the port's own timeout paces the loop

    def next_frame(self, types=None, timeout=5.0):
        """Next frame of one of the given types; other frames are handled."""
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            while self.pending:
                f = self.pending.pop(0)
                if types is not None and f.type in types:
                    return f
                self.handle(f)
            self.poll()
        raise DetectorError(f"no {[P.TYPE_NAMES.get(t) for t in types]} frame within {timeout} s")

    def command(self, cmd, timeout=5.0):
        self.send(cmd)
        f = self.next_frame({P.FT_ACK}, timeout)
        if f.payload[:1] != cmd:
            raise DetectorError(f"ACK for {f.payload!r}, expected {cmd!r}")

    # ---------------------------------------------------------------- files
    def open_csv(self, name, header):
        f = open(os.path.join(self.outdir, name), "w", newline="")
        w = csv.writer(f)
        w.writerow(header)
        return f, w

    def now(self):
        return round(self.clock() - self.t0, 3) if self.t0 is not None else 0.0

    # ---------------------------------------------------------------- events
    def handle(self, f):
        if f.type in (P.FT_SOFT, P.FT_CONFIG):
            ev = P.parse_event(f, self.info)
            self.record(ev)
            if ev.kind == "config":
                self.reconfigure("config")
        elif f.type == P.FT_SWEEP:
            self.sweeps = max(self.sweeps, P.parse_sweep(f, self.info) + 1)
        elif f.type == P.FT_CLOCK:
            ok = f.payload[0] == 1
            if not ok and self.clock_ok and not self.reconfiguring:
                self.totals["clock_losses"] += 1
                self.ev_w.writerow([self.now(), self.wall(), "clock_loss", self.sweeps,
                                    "", "", "", "", "", ""])
                self.log(f"[{self.now():9.1f}s] forwarded clock lost")
                if self.cfg.getboolean("reconfigure", "clock_loss_is_config", fallback=True):
                    self.reconfigure("clock_loss")
            self.clock_ok = ok
        elif f.type == P.FT_STATUS:
            st = P.parse_status(f)
            if st["overflow"] and not self.overflow:
                self.log("detector event FIFO overflowed: some events were lost")
            self.overflow = self.overflow or st["overflow"]
        # ACK, CLEARED, COUNTER, DUMPEND outside a request: nothing to do

    def wall(self):
        return datetime.datetime.now().isoformat(timespec="milliseconds")

    def record(self, ev):
        lanes = self.info.lanes
        m1, m2 = ev.error_bits(1), ev.error_bits(2)
        if ev.kind == "config":
            bits = [(l, "both", ev.replicas(1, l) + "/" + ev.replicas(2, l))
                    for l in range(lanes) if (m1 & m2) >> l & 1]
            self.totals["config_errors"] += 1
        else:
            bits = []
            for l in range(lanes):
                if (m1 >> l & 1) and not (m2 >> l & 1):
                    bits.append((l, "1", ev.replicas(1, l)))
                elif (m2 >> l & 1) and not (m1 >> l & 1):
                    bits.append((l, "2", ev.replicas(2, l)))
            self.totals["soft_events"] += 1
            self.totals["soft_bits"] += len(bits)
        for lane, sample, reps in bits:
            bit = ev.address * lanes + lane
            self.ev_w.writerow([self.now(), self.wall(), ev.kind, ev.sweep, ev.address, lane, bit,
                                self.layout.describe(bit), sample, reps])
            c = self.per_bit.setdefault(bit, [0, 0])
            c[0 if ev.kind == "soft" else 1] += 1
        self.ev_f.flush()
        self.log(f"[{self.now():9.1f}s] {ev.kind} error at address {ev.address}: "
                 + ", ".join(f"bit {ev.address * lanes + l} {self.layout.describe(ev.address * lanes + l)}"
                             for l, _, _ in bits))

    # ---------------------------------------------------------------- reconfiguration
    def reconfigure(self, trigger):
        cmd = self.cfg.get("reconfigure", "command", fallback="").strip()
        limit = self.cfg.getint("reconfigure", "max_reconfigs", fallback=0)
        n = self.totals["reconfigs"] + 1
        start = self.clock()
        rc, out = None, ""
        self.reconfiguring = True
        if limit and n > limit:
            self.log(f"reconfiguration limit ({limit}) reached; stopping the test")
            self.stop_requested = True
            self.reconfiguring = False
            return
        if cmd:
            self.log(f"[{self.now():9.1f}s] reconfiguring ({trigger}): {cmd}")
            try:
                r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                                   timeout=self.cfg.getfloat("reconfigure", "timeout_s", fallback=120))
                rc, out = r.returncode, (r.stdout + r.stderr).strip()
            except subprocess.TimeoutExpired:
                rc, out = "timeout", ""
        else:
            self.log(f"[{self.now():9.1f}s] configuration error ({trigger}); no reconfigure command set")
        time.sleep(self.cfg.getfloat("reconfigure", "settle_s", fallback=2.0))
        # drain what arrived meanwhile (clock lost/back while reprogramming)
        self.poll(wait=0)
        end = self.clock()
        self.downtime += end - start
        self.totals["reconfigs"] += 1
        if rc not in (0, None):
            self.totals["reconfig_failures"] += 1
        self.rc_w.writerow([n, trigger, round(start - self.t0, 3), round(end - self.t0, 3),
                            round(end - start, 3), rc, out[-200:].replace("\n", " | ")])
        self.rc_f.flush()
        self.reconfiguring = False
        self.send(P.CMD_RESUME)      # leaves the halt; ignored if not halted
        self.clock_ok = True

    # ---------------------------------------------------------------- counters
    def dump(self):
        self.send(P.CMD_DUMP)
        rows = []
        while True:
            f = self.next_frame({P.FT_COUNTER, P.FT_DUMPEND}, timeout=60)
            if f.type == P.FT_DUMPEND:
                return rows
            rows.append(P.parse_counter(f, self.info))

    # ---------------------------------------------------------------- the test
    def run(self, duration):
        os.makedirs(self.outdir, exist_ok=True)
        self.ev_f, self.ev_w = self.open_csv("events.csv", [
            "time_s", "wall_time", "kind", "sweep", "address", "lane", "bit",
            "register", "sample", "replicas"])
        self.rc_f, self.rc_w = self.open_csv("reconfig.csv", [
            "n", "trigger", "start_s", "end_s", "duration_s", "return_code", "output"])
        self.stop_requested = False
        count_down = self.cfg.getboolean("test", "count_downtime", fallback=False)
        status_iv = self.cfg.getfloat("test", "status_interval_s", fallback=10)
        dump_iv = self.cfg.getfloat("test", "dump_interval_s", fallback=0)
        hist = None

        self.send(P.CMD_STOP)
        self.send(P.CMD_INFO)
        self.info = P.parse_info(self.next_frame({P.FT_INFO}))
        self.log(f"detector: {self.info.lanes} lanes x {self.info.num_addr} addresses "
                 f"({self.info.lanes * self.info.num_addr} bits per replica)")
        if self.layout.width and self.layout.width > self.info.lanes * self.info.num_addr:
            self.log(f"warning: the map has {self.layout.width} bits, the detector reads "
                     f"{self.info.lanes * self.info.num_addr}")
        self.pending = [f for f in self.pending if f.type != P.FT_ACK]
        self.send(P.CMD_CLEAR)
        self.next_frame({P.FT_CLEARED}, timeout=30)
        self.t0 = self.clock()
        start_wall = self.wall()
        self.command(P.CMD_START)
        next_status = self.t0 + status_iv
        next_dump = self.t0 + dump_iv if dump_iv > 0 else None
        try:
            while not self.stop_requested:
                t = self.clock()
                active = (t - self.t0) - (0 if count_down else self.downtime)
                if active >= duration:
                    break
                while self.pending:
                    self.handle(self.pending.pop(0))
                if t >= next_status:
                    self.send(P.CMD_STATUS)
                    next_status = t + status_iv
                if next_dump is not None and t >= next_dump:
                    if hist is None:
                        hist_f, hist = self.open_csv("counters_history.csv",
                                                     ["time_s", "address", "soft", "config"])
                    for a, s, c in self.dump():
                        hist.writerow([self.now(), a, s, c])
                    hist_f.flush()
                    next_dump = t + dump_iv
                self.poll()
        except KeyboardInterrupt:
            self.log("interrupted; finishing up")
        end = self.clock()
        self.command(P.CMD_STOP)
        counters = self.dump()
        self.send(P.CMD_STATUS)
        try:
            self.next_frame({P.FT_STATUS}, timeout=5)
        except DetectorError:
            pass
        self.write_results(counters, start_wall, end)

    def write_results(self, counters, start_wall, end):
        lanes = self.info.lanes
        f, w = self.open_csv("counters.csv", ["address", "bit_lo", "bit_hi", "registers", "soft", "config"])
        for a, s, c in counters:
            w.writerow([a, a * lanes, a * lanes + lanes - 1,
                        " ".join(self.layout.registers_in(a, lanes)), s, c])
        f.close()
        f, w = self.open_csv("per_bit.csv", ["bit", "address", "lane", "register", "soft", "config"])
        for bit in sorted(self.per_bit):
            s, c = self.per_bit[bit]
            w.writerow([bit, bit // lanes, bit % lanes, self.layout.describe(bit), s, c])
        f.close()
        wall = end - self.t0
        hw_soft = sum(s for _, s, _ in counters)
        hw_cfg = sum(c for _, _, c in counters)
        f, w = self.open_csv("summary.csv", ["item", "value"])
        for k, v in [("start", start_wall), ("end", self.wall()),
                     ("wall_s", round(wall, 3)), ("downtime_s", round(self.downtime, 3)),
                     ("active_s", round(wall - self.downtime, 3)),
                     ("lanes", lanes), ("num_addr", self.info.num_addr),
                     ("bits_per_replica", lanes * self.info.num_addr),
                     ("sweeps_reported", self.sweeps),
                     ("soft_events", self.totals["soft_events"]),
                     ("soft_bits", self.totals["soft_bits"]),
                     ("config_errors", self.totals["config_errors"]),
                     ("clock_losses", self.totals["clock_losses"]),
                     ("reconfigs", self.totals["reconfigs"]),
                     ("reconfig_failures", self.totals["reconfig_failures"]),
                     ("hw_soft_count", hw_soft), ("hw_config_count", hw_cfg),
                     ("fifo_overflow", int(self.overflow)),
                     ("frame_errors", self.reader.bad)]:
            w.writerow([k, v])
        f.close()
        self.ev_f.close()
        self.rc_f.close()
        if hw_soft != self.totals["soft_bits"] or hw_cfg != self.totals["config_errors"]:
            self.log(f"note: hardware counters ({hw_soft} soft, {hw_cfg} config) differ from the "
                     f"events received ({self.totals['soft_bits']}, {self.totals['config_errors']})"
                     + (" -- the event FIFO overflowed" if self.overflow else ""))
        self.log(f"results in {self.outdir}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("config")
    ap.add_argument("--duration", type=float, help="test time in seconds (overrides [test] duration_s)")
    ap.add_argument("--out", help="output directory (default: [output] dir/<timestamp>)")
    a = ap.parse_args(argv)
    cfg = configparser.ConfigParser()
    if not cfg.read(a.config):
        sys.exit(f"cannot read {a.config}")
    duration = a.duration if a.duration is not None else cfg.getfloat("test", "duration_s")
    out = a.out or os.path.join(cfg.get("output", "dir", fallback="results"),
                                datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
    import serial                       # pyserial
    port = serial.Serial(cfg.get("serial", "port"), cfg.getint("serial", "baud", fallback=115200),
                         timeout=0.1)
    try:
        Monitor(cfg, port, out).run(duration)
    finally:
        port.close()


if __name__ == "__main__":
    main()
