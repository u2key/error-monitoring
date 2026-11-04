#!/usr/bin/env python3
"""Run an error-monitoring test from the host PC.

    python3 em_host.py config.ini [--duration SECONDS] [--out DIR] [--ratio R]
                       [--mode sweep|fixed|list] [--addr A] [--list-file FILE]

The host tells the tester the design clock ratio and what to scan, calibrates
the link, keeps the test time, reconfigures the FPGA under test when the tester
reports a configuration error, and writes the results as CSV files in a new
directory under [output] dir:

    events.csv     every error, one row per bit: time, kind (soft / config /
                   link_loss), sweep, address, lane, bit, register, read,
                   replicas that disagreed, bad echoes
    reconfig.csv   every reconfiguration: trigger, start, end, return code
    calib.csv      every calibration: latency, and tap and eye width per line
    counters.csv   the tester's per-address counters, dumped at the end
                   (counters_history.csv: the periodic dumps, if enabled)
    per_bit.csv    errors per TMRDFF bit, from the events
    summary.csv    settings, test time, active time, downtime, totals, bad echo
                   counts, and the flip-flop error estimate corrected for what
                   the scan can see

Scan: per design clock period, the last W read cycles read a window of W =
min(N, R - GUARD, WMAX) addresses, where N is the number of addresses or list
entries and R the ratio; each window is read in two successive periods.

Correction: a voting triplicated flip-flop repairs an upset at the next design
clock edge. A read at phase k of the period samples k + 0.5 read cycles after
the edge (see em_scanner.vhd), so it sees an upset that struck within that time:
(k + 0.5) / R of the period. Each window takes 2 of the 2 * windows periods of a
sweep, so for an address read at phase k the visibility is

    P = ((k + 0.5) / R) / windows

and the estimate sums 1 / P over the observed flips. It assumes upsets last to
the next design edge and fall uniformly in time, and it is statistical: with few
observed errors its uncertainty is large (the 95 % limits in summary.csv), and
with none it only gives an upper limit.
"""
import argparse
import configparser
import csv
import datetime
import math
import os
import subprocess
import sys
import time

import em_protocol as P
from em_layout import Layout


class TesterError(Exception):
    pass


class Scan:
    """The read schedule of a sweep over n addresses or list entries."""

    def __init__(self, ratio, guard, wmax, n, addresses=None):
        self.ratio, self.n = ratio, n
        self.addresses = addresses       # the list, in order; None = every address
        self.window = max(1, min(n, ratio - guard, wmax))
        self.windows = math.ceil(n / self.window)
        if addresses is not None:
            self.by_address = {}
            for i, a in enumerate(addresses):
                self.by_address[a] = self.by_address.get(a, 0.0) + self.at_index(i)

    def phase(self, i):
        """Phase in the design clock period of the read of entry i."""
        j = i // self.window
        w = self.window if j < self.windows - 1 else self.n - self.window * (self.windows - 1)
        return self.ratio - w + (i - self.window * j)

    def at_index(self, i):
        return (self.phase(i) + 0.5) / self.ratio / self.windows

    def visibility(self, address):
        """Probability that the scan sees a flip of a bit of this address."""
        if self.addresses is None:
            return self.at_index(address)
        return self.by_address.get(address, 0.0)

    def mean_visibility(self):
        return sum(self.at_index(i) for i in range(self.n)) / self.n

    def sweep_cycles(self):
        """Read clock cycles of one sweep."""
        return 2 * self.windows * self.ratio


def poisson_limits(n):
    """Approximate 95 % limits of a Poisson mean, given n observed (Wilson-Hilferty)."""
    lo = 0.0 if n == 0 else n * (1 - 1 / (9 * n) - 1.96 / (3 * math.sqrt(n))) ** 3
    m = n + 1
    return lo, m * (1 - 1 / (9 * m) + 1.96 / (3 * math.sqrt(m))) ** 3


def pick_calibration(passes, lines, lmax, taps):
    """Choose from training results: passes[(latency, tap)] = mask of passing lines.

    One latency applies to all lines, each with its own tap: the latency whose
    narrowest eye is the widest, and on each line the centre of its longest run
    of passing taps. Returns (latency, [tap], [eye width]), or None if no latency
    passes on every line.
    """
    best, best_key = None, None
    for lat in range(1, lmax + 1):
        chosen, eyes = [], []
        for line in range(lines):
            top = top_end = run = 0
            for t in range(taps):
                run = run + 1 if passes.get((lat, t), 0) >> line & 1 else 0
                if run > top:
                    top, top_end = run, t
            chosen.append(top_end - top // 2)
            eyes.append(top)
        key = (min(eyes), sum(eyes))
        if key[0] > 0 and (best_key is None or key > best_key):
            best, best_key = (lat, chosen, eyes), key
    return best


def read_list_file(path, info, layout):
    """Addresses to scan. Either a counters.csv from an earlier test (the addresses
    with errors), or one entry per line: an address, "bit N", or a register name."""
    with open(path, newline="") as f:
        text = f.read()
    first = text.split("\n", 1)[0]
    if "address" in first and "," in first:
        rows = csv.DictReader(text.splitlines())
        found = [int(r["address"]) for r in rows if int(r["soft"]) + int(r["config"]) > 0]
    else:
        found = []
        for raw in text.splitlines():
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            if line.isdigit():
                found.append(int(line))
            elif len(parts) == 2 and parts[0] == "bit" and parts[1].isdigit():
                found.append(int(parts[1]) // info.lanes)
            else:
                addrs = layout.addresses_of(line, info.lanes)
                if not addrs:
                    raise TesterError(f"{path}: {line!r} is not an address, a bit or a register in the map")
                found.extend(addrs)
    return list(dict.fromkeys(found))


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
        self.scan = None
        mp = cfg.get("layout", "map", fallback="").strip()
        self.layout = Layout.from_map(mp) if mp else Layout()
        self.t0 = None
        self.downtime = 0.0
        self.sweeps = 0
        self.per_bit = {}            # bit -> [soft, config]
        self.totals = {"soft_events": 0, "soft_bits": 0, "config_errors": 0,
                       "link_losses": 0, "reconfigs": 0, "reconfig_failures": 0,
                       "calibrations": 0}
        self.soft_estimate = 0.0
        self.overflow = False
        self.link_ok = True
        self.reconfiguring = False
        self.stop_requested = False
        self.echo_bad = (0, 0, 0)
        self.abort_reason = ""
        self.ratio = None
        self.mode = "sweep"
        self.calib_w = None

    # ---------------------------------------------------------------- link
    def send(self, cmd):
        self.port.write(cmd)

    def poll(self):
        n = getattr(self.port, "in_waiting", 0) or 1
        data = self.port.read(n)
        if data:
            self.pending += self.reader.feed(data)

    def next_frame(self, types, timeout=5.0):
        """Next frame of one of the given types; other frames are handled."""
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            while self.pending:
                f = self.pending.pop(0)
                if f.type in types:
                    return f
                self.handle(f)
            self.poll()
        raise TesterError(f"no {[P.TYPE_NAMES.get(t) for t in types]} frame within {timeout} s")

    def command(self, cmd, timeout=5.0):
        self.send(cmd)
        f = self.next_frame({P.FT_ACK, P.FT_NACK}, timeout)
        if f.payload[:1] != cmd[:1]:
            raise TesterError(f"answer for {f.payload!r} to command {cmd[:1]!r}")
        if f.type == P.FT_NACK:
            raise TesterError(f"the tester refused command {cmd!r}")

    # ---------------------------------------------------------------- files
    def open_csv(self, name, header):
        f = open(os.path.join(self.outdir, name), "w", newline="")
        w = csv.writer(f)
        w.writerow(header)
        return f, w

    def now(self):
        return round(self.clock() - self.t0, 3) if self.t0 is not None else 0.0

    def wall(self):
        return datetime.datetime.now().isoformat(timespec="milliseconds")

    # ---------------------------------------------------------------- events
    def handle(self, f):
        if self.info is None:
            return                   # left over from before this run
        if f.type in (P.FT_SOFT, P.FT_CONFIG):
            ev = P.parse_event(f, self.info)
            self.record(ev)
            if ev.kind == "config" and not self.reconfiguring:
                self.reconfigure("config")
        elif f.type == P.FT_SWEEP:
            self.sweeps = max(self.sweeps, P.parse_sweep(f, self.info) + 1)
        elif f.type == P.FT_LINK:
            ok = f.payload[0] == 1
            if not ok and self.link_ok and not self.reconfiguring:
                self.totals["link_losses"] += 1
                self.ev_w.writerow([self.now(), self.wall(), "link_loss", self.sweeps,
                                    "", "", "", "", "", "", ""])
                self.log(f"[{self.now():9.1f}s] readout link lost")
                self.link_ok = ok
                if self.cfg.getboolean("reconfigure", "link_loss_is_config", fallback=True):
                    self.reconfigure("link_loss")
            else:
                self.link_ok = ok
        elif f.type == P.FT_STATUS:
            st = P.parse_status(f)
            if st["overflow"] and not self.overflow:
                self.log("tester event queue overflowed: some events were lost")
            self.overflow = self.overflow or st["overflow"]
            if st["echo_bad"] != self.echo_bad:
                if st["echo_bad"] > self.echo_bad:
                    self.log(f"[{self.now():9.1f}s] bad echoes A/B/C: {st['echo_bad']}"
                             " -- timing may have drifted")
                self.echo_bad = st["echo_bad"]
        # ACK, CLEARED, COUNTER, DUMPEND, TRAIN outside a request: nothing to do

    def record(self, ev):
        lanes = self.info.lanes
        m1, m2 = ev.error_bits(1), ev.error_bits(2)
        bits = []
        for l in range(lanes):
            one, two = m1 >> l & 1, m2 >> l & 1
            if one and two:
                bits.append(("config", l, "both", ev.replicas(1, l) + "/" + ev.replicas(2, l)))
            elif one:
                bits.append(("soft", l, "1", ev.replicas(1, l)))
            elif two:
                bits.append(("soft", l, "2", ev.replicas(2, l)))
        if ev.kind == "config":
            self.totals["config_errors"] += 1
        else:
            self.totals["soft_events"] += 1
        vis = self.scan.visibility(ev.address) if self.scan else 0.0
        for kind, lane, read, reps in bits:
            bit = ev.address * lanes + lane
            self.ev_w.writerow([self.now(), self.wall(), kind, ev.sweep, ev.address, lane, bit,
                                self.layout.describe(bit), read, reps, ev.echo_text()])
            c = self.per_bit.setdefault(bit, [0, 0, 0.0])
            if kind == "soft":
                c[0] += 1
                self.totals["soft_bits"] += 1
                if vis > 0:
                    c[2] += 1 / vis
                    self.soft_estimate += 1 / vis
            else:
                c[1] += 1
        self.ev_f.flush()
        self.log(f"[{self.now():9.1f}s] {ev.kind} error at address {ev.address}: "
                 + ", ".join(f"bit {ev.address * lanes + l} {self.layout.describe(ev.address * lanes + l)}"
                             for _, l, _, _ in bits))

    # ---------------------------------------------------------------- calibration
    def train_map(self):
        """A training run at every latency and tap: {(latency, tap): passing lines}."""
        passes = {}
        for lat in range(1, self.info.lmax + 1):
            self.command(P.cmd_latency(lat))
            for tap in range(self.info.taps):
                self.command(P.cmd_tap(P.ALL_LINES, tap))
                self.send(P.CMD_TRAIN)
                f = self.next_frame({P.FT_TRAIN, P.FT_NACK})
                if f.type == P.FT_NACK:
                    raise TesterError("the tester refused a training run")
                passes[lat, tap] = P.parse_train(f, self.info)
        return passes

    def calibrate(self):
        """Find the read latency and each line's tap, and set them. Must be stopped."""
        picked = pick_calibration(self.train_map(), self.info.lines, self.info.lmax, self.info.taps)
        if picked is None:
            raise TesterError("calibration: no latency passes on every input line")
        lat, taps, eyes = picked
        self.command(P.cmd_latency(lat))
        for line, tap in enumerate(taps):
            self.command(P.cmd_tap(line, tap))
        self.send(P.CMD_TRAIN)
        passed = P.parse_train(self.next_frame({P.FT_TRAIN}), self.info)
        if passed != (1 << self.info.lines) - 1:
            raise TesterError(f"calibration: training fails at the chosen settings ({passed:#x})")
        self.totals["calibrations"] += 1
        n = self.totals["calibrations"]
        for line, (tap, eye) in enumerate(zip(taps, eyes)):
            self.calib_w.writerow([n, self.now(), lat, line, self.info.line_name(line), tap, eye])
        self.calib_f.flush()
        self.log(f"calibration {n}: latency {lat}, narrowest eye {min(eyes)} of {self.info.taps} taps")

    # ---------------------------------------------------------------- reconfiguration
    def reconfigure(self, trigger):
        cmd = self.cfg.get("reconfigure", "command", fallback="").strip()
        limit = self.cfg.getint("reconfigure", "max_reconfigs", fallback=0)
        n = self.totals["reconfigs"] + 1
        if limit and n > limit:
            self.log(f"reconfiguration limit ({limit}) reached; stopping the test")
            self.stop_requested = True
            return
        start = self.clock()
        rc, out = None, ""
        self.reconfiguring = True
        try:
            self.command(P.CMD_STOP)  # also leaves a halt; nothing is read while programming
        except TesterError as e:
            self.abort_reason = str(e)
            self.log(f"stopping the test: {e}")
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
        self.totals["reconfigs"] += 1
        if rc not in (0, None):
            self.totals["reconfig_failures"] += 1
        try:
            if self.cfg.getboolean("scan", "recalibrate", fallback=True) and \
                    self.cfg.getboolean("scan", "calibrate", fallback=True):
                self.calibrate()
            self.command(P.CMD_START)
        except TesterError as e:
            self.abort_reason = str(e)
            self.log(f"stopping the test after reconfiguration: {e}")
            self.stop_requested = True
        end = self.clock()
        self.downtime += end - start
        self.rc_w.writerow([n, trigger, round(start - self.t0, 3), round(end - self.t0, 3),
                            round(end - start, 3), rc, out[-200:].replace("\n", " | ")])
        self.rc_f.flush()
        self.reconfiguring = False
        self.link_ok = True

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
    def setup_scan(self, ratio, mode, addresses):
        """Set the ratio and what to scan, and work out the read schedule."""
        info = self.info
        if ratio < 2 or ratio <= info.guard or ratio >= 1 << 24:
            raise TesterError(f"ratio {ratio}: must be at least 2, above the guard of {info.guard},"
                              " and below 2**24")
        self.ratio, self.mode = ratio, mode
        self.command(P.cmd_ratio(ratio))
        if mode == "sweep":
            self.command(P.cmd_mode(False))
            self.scan = Scan(ratio, info.guard, info.wmax, info.num_addr)
        else:
            if not addresses:
                raise TesterError(f"{mode} mode needs at least one address")
            if len(addresses) > info.list_depth:
                raise TesterError(f"{len(addresses)} addresses; the tester's list holds {info.list_depth}")
            for a in addresses:
                if a >= info.num_addr:
                    raise TesterError(f"address {a} is beyond the last address {info.num_addr - 1}")
            self.command(P.cmd_list(addresses, info.addr_bytes), timeout=60)
            self.command(P.cmd_mode(True))
            self.scan = Scan(ratio, info.guard, info.wmax, len(addresses), addresses)
        self.log(f"scan: {mode}, ratio {ratio}, {self.scan.n} addresses in {self.scan.windows} windows "
                 f"of {self.scan.window}; design clock {info.clk_hz / ratio / 1e6:g} MHz, "
                 f"{self.scan.sweep_cycles() / info.clk_hz * 1e3:g} ms per sweep")

    def run(self, duration, ratio, mode="sweep", addresses=None):
        os.makedirs(self.outdir, exist_ok=True)
        self.ev_f, self.ev_w = self.open_csv("events.csv", [
            "time_s", "wall_time", "kind", "sweep", "address", "lane", "bit",
            "register", "read", "replicas", "echo_bad"])
        self.rc_f, self.rc_w = self.open_csv("reconfig.csv", [
            "n", "trigger", "start_s", "end_s", "duration_s", "return_code", "output"])
        self.calib_f, self.calib_w = self.open_csv("calib.csv", [
            "n", "time_s", "latency", "line", "name", "tap", "eye_taps"])
        count_down = self.cfg.getboolean("test", "count_downtime", fallback=False)
        status_iv = self.cfg.getfloat("test", "status_interval_s", fallback=10)
        dump_iv = self.cfg.getfloat("test", "dump_interval_s", fallback=0)
        hist = None

        self.send(P.CMD_STOP)
        self.send(P.CMD_INFO)
        self.info = P.parse_info(self.next_frame({P.FT_INFO}))
        self.log(f"tester: {self.info.lanes} lanes x {self.info.num_addr} addresses "
                 f"({self.info.lanes * self.info.num_addr} bits per replica), "
                 f"read clock {self.info.clk_hz / 1e6:g} MHz")
        if self.layout.width and self.layout.width > self.info.lanes * self.info.num_addr:
            self.log(f"warning: the map has {self.layout.width} bits, the tester reads "
                     f"{self.info.lanes * self.info.num_addr}")
        self.pending = [f for f in self.pending if f.type != P.FT_ACK]
        self.setup_scan(ratio, mode, addresses)
        if self.cfg.getboolean("scan", "calibrate", fallback=True):
            self.calibrate()
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
                while self.pending and not self.stop_requested:
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
            self.handle(self.next_frame({P.FT_STATUS}, timeout=5))
        except TesterError:
            pass
        self.write_results(counters, start_wall, end)

    def write_results(self, counters, start_wall, end):
        lanes = self.info.lanes
        f, w = self.open_csv("counters.csv", ["address", "bit_lo", "bit_hi", "registers", "soft", "config"])
        for a, s, c in counters:
            w.writerow([a, a * lanes, a * lanes + lanes - 1,
                        " ".join(self.layout.registers_in(a, lanes)), s, c])
        f.close()
        f, w = self.open_csv("per_bit.csv", ["bit", "address", "lane", "register", "soft", "config",
                                             "visibility", "soft_estimated"])
        for bit in sorted(self.per_bit):
            s, c, est = self.per_bit[bit]
            w.writerow([bit, bit // lanes, bit % lanes, self.layout.describe(bit), s, c,
                        f"{self.scan.visibility(bit // lanes):.6g}", round(est, 1)])
        f.close()
        wall = end - self.t0
        active = wall - self.downtime
        observed = self.totals["soft_bits"]
        low, high = poisson_limits(observed)
        vis = self.scan.mean_visibility()
        # the limits scale the estimate; with nothing observed, use the mean visibility
        per_flip = self.soft_estimate / observed if observed else 1 / vis
        hw_soft = sum(s for _, s, _ in counters)
        hw_cfg = sum(c for _, _, c in counters)
        f, w = self.open_csv("summary.csv", ["item", "value"])
        for k, v in [("start", start_wall), ("end", self.wall()),
                     ("wall_s", round(wall, 3)), ("downtime_s", round(self.downtime, 3)),
                     ("active_s", round(active, 3)),
                     ("mode", self.mode), ("ratio", self.ratio),
                     ("read_clk_hz", self.info.clk_hz),
                     ("design_clk_hz", self.info.clk_hz / self.ratio),
                     ("guard", self.info.guard),
                     ("window", self.scan.window), ("windows_per_sweep", self.scan.windows),
                     ("sweep_s", f"{self.scan.sweep_cycles() / self.info.clk_hz:.6g}"),
                     ("lanes", lanes), ("num_addr", self.info.num_addr),
                     ("addresses_scanned", self.scan.n),
                     ("bits_per_replica_scanned", lanes * self.scan.n),
                     ("sweeps_reported", self.sweeps),
                     ("soft_events", self.totals["soft_events"]),
                     ("soft_bits", self.totals["soft_bits"]),
                     ("config_errors", self.totals["config_errors"]),
                     ("link_losses", self.totals["link_losses"]),
                     ("reconfigs", self.totals["reconfigs"]),
                     ("reconfig_failures", self.totals["reconfig_failures"]),
                     ("calibrations", self.totals["calibrations"]),
                     ("echo_bad_a", self.echo_bad[0]), ("echo_bad_b", self.echo_bad[1]),
                     ("echo_bad_c", self.echo_bad[2]),
                     ("hw_soft_count", hw_soft), ("hw_config_count", hw_cfg),
                     ("soft_visibility_mean", f"{vis:.6g}"),
                     ("soft_estimated", round(self.soft_estimate, 1)),
                     ("soft_estimated_low95", round(low * per_flip, 1)),
                     ("soft_estimated_high95", round(high * per_flip, 1)),
                     ("soft_estimated_per_s", f"{self.soft_estimate / active:.6g}" if active > 0 else ""),
                     ("fifo_overflow", int(self.overflow)),
                     ("frame_errors", self.reader.bad),
                     ("aborted", self.abort_reason)]:
            w.writerow([k, v])
        f.close()
        self.ev_f.close()
        self.rc_f.close()
        self.calib_f.close()
        if hw_soft != self.totals["soft_bits"] or hw_cfg != self.totals["config_errors"]:
            self.log(f"note: hardware counters ({hw_soft} soft, {hw_cfg} config) differ from the "
                     f"events received ({self.totals['soft_bits']}, {self.totals['config_errors']})"
                     + (" -- the event queue overflowed" if self.overflow else ""))
        self.log(f"results in {self.outdir}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("config")
    ap.add_argument("--duration", type=float, help="test time in seconds (overrides [test] duration_s)")
    ap.add_argument("--out", help="output directory (default: [output] dir/<timestamp>)")
    ap.add_argument("--ratio", type=int, help="read clocks per design clock (overrides [scan] ratio)")
    ap.add_argument("--mode", choices=["sweep", "fixed", "list"], help="what to scan (overrides [scan] mode)")
    ap.add_argument("--addr", type=int, help="the address of fixed mode (overrides [scan] addr)")
    ap.add_argument("--list-file", help="the addresses of list mode (overrides [scan] list_file)")
    a = ap.parse_args(argv)
    cfg = configparser.ConfigParser()
    if not cfg.read(a.config):
        sys.exit(f"cannot read {a.config}")
    duration = a.duration if a.duration is not None else cfg.getfloat("test", "duration_s")
    ratio = a.ratio if a.ratio is not None else cfg.getint("scan", "ratio")
    mode = a.mode or cfg.get("scan", "mode", fallback="sweep")
    out = a.out or os.path.join(cfg.get("output", "dir", fallback="results"),
                                datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
    import serial                       # pyserial
    port = serial.Serial(cfg.get("serial", "port"), cfg.getint("serial", "baud", fallback=115200),
                         timeout=0.1)
    try:
        mon = Monitor(cfg, port, out)
        addresses = None
        if mode == "fixed":
            addr = a.addr if a.addr is not None else cfg.getint("scan", "addr")
            addresses = [addr]
        elif mode == "list":
            # the file may name bits and registers, which takes the tester's lanes
            port.write(P.CMD_STOP + P.CMD_INFO)
            info = P.parse_info(mon.next_frame({P.FT_INFO}))
            path = a.list_file or cfg.get("scan", "list_file")
            addresses = read_list_file(path, info, mon.layout)
        mon.run(duration, ratio, mode, addresses)
    except TesterError as e:
        sys.exit(f"error: {e}")
    finally:
        port.close()


if __name__ == "__main__":
    main()
