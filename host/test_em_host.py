"""Tests of the host side against a model of the tester.

    python3 -m unittest test_em_host -v
"""
import configparser
import csv
import os
import sys
import tempfile
import unittest

import em_protocol as P
from em_host import Monitor, Scan, TesterError, pick_calibration, poisson_limits, read_list_file
from em_layout import Layout

LANES, NUM_ADDR, ADDR_W, GUARD, WMAX, TAPS, LMAX = 8, 5, 3, 1, 16, 8, 4
LINES = 3 * LANES + 3
INFO = P.Info(2, LANES, ADDR_W, NUM_ADDR, GUARD, WMAX, TAPS, LMAX, 4, 50000000)
LATENCY = 3                    # the model's true read latency


class FakeTester:
    """Speaks the tester's protocol. Training passes at LATENCY, and at taps 2..5
    (line 0: 3..6). After START it reports, once each: a flip-flop error, then a
    configuration error (and halts), then after a restart another flip-flop error."""

    def __init__(self, link_loss=False):
        self.out = bytearray()
        self.running = self.halted = False
        self.counters = {a: [0, 0] for a in range(NUM_ADDR)}
        self.script = []
        self.starts = 0
        self.ticks = 0
        self.lat = 0
        self.taps = [0] * LINES
        self.ratio = self.mode = None
        self.list = []
        self.link_loss = link_loss
        self.commands = []

    # pyserial-like interface
    in_waiting = 0

    def write(self, cmd):
        c = bytes(cmd)
        self.commands.append(c)
        op = c[:1]
        if op == P.CMD_STOP + b"" and len(c) > 1:      # several commands in one write
            self.write(c[:1])
            self.write(c[1:])
        elif op == P.CMD_INFO:
            self.frame(P.FT_INFO, bytes([2, 0, LANES, ADDR_W]) + NUM_ADDR.to_bytes(4, "big")
                       + bytes([GUARD, 0, WMAX, TAPS, LMAX, 0, 4]) + (50000000).to_bytes(4, "big"))
        elif op == P.CMD_STATUS:
            self.frame(P.FT_STATUS, bytes([int(self.running) | int(self.halted) << 1 | 4])
                       + (0).to_bytes(4, "big") + (3).to_bytes(4, "big") + (0).to_bytes(4, "big"))
        elif op == P.CMD_DUMP:
            for a in range(NUM_ADDR):
                s, k = self.counters[a]
                self.frame(P.FT_COUNTER, bytes([a]) + s.to_bytes(2, "big") + bytes([k]))
            self.frame(P.FT_DUMPEND)
        elif op == P.CMD_TRAIN:
            mask = 0
            if self.lat == LATENCY:
                for line in range(LINES):
                    lo = 3 if line == 0 else 2
                    if lo <= self.taps[line] <= lo + 3:
                        mask |= 1 << line
            self.frame(P.FT_TRAIN, bytes([mask & 255, mask >> 8 & 255, mask >> 16 & 255, mask >> 24 & 7]))
        elif op == b"L":
            self.lat = c[1]
            self.frame(P.FT_ACK, op)
        elif op == b"A":
            if c[1] == P.ALL_LINES:
                self.taps = [c[2]] * LINES
            else:
                self.taps[c[1]] = c[2]
            self.frame(P.FT_ACK, op)
        elif op == b"F":
            self.ratio = P.be(c[1:4])
            self.frame(P.FT_NACK if self.running else P.FT_ACK, op)
        elif op == b"M":
            self.mode = c[1]
            self.frame(P.FT_ACK, op)
        elif op == b"U":
            self.list = list(c[3:])
            self.frame(P.FT_ACK, op)
        else:
            self.frame(P.FT_ACK, c)
            if c == P.CMD_CLEAR:
                self.counters = {a: [0, 0] for a in range(NUM_ADDR)}
                self.frame(P.FT_CLEARED)
            elif c == P.CMD_START:
                self.running = True
                self.starts += 1
                if self.starts == 1:
                    self.script = [("soft", 2, 0b00000010, 0, 0b001000),
                                   ("config", 3, 0b00010001, 0b00010000, 0)]
                else:
                    self.script = [("soft", 4, 0, 0b00000001, 0)]
                if self.link_loss and self.starts == 1:
                    self.script = [("link",)]
            elif c == P.CMD_STOP:
                self.running = self.halted = False

    def frame(self, t, pay=b""):
        self.out += P.frame_bytes(t, pay)

    def read(self, n):
        self.ticks += 1
        if self.running and not self.halted and self.script and self.ticks % 3 == 0:
            item = self.script.pop(0)
            if item[0] == "link":
                self.frame(P.FT_LINK, bytes([0]))
            else:
                kind, addr, a1, a2, echo = item
                # replica A disagrees: masks a1 b1 c1 a2 b2 c2
                pay = bytes([addr]) + (7).to_bytes(4, "big") + bytes([a1, 0, 0, a2, 0, 0, echo])
                self.frame(P.FT_SOFT if kind == "soft" else P.FT_CONFIG, pay)
                self.counters[addr][0] += bin(a1 ^ a2).count("1")
                if kind == "config":
                    self.counters[addr][1] += 1
                    self.halted = True
        data, self.out = bytes(self.out[:max(n, 64)]), self.out[max(n, 64):]
        return data


def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def write_map(d):
    mp = os.path.join(d, "top.tmrdff.map")
    with open(mp, "w") as f:
        f.write("-- TOP: TMRDFF_A/_B/_C, 40 bits each, same layout\n"
                "0              Q_TMP\n16:1           MEM\n20:17          HOLD\n39:21          REST\n")
    return mp


class ProtocolTest(unittest.TestCase):
    def test_frames_and_resync(self):
        good = P.frame_bytes(P.FT_ACK, b"S")
        bad = bytearray(P.frame_bytes(P.FT_ACK, b"P"))
        bad[-1] ^= 0xFF
        r = P.FrameReader()
        frames = r.feed(b"\x00\x13" + bytes(bad) + good[:2])
        frames += r.feed(good[2:])
        self.assertEqual([(f.type, f.payload) for f in frames], [(P.FT_ACK, b"S")])
        self.assertGreater(r.bad, 0)

    def test_event_masks(self):
        f = P.Frame(P.FT_CONFIG, bytes([3]) + (9).to_bytes(4, "big") + bytes([1, 0, 0, 0, 1, 0, 0b100001]))
        ev = P.parse_event(f, INFO)
        self.assertEqual((ev.kind, ev.address, ev.sweep), ("config", 3, 9))
        self.assertEqual(ev.replicas(1, 0), "A")
        self.assertEqual(ev.replicas(2, 0), "B")
        self.assertEqual(ev.echo_text(), "C1 A2")

    def test_train_lines(self):
        f = P.Frame(P.FT_TRAIN, bytes([0x01, 0x02, 0x80, 0b101]))
        self.assertEqual(P.parse_train(f, INFO), 1 | 2 << 8 | 0x80 << 16 | 0b101 << 24)
        self.assertEqual(INFO.line_name(9), "lane_b[1]")
        self.assertEqual(INFO.line_name(25), "echo_b")

    def test_commands(self):
        self.assertEqual(P.cmd_ratio(0x010203), b"F\x01\x02\x03")
        self.assertEqual(P.cmd_list([1, 0x20304], 3), b"U\x00\x02\x00\x00\x01\x02\x03\x04")


class ScanTest(unittest.TestCase):
    def test_sweep(self):
        s = Scan(8, 1, 256, 10)
        self.assertEqual((s.window, s.windows), (7, 2))
        self.assertEqual(s.phase(4), 5)              # the last 7 phases: 1..7
        self.assertAlmostEqual(s.visibility(4), 5.5 / 8 / 2)
        self.assertEqual(s.phase(7), 5)              # short last window: the last 3 phases
        self.assertEqual(s.phase(9), 7)
        self.assertEqual(s.sweep_cycles(), 2 * 2 * 8)

    def test_list_and_fixed(self):
        s = Scan(8, 1, 256, 3, [2, 4, 7])
        self.assertAlmostEqual(s.visibility(4), 6.5 / 8)
        self.assertEqual(s.visibility(5), 0.0)
        self.assertAlmostEqual(Scan(8, 1, 256, 1, [5]).visibility(5), 7.5 / 8)

    def test_slow_design_clock(self):
        # a ratio much larger than the addresses: all read near the end of the period
        s = Scan(1000, 1, 8, 8)
        self.assertEqual(s.window, 8)
        self.assertEqual(s.phase(0), 992)

    def test_limits(self):
        # exact Poisson 95 % limits: n=0 (0, 3.69), n=10 (4.80, 18.39)
        self.assertEqual(poisson_limits(0)[0], 0.0)
        self.assertAlmostEqual(poisson_limits(0)[1], 3.69, delta=0.1)
        self.assertAlmostEqual(poisson_limits(10)[0], 4.80, delta=0.1)
        self.assertAlmostEqual(poisson_limits(10)[1], 18.39, delta=0.1)


class CalibrationTest(unittest.TestCase):
    def test_pick(self):
        # 2 lines. Latency 2: line 1 never passes. Latency 3: both, at different taps.
        passes = {(2, t): 0b01 for t in range(8)}
        passes.update({(3, t): (0b01 if 1 <= t <= 4 else 0) | (0b10 if 3 <= t <= 7 else 0)
                       for t in range(8)})
        lat, taps, eyes = pick_calibration(passes, 2, 4, 8)
        self.assertEqual(lat, 3)
        self.assertEqual(eyes, [4, 5])
        self.assertEqual(taps, [2, 5])

    def test_prefers_wider(self):
        passes = {(1, 0): 0b1, (2, 2): 0b1, (2, 3): 0b1, (2, 4): 0b1}
        self.assertEqual(pick_calibration(passes, 1, 3, 8)[0], 2)

    def test_nothing_passes(self):
        self.assertIsNone(pick_calibration({(1, 0): 0b01}, 2, 4, 8))


class LayoutTest(unittest.TestCase):
    def test_map(self):
        with tempfile.NamedTemporaryFile("w", suffix=".map", delete=False) as f:
            f.write("-- TOP: TMRDFF_A/_B/_C, 37 bits each, same layout\n"
                    "-- bits         register (hierarchical path)\n"
                    "0              Q_TMP\n16:1           MEM\n20:17          U1/HOLD\n")
        lay = Layout.from_map(f.name)
        os.unlink(f.name)
        self.assertEqual(lay.width, 37)
        self.assertEqual(lay.describe(0), "Q_TMP")
        self.assertEqual(lay.describe(18), "U1/HOLD[1]")
        self.assertEqual(lay.describe(30), "")
        self.assertEqual(lay.registers_in(2, 8), ["MEM", "U1/HOLD"])
        self.assertEqual(lay.addresses_of("HOLD", 8), [2])
        self.assertEqual(lay.addresses_of("U1/HOLD", 4), [4, 5])


class ListFileTest(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.lay = Layout.from_map(write_map(self.d))
        self.path = os.path.join(self.d, "list")

    def test_entries(self):
        with open(self.path, "w") as f:
            f.write("# suspects\n3\nbit 17  # in address 2\nHOLD\n3\n")
        self.assertEqual(read_list_file(self.path, INFO, self.lay), [3, 2])

    def test_counters(self):
        with open(self.path, "w") as f:
            f.write("address,bit_lo,bit_hi,registers,soft,config\n"
                    "0,0,7,Q_TMP,0,0\n1,8,15,MEM,5,0\n2,16,23,MEM,0,1\n")
        self.assertEqual(read_list_file(self.path, INFO, self.lay), [1, 2])

    def test_unknown(self):
        with open(self.path, "w") as f:
            f.write("NO_SUCH_REGISTER\n")
        with self.assertRaises(TesterError):
            read_list_file(self.path, INFO, self.lay)


class MonitorTest(unittest.TestCase):
    def config(self, d, marker=None):
        cfg = configparser.ConfigParser()
        cfg.read_dict({
            "layout": {"map": write_map(d)},
            "reconfigure": {"command": f"{sys.executable} -c \"open(r'{marker}', 'a').write('x')\"",
                            "settle_s": "0", "timeout_s": "30"},
            "test": {"status_interval_s": "0.2", "dump_interval_s": "0"},
        })
        return cfg

    def test_run(self):
        d = tempfile.mkdtemp()
        marker = os.path.join(d, "reprogrammed")
        dev = FakeTester()
        out = os.path.join(d, "out")
        Monitor(self.config(d, marker), dev, out, log=lambda *a: None).run(1.0, ratio=8)

        self.assertEqual(open(marker).read(), "x")          # reconfigured once
        self.assertEqual(dev.starts, 2)                      # and started again
        self.assertEqual(dev.ratio, 8)
        self.assertEqual(dev.lat, LATENCY)
        self.assertEqual(dev.taps[:2], [4, 3])               # eyes 3..6 and 2..5
        ev = read_csv(os.path.join(out, "events.csv"))
        got = [(e["kind"], e["address"], e["lane"], e["bit"], e["register"], e["read"], e["replicas"],
                e["echo_bad"]) for e in ev]
        self.assertEqual(got, [
            ("soft", "2", "1", "17", "HOLD[0]", "1", "A", "A1"),
            ("soft", "3", "0", "24", "REST[3]", "1", "A", ""),   # beside the configuration error
            ("config", "3", "4", "28", "REST[7]", "both", "A/A", ""),
            ("soft", "4", "0", "32", "REST[11]", "2", "A", ""),
        ])
        rc = read_csv(os.path.join(out, "reconfig.csv"))
        self.assertEqual([(r["trigger"], r["return_code"]) for r in rc], [("config", "0")])
        cal = read_csv(os.path.join(out, "calib.csv"))
        self.assertEqual(len(cal), 2 * LINES)                # at the start and after reconfiguring
        self.assertEqual((cal[0]["n"], cal[0]["latency"], cal[0]["name"], cal[0]["tap"], cal[0]["eye_taps"]),
                         ("1", "3", "lane_a[0]", "4", "4"))
        cnt = {int(r["address"]): r for r in read_csv(os.path.join(out, "counters.csv"))}
        self.assertEqual((cnt[2]["soft"], cnt[3]["config"], cnt[4]["soft"]), ("1", "1", "1"))
        self.assertEqual(cnt[2]["registers"], "MEM HOLD REST")   # bits 16..23
        summ = {r["item"]: r["value"] for r in read_csv(os.path.join(out, "summary.csv"))}
        self.assertEqual((summ["soft_bits"], summ["config_errors"], summ["reconfigs"]), ("3", "1", "1"))
        self.assertEqual((summ["hw_soft_count"], summ["hw_config_count"]), ("3", "1"))
        self.assertEqual((summ["ratio"], summ["window"], summ["windows_per_sweep"], summ["guard"]),
                         ("8", "5", "1", "1"))
        self.assertEqual((summ["echo_bad_a"], summ["echo_bad_b"]), ("0", "3"))
        # 5 addresses in one window at phases 3..7: address k read at phase 3 + k
        est = 8 / 5.5 + 8 / 6.5 + 8 / 7.5
        self.assertAlmostEqual(float(summ["soft_estimated"]), est, delta=0.1)
        self.assertLess(float(summ["soft_estimated_low95"]), est)
        self.assertGreater(float(summ["soft_estimated_high95"]), est)
        pb = read_csv(os.path.join(out, "per_bit.csv"))
        self.assertEqual([(r["bit"], r["soft"], r["config"]) for r in pb],
                         [("17", "1", "0"), ("24", "1", "0"), ("28", "0", "1"), ("32", "1", "0")])
        self.assertAlmostEqual(float(pb[0]["soft_estimated"]), 8 / 5.5, delta=0.1)

    def test_list_mode(self):
        d = tempfile.mkdtemp()
        dev = FakeTester()
        out = os.path.join(d, "out")
        Monitor(self.config(d, os.path.join(d, "m")), dev, out, log=lambda *a: None).run(
            0.5, ratio=8, mode="list", addresses=[2, 4])
        self.assertEqual(dev.list, [2, 4])
        self.assertEqual(dev.mode, 1)
        summ = {r["item"]: r["value"] for r in read_csv(os.path.join(out, "summary.csv"))}
        self.assertEqual((summ["mode"], summ["addresses_scanned"], summ["window"]), ("list", "2", "2"))

    def test_list_too_long(self):
        d = tempfile.mkdtemp()
        with self.assertRaises(TesterError):
            Monitor(self.config(d, os.path.join(d, "m")), FakeTester(), os.path.join(d, "out"),
                    log=lambda *a: None).run(0.1, ratio=8, mode="list", addresses=[0, 1, 2, 3, 4])

    def test_link_loss(self):
        d = tempfile.mkdtemp()
        marker = os.path.join(d, "reprogrammed")
        dev = FakeTester(link_loss=True)
        out = os.path.join(d, "out")
        Monitor(self.config(d, marker), dev, out, log=lambda *a: None).run(0.5, ratio=8)
        self.assertEqual(open(marker).read(), "x")
        rc = read_csv(os.path.join(out, "reconfig.csv"))
        self.assertEqual([r["trigger"] for r in rc], ["link_loss"])
        ev = read_csv(os.path.join(out, "events.csv"))
        self.assertEqual(ev[0]["kind"], "link_loss")


if __name__ == "__main__":
    unittest.main()
