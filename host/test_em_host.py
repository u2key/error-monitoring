"""Tests of the host side against a model of the detector.

    python3 -m unittest test_em_host -v
"""
import configparser
import csv
import os
import sys
import tempfile
import unittest

import em_protocol as P
from em_layout import Layout
from em_host import Monitor, visibility, poisson_limits

LANES, NUM_ADDR, ADDR_W = 8, 5, 3


class FakeDetector:
    """Speaks the detector's protocol. After START it reports, once each:
    a flip-flop error, then a configuration error (and halts until RESUME),
    then after RESUME another flip-flop error."""

    def __init__(self):
        self.out = bytearray()
        self.running = self.halted = False
        self.counters = {a: [0, 0] for a in range(NUM_ADDR)}
        self.script = []
        self.resumes = 0
        self.ticks = 0

    # pyserial-like interface
    in_waiting = 0

    def write(self, cmd):
        c = bytes(cmd)
        if c == P.CMD_INFO:
            self.frame(P.FT_INFO, bytes([1, 0, LANES, ADDR_W]) + NUM_ADDR.to_bytes(4, "big") + bytes([8]))
        elif c == P.CMD_STATUS:
            self.frame(P.FT_STATUS, bytes([int(self.running) | int(self.halted) << 1 | 4]))
        elif c == P.CMD_DUMP:
            for a in range(NUM_ADDR):
                s, k = self.counters[a]
                self.frame(P.FT_COUNTER, bytes([a]) + s.to_bytes(2, "big") + bytes([k]))
            self.frame(P.FT_DUMPEND)
        else:
            self.frame(P.FT_ACK, c)
            if c == P.CMD_CLEAR:
                self.counters = {a: [0, 0] for a in range(NUM_ADDR)}
                self.frame(P.FT_CLEARED)
            elif c == P.CMD_START:
                self.running = True
                self.script = [("soft", 2, 0b00000010, 0), ("config", 3, 0b00010000, 0b00010000)]
            elif c == P.CMD_STOP:
                self.running = False
            elif c == P.CMD_RESUME and self.halted:
                self.halted = False
                self.resumes += 1
                self.script = [("soft", 4, 0, 0b00000001)]

    def frame(self, t, pay=b""):
        self.out += P.frame_bytes(t, pay)

    def read(self, n):
        self.ticks += 1
        if self.running and not self.halted and self.script and self.ticks % 3 == 0:
            kind, addr, a1, a2 = self.script.pop(0)
            # replica A disagrees: masks a1 b1 c1 a2 b2 c2
            pay = bytes([addr]) + (7).to_bytes(4, "big") + bytes([a1, 0, 0, a2, 0, 0])
            self.frame(P.FT_SOFT if kind == "soft" else P.FT_CONFIG, pay)
            m1, m2 = a1, a2
            if kind == "soft":
                self.counters[addr][0] += bin((m1 | m2) & ~(m1 & m2)).count("1")
            else:
                self.counters[addr][1] += 1
                self.halted = True
        data, self.out = bytes(self.out[:max(n, 64)]), self.out[max(n, 64):]
        return data


def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


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
        info = P.Info(1, LANES, ADDR_W, NUM_ADDR, 8)
        f = P.Frame(P.FT_CONFIG, bytes([3]) + (9).to_bytes(4, "big") + bytes([1, 0, 0, 0, 1, 0]))
        ev = P.parse_event(f, info)
        self.assertEqual((ev.kind, ev.address, ev.sweep), ("config", 3, 9))
        self.assertEqual(ev.replicas(1, 0), "A")
        self.assertEqual(ev.replicas(2, 0), "B")


class CorrectionTest(unittest.TestCase):
    def test_limits(self):
        # exact Poisson 95 % limits: n=0 (0, 3.69), n=10 (4.80, 18.39)
        self.assertEqual(poisson_limits(0)[0], 0.0)
        self.assertAlmostEqual(poisson_limits(0)[1], 3.69, delta=0.1)
        self.assertAlmostEqual(poisson_limits(10)[0], 4.80, delta=0.1)
        self.assertAlmostEqual(poisson_limits(10)[1], 18.39, delta=0.1)


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


class MonitorTest(unittest.TestCase):
    def test_run(self):
        d = tempfile.mkdtemp()
        mp = os.path.join(d, "top.tmrdff.map")
        with open(mp, "w") as f:
            f.write("-- TOP: TMRDFF_A/_B/_C, 40 bits each, same layout\n"
                    "0              Q_TMP\n16:1           MEM\n20:17          HOLD\n39:21          REST\n")
        marker = os.path.join(d, "reprogrammed")
        cfg = configparser.ConfigParser()
        cfg.read_dict({
            "layout": {"map": mp},
            "reconfigure": {"command": f"{sys.executable} -c \"open(r'{marker}', 'a').write('x')\"",
                            "settle_s": "0", "timeout_s": "30"},
            "test": {"status_interval_s": "0.2", "dump_interval_s": "0"},
        })
        dev = FakeDetector()
        out = os.path.join(d, "out")
        Monitor(cfg, dev, out, log=lambda *a: None).run(duration=1.0)

        self.assertEqual(open(marker).read(), "x")          # reconfigured once
        self.assertEqual(dev.resumes, 1)                     # and resumed
        ev = read_csv(os.path.join(out, "events.csv"))
        got = [(e["kind"], e["address"], e["lane"], e["bit"], e["register"], e["sample"], e["replicas"])
               for e in ev]
        self.assertEqual(got, [
            ("soft", "2", "1", "17", "HOLD[0]", "1", "A"),
            ("config", "3", "4", "28", "REST[7]", "both", "A/A"),
            ("soft", "4", "0", "32", "REST[11]", "2", "A"),
        ])
        rc = read_csv(os.path.join(out, "reconfig.csv"))
        self.assertEqual([(r["trigger"], r["return_code"]) for r in rc], [("config", "0")])
        cnt = {int(r["address"]): r for r in read_csv(os.path.join(out, "counters.csv"))}
        self.assertEqual((cnt[2]["soft"], cnt[3]["config"], cnt[4]["soft"]), ("1", "1", "1"))
        self.assertEqual(cnt[2]["registers"], "MEM HOLD REST")   # bits 16..23
        summ = {r["item"]: r["value"] for r in read_csv(os.path.join(out, "summary.csv"))}
        self.assertEqual((summ["soft_bits"], summ["config_errors"], summ["reconfigs"]), ("2", "1", "1"))
        self.assertEqual((summ["hw_soft_count"], summ["hw_config_count"]), ("2", "1"))
        vis = visibility(NUM_ADDR, 8)                        # 2 of 5 * 12 cycles
        self.assertAlmostEqual(vis, 2 / 60)
        self.assertEqual(float(summ["soft_estimated"]), 60.0)   # 2 observed / vis
        self.assertLess(float(summ["soft_estimated_low95"]), 60.0)
        self.assertGreater(float(summ["soft_estimated_high95"]), 60.0)
        pb = read_csv(os.path.join(out, "per_bit.csv"))
        self.assertEqual(float(pb[0]["soft_estimated"]), 30.0)
        self.assertEqual([(r["bit"], r["soft"], r["config"]) for r in pb],
                         [("17", "1", "0"), ("28", "0", "1"), ("32", "1", "0")])


if __name__ == "__main__":
    unittest.main()
