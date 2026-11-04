"""Frames between the host and the detector (see hdl/em_pkg.vhd, em_host_if.vhd).

Detector -> host: 0xA5, type, length, payload, checksum (xor of type, length
and payload). Host -> detector: single command bytes.
"""
from dataclasses import dataclass, field

SYNC = 0xA5

FT_SOFT, FT_CONFIG, FT_SWEEP, FT_COUNTER = 0x01, 0x02, 0x03, 0x04
FT_STATUS, FT_DUMPEND, FT_ACK, FT_INFO = 0x05, 0x06, 0x07, 0x08
FT_CLOCK, FT_CLEARED = 0x09, 0x0A

CMD_START, CMD_STOP, CMD_CLEAR, CMD_RESUME = b"S", b"P", b"C", b"R"
CMD_DUMP, CMD_INFO, CMD_STATUS = b"D", b"I", b"?"

TYPE_NAMES = {FT_SOFT: "soft", FT_CONFIG: "config", FT_SWEEP: "sweep",
              FT_COUNTER: "counter", FT_STATUS: "status", FT_DUMPEND: "dumpend",
              FT_ACK: "ack", FT_INFO: "info", FT_CLOCK: "clock", FT_CLEARED: "cleared"}


@dataclass
class Info:
    version: int
    lanes: int
    addr_width: int
    num_addr: int
    settle: int

    @property
    def addr_bytes(self):
        return (self.addr_width + 7) // 8

    @property
    def lane_bytes(self):
        return (self.lanes + 7) // 8


@dataclass
class Frame:
    type: int
    payload: bytes

    @property
    def name(self):
        return TYPE_NAMES.get(self.type, f"0x{self.type:02x}")


@dataclass
class ErrorEvent:
    """A SOFT or CONFIG event: the replica masks of both samples."""
    kind: str                 # "soft" or "config"
    address: int
    sweep: int
    masks: list = field(default_factory=list)   # [a1, b1, c1, a2, b2, c2] as ints

    def error_bits(self, sample):
        """Lanes in error in sample 1 or 2: some replica disagreed."""
        a, b, c = self.masks[3 * (sample - 1): 3 * sample]
        return a | b | c

    def replicas(self, sample, lane):
        """Which replicas disagreed with the majority at a lane, e.g. "A"."""
        a, b, c = self.masks[3 * (sample - 1): 3 * sample]
        return "".join(r for r, m in zip("ABC", (a, b, c)) if m >> lane & 1)


class FrameReader:
    """Turns a byte stream into frames, resynchronising on bad checksums."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            try:
                i = self.buf.index(SYNC)
            except ValueError:
                self.buf.clear()
                break
            if i:
                self.bad += 1
                del self.buf[:i]
            if len(self.buf) < 3:
                break
            typ, n = self.buf[1], self.buf[2]
            if len(self.buf) < 4 + n:
                break
            pay = bytes(self.buf[3:3 + n])
            s = typ ^ n
            for x in pay:
                s ^= x
            if s != self.buf[3 + n]:
                self.bad += 1
                del self.buf[:1]          # not a frame start after all
                continue
            frames.append(Frame(typ, pay))
            del self.buf[:4 + n]
        return frames


def be(b):
    return int.from_bytes(b, "big")


def parse_info(f):
    p = f.payload
    return Info(version=p[0], lanes=be(p[1:3]), addr_width=p[3], num_addr=be(p[4:8]), settle=p[8])


def parse_event(f, info):
    ab, lb = info.addr_bytes, info.lane_bytes
    p = f.payload
    address, sweep = be(p[:ab]), be(p[ab:ab + 4])
    masks = [be(p[ab + 4 + m * lb: ab + 4 + (m + 1) * lb]) for m in range(6)]
    return ErrorEvent("soft" if f.type == FT_SOFT else "config", address, sweep, masks)


def parse_sweep(f, info):
    ab = info.addr_bytes
    return be(f.payload[ab:ab + 4])


def parse_counter(f, info):
    ab = info.addr_bytes
    p = f.payload
    return be(p[:ab]), be(p[ab:ab + 2]), p[ab + 2]


def parse_status(f):
    b = f.payload[0]
    return {"run": bool(b & 1), "halted": bool(b & 2), "clock": bool(b & 4), "overflow": bool(b & 8)}


def frame_bytes(typ, payload=b""):
    """Encode a frame (used by the tests' detector model)."""
    s = typ ^ len(payload)
    for x in payload:
        s ^= x
    return bytes([SYNC, typ, len(payload)]) + bytes(payload) + bytes([s])
