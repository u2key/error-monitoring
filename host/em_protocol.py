"""Frames and commands between the host and the tester (see hdl/em_host_if.vhd).

Tester -> host: 0xA5, type, length, payload, checksum (xor of type, length and
payload). Host -> tester: a command byte, some with a payload.
"""
from dataclasses import dataclass, field

SYNC = 0xA5

FT_SOFT, FT_CONFIG, FT_SWEEP, FT_COUNTER = 0x01, 0x02, 0x03, 0x04
FT_STATUS, FT_DUMPEND, FT_ACK, FT_INFO = 0x05, 0x06, 0x07, 0x08
FT_LINK, FT_CLEARED, FT_TRAIN, FT_NACK = 0x09, 0x0A, 0x0B, 0x0C

CMD_START, CMD_STOP, CMD_CLEAR, CMD_RESUME = b"S", b"P", b"C", b"R"
CMD_DUMP, CMD_INFO, CMD_STATUS, CMD_TRAIN = b"D", b"I", b"?", b"T"

ALL_LINES = 0xFF

TYPE_NAMES = {FT_SOFT: "soft", FT_CONFIG: "config", FT_SWEEP: "sweep",
              FT_COUNTER: "counter", FT_STATUS: "status", FT_DUMPEND: "dumpend",
              FT_ACK: "ack", FT_INFO: "info", FT_LINK: "link", FT_CLEARED: "cleared",
              FT_TRAIN: "train", FT_NACK: "nack"}


def cmd_ratio(r):
    return b"F" + r.to_bytes(3, "big")


def cmd_latency(l):
    return b"L" + bytes([l])


def cmd_tap(line, tap):
    return b"A" + bytes([line, tap])


def cmd_mode(use_list):
    return b"M" + bytes([1 if use_list else 0])


def cmd_list(addresses, addr_bytes):
    return b"U" + len(addresses).to_bytes(2, "big") + b"".join(
        a.to_bytes(addr_bytes, "big") for a in addresses)


@dataclass
class Info:
    version: int
    lanes: int
    addr_width: int
    num_addr: int
    guard: int
    wmax: int
    taps: int
    lmax: int
    list_depth: int
    clk_hz: int

    @property
    def addr_bytes(self):
        return (self.addr_width + 7) // 8

    @property
    def lane_bytes(self):
        return (self.lanes + 7) // 8

    @property
    def lines(self):
        """Input lines with their own delay: lanes a, b, c, then echoes a, b, c."""
        return 3 * self.lanes + 3

    def line_name(self, line):
        if line >= 3 * self.lanes:
            return "echo_" + "abc"[line - 3 * self.lanes]
        return f"lane_{'abc'[line // self.lanes]}[{line % self.lanes}]"


@dataclass
class Frame:
    type: int
    payload: bytes

    @property
    def name(self):
        return TYPE_NAMES.get(self.type, f"0x{self.type:02x}")


@dataclass
class ErrorEvent:
    """A SOFT or CONFIG event: the replica masks of both reads."""
    kind: str                 # "soft" or "config"
    address: int
    sweep: int
    masks: list = field(default_factory=list)   # [a1, b1, c1, a2, b2, c2] as ints
    echo: int = 0             # bad echoes: read 1 c b a (bits 5..3), read 2 c b a (2..0)

    def error_bits(self, read):
        """Lanes in error in read 1 or 2: some replica disagreed."""
        a, b, c = self.masks[3 * (read - 1): 3 * read]
        return a | b | c

    def replicas(self, read, lane):
        """Which replicas disagreed with the majority at a lane, e.g. "A"."""
        a, b, c = self.masks[3 * (read - 1): 3 * read]
        return "".join(r for r, m in zip("ABC", (a, b, c)) if m >> lane & 1)

    def echo_text(self):
        """Replicas whose echo was wrong, e.g. "A1 C2"; empty if none."""
        return " ".join(f"{r}{read}" for read, shift in ((1, 3), (2, 0))
                        for i, r in enumerate("ABC") if self.echo >> (shift + i) & 1)


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
    return Info(version=p[0], lanes=be(p[1:3]), addr_width=p[3], num_addr=be(p[4:8]), guard=p[8],
                wmax=be(p[9:11]), taps=p[11], lmax=p[12], list_depth=be(p[13:15]), clk_hz=be(p[15:19]))


def parse_event(f, info):
    ab, lb = info.addr_bytes, info.lane_bytes
    p = f.payload
    address, sweep = be(p[:ab]), be(p[ab:ab + 4])
    masks = [be(p[ab + 4 + m * lb: ab + 4 + (m + 1) * lb]) for m in range(6)]
    return ErrorEvent("soft" if f.type == FT_SOFT else "config", address, sweep, masks,
                      p[ab + 4 + 6 * lb])


def parse_sweep(f, info):
    ab = info.addr_bytes
    return be(f.payload[ab:ab + 4])


def parse_counter(f, info):
    ab = info.addr_bytes
    p = f.payload
    return be(p[:ab]), be(p[ab:ab + 2]), p[ab + 2]


def parse_status(f):
    p = f.payload
    b = p[0]
    return {"run": bool(b & 1), "halted": bool(b & 2), "link": bool(b & 4), "overflow": bool(b & 8),
            "busy": bool(b & 16), "echo_bad": (be(p[1:5]), be(p[5:9]), be(p[9:13]))}


def parse_train(f, info):
    """The lines that passed a training run, as a mask (bit i = line i)."""
    lb, lanes = info.lane_bytes, info.lanes
    p = f.payload
    a, b, c = (be(p[m * lb: (m + 1) * lb]) for m in range(3))
    return a | b << lanes | c << 2 * lanes | (p[3 * lb] & 7) << 3 * lanes


def frame_bytes(typ, payload=b""):
    """Encode a frame (used by the tests' tester model)."""
    s = typ ^ len(payload)
    for x in payload:
        s ^= x
    return bytes([SYNC, typ, len(payload)]) + bytes(payload) + bytes([s])
