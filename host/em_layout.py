"""Which register a TMRDFF bit belongs to.

Reads <top>.tmrdff.map written by generate-tmr-vhdl --probe:

    -- TOP: TMRDFF_A/_B/_C, 37 bits each, same layout
    -- bits         register (hierarchical path)
    0              Q_TMP
    16:1           MEM
    ...

Bit b of TMRDFF is lane b % LANES of address b // LANES.
"""
import bisect
import re


class Layout:
    def __init__(self, ranges=None, width=None):
        # sorted [(lo, hi, path)]
        self.ranges = sorted(ranges or [])
        self.los = [r[0] for r in self.ranges]
        self.width = width

    @classmethod
    def from_map(cls, path):
        ranges, width = [], None
        for line in open(path):
            if line.startswith("--"):
                m = re.search(r"(\d+) bits each", line)
                if m:
                    width = int(m.group(1))
                continue
            parts = line.split(None, 1)
            if len(parts) != 2:
                continue
            rng, name = parts[0], parts[1].strip()
            if not re.fullmatch(r"\d+(:\d+)?", rng):
                continue                       # an entry the converter could not evaluate
            if ":" in rng:
                hi, lo = (int(x) for x in rng.split(":"))
            else:
                hi = lo = int(rng)
            ranges.append((lo, hi, name))
        return cls(ranges, width)

    def register(self, bit):
        """(path, index within the register) of a TMRDFF bit, or ("", None)."""
        i = bisect.bisect_right(self.los, bit) - 1
        if i >= 0:
            lo, hi, name = self.ranges[i]
            if bit <= hi:
                return name, bit - lo
        return "", None

    def describe(self, bit):
        name, k = self.register(bit)
        if not name:
            return ""
        lo, hi = next((r[0], r[1]) for r in self.ranges if r[2] == name and r[0] <= bit <= r[1])
        return name if lo == hi else f"{name}[{k}]"

    def registers_in(self, address, lanes):
        """Registers that have bits in one address (word), in order."""
        lo, hi = address * lanes, address * lanes + lanes - 1
        names = []
        for r in self.ranges:
            if r[0] <= hi and r[1] >= lo and r[2] not in names:
                names.append(r[2])
        return names
