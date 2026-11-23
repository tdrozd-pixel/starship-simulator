"""Tests for the telemetry dictionary generator. Run: python -I tools/telemetry/test_generate.py"""

import copy
import os
import struct
import subprocess
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate  # noqa: E402

EXPECTED_LENGTHS = {
    "fsw_status": 20, "mission_config": 36, "navigation": 80,
    "guidance_control": 84, "actuator_feedback": 36, "gnss": 56,
}


def pack(d, packet_name, values):
    """Reference packer: big-endian struct bytes built from the dictionary, independent of the generator layout."""
    packet = next(p for p in d["packets"] if p["name"] == packet_name)
    return _pack_fields(d, packet["fields"], values)


def _pack_fields(d, fields, values):
    out = b""
    for f in fields:
        typ = f["type"]
        n = f.get("count", 1)
        if typ == "pad":
            out += b"\x00" * f["count"]
        elif typ.startswith("record:"):
            rec = d["records"][typ[len("record:"):]]
            for i in range(n):
                out += _pack_fields(d, rec["fields"], values[f["name"]][i])
        elif "bits" in f:
            flags = 0
            for i, b in enumerate(f["bits"]):
                flags |= (1 if values[b["name"]] else 0) << i
            out += struct.pack(">B", flags)
        else:
            fmt = ">" + generate.SCALARS[typ][3]
            v = values[f["name"]]
            for i in range(n):
                x = v[i] if "count" in f else v
                out += struct.pack(fmt, int(x) if typ[0] == "u" else float(x))
    return out


class GeneratorTests(unittest.TestCase):
    def setUp(self):
        self.d = generate.load_dictionary()

    def test_lengths_match_the_documented_contract(self):
        lay = generate.layout(self.d)
        got = {p["name"]: p["userDataLength"] for p in lay["packets"]}
        self.assertEqual(got, EXPECTED_LENGTHS)

    def test_validation_rejects_duplicate_apid(self):
        d = copy.deepcopy(self.d)
        d["packets"][1]["apid"] = d["packets"][0]["apid"]
        with self.assertRaisesRegex(generate.DictionaryError, "duplicate apid"):
            generate.validate(d)

    def test_validation_rejects_unknown_type(self):
        d = copy.deepcopy(self.d)
        d["packets"][0]["fields"][0]["type"] = "u7"
        with self.assertRaisesRegex(generate.DictionaryError, "unknown type"):
            generate.validate(d)

    def test_validation_rejects_oversized_packet(self):
        d = copy.deepcopy(self.d)
        d["packets"][0]["fields"].append({"name": "big", "type": "f64", "count": 30})  # 240 more bytes
        with self.assertRaisesRegex(generate.DictionaryError, "maximum is 244"):
            generate.validate(d)

    def test_validation_rejects_bad_pad_name_and_unknown_enum_and_record(self):
        d = copy.deepcopy(self.d)
        d["packets"][2]["fields"][7]["name"] = "spare"
        with self.assertRaisesRegex(generate.DictionaryError, "named 'pad'"):
            generate.validate(d)
        d = copy.deepcopy(self.d)
        d["packets"][0]["fields"][0]["enum"] = "nope"
        with self.assertRaisesRegex(generate.DictionaryError, "unknown enum"):
            generate.validate(d)
        d = copy.deepcopy(self.d)
        d["packets"][3]["fields"][-1]["type"] = "record:nope"
        with self.assertRaisesRegex(generate.DictionaryError, "unknown record"):
            generate.validate(d)

    def test_generation_is_deterministic(self):
        a = generate.generate_all()
        b = generate.generate_all()
        self.assertEqual(a, b)
        for text in a.values():
            self.assertNotIn("\r", text)
            self.assertIn(generate.BANNER, text.split("\n")[0])

    def test_generated_files_are_current(self):
        result = subprocess.run([sys.executable, "-I", os.path.join(generate.ROOT, "tools", "telemetry", "generate.py"),
                                 "--check"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_reference_packer_lengths_and_bytes(self):
        d = self.d
        gnss = {"fixType": 2, "satellitesUsed": 20, "satellitesVisible": 24, "fixTime": 1.5, "latitude": 0.5,
                "longitude": -1.0, "height": 12.0, "velocity": [1, 2, 3], "horizontalSigma": 0.1, "verticalSigma": 0.2}
        data = pack(d, "gnss", gnss)
        self.assertEqual(len(data), 56)
        self.assertEqual(data[:4], bytes([2, 20, 24, 0]))
        self.assertEqual(struct.unpack(">d", data[4:12])[0], 1.5)
        self.assertEqual(struct.unpack(">f", data[48:52])[0], struct.unpack(">f", struct.pack(">f", 0.1))[0])
        status = {"mode": 3, "enginesOn": True, "navValid": False, "translating": True, "aligned": False,
                  "gnssReady": True, "gnssFixCount": 7, "cycleCount": 99, "timeInMode": 1.0, "propellant": 2.0,
                  "mass": 3.0}
        data = pack(d, "fsw_status", status)
        self.assertEqual(len(data), 20)
        self.assertEqual(data[:2], bytes([3, 1 | 4 | 16]))
        engine = {"on": True, "throttle": 0.5, "pitch": 0.1, "yaw": -0.1}
        gc = {k: 0.0 for k in ("altitudeSetpoint", "velocitySetpoint", "eastSetpoint", "northSetpoint",
                               "eastAccelCommand", "northAccelCommand")}
        gc["attitudeError"] = [0, 0, 0]
        gc["engines"] = [engine] * 3
        data = pack(d, "guidance_control", gc)
        self.assertEqual(len(data), 84)
        self.assertEqual(data[36:40], bytes([1, 0, 0, 0]))
        self.assertEqual(data[52:56], bytes([1, 0, 0, 0]))

    def test_packer_matches_generator_offsets(self):
        # Every packet's struct size equals the generator's computed length.
        lay = generate.layout(self.d)
        for p in lay["packets"]:
            fmt = ">"
            for f in p["fields"]:
                if f["type"].startswith("record:"):
                    for _ in range(f.get("count", 1)):
                        for rf in self.d["records"][f["type"][7:]]["fields"]:
                            fmt += self._fmt(rf)
                else:
                    fmt += self._fmt(f)
            self.assertEqual(struct.calcsize(fmt), p["userDataLength"], p["name"])

    @staticmethod
    def _fmt(f):
        if f["type"] == "pad":
            return f"{f['count']}x"
        return str(f.get("count", 1)) + generate.SCALARS[f["type"]][3]


if __name__ == "__main__":
    unittest.main()
