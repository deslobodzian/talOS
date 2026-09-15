"""Python side of the cross-language wire test (item 50).

Encodes with the real flatc-generated Python bindings and pins the exact
golden bytes; decodes the goldens back with the same generated code. The
C++ mirror (crosslang_test.cc) decodes these same golden files with the
real flatc-generated C++ headers and re-encodes for a byte-identical
comparison. The two sides share no implementation: the golden file is the
only join, so a schema drift that changes one generator's layout fails
loudly here (generator side) or there (decoder side), never silently.
"""

import json
import os
import unittest
from pathlib import Path

import flatbuffers

from talos.drive import ChassisTarget, DrivetrainState
from talos.drive.DrivetrainState import CreateDrivetrainState

HERE = Path(__file__).parent
DATA = Path(os.environ.get("TEST_SRCDIR", "")) / "_main" / "tools" / "delivery" / "testdata"
if not (DATA / "goldens.json").exists():
    DATA = HERE / "testdata"


def load(name):
    return (DATA / name).read_bytes()


class CrossLangGoldenTest(unittest.TestCase):
    def test_target_encode_matches_golden(self):
        gold = json.loads((DATA / "goldens.json").read_text())["chassis_target"]
        builder = flatbuffers.Builder(64)
        off = ChassisTarget.CreateChassisTarget(
            builder, gold["vx_mps"], gold["vy_mps"], gold["omega_radps"],
            gold["issued_ns"], gold["enabled"],
        )
        builder.Finish(off)
        self.assertEqual(bytes(builder.Output()), load("chassis_target_golden.bin"))

    def test_state_encode_matches_golden(self):
        gold = json.loads((DATA / "goldens.json").read_text())["drivetrain_state"]
        builder = flatbuffers.Builder(80)
        off = CreateDrivetrainState(
            builder, gold["timestamp_ns"], gold["enabled"], gold["vx_mps"],
            gold["vy_mps"], gold["omega_radps"], gold["yaw_rot"],
            gold["yaw_rate_rps"],
        )
        builder.Finish(off)
        self.assertEqual(bytes(builder.Output()), load("drivetrain_state_golden.bin"))

    def test_target_decode_roundtrip(self):
        # Same generated code, decode direction: documents the field values
        # the C++ test must observe on the same bytes.
        wire = load("chassis_target_golden.bin")
        root = int.from_bytes(wire[0:4], "little")
        t = ChassisTarget.ChassisTarget()
        t.Init(wire, root)
        self.assertAlmostEqual(t.VxMps(), 1.5)
        self.assertAlmostEqual(t.VyMps(), -0.5)
        self.assertAlmostEqual(t.OmegaRadps(), 3.141592653589793)
        self.assertEqual(t.IssuedNs(), 123456789)
        self.assertTrue(t.Enabled())

    def test_state_decode_roundtrip(self):
        wire = load("drivetrain_state_golden.bin")
        root = int.from_bytes(wire[0:4], "little")
        s = DrivetrainState.DrivetrainState()
        s.Init(wire, root)
        self.assertEqual(s.TimestampNs(), 987654321)
        self.assertTrue(s.Enabled())
        self.assertAlmostEqual(s.VxMps(), 2.0)
        self.assertAlmostEqual(s.VyMps(), -1.0)
        self.assertAlmostEqual(s.OmegaRadps(), 0.5)
        self.assertAlmostEqual(s.YawRot(), 1.25)
        self.assertAlmostEqual(s.YawRateRps(), -0.2)


if __name__ == "__main__":
    unittest.main()
