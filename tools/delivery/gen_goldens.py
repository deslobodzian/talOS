#!/usr/bin/env python3
"""Regenerate cross-language golden packets from the real flatc output.

The goldens join two independent implementations: the flatc-generated
Python bindings encode them here, and tools/delivery/crosslang_test.cc
decodes them with the flatc-generated C++ headers (and re-encodes for a
byte-identical comparison). Neither side mirrors the other; the golden
file is the only shared artifact, with provenance in goldens.json.

Run from the repository root:
    python3 tools/delivery/gen_goldens.py
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT / "2026-robot/main_processor/drivetrain"))

import flatbuffers  # noqa: E402

from talos.drive import ChassisTarget, DrivetrainState  # noqa: E402
from talos.drive.DrivetrainState import (  # noqa: E402
    CreateDrivetrainState,
)

# Fixed probe values. Chosen for wire visibility, not realism: distinct
# fractional doubles, a large issued_ns, and enabled=True so the trailing
# bool byte is nonzero.
TARGET_VALUES = {
    "vx_mps": 1.5,
    "vy_mps": -0.5,
    "omega_radps": 3.141592653589793,
    "issued_ns": 123456789,
    "enabled": True,
}
STATE_VALUES = {
    "timestamp_ns": 987654321,
    "enabled": True,
    "vx_mps": 2.0,
    "vy_mps": -1.0,
    "omega_radps": 0.5,
    "yaw_rot": 1.25,
    "yaw_rate_rps": -0.2,
}


def encode_target(v):
    builder = flatbuffers.Builder(64)
    off = ChassisTarget.CreateChassisTarget(
        builder, v["vx_mps"], v["vy_mps"], v["omega_radps"],
        v["issued_ns"], v["enabled"],
    )
    builder.Finish(off)
    return bytes(builder.Output())


def encode_state(v):
    builder = flatbuffers.Builder(80)
    off = CreateDrivetrainState(
        builder, v["timestamp_ns"], v["enabled"], v["vx_mps"],
        v["vy_mps"], v["omega_radps"], v["yaw_rot"], v["yaw_rate_rps"],
    )
    builder.Finish(off)
    return bytes(builder.Output())


def main():
    out = ROOT / "tools/delivery/testdata"
    out.mkdir(parents=True, exist_ok=True)
    target_wire = encode_target(TARGET_VALUES)
    state_wire = encode_state(STATE_VALUES)
    (out / "chassis_target_golden.bin").write_bytes(target_wire)
    (out / "drivetrain_state_golden.bin").write_bytes(state_wire)
    provenance = {
        "generator": "tools/delivery/gen_goldens.py",
        "bindings": "2026-robot/main_processor/drivetrain/talos/drive/*.py (flatc output, checked in)",
        "schema": "talos-chassis_contracts/drive_message.fbs",
        "chassis_target": TARGET_VALUES,
        "drivetrain_state": STATE_VALUES,
    }
    (out / "goldens.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(f"wrote {len(target_wire)}B chassis_target + {len(state_wire)}B drivetrain_state goldens")


if __name__ == "__main__":
    main()
