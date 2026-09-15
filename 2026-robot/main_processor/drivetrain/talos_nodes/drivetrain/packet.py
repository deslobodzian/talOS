"""Drivetrain wire codecs. Standard library only (struct/math): import-light so
``--describe`` can use the sizes without touching shared memory, flatbuffers,
or ctypes.

Byte layouts mirror the C++ ground truth exactly.

Topics (mirroring 2026-robot/main_processor/drivetrain/packet.h):
  /drivetrain/target        - ChassisTarget flatbuffers struct (arbiter -> this node)
  /drivetrain/target/teleop - ChassisTarget flatbuffers struct (teleop -> arbiter)
  /drivetrain/target/auto   - ChassisTarget flatbuffers struct (auto -> arbiter)
  /drivetrain/state         - DrivetrainState flatbuffers struct (this node -> consumers)
  /hw/state                 - hardware::Packet envelope (bridge -> this node)
  /hw/request/drivetrain    - hardware::Packet envelope (this node -> bridge)
"""

import math
import struct

NODE_NAME = "drivetrain"
NODE_TARGET = "//2026-robot/main_processor/drivetrain:node"

TARGET_TOPIC = "/drivetrain/target"
TELEOP_TARGET_TOPIC = "/drivetrain/target/teleop"
AUTO_TARGET_TOPIC = "/drivetrain/target/auto"
STATE_TOPIC = "/drivetrain/state"
HW_STATE_TOPIC = "/hw/state"
REQUEST_TOPIC = "/hw/request/drivetrain"

PERIOD_US = 5000
COMMAND_TIMEOUT_US = 100000

CHASSIS_TARGET_SIZE = 40
CHASSIS_TARGET_ALIGN = 8
DRIVETRAIN_STATE_SIZE = 56
DRIVETRAIN_STATE_ALIGN = 8
PACKET_SIZE = 1404
PACKET_ALIGN = 4
MAX_PAYLOAD = 1400

# ChassisTarget wire image: vx_mps f64 @0, vy_mps f64 @8, omega_radps f64 @16,
# issued_ns i64 @24, enabled bool @32, 7 trailing pad bytes (struct size 40, align 8).
_TARGET_STRUCT = struct.Struct("<dddq?7x")
assert _TARGET_STRUCT.size == CHASSIS_TARGET_SIZE

# DrivetrainState wire image: timestamp_ns u64 @0, enabled bool @8, 7 pad bytes @9,
# vx_mps f64 @16, vy_mps f64 @24, omega_radps f64 @32, yaw_rot f64 @40,
# yaw_rate_rps f64 @48 (struct size 56, align 8).
_STATE_STRUCT = struct.Struct("<Q?7xddddd")
assert _STATE_STRUCT.size == DRIVETRAIN_STATE_SIZE


def describe_sources(timer_kind, watcher_kind, sender_kind):
    """The node's --describe source table, mirroring DrivetrainNode constructor order."""
    return [
        (watcher_kind, HW_STATE_TOPIC, PACKET_SIZE, False, False),
        (watcher_kind, TARGET_TOPIC, CHASSIS_TARGET_SIZE, False, False),
        (sender_kind, REQUEST_TOPIC, PACKET_SIZE, False, False),
        (sender_kind, STATE_TOPIC, DRIVETRAIN_STATE_SIZE, False, False),
        (timer_kind, "swerve", 0, False, False),
    ]


def encode_chassis_target(vx_mps, vy_mps, omega_radps, issued_ns, enabled):
    """Pack a ChassisTarget to its 40 payload bytes."""
    for val in (vx_mps, vy_mps, omega_radps):
        if not math.isfinite(val):
            raise ValueError("ChassisTarget velocity components must be finite")
    return _TARGET_STRUCT.pack(float(vx_mps), float(vy_mps), float(omega_radps),
                               int(issued_ns), bool(enabled))


def decode_chassis_target(payload):
    """Unpack 40 ChassisTarget payload bytes -> (vx_mps, vy_mps, omega_radps, issued_ns, enabled)."""
    if len(payload) != CHASSIS_TARGET_SIZE:
        raise ValueError("ChassisTarget payload must be %d bytes, got %d"
                         % (CHASSIS_TARGET_SIZE, len(payload)))
    return _TARGET_STRUCT.unpack(bytes(payload[:CHASSIS_TARGET_SIZE]))


def encode_drivetrain_state(timestamp_ns, enabled, vx_mps, vy_mps, omega_radps,
                            yaw_rot, yaw_rate_rps):
    """Pack a DrivetrainState to its 56 payload bytes."""
    for val in (vx_mps, vy_mps, omega_radps, yaw_rot, yaw_rate_rps):
        if not math.isfinite(val):
            raise ValueError("DrivetrainState float components must be finite")
    return _STATE_STRUCT.pack(int(timestamp_ns), bool(enabled), float(vx_mps),
                               float(vy_mps), float(omega_radps), float(yaw_rot),
                               float(yaw_rate_rps))


def decode_drivetrain_state(payload):
    """Unpack 56 DrivetrainState payload bytes -> (timestamp_ns, enabled, vx, vy, omega, yaw, yaw_rate)."""
    if len(payload) != DRIVETRAIN_STATE_SIZE:
        raise ValueError("DrivetrainState payload must be %d bytes, got %d"
                         % (DRIVETRAIN_STATE_SIZE, len(payload)))
    return _STATE_STRUCT.unpack(bytes(payload[:DRIVETRAIN_STATE_SIZE]))
