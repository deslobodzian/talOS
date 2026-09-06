"""Intake wire codecs. Standard library only (struct/math): import-light so
``main.py --describe`` can use the sizes without touching shared memory,
flatbuffers, or ctypes.

Byte layouts mirror the C++ ground truth exactly; every field offset cites
the C++ line that owns it.

Topics (packet names mirror 2026-robot/main_processor/shooter/packet.h):
  /intake/target   - IntakeTarget flatbuffers struct (arbiter -> this node)
  /intake/state    - IntakeState flatbuffers struct (this node -> consumers)
  /hw/state        - hardware::Packet envelope (bridge -> this node)
  /hw/request/intake - hardware::Packet envelope (this node -> bridge)
"""

import math
import struct

NODE_NAME = "intake"
NODE_TARGET = "//2026-robot/main_processor/intake:node"

TARGET_TOPIC = "/intake/target"
STATE_TOPIC = "/intake/state"
HW_STATE_TOPIC = "/hw/state"
REQUEST_TOPIC = "/hw/request/intake"

PERIOD_US = 20000
COMMAND_TIMEOUT_US = 100000  # talOS/hardware/config.h:122 default.

# RTMS message sizes: sizeof/alignment of the C++ message types, which is what
# event::watch/make_sender registers (talOS/events/handles.h:133-150 passes
# sizeof(Message)/alignof(Message)).
#   IntakeTarget: flatbuffers struct {ulong, bool, float} -> 16 bytes, align 8
#     (intake_message.fbs:3-7; SizeOf 16 in talos/intake/IntakeTarget.py:14).
#   IntakeState: {ulong, bool, float, bool} -> 24 bytes, align 8
#     (intake_message.fbs:9-14; SizeOf 24 in talos/intake/IntakeState.py:13).
#   hardware::Packet: {uint32 size, uint8 data[1400]} -> 1404 bytes, align 4
#     (talOS/hardware/packet.h:14-21; kMaxPayloadSize 1400 in
#     talOS/protocol/frame.h:16).
TARGET_SIZE = 16
TARGET_ALIGN = 8
STATE_SIZE = 24
STATE_ALIGN = 8
PACKET_SIZE = 1404
PACKET_ALIGN = 4
MAX_PAYLOAD = 1400

# hardware::Mode values (talOS/hardware/messages.h:11-18).
MODE_NEUTRAL = 0
MODE_DUTY_CYCLE = 1
MODE_VOLTAGE = 2
MODE_VELOCITY = 3
MODE_POSITION = 4
MODE_MOTION_MAGIC = 5

# hardware::StateFlags bits (talOS/hardware/messages.h:19-24).
FLAG_CONFIGURED = 1
FLAG_ENABLED = 2
FLAG_COMMAND_ACTIVE = 4
FLAG_HARDWARE_FAULT = 8

# IntakeTarget wire image: issued_ns u64 @0 (IntakeTarget.py:21), enabled
# bool @8 (IntakeTarget.py:23), 3 pad bytes (flatbuffers bool->float32
# alignment; see CreateIntakeTarget Prep/Pad in IntakeTarget.py:27-33),
# roller_velocity_rps float32 @12 (IntakeTarget.py:25).
_TARGET_STRUCT = struct.Struct("<Q?3xf")
assert _TARGET_STRUCT.size == TARGET_SIZE

# IntakeState wire image: issued_ns u64 @0 (IntakeState.py:21), enabled bool
# @8 (IntakeState.py:23), 3 pad, roller_velocity_rps float32 @12
# (IntakeState.py:25), beam_broken bool @16 (IntakeState.py:27), 7 trailing
# pad (struct size 24, align 8; CreateIntakeState in IntakeState.py:29-37).
_STATE_STRUCT = struct.Struct("<Q?3xf?7x")
assert _STATE_STRUCT.size == STATE_SIZE


def encode_target(issued_ns, enabled, roller_velocity_rps):
    """Pack an IntakeTarget to its 16 payload bytes."""
    return _TARGET_STRUCT.pack(issued_ns, bool(enabled),
                               float(roller_velocity_rps))


def decode_target(payload):
    """Unpack 16 IntakeTarget payload bytes -> (issued_ns, enabled, vel_rps)."""
    if len(payload) != TARGET_SIZE:
        raise ValueError("IntakeTarget payload must be %d bytes, got %d"
                         % (TARGET_SIZE, len(payload)))
    issued_ns, enabled, vel = _TARGET_STRUCT.unpack(bytes(payload[:TARGET_SIZE]))
    return (issued_ns, bool(enabled), float(vel))


def encode_state(issued_ns, enabled, roller_velocity_rps, beam_broken):
    """Pack an IntakeState to its 24 payload bytes."""
    return _STATE_STRUCT.pack(issued_ns, bool(enabled),
                              float(roller_velocity_rps), bool(beam_broken))


def decode_state(payload):
    """Unpack 24 IntakeState payload bytes -> (issued_ns, enabled, vel, beam)."""
    if len(payload) != STATE_SIZE:
        raise ValueError("IntakeState payload must be %d bytes, got %d"
                         % (STATE_SIZE, len(payload)))
    issued_ns, enabled, vel, beam = _STATE_STRUCT.unpack(
        bytes(payload[:STATE_SIZE]))
    return (issued_ns, bool(enabled), float(vel), bool(beam))


def encode_command(config_id, boot_id, epoch, observed_time_us, motor_id,
                   mode, slot, demand, feedforward_v=0.0):
    """Encode one-motor hardware::Command inside a hardware::Packet envelope.

    Returns the full 1404-byte RTMS payload (Packet envelope: size u32 @0 per
    talOS/hardware/packet.h:15, wire bytes @4, zero-filled tail so outputs are
    byte-stable per packet.h:12-13).

    Command wire offsets mirror talOS/hardware/messages.cc Encode(Command)
    (lines 51-83): config_id u64 @0 (:57), boot_id u64 @8 (:58), epoch u64
    @16 (:59), observed_time_us u64 @24 (:60), count u16 @32 (:61),
    digital_output_count u16 @34 (:62), pwm_output_count u16 @36 (:63), then
    motors[0] @38: id u16 @+0 (:66), mode u8 @+2 (:67), slot u8 @+3 (:68),
    demand f64 @+4 (:69), feedforward_v f64 @+12 (:70); stride 20, so one
    motor ends @58. Envelope offsets below are +4 for the size prefix.
    """
    if mode < MODE_NEUTRAL or mode > MODE_MOTION_MAGIC:
        raise ValueError("bad motor mode %r" % (mode,))
    if slot < 0 or slot > 2:  # Decode rejects slot > 2 (messages.cc:105).
        raise ValueError("bad motor slot %r" % (slot,))
    for name, value in (("demand", demand), ("feedforward_v", feedforward_v)):
        if not math.isfinite(value):
            raise ValueError("%s must be finite" % name)
    wire = struct.pack("<QQQQHHH", config_id, boot_id, epoch,
                       observed_time_us, 1, 0, 0)
    wire += struct.pack("<HBBdd", motor_id, mode, slot, float(demand),
                        float(feedforward_v))
    if len(wire) > MAX_PAYLOAD:
        raise ValueError("command does not fit one frame")
    envelope = struct.pack("<I", len(wire)) + wire
    return envelope + bytes(PACKET_SIZE - len(envelope))


def decode_hw_state_minimal(payload, roller_id, beam_id):
    """Decode the minimal /hw/state fields the intake policy needs.

    Walks the State encoding from talOS/hardware/messages.cc Encode(State)
    (lines 123-199) but extracts only: header words, the roller MotorSample
    (velocity_rps + valid), and the beam DigitalInputSample (value + valid).

    Header offsets (messages.cc:132-144, envelope +4 for the Packet size u32
    at packet.h:15): size u32 @0; config_id u64 @4 (:132); boot_id u64 @12
    (:133); epoch u64 @20 (:134); sample_time_us u64 @28 (:135);
    last_command_sequence u64 @36; flags u32 @44 (:137); motor_count u16 @48
    (:138); sensor_count u16 @50 (:139); digital_input_count u16 @52 (:140);
    digital_output_count u16 @54 (:141); analog_input_count u16 @56;
    encoder_count u16 @58; pwm_output_count u16 @60; arrays start @62.

    MotorSample stride 43 (messages.cc:145-155): id u16 @+0 (:147), valid u8
    @+2 (:148), position f64 @+3 (:149), velocity_rps f64 @+11 (:150),
    voltage f64 @+19 (:151), stator_current f64 @+27 (:152), pos_age u32 @+35
    (:153), vel_age u32 @+39 (:154).
    SensorSample stride 27 (messages.cc:156-163): id u16 @+0, valid u8 @+2,
    position f64 @+3, velocity f64 @+11, pos_age u32 @+19, vel_age u32 @+23.
    DigitalInputSample stride 4 (messages.cc:165-170): id u16 @+0 (:167),
    valid u8 @+2 (:168), value u8 @+3 (:169).

    Returns a dict, or None when the bytes are malformed/truncated.
    """
    try:
        buf = bytes(payload)
        if len(buf) < 62:
            return None
        (size,) = struct.unpack_from("<I", buf, 0)  # packet.h:15.
        if size > MAX_PAYLOAD or 4 + size > len(buf):
            return None
        end = 4 + size
        (config_id, boot_id, epoch, sample_time_us,
         _last_seq) = struct.unpack_from("<QQQQQ", buf, 4)  # :132-136.
        (flags,) = struct.unpack_from("<I", buf, 44)  # :137.
        (motor_count, sensor_count, di_count, do_count, ai_count, enc_count,
         pwm_count) = struct.unpack_from("<HHHHHHH", buf, 48)  # :138-144.
        pos = 62
        roller_valid = False
        roller_vel = 0.0
        for _ in range(motor_count):  # :145-155, stride 43.
            if pos + 43 > end:
                return None
            (mid,) = struct.unpack_from("<H", buf, pos)  # :147.
            (valid,) = struct.unpack_from("<B", buf, pos + 2)  # :148.
            if mid == roller_id:
                (roller_vel,) = struct.unpack_from("<d", buf,
                                                   pos + 11)  # :150.
                roller_valid = (valid == 1)
                if not math.isfinite(roller_vel):
                    roller_valid = False
            pos += 43
        pos += sensor_count * 27  # :156-163, stride 27.
        if pos > end:
            return None
        beam_valid = False
        beam_broken = False
        for _ in range(di_count):  # :165-170, stride 4.
            if pos + 4 > end:
                return None
            (did,) = struct.unpack_from("<H", buf, pos)  # :167.
            (valid,) = struct.unpack_from("<B", buf, pos + 2)  # :168.
            (value,) = struct.unpack_from("<B", buf, pos + 3)  # :169.
            if valid > 1 or value > 1:  # :250-255 reject these.
                return None
            if did == beam_id:
                beam_valid = (valid == 1)
                beam_broken = (value == 1)
            pos += 4
        # Remaining arrays (digital/analog/encoder/pwm samples) carry nothing
        # the intake policy reads, so they are range-checked, not parsed.
        if pos + do_count * 4 + ai_count * 15 + enc_count * 23 + pwm_count * 11 > end:
            return None
        return {
            "config_id": config_id,
            "boot_id": boot_id,
            "epoch": epoch,
            "sample_time_us": sample_time_us,
            "flags": flags,
            "roller_valid": roller_valid,
            "roller_velocity_rps": roller_vel,
            "beam_valid": beam_valid,
            "beam_broken": beam_broken,
        }
    except (struct.error, IndexError):
        return None
