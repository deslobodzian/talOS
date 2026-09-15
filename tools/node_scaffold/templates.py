"""Templates for generating standalone talOS nodes."""

from typing import Dict


def class_name(name: str) -> str:
    """Convert snake_case node name to CamelCase class name."""
    return "".join(part.capitalize() for part in name.split("_"))


def render_bazelrc() -> str:
    return """common --enable_bzlmod

build --enable_platform_specific_config

build:linux --cxxopt=-std=c++23 --cxxopt=-O3
build:macos --cxxopt=-std=c++23 --cxxopt=-O3
build:macos --cxxopt=-fexperimental-library --linkopt=-fexperimental-library
build:macos --macos_minimum_os=13.4
build:windows --cxxopt=/std:c++23 --cxxopt=-O3
"""


def render_module_bazel(name: str, lang: str, hardware: bool, sdk_path: str) -> str:
    lines = [
        f'module(name = "talos_{name}", version = "0.1.0")',
        "",
        'bazel_dep(name = "talos", version = "0.1.0")',
        "local_path_override(",
        '    module_name = "talos",',
        f'    path = "{sdk_path}",',
        ")",
        "",
        'bazel_dep(name = "apple_support", version = "2.0.0", repo_name = "build_bazel_apple_support")',
        'bazel_dep(name = "platforms", version = "1.1.0")',
        'bazel_dep(name = "rules_cc", version = "0.2.18")',
        'bazel_dep(name = "flatbuffers", version = "25.12.19")',
        'bazel_dep(name = "rules_foreign_cc", version = "0.14.0")',
        'bazel_dep(name = "aspect_bazel_lib", version = "2.22.5")',
        '# Floor aspect_rules_esbuild above the Bazel-8-incompatible 0.21.0',
        '# that flatbuffers pulls transitively (its toolchain .bzl uses the',
        '# removed incompatible_use_toolchain_transition kwarg, which breaks',
        '# analysis of any flatc target). 0.25.1 is the newest floor that',
        '# keeps aspect_rules_js 2.x: 0.26.0+ requires rules_js 3.x, whose',
        '# npm tag dropped the root_package attribute flatbuffers still',
        '# passes (see root MODULE.bazel).',
        'bazel_dep(name = "aspect_rules_esbuild", version = "0.25.1")',
        'bazel_dep(name = "rules_go", version = "0.63.0", repo_name = "io_bazel_rules_go")',
    ]

    if lang == "cpp":
        lines.append('bazel_dep(name = "googletest", version = "1.17.0.bcr.2")')
        if hardware:
            lines.append('bazel_dep(name = "eigen", version = "3.4.0.bcr.3")')
    elif lang == "python":
        lines.append('bazel_dep(name = "rules_python", version = "2.0.1")')
        lines.extend([
            "",
            'python = use_extension("@rules_python//python/extensions:python.bzl", "python")',
            "python.toolchain(",
            '    python_version = "3.11",',
            "    is_default = True,",
            ")",
            'use_repo(python, "python_3_11")',
        ])

    lines.append("")
    return "\n".join(lines)


def render_build(name: str, lang: str, hardware: bool) -> str:
    cls = class_name(name)
    if lang == "cpp":
        extra_deps = []
        if hardware:
            extra_deps.append('        "@eigen//:eigen",\n')
            extra_deps.append('        "@talos//talOS/process:hardware_main",\n')
        else:
            extra_deps.append('        "@talos//talOS/process:node_main",\n')
        extra_deps_str = "".join(extra_deps)
        hw_bool = "True" if hardware else "False"
        return f"""load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@talos//build:defs.bzl", "talos_cc_node", "talos_contract_cc")

exports_files(["subsystem.toml"])

talos_contract_cc(
    name = "contract",
    node_name = "{name}",
    fbs_srcs = ["{name}_message.fbs"],
    packet_hdr = "packet.h",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "{name}",
    hdrs = ["node.h"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/{name}",
    visibility = ["//visibility:public"],
    deps = [
        ":contract",
        "@talos//talOS/configuration:configuration",
        "@talos//talOS/events:events",
        "@talos//talOS/hardware:gate",
        "@talos//talOS/hardware:hardware",
{extra_deps_str}    ],
)

talos_cc_node(
    name = "node",
    node_name = "{name}",
    hdrs = ["node.h"],
    factory = "talos::{name}::from_context",
    contract = ":contract",
    hardware = {hw_bool},
    visibility = ["//visibility:public"],
    deps = [":{name}"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = [
        "testdata/robot.toml",
        "testdata/{name}.toml",
    ],
    env = {{
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
    }},
    deps = [
        ":{name}",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
        "@talos//talOS/configuration:configuration",
        "@talos//talOS/events:events",
    ],
)
"""
    else:
        return f"""load("@rules_python//python:defs.bzl", "py_library", "py_test")
load("@talos//build:defs.bzl", "flatbuffer_py_library", "talos_py_node")

exports_files(["subsystem.toml"])

flatbuffer_py_library(
    name = "messages_py",
    srcs = ["{name}_message.fbs"],
    outs = [
        "talos/{name}/__init__.py",
        "talos/{name}/{cls}Target.py",
        "talos/{name}/{cls}State.py",
    ],
    visibility = ["//visibility:public"],
)

py_library(
    name = "{name}",
    srcs = [
        "node.py",
        "packet.py",
    ],
    imports = ["."],
    visibility = ["//visibility:public"],
    deps = [
        ":messages_py",
        "@talos//talOS/ipc/python:rtms_py",
    ],
)

talos_py_node(
    name = "node",
    node_name = "{name}",
    entry_point = "node:{cls}Node",
    contract = ":messages_py",
    describe_sources = "node:describe_source_rows",
    default_config = "subsystem.toml",
    visibility = ["//visibility:public"],
    deps = [":{name}"],
)

py_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.py"],
    data = [
        "testdata/robot.toml",
        "testdata/{name}.toml",
    ],
    env = {{
        "TALOS_TEST_CONFIG": "testdata/robot.toml",
    }},
    deps = [
        ":{name}",
        "@rules_python//python/runfiles",
    ],
)
"""


def render_message_fbs(name: str) -> str:
    cls = class_name(name)
    return f"""namespace talos.{name};

struct {cls}Target {{
  issued_ns:ulong;
  target_velocity_rps:double;
  enabled:bool;
}}

struct {cls}State {{
  timestamp_ns:ulong;
  velocity_rps:double;
  target_velocity_rps:double;
  at_target:bool;
  enabled:bool;
}}
"""


def render_packet_h(name: str) -> str:
    return f"""#pragma once

#include <cstdint>
#include <string>

#include "talOS/hardware/packet.h"

namespace talos::{name} {{

using Packet = hardware::Packet;

inline constexpr const char* kHwStateTopic = hardware::kStateTopic;
inline constexpr const char* kStateTopic = "/{name}/state";
inline constexpr const char* kTargetTopic = "/{name}/target";
inline constexpr const char* kTeleopTargetTopic = "/{name}/target/teleop";
inline constexpr const char* kAutoTargetTopic = "/{name}/target/auto";

inline const std::string kRequestTopic = hardware::RequestTopic("{name}");

inline constexpr uint32_t kPeriodUs = 5000;
inline constexpr uint32_t kCommandTimeoutUs = 100000;

}}  // namespace talos::{name}
"""


def render_packet_py(name: str, hardware: bool) -> str:
    cls = class_name(name)
    hw_bool = "True" if hardware else "False"
    return f'''"""Packet and topic definitions for {name}."""

import math
import struct
from talos_runtime import node_api

NAME = "{name}"
PERIOD_US = 5000
COMMAND_TIMEOUT_US = 100000

HW_STATE_TOPIC = "/hw/state"
STATE_TOPIC = f"/{{NAME}}/state"
TARGET_TOPIC = f"/{{NAME}}/target"
REQUEST_TOPIC = f"/hw/request/{{NAME}}"

# RTMS message sizes: sizeof/alignment of the message types, which is what
# event::watch/make_sender registers (talOS/events/handles.h passes
# sizeof(Message)/alignof(Message)).
#   {cls}Target: flatbuffers struct {{ulong, double, bool}} -> 24 bytes, align 8
#     ({name}_message.fbs; fields in declaration order, trailing pad to
#     a multiple of the max field alignment 8).
#   {cls}State: flatbuffers struct {{ulong, double, double, bool, bool}}
#     -> 32 bytes, align 8 (26 payload bytes + 6 trailing pad).
#   hardware::Packet: {{uint32 size, uint8 data[1400]}} -> 1404 bytes, align 4
#     (talOS/hardware/packet.h:14-21; kMaxPayloadSize 1400 in
#     talOS/protocol/frame.h:16).
TARGET_SIZE = 24
TARGET_ALIGN = 8
STATE_SIZE = 32
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


def describe_sources(hardware={hw_bool}):
    if hardware:
        return [
            (node_api.TIMER, NAME, 0, False, False),
            (node_api.WATCHER, HW_STATE_TOPIC, PACKET_SIZE, False, False),
            (node_api.WATCHER, TARGET_TOPIC, TARGET_SIZE, False, False),
            (node_api.SENDER, REQUEST_TOPIC, PACKET_SIZE, False, False),
            (node_api.SENDER, STATE_TOPIC, STATE_SIZE, False, False),
        ]
    return [
        (node_api.TIMER, NAME, 0, False, False),
        (node_api.WATCHER, TARGET_TOPIC, TARGET_SIZE, False, False),
        (node_api.SENDER, STATE_TOPIC, STATE_SIZE, False, False),
    ]


# {cls}Target wire image: issued_ns u64 @0, target_velocity_rps float64 @8,
# enabled bool @16, 7 pad bytes (flatbuffers struct alignment; max field
# align 8, size rounded to 24).
_TARGET_STRUCT = struct.Struct("<Qd?7x")
assert _TARGET_STRUCT.size == TARGET_SIZE

# {cls}State wire image: timestamp_ns u64 @0, velocity_rps float64 @8,
# target_velocity_rps float64 @16, at_target bool @24, enabled bool @25,
# 6 trailing pad bytes (size 32, align 8; field order is the .fbs order).
_STATE_STRUCT = struct.Struct("<Qdd??6x")
assert _STATE_STRUCT.size == STATE_SIZE


def encode_target(issued_ns: int, target_vel: float, enabled: bool) -> bytes:
    """Pack a {cls}Target to its 24 payload bytes."""
    return _TARGET_STRUCT.pack(int(issued_ns), float(target_vel), bool(enabled))


def decode_target(payload: bytes):
    """Unpack 24 {cls}Target payload bytes -> (issued_ns, vel_rps, enabled)."""
    if len(payload) != TARGET_SIZE:
        raise ValueError(f"Expected {{TARGET_SIZE}} bytes, got {{len(payload)}}")
    issued_ns, target_vel, enabled = _TARGET_STRUCT.unpack(bytes(payload[:TARGET_SIZE]))
    return (issued_ns, float(target_vel), bool(enabled))


def encode_state(timestamp_ns: int, vel: float, target_vel: float, at_target: bool, enabled: bool) -> bytes:
    """Pack a {cls}State to its 32 payload bytes."""
    return _STATE_STRUCT.pack(
        int(timestamp_ns), float(vel), float(target_vel),
        bool(at_target), bool(enabled))


def decode_state(payload: bytes):
    """Unpack 32 {cls}State payload bytes."""
    if len(payload) != STATE_SIZE:
        raise ValueError(f"Expected {{STATE_SIZE}} bytes, got {{len(payload)}}")
    timestamp_ns, vel, target_vel, at_target, enabled = _STATE_STRUCT.unpack(
        bytes(payload[:STATE_SIZE]))
    return (timestamp_ns, float(vel), float(target_vel), bool(at_target), bool(enabled))


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
        raise ValueError(f"bad motor mode {{mode!r}}")
    if slot < 0 or slot > 2:  # Decode rejects slot > 2 (messages.cc:105).
        raise ValueError(f"bad motor slot {{slot!r}}")
    for _name, _value in (("demand", demand), ("feedforward_v", feedforward_v)):
        if not math.isfinite(_value):
            raise ValueError(f"{{_name}} must be finite")
    wire = struct.pack("<QQQQHHH", int(config_id), int(boot_id), int(epoch),
                       int(observed_time_us), 1, 0, 0)
    wire += struct.pack("<HBBdd", int(motor_id), int(mode), int(slot),
                        float(demand), float(feedforward_v))
    if len(wire) > MAX_PAYLOAD:
        raise ValueError("command does not fit one frame")
    envelope = struct.pack("<I", len(wire)) + wire
    return envelope + bytes(PACKET_SIZE - len(envelope))


def decode_command(payload):
    """Decode a hardware::Packet envelope holding a hardware::Command.

    Mirrors talOS/hardware/messages.cc Decode(Command) (lines 84-122):
    range-checks counts against config.h caps, mode/slot ranges, finite
    doubles, and exact wire length (Done at :119). Returns a dict with the
    header words and the motor list. Raises ValueError on malformed bytes
    (the C++ decoder returns false there).
    """
    buf = bytes(payload)
    if len(buf) != PACKET_SIZE:
        raise ValueError(f"Expected {{PACKET_SIZE}} bytes, got {{len(buf)}}")
    (size,) = struct.unpack_from("<I", buf, 0)  # packet.h:15.
    if size > MAX_PAYLOAD or 4 + size > len(buf):
        raise ValueError(f"bad Packet size {{size}}")
    end = 4 + size
    (config_id, boot_id, epoch, observed_time_us,
     count, do_count, pwm_count) = struct.unpack_from("<QQQQHHH", buf, 4)
    if count > 16 or do_count > 4 or pwm_count > 4:  # config.h caps.
        raise ValueError("command count exceeds device caps")
    pos = 4 + 38
    motors = []
    for _ in range(count):  # messages.cc:98-106, stride 20.
        if pos + 20 > end:
            raise ValueError("truncated motor request")
        (mid, mode, slot) = struct.unpack_from("<HBB", buf, pos)
        (demand,) = struct.unpack_from("<d", buf, pos + 4)
        (feedforward_v,) = struct.unpack_from("<d", buf, pos + 12)
        if mode > MODE_MOTION_MAGIC or slot > 2:
            raise ValueError("bad motor mode/slot")
        if not math.isfinite(demand) or not math.isfinite(feedforward_v):
            raise ValueError("non-finite motor demand")
        motors.append({{"id": mid, "mode": mode, "slot": slot,
                        "demand": float(demand),
                        "feedforward_v": float(feedforward_v)}})
        pos += 20
    for _ in range(do_count):  # messages.cc:107-113, stride 3.
        if pos + 3 > end:
            raise ValueError("truncated digital output request")
        (_did,) = struct.unpack_from("<H", buf, pos)
        (val,) = struct.unpack_from("<B", buf, pos + 2)
        if val > 1:
            raise ValueError("bad digital output value")
        pos += 3
    for _ in range(pwm_count):  # messages.cc:114-118, stride 10.
        if pos + 10 > end:
            raise ValueError("truncated pwm request")
        (pout,) = struct.unpack_from("<d", buf, pos + 2)
        if not math.isfinite(pout):
            raise ValueError("non-finite pwm output")
        pos += 10
    if pos != end:
        raise ValueError("trailing bytes after command")
    return {{"config_id": config_id, "boot_id": boot_id, "epoch": epoch,
             "observed_time_us": observed_time_us, "motors": motors}}


def encode_hw_state(config_id, boot_id, epoch, sample_time_us, motor_id,
                    velocity_rps, valid=True, flags=None,
                    position_rot=0.0, voltage=0.0, stator_current_a=0.0):
    """Encode a minimal hardware::State (one MotorSample) in a Packet envelope.

    Returns the full 1404-byte RTMS payload. State wire offsets mirror
    talOS/hardware/messages.cc Encode(State) (lines 123-199): config_id u64
    @0 (:132), boot_id u64 @8 (:133), epoch u64 @16 (:134), sample_time_us
    u64 @24 (:135), last_command_sequence u64 @32 (:136), flags u32 @40
    (:137), motor_count u16 @44 (:138), then 6 more u16 counts @46-58
    (:139-144); motors[0] @58 with stride 43 (:145-155). Envelope offsets
    are +4 for the Packet size prefix. Other sample arrays are empty.
    """
    if flags is None:
        flags = FLAG_CONFIGURED | FLAG_ENABLED
    if not math.isfinite(float(velocity_rps)):
        raise ValueError("velocity_rps must be finite")
    for _name, _value in (("position_rot", position_rot), ("voltage", voltage),
                          ("stator_current_a", stator_current_a)):
        if not math.isfinite(float(_value)):
            raise ValueError(f"{{_name}} must be finite")
    header = struct.pack("<QQQQQIHHHHHHH",
                         int(config_id), int(boot_id), int(epoch),
                         int(sample_time_us), 0, int(flags), 1, 0, 0, 0, 0, 0, 0)
    motor = struct.pack("<HBddddII", int(motor_id), 1 if valid else 0,
                        float(position_rot), float(velocity_rps), float(voltage),
                        float(stator_current_a), 0, 0)
    wire = header + motor
    if len(wire) > MAX_PAYLOAD:
        raise ValueError("state does not fit one frame")
    envelope = struct.pack("<I", len(wire)) + wire
    return envelope + bytes(PACKET_SIZE - len(envelope))


def decode_hw_state_minimal(payload, motor_id):
    """Decode the minimal /hw/state fields this node needs.

    Walks the State encoding from talOS/hardware/messages.cc Encode(State)
    (lines 123-199) but extracts only: header words and the one MotorSample
    (velocity_rps + valid).

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

    Returns a dict, or None when the bytes are malformed/truncated.
    """
    try:
        buf = bytes(payload)
        if len(buf) != PACKET_SIZE:
            return None
        (size,) = struct.unpack_from("<I", buf, 0)  # packet.h:15.
        if size > MAX_PAYLOAD or 4 + size > len(buf):
            return None
        end = 4 + size
        # Trailing envelope bytes are zero-filled (packet.h:12-13); the C++
        # decoder only reads packet.bytes() (the first 4+size bytes), so any
        # tail content is ignored here exactly as there.
        (config_id, boot_id, epoch, sample_time_us,
         _last_seq) = struct.unpack_from("<QQQQQ", buf, 4)  # :132-136.
        (flags,) = struct.unpack_from("<I", buf, 44)  # :137.
        (motor_count, sensor_count, di_count, do_count, ai_count, enc_count,
         pwm_count) = struct.unpack_from("<HHHHHHH", buf, 48)  # :138-144.
        if motor_count > 16 or sensor_count > 12 or di_count > 8:
            return None
        if do_count > 4 or ai_count > 4 or enc_count > 4 or pwm_count > 4:
            return None
        pos = 62
        motor_valid = False
        motor_vel = 0.0
        for _ in range(motor_count):  # :145-155, stride 43.
            if pos + 43 > end:
                return None
            (mid,) = struct.unpack_from("<H", buf, pos)  # :147.
            (valid,) = struct.unpack_from("<B", buf, pos + 2)  # :148.
            if valid > 1:
                return None
            if mid == int(motor_id):
                (motor_vel,) = struct.unpack_from("<d", buf,
                                                  pos + 11)  # :150.
                if not math.isfinite(motor_vel):
                    return None
                motor_valid = (valid == 1)
            else:
                (other_vel,) = struct.unpack_from("<d", buf, pos + 11)
                if not math.isfinite(other_vel):
                    return None
            pos += 43
        # Remaining arrays carry nothing this node reads; range-check them
        # against the wire end (strides from messages.cc:156-197).
        if pos + sensor_count * 27 + di_count * 4 + do_count * 4 > end:
            return None
        pos += sensor_count * 27 + di_count * 4 + do_count * 4
        if pos + ai_count * 15 + enc_count * 23 + pwm_count * 11 > end:
            return None
        pos += ai_count * 15 + enc_count * 23 + pwm_count * 11
        if pos != end:
            return None
        return {{
            "config_id": config_id,
            "boot_id": boot_id,
            "epoch": epoch,
            "sample_time_us": sample_time_us,
            "flags": flags,
            "motor_valid": motor_valid,
            "motor_velocity_rps": float(motor_vel),
        }}
    except (struct.error, IndexError):
        return None
'''


def render_node_h(name: str, hardware: bool) -> str:
    cls = class_name(name)
    if hardware:
        return f"""#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <span>
#include <utility>

#include "talOS/configuration/config_parser.h"
#include "talOS/events/handles.h"
#include "talOS/hardware/gate.h"
#include "talOS/hardware/messages.h"
#include "talOS/process/hardware_main.h"
#include "talos_nodes/{name}/packet.h"
#include "talos_nodes/{name}/{name}_message_generated.h"

namespace talos::{name} {{

template <typename Loop>
class {cls}Node {{
 public:
  struct NodeContext {{
    Loop& loop;
    hardware::Config hardware_config{{}};
    hardware::Devices devices{{}};
    bool simulation{{false}};
  }};

  static {cls}Node from_context(const NodeContext& ctx) {{
    return {cls}Node(ctx.loop, ctx.hardware_config, ctx.devices, ctx.simulation);
  }}

  static {cls}Node from_context(Loop& loop, const talos::process::HardwareSubsystem& sub, bool simulation) {{
    return from_context(NodeContext{{
        .loop = loop,
        .hardware_config = sub.robot.hardware,
        .devices = sub.devices,
        .simulation = simulation,
    }});
  }}

  explicit {cls}Node(Loop& loop, hardware::Config config,
                     hardware::Devices devices = {{}}, bool simulation = false)
      : config_(std::move(config)),
        devices_(std::move(devices)),
        simulation_(simulation),
        config_id_(hardware::ConfigurationId(config_)) {{
    if (!devices_.motors.empty()) {{
      leader_id_ = devices_.motors[0];
    }} else if (!config_.motors.empty()) {{
      leader_id_ = config_.motors[0].id;
    }}

    event::watch<Packet, &{cls}Node::OnHwState>(loop, kHwStateTopic, this);
    event::watch<{cls}Target, &{cls}Node::OnTarget>(loop, kTargetTopic, this);
    request_ = event::make_sender<Packet>(loop, kRequestTopic);
    state_sender_ = event::make_sender<{cls}State>(loop, kStateTopic);
    timer_ = event::make_timer<&{cls}Node::Tick>(loop, "{name}", this);
  }}

  void Start(event::MonotonicTime first) {{
    timer_.setup_periodic(first, std::chrono::microseconds{{config_.period_us > 0 ? config_.period_us : kPeriodUs}});
  }}

  uint16_t leader_id() const {{ return leader_id_; }}
  const {cls}State& latest_state() const {{ return latest_state_; }}
  uint64_t ticks() const {{ return ticks_; }}

 private:
  void OnHwState(const event::Context& context, const Packet& packet) {{
    hardware::State state;
    if (!hardware::Decode(packet.bytes(), state)) return;
    if (!hardware::AcceptState(have_state_, state_, config_id_, state)) return;
    state_ = state;
    state_received_ = context.now;
    have_state_ = true;
  }}

  void OnTarget(const event::Context&, const {cls}Target& target) {{
    target_ = target;
  }}

  void Tick(const event::Context& context) {{
    ++ticks_;
    double leader_velocity_rps = 0.0;
    bool leader_sample_valid = false;
    if (const hardware::MotorSample* sample = hardware::FindMotor(state_, leader_id_)) {{
      leader_sample_valid = sample->valid;
      leader_velocity_rps = sample->velocity_rps;
    }}

    const auto timeout = std::chrono::microseconds{{
        config_.command_timeout_us > 0 ? config_.command_timeout_us : kCommandTimeoutUs}};
    const auto issued = event::MonotonicTime::from_nanos(target_.issued_ns());
    const bool gateway_ok = hardware::GatewayReady(state_);
    const bool target_fresh = hardware::TargetFresh(
        context.now, issued, target_.enabled(), state_received_, timeout);
    const bool valid = (gateway_ok && target_fresh && leader_sample_valid) || simulation_;

    hardware::Command command = hardware::NeutralCommand(
        state_, std::span<const uint16_t>(&leader_id_, 1));
    if (!command.motors.empty()) {{
      command.motors[0].slot = 0;
      if (valid && target_.enabled()) {{
        command.motors[0].mode = hardware::Mode::kVelocity;
        command.motors[0].demand = target_.target_velocity_rps();
        command.motors[0].feedforward_v = 0.0;
      }} else {{
        command.motors[0].mode = hardware::Mode::kNeutral;
        command.motors[0].demand = 0.0;
        command.motors[0].feedforward_v = 0.0;
      }}
      Packet packet{{}};
      packet.size = static_cast<uint32_t>(hardware::Encode(command, packet.data));
      if (packet.size > 0) {{
        request_.send(packet);
      }}
    }}

    const double target_vel = target_.enabled() ? target_.target_velocity_rps() : 0.0;
    constexpr double kAtTargetToleranceRps = 2.0;
    const bool at_target =
        target_.enabled() && std::abs(target_vel) > 1e-3 &&
        std::abs(leader_velocity_rps - target_vel) <= kAtTargetToleranceRps;
    const bool enabled = (state_.flags & (hardware::kConfigured | hardware::kEnabled)) ==
                         (hardware::kConfigured | hardware::kEnabled);

    latest_state_ = {cls}State{{
        state_.sample_time_us * 1000,
        leader_velocity_rps,
        target_vel,
        at_target,
        enabled || simulation_,
    }};
    state_sender_.send(latest_state_);
  }}

  hardware::Config config_;
  hardware::Devices devices_;
  bool simulation_{{false}};
  uint64_t config_id_{{0}};
  uint16_t leader_id_{{0}};
  hardware::State state_{{}};
  bool have_state_{{false}};
  event::MonotonicTime state_received_{{}};
  {cls}Target target_{{0, 0.0, false}};
  {cls}State latest_state_{{0, 0.0, 0.0, false, false}};
  uint64_t ticks_{{0}};

  event::Sender<Loop, Packet> request_;
  event::Sender<Loop, {cls}State> state_sender_;
  event::Timer<Loop> timer_;
}};

template <typename Loop>
inline auto from_context(Loop& loop, const talos::process::HardwareSubsystem& sub, bool simulation) {{
  return {cls}Node<Loop>::from_context(loop, sub, simulation);
}}

}}  // namespace talos::{name}
"""
    else:
        return f"""#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <utility>

#include "talOS/configuration/config_parser.h"
#include "talOS/events/handles.h"
#include "talOS/process/node_main.h"
#include "talos_nodes/{name}/packet.h"
#include "talos_nodes/{name}/{name}_message_generated.h"

namespace talos::{name} {{

template <typename Loop>
class {cls}Node {{
 public:
  struct NodeContext {{
    Loop& loop;
    bool simulation{{false}};
  }};

  static {cls}Node from_context(const NodeContext& ctx) {{
    return {cls}Node(ctx.loop, ctx.simulation);
  }}

  static {cls}Node from_context(Loop& loop, const talos::process::NodeFlags& flags) {{
    return from_context(NodeContext{{
        .loop = loop,
        .simulation = flags.simulation,
    }});
  }}

  explicit {cls}Node(Loop& loop, bool simulation = false)
      : simulation_(simulation) {{
    event::watch<{cls}Target, &{cls}Node::OnTarget>(loop, kTargetTopic, this);
    state_sender_ = event::make_sender<{cls}State>(loop, kStateTopic);
    timer_ = event::make_timer<&{cls}Node::Tick>(loop, "{name}", this);
  }}

  void Start(event::MonotonicTime first) {{
    timer_.setup_periodic(first, std::chrono::microseconds{{kPeriodUs}});
  }}

  const {cls}State& latest_state() const {{ return latest_state_; }}
  uint64_t ticks() const {{ return ticks_; }}

 private:
  void OnTarget(const event::Context&, const {cls}Target& target) {{
    target_ = target;
  }}

  void Tick(const event::Context& context) {{
    ++ticks_;
    const double target_vel = target_.enabled() ? target_.target_velocity_rps() : 0.0;
    latest_state_ = {cls}State{{
        static_cast<uint64_t>(context.now.nanos()),
        target_vel,
        target_vel,
        target_.enabled(),
        target_.enabled() || simulation_,
    }};
    state_sender_.send(latest_state_);
  }}

  bool simulation_{{false}};
  {cls}Target target_{{0, 0.0, false}};
  {cls}State latest_state_{{0, 0.0, 0.0, false, false}};
  uint64_t ticks_{{0}};

  event::Sender<Loop, {cls}State> state_sender_;
  event::Timer<Loop> timer_;
}};

template <typename Loop>
inline auto from_context(Loop& loop, const talos::process::NodeFlags& flags) {{
  return {cls}Node<Loop>::from_context(loop, flags);
}}

}}  // namespace talos::{name}
"""


def render_node_py(name: str, hardware: bool) -> str:
    cls = class_name(name)
    hw_bool = "True" if hardware else "False"
    if hardware:
        return f'''"""Node implementation for {name} (hardware)."""

import packet
from talos_runtime import node_api, runner


# Logical motor name in subsystem.toml ([motors.{name}_motor]); resolved to
# the global logical ID from the robot config, never hard-coded.
MOTOR_NAME = "{name}_motor"


def describe_source_rows():
    return packet.describe_sources(hardware={hw_bool})


def resolve_ids(config_path, subsystem="{name}", motor=MOTOR_NAME):
    """Resolve the motor's global logical ID from a config file.

    Returns (motor_id, period_us); raises RuntimeError naming a missing device.
    """
    with node_api.load_config(config_path) as cfg:
        motor_id = cfg.get_device_id(subsystem, motor)
        if motor_id == 0:
            raise RuntimeError(
                "device %r not found for subsystem %r in %s"
                % (motor, subsystem, config_path))
        period_us = cfg.period_us if cfg.period_us else packet.PERIOD_US
        return (motor_id, period_us)


class {cls}Node:
    """Event-driven hardware node for {name}."""

    @classmethod
    def from_context(cls, ctx: runner.NodeContext):
        if ctx.config is not None:
            motor_id = ctx.get_device_id(MOTOR_NAME)
            period_us = ctx.period_us or packet.PERIOD_US
            command_timeout_us = (
                ctx.command_timeout_us or packet.COMMAND_TIMEOUT_US)
            config_id = ctx.config_id
        else:
            motor_id = 1
            period_us = packet.PERIOD_US
            command_timeout_us = packet.COMMAND_TIMEOUT_US
            config_id = 0
        return cls(
            node_name=ctx.node_name,
            period_us=period_us,
            simulation=ctx.simulation,
            loop=ctx.loop,
            motor_id=motor_id,
            config_id=config_id,
            command_timeout_us=command_timeout_us,
        )

    def __init__(self, node_name="{name}", period_us=packet.PERIOD_US, simulation=False, loop=None,
                 motor_id=0, config_id=0, command_timeout_us=packet.COMMAND_TIMEOUT_US):
        self.node_name = node_name
        self.period_us = period_us
        self.simulation = simulation
        self.loop = loop
        self.motor_id = motor_id
        self.config_id = config_id
        self.command_timeout_us = command_timeout_us
        self.boot_id = 0
        self.epoch = 0
        self.ticks = 0
        self.target = (0, 0.0, False)
        self.measured_vel = 0.0
        self.motor_valid = False
        self.flags = 0
        self.sample_time_us = 0
        self.have_state = False
        self.state_rx_ns = 0
        self._prev_state = None
        self.last_request_demand = 0.0
        self.last_request_bytes = None
        self.latest_state = (0, 0.0, 0.0, False, False)

        self.timer_id = None
        self.target_watcher_id = None
        self.hw_state_watcher_id = None
        self.request_sender_id = None
        self.state_sender_id = None

        if loop is not None:
            self.register_sources(loop)

    def register_sources(self, loop):
        self.loop = loop
        self.timer_id = loop.register_timer(
            self.node_name,
            period_ns=self.period_us * 1000,
            offset_ns=0,
            callback=self._on_timer,
        )
        self.target_watcher_id = loop.register_watcher(
            packet.TARGET_TOPIC,
            packet.TARGET_SIZE,
            packet.TARGET_ALIGN,
            self._on_target,
        )
        self.hw_state_watcher_id = loop.register_watcher(
            packet.HW_STATE_TOPIC,
            packet.PACKET_SIZE,
            packet.PACKET_ALIGN,
            self._on_hw_state,
        )
        self.request_sender_id = loop.register_sender(
            packet.REQUEST_TOPIC,
            packet.PACKET_SIZE,
            packet.PACKET_ALIGN,
        )
        self.state_sender_id = loop.register_sender(
            packet.STATE_TOPIC,
            packet.STATE_SIZE,
            packet.STATE_ALIGN,
        )

    def _on_target(self, ctx, payload):
        try:
            self.target = packet.decode_target(payload)
        except Exception:
            pass

    def _on_hw_state(self, ctx, payload):
        try:
            info = packet.decode_hw_state_minimal(bytes(payload), self.motor_id)
        except Exception:
            return
        if info is None:
            return
        # Shared accept gate (talOS/hardware/gate.h AcceptState via the
        # node_api ABI): config match plus boot/staleness. Drops never touch
        # the latched sample, and the expected config_id is never overwritten
        # from the wire.
        try:
            accepted = node_api.accept_state(
                self.config_id, self._prev_state, info)
        except Exception:
            return
        if not accepted:
            return
        self._prev_state = info
        self.measured_vel = info["motor_velocity_rps"]
        self.motor_valid = info["motor_valid"]
        self.flags = info["flags"]
        self.sample_time_us = info["sample_time_us"]
        self.boot_id = info["boot_id"]
        self.epoch = info["epoch"]
        self.have_state = True
        try:
            self.state_rx_ns = ctx.now_ns
        except Exception:
            pass

    def _on_timer(self, ctx, payload):
        del payload
        self.ticks += 1
        now_ns = ctx.now_ns
        issued_ns, target_vel, enabled = self.target
        # Shared drive gates (gate.h via the node_api ABI): the gateway must
        # be ready and the latched target fresh, and this motor's sample must
        # be valid. Simulation bypasses the gates like the C++ node.
        try:
            gateway_ok = bool(node_api.gateway_ready(self.flags))
        except Exception:
            gateway_ok = False
        if not self.have_state:
            gateway_ok = False
        try:
            target_fresh = bool(node_api.target_fresh_ns(
                now_ns, issued_ns, enabled, self.state_rx_ns,
                self.command_timeout_us))
        except Exception:
            target_fresh = False
        valid = (
            (gateway_ok and target_fresh and self.motor_valid)
            or self.simulation)
        if valid and enabled:
            mode = packet.MODE_VELOCITY
            cmd_demand = target_vel
        else:
            mode = packet.MODE_NEUTRAL
            cmd_demand = 0.0
        # Header words echo the accepted gateway sample (NeutralCommand in
        # gate.h); the observed timestamp is the gateway's sample time, not
        # the local clock, so the bridge can match commands to state.
        if self.have_state and self._prev_state is not None:
            cmd_config = self._prev_state["config_id"]
            cmd_boot = self._prev_state["boot_id"]
            cmd_epoch = self._prev_state["epoch"]
            observed_us = self._prev_state["sample_time_us"]
        else:
            cmd_config = self.config_id
            cmd_boot = self.boot_id
            cmd_epoch = self.epoch
            observed_us = now_ns // 1000
        request_bytes = packet.encode_command(
            cmd_config, cmd_boot, cmd_epoch,
            observed_us, self.motor_id, mode, 0, cmd_demand,
        )
        self.last_request_demand = cmd_demand
        self.last_request_bytes = request_bytes
        if self.request_sender_id is not None and self.loop is not None:
            self.loop.send(self.request_sender_id, request_bytes)
        at_target = (
            enabled
            and abs(target_vel) > 1e-3
            and abs(self.measured_vel - target_vel) <= 2.0
        )
        # Published timestamp is the gateway's sample time (C++ node.h);
        # enabled mirrors the gateway flags, or simulation.
        if self.have_state:
            timestamp_ns = self.sample_time_us * 1000
            enabled_out = (
                (self.flags
                 & (packet.FLAG_CONFIGURED | packet.FLAG_ENABLED))
                == (packet.FLAG_CONFIGURED | packet.FLAG_ENABLED)
            ) or self.simulation
        else:
            timestamp_ns = now_ns
            enabled_out = bool(self.simulation)
        state_bytes = packet.encode_state(
            timestamp_ns=timestamp_ns,
            vel=self.measured_vel,
            target_vel=target_vel if enabled else 0.0,
            at_target=at_target,
            enabled=enabled_out,
        )
        self.latest_state = packet.decode_state(state_bytes)
        if self.state_sender_id is not None and self.loop is not None:
            self.loop.send(self.state_sender_id, state_bytes)
'''
    return f'''"""Node implementation for {name}."""

import packet
from talos_runtime import node_api, runner


def describe_source_rows():
    return packet.describe_sources(hardware={hw_bool})


class {cls}Node:
    """Event-driven node for {name}."""

    @classmethod
    def from_context(cls, ctx: runner.NodeContext):
        return cls(
            node_name=ctx.node_name,
            period_us=ctx.period_us,
            simulation=ctx.simulation,
            loop=ctx.loop,
        )

    def __init__(self, node_name="{name}", period_us=packet.PERIOD_US, simulation=False, loop=None):
        self.node_name = node_name
        self.period_us = period_us
        self.simulation = simulation
        self.loop = loop
        self.ticks = 0
        self.target = (0, 0.0, False)
        self.latest_state = (0, 0.0, 0.0, False, False)

        self.timer_id = None
        self.target_watcher_id = None
        self.state_sender_id = None

        if loop is not None:
            self.register_sources(loop)

    def register_sources(self, loop):
        self.loop = loop
        self.timer_id = loop.register_timer(
            self.node_name,
            period_ns=self.period_us * 1000,
            offset_ns=0,
            callback=self._on_timer,
        )
        self.target_watcher_id = loop.register_watcher(
            packet.TARGET_TOPIC,
            packet.TARGET_SIZE,
            packet.TARGET_ALIGN,
            self._on_target,
        )
        self.state_sender_id = loop.register_sender(
            packet.STATE_TOPIC,
            packet.STATE_SIZE,
            packet.STATE_ALIGN,
        )

    def _on_target(self, ctx, payload):
        try:
            self.target = packet.decode_target(payload)
        except Exception:
            pass

    def _on_timer(self, ctx, payload):
        del payload
        self.ticks += 1
        issued_ns, target_vel, enabled = self.target
        vel = target_vel if enabled else 0.0
        at_target = enabled and abs(vel) > 1e-3
        state_bytes = packet.encode_state(
            timestamp_ns=ctx.now_ns,
            vel=vel,
            target_vel=target_vel if enabled else 0.0,
            at_target=at_target,
            enabled=enabled or self.simulation,
        )
        self.latest_state = packet.decode_state(state_bytes)
        if self.state_sender_id is not None and self.loop is not None:
            self.loop.send(self.state_sender_id, state_bytes)
'''


def render_node_test_cc(name: str, hardware: bool) -> str:
    cls = class_name(name)
    return f"""#include "node.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/simulated_event_loop.h"

namespace talos::{name} {{
namespace {{

using namespace std::chrono_literals;

std::string TestConfigPath() {{
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(
          BAZEL_CURRENT_REPOSITORY, &error));
  const char* location = std::getenv("TALOS_TEST_CONFIG");
  if (!runfiles || !location) {{
    return "testdata/robot.toml";
  }}
  return runfiles->Rlocation(location);
}}

TEST({cls}NodeTest, SimulatedLoopExecutesAndUpdatesState) {{
  event::SimulationEnvironment environment;
  event::SimulatedEventLoop<> loop{{environment}};

  const std::string cfg_path = TestConfigPath();
  auto robot_cfg = config::ParseRobotConfig(cfg_path);
  robot_cfg.hardware.commissioned = true;

  auto node = {cls}Node<decltype(loop)>::from_context({cls}Node<decltype(loop)>::NodeContext{{
      .loop = loop,
      .hardware_config = robot_cfg.hardware,
      .devices = {{}},
      .simulation = true,
  }});

  node.Start(loop.monotonic_now());
  EXPECT_GT(loop.manifest().size(), 0u);

  // Inject target
  {cls}Target target{{static_cast<uint64_t>(loop.monotonic_now().nanos()), 25.0, true}};
  loop.inject(kTargetTopic, target);

  // Step the simulation
  loop.run_for(20ms);

  EXPECT_GT(node.ticks(), 0u);
  EXPECT_TRUE(node.latest_state().enabled());
  EXPECT_DOUBLE_EQ(node.latest_state().target_velocity_rps(), 25.0);
}}

}}  // namespace
}}  // namespace talos::{name}
"""


def render_node_test_py(name: str, hardware: bool = False) -> str:
    cls = class_name(name)
    if hardware:
        return f'''"""Hardware acceptance tests for {name} running on SimulatedEventLoop."""

import os
import unittest
import packet
from node import {cls}Node, MOTOR_NAME, resolve_ids
from talos_runtime import node_api, runner

_HERE = os.path.dirname(os.path.abspath(__file__))


def _candidate_configs():
    paths = []
    env = os.environ.get("TALOS_TEST_CONFIG")
    if env:
        paths.append(env)
        try:
            from python.runfiles import runfiles as _rf
        except ImportError:
            try:
                from rules_python.python.runfiles import runfiles as _rf
            except ImportError:
                _rf = None
        if _rf is not None:
            try:
                created = _rf.Create()
            except Exception:
                created = None
            if created is not None:
                try:
                    for cand in (created.Rlocation(env),
                                 created.Rlocation("_main/" + env)):
                        if cand:
                            paths.append(cand)
                except Exception:
                    pass
    paths.extend(["testdata/robot.toml", "testdata/{name}.toml",
                  os.path.join(_HERE, "testdata", "robot.toml"),
                  os.path.join(_HERE, "testdata", "{name}.toml")])
    seen = []
    for path in paths:
        if path and path not in seen and os.path.isfile(path):
            seen.append(path)
    return seen


def _load_config():
    for path in _candidate_configs():
        try:
            return (node_api.load_config(path), path)
        except Exception:
            continue
    return (None, None)


class {cls}NodeTest(unittest.TestCase):
    def _configured_node(self, loop, simulation=False):
        cfg, _ = _load_config()
        if cfg is None:
            self.skipTest("test config not available")
        self.addCleanup(cfg.close)
        ctx = runner.NodeContext(
            node_name="{name}",
            config=cfg,
            simulation=simulation,
            session_id=1,
            loop=loop,
        )
        return (cfg, {cls}Node.from_context(ctx))

    def test_resolves_device_id_from_config(self):
        cfg, path = _load_config()
        if cfg is None:
            self.skipTest("test config not available")
        self.addCleanup(cfg.close)
        with node_api.Loop.create_simulated() as loop:
            ctx = runner.NodeContext(
                node_name="{name}",
                config=cfg,
                simulation=False,
                session_id=1,
                loop=loop,
            )
            node = {cls}Node.from_context(ctx)
            self.assertNotEqual(node.motor_id, 0)
            self.assertEqual(
                node.motor_id, cfg.get_device_id("{name}", MOTOR_NAME))
            self.assertEqual(node.config_id, cfg.config_id)
            self.assertEqual(
                node.command_timeout_us,
                cfg.command_timeout_us or packet.COMMAND_TIMEOUT_US)

    def test_missing_device_raises(self):
        cfg, path = _load_config()
        if cfg is None:
            self.skipTest("test config not available")
        self.addCleanup(cfg.close)
        with self.assertRaises(RuntimeError):
            resolve_ids(path, motor="nonexistent_motor")

    def test_hardware_request_and_measured_state(self):
        with node_api.Loop.create_simulated() as loop:
            cfg, node = self._configured_node(loop)
            self.assertIsNotNone(node.hw_state_watcher_id)
            self.assertIsNotNone(node.request_sender_id)

            sent = []
            orig_send = loop.send
            def spy(sender_id, payload):
                sent.append((sender_id, bytes(payload)))
                return orig_send(sender_id, payload)
            loop.send = spy

            # Wire-format expectations: the generated packet must use the
            # canonical hardware layout (packet.h / messages.cc), not the old
            # scaffold-local double-at-offset-0 stand-in.
            self.assertEqual(packet.TARGET_SIZE, 24)
            self.assertEqual(packet.TARGET_ALIGN, 8)
            self.assertEqual(packet.STATE_SIZE, 32)
            self.assertEqual(packet.STATE_ALIGN, 8)
            self.assertEqual(packet.PACKET_SIZE, 1404)
            self.assertEqual(packet.PACKET_ALIGN, 4)

            # Measured velocity differs from the requested target so the
            # test can tell echo from measurement.
            sample_us = loop.now_ns() // 1000
            loop.sim_inject(
                packet.HW_STATE_TOPIC,
                packet.encode_hw_state(
                    node.config_id, 7, 3, sample_us, node.motor_id, 10.0,
                ),
            )
            loop.sim_inject(
                packet.TARGET_TOPIC,
                packet.encode_target(
                    issued_ns=loop.now_ns(),
                    target_vel=42.0,
                    enabled=True,
                ),
            )

            # Advance virtual clock by 30 ms
            loop.run_for(30_000_000)

            self.assertGreater(node.ticks, 0)
            self.assertTrue(node.have_state)
            self.assertAlmostEqual(node.measured_vel, 10.0)
            requests = [p for sid, p in sent if sid == node.request_sender_id]
            self.assertGreater(len(requests), 0, "expected motor request on REQUEST_TOPIC")
            cmd = packet.decode_command(requests[-1])
            self.assertEqual(len(cmd["motors"]), 1)
            demand = cmd["motors"][0]["demand"]
            self.assertEqual(cmd["motors"][0]["mode"], packet.MODE_VELOCITY)
            self.assertEqual(cmd["motors"][0]["id"], node.motor_id)
            self.assertNotEqual(demand, 0.0)
            self.assertAlmostEqual(demand, 42.0)
            self.assertAlmostEqual(node.last_request_demand, demand)
            # Command header echoes the accepted gateway sample: config match
            # plus the gateway's observed timestamp, not the local clock.
            self.assertEqual(cmd["config_id"], node.config_id)
            self.assertEqual(cmd["observed_time_us"], sample_us)
            # hardware::Packet envelope: size u32 @0, wire @4, zero tail.
            import struct as _struct
            (size,) = _struct.unpack_from("<I", requests[-1], 0)
            self.assertGreater(size, 0)
            self.assertLessEqual(size, packet.MAX_PAYLOAD)
            self.assertEqual(len(requests[-1]), packet.PACKET_SIZE)
            self.assertEqual(requests[-1][4 + size:], bytes(len(requests[-1]) - 4 - size))
            ts, vel, target_vel, at_target, enabled = node.latest_state
            # Published state carries the gateway sample time and measurement.
            self.assertEqual(ts, sample_us * 1000)
            self.assertAlmostEqual(vel, 10.0)
            self.assertAlmostEqual(target_vel, 42.0)
            self.assertTrue(enabled)
            self.assertNotEqual(vel, demand)

    def test_stale_target_coasts_neutral(self):
        with node_api.Loop.create_simulated() as loop:
            cfg, node = self._configured_node(loop)
            sample_us = loop.now_ns() // 1000
            loop.sim_inject(
                packet.HW_STATE_TOPIC,
                packet.encode_hw_state(
                    node.config_id, 7, 3, sample_us, node.motor_id, 10.0,
                ),
            )
            old = loop.now_ns() - (node.command_timeout_us * 1000 + 1)
            loop.sim_inject(
                packet.TARGET_TOPIC,
                packet.encode_target(
                    issued_ns=old,
                    target_vel=42.0,
                    enabled=True,
                ),
            )
            loop.run_for(30_000_000)

            self.assertGreater(node.ticks, 0)
            self.assertTrue(node.have_state)
            sent = [p for sid, p in [(node.request_sender_id, node.last_request_bytes)]
                    if p is not None]
            self.assertGreater(len(sent), 0)
            cmd = packet.decode_command(sent[-1])
            self.assertEqual(cmd["motors"][0]["mode"], packet.MODE_NEUTRAL)
            self.assertEqual(cmd["motors"][0]["demand"], 0.0)
            self.assertEqual(node.last_request_demand, 0.0)

    def test_disabled_target_coasts_neutral(self):
        with node_api.Loop.create_simulated() as loop:
            cfg, node = self._configured_node(loop)
            sample_us = loop.now_ns() // 1000
            loop.sim_inject(
                packet.HW_STATE_TOPIC,
                packet.encode_hw_state(
                    node.config_id, 7, 3, sample_us, node.motor_id, 10.0,
                ),
            )
            loop.sim_inject(
                packet.TARGET_TOPIC,
                packet.encode_target(
                    issued_ns=loop.now_ns(),
                    target_vel=42.0,
                    enabled=False,
                ),
            )
            loop.run_for(30_000_000)

            self.assertTrue(node.have_state)
            cmd = packet.decode_command(node.last_request_bytes)
            self.assertEqual(cmd["motors"][0]["mode"], packet.MODE_NEUTRAL)
            self.assertEqual(cmd["motors"][0]["demand"], 0.0)

    def test_gateway_fault_coasts_neutral(self):
        with node_api.Loop.create_simulated() as loop:
            cfg, node = self._configured_node(loop)
            sample_us = loop.now_ns() // 1000
            loop.sim_inject(
                packet.HW_STATE_TOPIC,
                packet.encode_hw_state(
                    node.config_id, 7, 3, sample_us, node.motor_id, 10.0,
                    flags=(packet.FLAG_CONFIGURED | packet.FLAG_ENABLED
                           | packet.FLAG_HARDWARE_FAULT),
                ),
            )
            loop.sim_inject(
                packet.TARGET_TOPIC,
                packet.encode_target(
                    issued_ns=loop.now_ns(),
                    target_vel=42.0,
                    enabled=True,
                ),
            )
            loop.run_for(30_000_000)

            self.assertTrue(node.have_state)
            cmd = packet.decode_command(node.last_request_bytes)
            self.assertEqual(cmd["motors"][0]["mode"], packet.MODE_NEUTRAL)
            self.assertEqual(cmd["motors"][0]["demand"], 0.0)

    def test_wrong_config_state_dropped(self):
        with node_api.Loop.create_simulated() as loop:
            cfg, node = self._configured_node(loop)
            if node.config_id == 0:
                self.skipTest("config gate accepts any without a config id")
            sample_us = loop.now_ns() // 1000
            loop.sim_inject(
                packet.HW_STATE_TOPIC,
                packet.encode_hw_state(
                    node.config_id + 1, 7, 3, sample_us, node.motor_id, 10.0,
                ),
            )
            loop.run_for(10_000_000)
            self.assertFalse(node.have_state)
            loop.sim_inject(
                packet.HW_STATE_TOPIC,
                packet.encode_hw_state(
                    node.config_id, 7, 3, sample_us, node.motor_id, 10.0,
                ),
            )
            loop.run_for(10_000_000)
            self.assertTrue(node.have_state)
            self.assertAlmostEqual(node.measured_vel, 10.0)

    def test_stale_sample_dropped(self):
        with node_api.Loop.create_simulated() as loop:
            cfg, node = self._configured_node(loop)
            loop.sim_inject(
                packet.HW_STATE_TOPIC,
                packet.encode_hw_state(
                    node.config_id, 7, 3, 2000, node.motor_id, 10.0,
                ),
            )
            loop.run_for(10_000_000)
            self.assertAlmostEqual(node.measured_vel, 10.0)
            loop.sim_inject(
                packet.HW_STATE_TOPIC,
                packet.encode_hw_state(
                    node.config_id, 7, 3, 1000, node.motor_id, 20.0,
                ),
            )
            loop.run_for(10_000_000)
            self.assertAlmostEqual(node.measured_vel, 10.0)
            self.assertEqual(node.sample_time_us, 2000)


if __name__ == "__main__":
    unittest.main()
'''
    return f'''"""Tests for {name} running on SimulatedEventLoop."""

import unittest
import packet
from node import {cls}Node
from talos_runtime import node_api, runner


class {cls}NodeTest(unittest.TestCase):
    def test_simulation_dispatch_and_state(self):
        with node_api.Loop.create_simulated() as loop:
            ctx = runner.NodeContext(
                node_name="{name}",
                config=None,
                simulation=True,
                session_id=1,
                loop=loop,
            )
            node = {cls}Node.from_context(ctx)

            # Inject a valid target
            target_bytes = packet.encode_target(
                issued_ns=loop.now_ns(),
                target_vel=42.0,
                enabled=True,
            )
            loop.sim_inject(packet.TARGET_TOPIC, target_bytes)

            # Advance virtual clock by 30 ms
            loop.run_for(30_000_000)

            self.assertGreater(node.ticks, 0)
            self.assertGreater(loop.dispatch_count, 0)
            ts, vel, target_vel, at_target, enabled = node.latest_state
            self.assertTrue(enabled)
            self.assertEqual(target_vel, 42.0)
            self.assertEqual(vel, 42.0)
            self.assertTrue(at_target)


if __name__ == "__main__":
    unittest.main()
'''


def render_subsystem_toml(name: str, hardware: bool) -> str:
    if hardware:
        return f"""[subsystem]
name = "{name}"
node = "//:node"
period_us = 5000

[motors.{name}_motor]
type = "TalonFX"
bus = "rio"
can_id = 1
inverted = false
brake = true
supply_limit_a = 40.0
stator_limit_a = 80.0
max_voltage = 12.0
sensor_to_mechanism_ratio = 1.0
max_velocity_rps = 100.0
cruise_velocity_rps = 80.0
acceleration_rps2 = 160.0
slot0 = {{ p = 0.1, i = 0.0, d = 0.0, s = 0.0, v = 0.12, a = 0.0, g = 0.0 }}
"""
    else:
        return f"""[subsystem]
name = "{name}"
node = "//:node"
period_us = 5000
"""


def render_test_robot_toml(name: str) -> str:
    return f"""# Test fixture configuration for {name}
[robot]
name = "{name}_test"
period_us = 5000
command_timeout_us = 100000
status_hz = 200.0
commissioned = false

[[subsystems]]
name = "{name}"
path = "{name}.toml"
"""


def render_test_sh() -> str:
    return """#!/usr/bin/env bash
set -euo pipefail
bazel test //... "$@"
"""


def render_readme(name: str, lang: str, hardware: bool) -> str:
    cls = class_name(name)
    lang_desc = "C++" if lang == "cpp" else "Python"
    hw_desc = "with hardware control (motors/sensors)" if hardware else "hardware-free"
    return f"""# talos_{name}

Standalone {lang_desc} talOS node repository ({hw_desc}).

## Directory Structure

- `MODULE.bazel`: Standalone Bzlmod module declaration.
- `BUILD`: Minimal build declaration using `@talos//build:defs.bzl`.
- `{name}_message.fbs`: FlatBuffers message schema for targets and states.
- `packet.h`: Topic constants and layout definitions.
- `{"node.h" if lang == "cpp" else "node.py"}`: Core node implementation.
- `{"node_test.cc" if lang == "cpp" else "node_test.py"}`: Deterministic unit tests running on `SimulatedEventLoop`.
- `subsystem.toml`: Subsystem definition and device mappings.
- `testdata/`: Local test fixtures (`robot.toml` and `{name}.toml`).
- `test.sh`: Convenience test execution script.

## Building and Testing

Run unit tests locally:
```bash
./test.sh
# or
bazel test //...
```

Run the node binary directly:
```bash
bazel run //:node -- --describe
```

## Robot Integration Workflow

1. **Add module dependency to the robot's `MODULE.bazel`**:
   ```starlark
   bazel_dep(name = "talos_{name}", version = "0.1.0")
   local_path_override(
       module_name = "talos_{name}",
       path = "../talos_{name}",
   )
   ```

2. **Assemble the node in the robot's `talos_bundle`**:
   ```starlark
   talos_bundle(
       name = "robot_bundle",
       config = "//configuration:robot.toml",
       nodes = [
           "@talos_{name}//:node",
           # other nodes...
       ],
   )
   ```

3. **Declare the subsystem in the robot manifest**:
   In `2026-robot/main_processor/configuration/robot.toml`:
   ```toml
   [[subsystems]]
   name = "{name}"
   path = "subsystems/{name}.toml"
   ```
"""
