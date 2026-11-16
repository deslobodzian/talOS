"""Scaffolds a talos subsystem: a node package plus its robot wiring.

    bazel run //tools:new_subsystem -- <name> --lang cpp|py \\
        [--type velocity|position|estimator] [--raw] [--robot 2026-robot] \\
        [--standalone DIR] [--dry-run]

Writes <robot>/main_processor/<name>/ (messages, node, Sim test, testdata),
modelled on that robot's shooter (C++) and intake (Python) packages (a
--type position node wraps talos::mechanisms::PositionMechanism, in Python
talos.mechanisms.PositionMechanism; with --raw it writes the homing and
limits out in the node's own file instead, on its Motor), or for
--type estimator a hardware-free node publishing geometry.PoseEstimate from
the PoseObservations of its `cameras`, then adds
configuration/subsystems/<name>.toml and its [[subsystems]] entry in
configuration/robot.toml, and regenerates configuration/robot_manifest.bzl
(//tools:sync_manifest), which the bundle and graph test read, plus the
manifest of every other config in the robot that includes robot.toml.
Names and CAN ids already taken are read from the effective config, so a
subsystem inherited through `include`, `defaults` or a custom `path` counts;
the teleop binding it writes commented, and its `Next:` simulation block,
take the lowest joystick 0 button from 3 that no other binding of the robot
(live, or commented in a subsystem file) uses.
Everything is checked before anything is written, so a refused run leaves the
tree untouched. --standalone writes the package as its own Bzlmod repository
in DIR instead, building against this checkout (from a robot repository:
the talOS checkout the tool runs from, such as its talos/ submodule, or
--talos-checkout DIR; it refuses when talOS is a module Bazel fetched).

From a robot repository that depends on talOS (sync_manifest's test: this
script runs from the talOS module, not the main repository):

    bazel run @talos//tools:new_subsystem -- <name> --lang cpp|py \\
        [--type ...] [--raw] [--config PATH/robot.toml] [--package-dir DIR]

writes the same package with every talOS label as @talos//..., into the
directory that holds most of the config's own nodes (`//gate:node`: the
root; `//main_processor/shooter:node`: main_processor/), else beside the
config (its parent when that is `configuration/`), and the subsystem file,
entry and manifests as above, with every config of the repository that
includes the edited one, outside a nested repository such as the talos/
submodule. --config defaults to robot.toml, else
main_processor/configuration/robot.toml.

Run it with `bazel run` (`bazel run //tools:new_subsystem -- --help` lists the
options): it imports //tools:sync_manifest, which finds the configuration
resolver in Bazel runfiles, so `python3 tools/new_subsystem.py` fails with
`No module named 'tools'`.
"""

import argparse
import ast
import collections
import difflib
import keyword
import os
import posixpath
import re
import sys
import textwrap
import tomllib
import typing

from tools import sync_manifest

# Mirrors talOS/introspection/names.h (CheckSegment, CheckNodeName): no tool
# runs those checks on a bare name before its package exists.
# new_subsystem_test parses the header and fails when the two differ; the
# launcher re-checks every name at --describe-only.
MAX_SEGMENT_BYTES = 24
MAX_TOPIC_BYTES = 31
RESERVED_OWNERS = {"hw", "talos", "sim", "replay", "test"}
ABBREVIATIONS = {
    "tgt": "target", "cmd": "command", "req": "request",
    "ds": "driver_station", "st": "state", "stat": "status",
    "pos": "position", "vel": "velocity", "acc": "acceleration",
    "dt": "drivetrain", "drive": "drivetrain", "tlm": "telemetry",
    "odom": "odometry", "msg": "message", "pkt": "packet",
    "cfg": "config", "idx": "index", "cnt": "count",
}
DIRECTION_SUFFIXES = ("_in", "_out", "_pub", "_sub", "_tx", "_rx", "_topic",
                      "_send", "_recv")

# The name is also a C++ namespace and a top-level Python package (the
# generated messages), so it must not be a keyword or shadow a module every
# node imports.
CPP_KEYWORDS = {
    "alignas", "alignof", "and", "and_eq", "asm", "auto", "bitand", "bitor",
    "bool", "break", "case", "catch", "char", "char8_t", "char16_t",
    "char32_t", "class", "compl", "concept", "const", "consteval",
    "constexpr", "constinit", "const_cast", "continue", "co_await",
    "co_return", "co_yield", "decltype", "default", "delete", "do", "double",
    "dynamic_cast", "else", "enum", "explicit", "export", "extern", "false",
    "float", "for", "friend", "goto", "if", "inline", "int", "long",
    "mutable", "namespace", "new", "noexcept", "not", "not_eq", "nullptr",
    "operator", "or", "or_eq", "private", "protected", "public", "register",
    "reinterpret_cast", "requires", "return", "short", "signed", "sizeof",
    "static", "static_assert", "static_cast", "struct", "switch", "template",
    "this", "thread_local", "throw", "true", "try", "typedef", "typeid",
    "typename", "union", "unsigned", "using", "virtual", "void", "volatile",
    "wchar_t", "while", "xor", "xor_eq", "std",
}
TAKEN_PACKAGES = {"python", "flatbuffers", "talos"}
# Targets the generated BUILD declares besides the cc_library named <name>.
PACKAGE_TARGETS = {"messages", "node", "node_bin", "node_test"}

# The lane a teleop or autonomous producer would add; the name must leave
# room for it.
LONGEST_LANE = "teleop"
PERIOD_US = 20000
MAX_CAN_ID = 62  # 63 is the CAN broadcast id
MAX_DIO = 31  # talOS/hardware/config.cc: DIO channels 0..31
# The joystick 0 buttons a scaffold's teleop binding may take, lowest first
# (talOS/teleop/README.md: buttons 1..32). It starts at 3, the button every
# scaffold took before it skipped the ones a robot already binds.
TELEOP_BUTTONS = range(3, 33)
# The teleop node's own subsystem file, whose comments show a binding as an
# example; no button of its is taken. A robot repository names it with the
# talOS module's prefix (`@talos//talOS/teleop:node`).
TELEOP_NODE = "//talOS/teleop:node"
# Keys of a subsystem file that the framework reads or refuses
# (talOS/configuration/config_parser.h: kFrameworkSubsystemKeys and
# CheckSubsystemKeys' singular spellings), and its [subsystem] header. A
# position mechanism's parameter table is [<name>], so <name> cannot be one.
# new_subsystem_test parses the header and fails when the two differ.
SUBSYSTEM_FILE_KEYS = {
    "subsystem", "node", "critical", "period_us", "remap", "targets", "teleop",
    "motors", "sensors", "encoders", "digital_inputs", "digital_outputs",
    "analog_inputs", "pwm_outputs", "trajectories", "target", "motor",
    "sensor", "encoder", "digital_input", "digital_output", "analog_input",
    "pwm_output", "trajectory",
}


class Refused(Exception):
    pass


def check_name(name):
    """Why `name` cannot name a subsystem, or None."""
    quoted = f"node name '{name}'"
    if not name:
        return "node name is empty"
    if len(name) > MAX_SEGMENT_BYTES:
        return (f"{quoted} is {len(name)} characters; the limit is "
                f"{MAX_SEGMENT_BYTES}")
    if not "a" <= name[0] <= "z":
        return f"{quoted} must start with a lowercase letter"
    for c in name:
        if not ("a" <= c <= "z" or "0" <= c <= "9" or c == "_"):
            return (f"{quoted} contains '{c}'; segments are lowercase "
                    "letters, digits and underscores")
    if "__" in name:
        return f"{quoted} has a doubled underscore"
    if name.endswith("_"):
        return f"{quoted} ends with an underscore"
    if name in ABBREVIATIONS:
        return f"{quoted} is an abbreviation; use '{ABBREVIATIONS[name]}'"
    for suffix in DIRECTION_SUFFIXES:
        if len(name) > len(suffix) and name.endswith(suffix):
            return (f"{quoted} encodes direction in '{suffix}'; direction "
                    "belongs to the endpoint, not the name")
    if name in RESERVED_OWNERS:
        return f"{quoted} is a reserved topic namespace"
    lane = f"/{name}/target/{LONGEST_LANE}"
    if len(lane) > MAX_TOPIC_BYTES:
        return (f"{quoted} leaves no room for a producer lane: '{lane}' is "
                f"{len(lane)} bytes; topics are at most {MAX_TOPIC_BYTES}")
    if (name in CPP_KEYWORDS or keyword.iskeyword(name)
            or name in sys.stdlib_module_names or name in TAKEN_PACKAGES):
        return (f"{quoted} is a C++/Python keyword or an existing module; "
                "it becomes a namespace and a Python package")
    if name in PACKAGE_TARGETS:
        return f"{quoted} collides with a target the package BUILD declares"
    return None


def check_kind(name, lang, kind, raw=False):
    """Why `name` cannot name a subsystem of this language and kind, or
    None. Both languages' library position nodes take the same [<name>]
    table; a --raw one has none."""
    del lang
    if kind == "position" and not raw and name in SUBSYSTEM_FILE_KEYS:
        return (f"node name '{name}' would name the position mechanism's "
                f"parameter table [{name}], which the subsystem file "
                "reserves")
    return None


def pascal(name):
    return "".join(part[:1].upper() + part[1:] for part in name.split("_"))


def fill(template, **values):
    for key, value in values.items():
        template = template.replace(f"@{key}@", str(value))
    return template


def label_order(label):
    """Buildifier's list order: files, then :local, //absolute, @external."""
    if label.startswith("@"):
        return 3, label
    if label.startswith("//"):
        return 2, label
    if label.startswith(":"):
        return 1, label
    return 0, label


def buildifier_sorted(labels):
    return sorted(labels, key=label_order)


def string_list(labels, indent):
    pad = " " * indent
    return "".join(f'{pad}    "{label}",\n'
                   for label in buildifier_sorted(labels)) + f"{pad}]"


# ---------------------------------------------------------------- templates

FBS = """\
namespace @NAME@;

// What a producer asks for on its lane, /@NAME@/target/<source>. The node
// drives the motor only while the robot is enabled and a lane of its target
// policy is live.
struct @PASCAL@Target {
  velocity_rps:double;
}

// What the @NAME@ is doing, published every tick.
struct @PASCAL@State {
  enabled:bool;
  velocity_rps:double;
  connected:bool;
}
"""

POSITION_FBS = """\
namespace @NAME@;

// What a producer asks for on its lane, /@NAME@/target/<source>. The node
// drives the motor with Motion Magic only while the robot is enabled and a
// lane of its target policy is live.
struct @PASCAL@Target {
  position_rot:double;
}

// Whether positions count from the home switch yet: HOMING until the switch
// zeroes the motor (it drives toward the switch only with a live target);
// FAILED once homing timed out or stalled, or the switch read closed well
// away from the zero (neutral until the Driver Station reports disabled).
enum @PASCAL@Homing : ubyte { HOMING, HOMED, FAILED }

// What the @NAME@ is doing, published every tick. enabled: it drove the motor
// this tick (homing or Motion Magic). position_rot counts from the home
// switch once HOMED; before that it is the motor's unhomed position.
struct @PASCAL@State {
  enabled:bool;
  position_rot:double;
  connected:bool;
  at_goal:bool;
  homing:@PASCAL@Homing;
}
"""

POSITION_RAW_FBS = POSITION_FBS.replace(
    "// FAILED once homing timed out or stalled, or the switch read closed well\n"
    "// away from the zero (neutral until the Driver Station reports disabled).\n",
    "// FAILED once the push toward the switch ran past its bound, or the switch\n"
    "// still read closed after the first move off it (neutral until the Driver\n"
    "// Station reports disabled).\n")
assert POSITION_RAW_FBS != POSITION_FBS

MESSAGES_BUILD = """\
# @PASCAL@Target / @PASCAL@State (namespace @NAME@) and the @NAME@'s topics.
# Producers write their own lanes, /@NAME@/target/<source>, through
# talos::TargetLane / talos.TargetLane; the @NAME@ reads them with
# talos::Target, whose policy comes from [robot.targets] and the subsystem's
# [targets].
talos_messages(
    name = "messages",
    srcs = ["@NAME@_message.fbs"],
    include_prefix = "@NAME@",
    topics = {
        "state": "/@NAME@/state",
    },
)
"""

CC_BUILD = """\
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

@MESSAGES@
cc_library(
    name = "@NAME@",
    hdrs = ["@NAME@.h"],
    include_prefix = "@NAME@",
    strip_include_prefix = ".",
    visibility = ["//visibility:private"],
    deps = [
        ":messages",
        "//talOS/devices",
        "//talOS/node",
    ],
)

talos_cc_node(
    name = "node",
    srcs = ["@NAME@.cc"],
    node_name = "@NAME@",
    deps = [":@NAME@"],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    data = @DATA@,
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)",
        "TALOS_TEST_NODE": "$(rlocationpath :node)",
    },
    deps = [
        ":@NAME@",
        "//talOS/devices:testing",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
"""

CC_HEADER = """\
#pragma once

#include <cmath>

@INCLUDES@

namespace @NAME@ {

// Drives the motor at the velocity its target asks for and reports what it
// did on /@NAME@/state. Gains live in the robot config, slot0 of
// [motors.@NAME@_motor]; the motor controller closes the velocity loop
// onboard.
//
// Nothing asks yet. A teleop or autonomous producer writes its own lane,
// /@NAME@/target/<source>, with talos::TargetLane; talos::Target
// picks the live lane by the policy in [robot.targets] or the subsystem's
// [targets].
class @PASCAL@ {
 public:
  using Target = @PASCAL@Target;
  using State = @PASCAL@State;

  explicit @PASCAL@(talos::Node& node)
      : motor_{node, "@NAME@_motor"},
        target_{node},
        state_{node.publish<State>(topics::kState)} {}

  void update() {
    const Target* t = target_.get();  // null: disabled, stale or released
    const bool drive =
        t != nullptr && motor_.connected() && std::isfinite(t->velocity_rps());
    // Otherwise nothing is set and the flush sends neutral.
    if (drive) motor_.set_velocity(t->velocity_rps());
    state_.send(State{drive, motor_.velocity(), motor_.connected()});
  }

 private:
  talos::Motor motor_;
  talos::Target<Target> target_;
  talos::Publisher<State> state_;
};

}  // namespace @NAME@
"""
CC_POSITION_HEADER = """\
#pragma once

@INCLUDES@

namespace @NAME@ {

// Drives the mechanism to the position its target asks for and reports
// what it did on /@NAME@/state. Producers write lanes
// /@NAME@/target/<source> with talos::TargetLane; the
// policy comes from [robot.targets] or the subsystem's [targets].
//
// The control is a talos::mechanisms::PositionMechanism, configured by the
// subsystem file: the motor [motors.@NAME@] and the
// parameter table [@NAME@] (keys and units in the talos repository's
// talOS/mechanisms/README.md). It runs Motion Magic to the goal, clamped to
// min_rot..max_rot, and homes on the switch @NAME@_home:
// positions count from that switch, not from where the motor powered on.
// Until the switch has zeroed the motor, and again after a motor reset or a
// RoboRIO reboot drops that zero, a live goal drives toward the switch at
// homing_v instead. A homing timeout, a homing push that does not move, or
// the switch reading closed well into the travel (stuck) latches FAILED,
// neutral, until the Driver Station reports disabled.
class @PASCAL@ {
 public:
  using Target = @PASCAL@Target;
  using State = @PASCAL@State;
  using Homing = @PASCAL@Homing;

  explicit @PASCAL@(talos::Node& node)
      : mechanism_{node, "@NAME@"},
        target_{node},
        state_{node.publish<State>(topics::kState)} {}

  void update() {
    // The mechanism drives only on this tick's request. Without a live
    // target (disabled, stale, released, another mode's lane) stop(): the
    // motor goes neutral rather than holding an old goal.
    if (const Target* t = target_.get()) {
      mechanism_.set_goal(t->position_rot());  // homes first; NaN: neutral
    } else {
      mechanism_.stop();
    }
    mechanism_.update();
    state_.send(State{mechanism_.driving(), mechanism_.position_rot(),
                      mechanism_.connected(), mechanism_.at_goal(),
                      HomingState()});
  }

 private:
  Homing HomingState() const {
    using Phase = talos::mechanisms::PositionPhase;
    if (mechanism_.phase() == Phase::kFaulted) return Homing::FAILED;
    return mechanism_.homed() ? Homing::HOMED : Homing::HOMING;
  }

  // A helper, not a requirement: it homes, clamps goals to the travel and
  // latches faults. The node may drive its talos::Motor directly instead
  // (docs/FIRST_MECHANISM.md in the talos repository).
  talos::mechanisms::PositionMechanism mechanism_;
  talos::Target<Target> target_;
  talos::Publisher<State> state_;
};

}  // namespace @NAME@
"""

# --type position --raw: the same mechanism with every homing and limit rule
# written out in the student's own file, no talOS/mechanisms. The behaviour
# is docs/FIRST_MECHANISM.md §6's.
CC_POSITION_RAW_HEADER = """\
#pragma once

#include <algorithm>
#include <cmath>

@INCLUDES@

namespace @NAME@ {

// Drives the mechanism to the position its target asks for and reports
// what it did on /@NAME@/state. Producers write lanes
// /@NAME@/target/<source> with talos::TargetLane; the
// policy comes from [robot.targets] or the subsystem's [targets].
//
// Every line of the control is here, in this file: it homes on the switch
// @NAME@_home, then runs Motion Magic inside a fixed travel.
// Positions count from that switch, not from where the motor powered on.
// Until the switch has zeroed the motor, and again after a motor reset or a
// RoboRIO reboot drops that zero, a live goal pushes toward the switch at
// kHomingVolts instead. A push that runs past its bound, or the switch still
// reading closed after the first move off it, latches FAILED, neutral, until
// the Driver Station reports disabled. docs/FIRST_MECHANISM.md §6 in the
// talos repository explains each rule.
class @PASCAL@ {
 public:
  using Target = @PASCAL@Target;
  using State = @PASCAL@State;
  using Homing = @PASCAL@Homing;

  explicit @PASCAL@(talos::Node& node)
      : motor_{node, "@NAME@"},
        home_{node, "@NAME@_home"},
        ds_{node},
        target_{node},
        state_{node.publish<State>(topics::kState)} {}

  void update() {
    // The switch reads false when closed, true when open.
    const bool on_switch = home_.connected() && !home_.get();
    const bool off_switch = home_.connected() && home_.get();
    // Zero on the switch once the motor is at rest, so the first move does
    // not start at homing speed.
    if (!motor_.zeroed() && on_switch &&
        std::abs(motor_.velocity()) < kRestRps) {
      motor_.rezero(0.0);
    }
    if (motor_.zeroed()) {
      push_ticks_ = 0;  // homed: a later re-home pushes from zero
      if (off_switch) left_switch_ = true;
    } else {
      left_switch_ = false;  // a dropped zero homes and checks again
    }
    // Only a Driver Station that reports disabled clears a failure: not a
    // release, a stale lane or a feedback blip, any of which would otherwise
    // start another push into the stop.
    if (ds_.connected() && !ds_.enabled()) {
      failed_ = false;
      push_ticks_ = 0;
    }
    // Still closed after the first move off it: the switch is stuck, or the
    // zero is wrong. Stop before the move reaches kHomeMarginRot.
    if (motor_.zeroed() && !left_switch_ &&
        motor_.position() > kHomeMarginRot - kToleranceRot) {
      failed_ = true;
    }
    const Target* t = target_.get();  // null: disabled, stale or released
    const bool live = t != nullptr && motor_.connected() &&
                      std::isfinite(t->position_rot()) && !failed_;
    bool drove = false;
    double goal = 0.0;
    if (live && motor_.zeroed()) {
      // Until the switch has opened, no farther than kHomeMarginRot.
      const double top = left_switch_ ? kMaxRot : kHomeMarginRot;
      goal = std::clamp(t->position_rot(), 0.0, top);
      motor_.set_motion_magic(goal);
      drove = true;
    } else if (live && !on_switch) {
      // The push is bounded: a switch that never closes (a cut wire reads
      // open) would otherwise push into the stop while the button is held.
      if (++push_ticks_ > kMaxPushTicks) {
        failed_ = true;  // pushed long enough to cross the travel: no switch
      } else {
        motor_.set_voltage(kHomingVolts);  // toward the switch
        drove = true;
      }
    }
    // Otherwise nothing is set and the flush sends neutral.
    const bool at_goal = drove && motor_.zeroed() &&
                         std::abs(motor_.position() - goal) < kToleranceRot;
    Homing homing = motor_.zeroed() ? Homing::HOMED : Homing::HOMING;
    if (failed_) homing = Homing::FAILED;
    state_.send(
        State{drove, motor_.position(), motor_.connected(), at_goal, homing});
  }

 private:
  // PLACEHOLDER values, sized for the simulation new_subsystem's `Next:`
  // describes. Set each for your mechanism before the robot drives it.
  //
  // Travel above the switch, in rotations: goals clamp to 0..kMaxRot.
  // Measure it on the mechanism.
  static constexpr double kMaxRot = 2.0;
  // The homing push, in volts; negative drives toward the switch. Keep it
  // low: with the motor's stator_limit_a it bounds how hard the push meets
  // the stop. Check its direction before the first enable.
  static constexpr double kHomingVolts = -0.5;
  // The push bound, in ticks of the subsystem's period_us: time a push
  // across the full travel at kHomingVolts and allow about twice that.
  // 50 ticks is 1 s at period_us = 20000.
  static constexpr int kMaxPushTicks = 50;
  // The switch must read open by here after each zero, in rotations: the
  // distance from the zero at which it reliably opens, plus a little. Keep
  // it small: from a wrong zero the mechanism can still move this far.
  static constexpr double kHomeMarginRot = 0.15;
  // Within this of the goal, /@NAME@/state reports at_goal.
  static constexpr double kToleranceRot = 0.05;
  // Slower than this the motor is at rest, and the switch may zero it.
  static constexpr double kRestRps = 0.05;

  talos::Motor motor_;
  talos::DigitalInput home_;
  talos::DriverStation ds_;
  talos::Target<Target> target_;
  talos::Publisher<State> state_;
  bool failed_ = false;
  bool left_switch_ = false;
  int push_ticks_ = 0;
};

}  // namespace @NAME@
"""


CC_SOURCE = """\
#include "@NAME@/@NAME@.h"

TALOS_NODE(@NAME@::@PASCAL@)
"""

CC_TEST = """\
#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

@INCLUDES@

namespace @NAME@ {
namespace {

using namespace std::chrono_literals;
using talos::hardware::Mode;
using talos::testing::AllocationProbe;
using talos::testing::DriverStationGeneration;
using talos::testing::DsPacket;
using talos::testing::PublishTarget;
using talos::testing::ReleaseTarget;
using talos::testing::Sim;
using talos::testing::SimHardware;
namespace ds = talos::driver_station;

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"runfile unavailable: "} + env + " " +
                             error);
  }
  return runfiles->Rlocation(location);
}

// The node test matrix, docs/TESTING.md §7 in the talos repository: the case
// names are the matrix's, which talos's //tools:check_node_matrix looks for.
class @PASCAL@Test : public ::testing::Test {
 protected:
  using Target = @PASCAL@::Target;
  using State = @PASCAL@::State;

  // An enabled teleop robot: the default policy reads the teleop lane.
  void Teleop() {
    hw.enable();
    hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
    sim.step(10ms);  // the mode is current before any lane arrives
  }
  // Holds a lane the way a producer does: republished every 20 ms.
  void Hold(double demand, std::chrono::milliseconds duration,
            const char* source = "teleop") {
    for (auto t = 0ms; t < duration; t += 20ms) {
      PublishTarget(sim, "@NAME@", source, Target{demand});
      sim.step(20ms);
    }
  }
  // What the node asked for (the flush sends neutral for an unset motor).
  Mode Request() const {
    return hw.motor_request("@NAME@", "@NAME@_motor").mode;
  }
  // What the gateway applies, after its enable and expiry gates.
  Mode Applied() const {
    return hw.applied_motor("@NAME@", "@NAME@_motor").mode;
  }
  State Latest() const {
    const auto state = sim.latest<State>(topics::kState);
    EXPECT_TRUE(state.has_value());
    return state.value_or(State{});
  }

  Sim sim{Runfile("TALOS_TEST_CONFIG")};
  @PASCAL@& node = sim.add<@PASCAL@>("@NAME@");
  SimHardware hw{sim};
};

// REQ-SAFE-07
TEST_F(@PASCAL@Test, DrivesAtALiveLane) {
  hw.ideal_motors();
  Teleop();
  Hold(30.0, 60ms);

  const auto request = hw.applied_motor("@NAME@", "@NAME@_motor");
  EXPECT_EQ(request.mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(request.demand, 30.0);
  const State state = Latest();
  EXPECT_TRUE(state.enabled());
  EXPECT_DOUBLE_EQ(state.velocity_rps(), 30.0);
  EXPECT_TRUE(state.connected());
}

// REQ-CONTRACT-25: matrix case 2, `state` still published with
// `enabled == false` while disabled. The neutral request is the
// devices flush's (REQ-SAFE-02, proved in talOS/devices), not this
// node's evidence.
TEST_F(@PASCAL@Test, DisabledRobotGoesNeutral) {
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  hw.enable(false);
  sim.step(10ms);
  Hold(30.0, 60ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Applied(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());  // state keeps publishing
}

// REQ-CONTRACT-25: matrix case 3, the node's `state` still published, `enabled`
// false, once its lane is stale. The neutral request is the devices flush's
// (Target::get returns no target: REQ-SAFE-41, proved in talOS/devices), not
// this node's evidence.
TEST_F(@PASCAL@Test, StaleLaneGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVelocity);

  sim.step(150ms);  // past command_timeout_us: the lane is stale
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Applied(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());  // state keeps publishing
}

// Matrix case 4: the node keeps running when its lane is released. The neutral
// request is the devices flush's (Target::get returns no target: REQ-SAFE-41,
// proved in talOS/devices), not this node's evidence.
TEST_F(@PASCAL@Test, ReleasedLaneGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVelocity);

  ReleaseTarget<Target>(sim, "@NAME@", "teleop");
  sim.step(40ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
}

// REQ-CONTRACT-25: matrix case 5, the node's `state` published, `enabled`
// false, while no target was ever sent. The neutral request is the devices
// flush's (Target::get returns no target: REQ-SAFE-41, proved in
// talOS/devices), not this node's evidence.
TEST_F(@PASCAL@Test, NeverSentGoesNeutral) {
  Teleop();
  sim.step(60ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());
}

// Matrix case 6: the node keeps running once the Driver Station view is stale.
// The neutral request is the devices flush's (Target::get returns no
// target: REQ-SAFE-41, proved in talOS/devices), not this node's evidence.
TEST_F(@PASCAL@Test, DriverStationLossGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVelocity);

  hw.stop_driver_station();  // the bridge stops forwarding DS packets
  Hold(30.0, 160ms);         // past command_timeout_us: the DS is stale
  EXPECT_EQ(Request(), Mode::kNeutral);
}

// REQ-CONTRACT-25
TEST_F(@PASCAL@Test, DisconnectedFeedbackGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVelocity);

  hw.set_connected("@NAME@", "@NAME@_motor", false);
  Hold(30.0, 40ms);
  // The neighbor's motor keeps the robot enabled (testdata/robot.toml), so
  // the node's own feedback gate is what stops it.
  ASSERT_TRUE(talos::robot_enabled(sim.node("@NAME@")));
  EXPECT_EQ(Request(), Mode::kNeutral);
  const State state = Latest();
  EXPECT_FALSE(state.connected());
  EXPECT_FALSE(state.enabled());
}

// REQ-SAFE-04
TEST_F(@PASCAL@Test, NonFiniteDemandGoesNeutral) {
  Teleop();
  Hold(std::numeric_limits<double>::quiet_NaN(), 40ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());
}

// matrix: n/a 9 (the @NAME@ reads no parameters)

// REQ-SAFE-07: matrix case 10, the current mode's lane drives the node end to
// end through its Target; the other mode's lane going neutral is the
// flush's (REQ-SAFE-41).
TEST_F(@PASCAL@Test, OtherModesLaneIsIgnored) {
  Teleop();
  Hold(30.0, 40ms, "auto");  // autonomous's lane, live, in teleop
  EXPECT_EQ(Request(), Mode::kNeutral);
  ReleaseTarget<Target>(sim, "@NAME@", "auto");

  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kAutonomous));
  sim.step(10ms);
  Hold(30.0, 40ms);  // teleop's lane, live, in autonomous
  EXPECT_EQ(Request(), Mode::kNeutral);
  Hold(20.0, 40ms, "auto");
  const auto request = hw.motor_request("@NAME@", "@NAME@_motor");
  EXPECT_EQ(request.mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(request.demand, 20.0);
}

// Matrix case 11: the node drives again on a target of the current generation.
// Refusing the earlier one is Target's and the flush's (REQ-SAFE-09 in
// talOS/devices/target_test.cc, and REQ-SAFE-41).
TEST_F(@PASCAL@Test, EarlierGenerationTargetIsIgnored) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVelocity);
  const std::uint64_t before = DriverStationGeneration(sim);
  hw.set_driver_station(DsPacket(ds::kTeleop));  // disabled
  sim.step(40ms);
  // Re-enabled. SimHardware publishes the packet at its next 20 ms hook,
  // which the Sim runs after the node's tick at that instant, so the packet
  // waits for the node's next drain; a target its producer queued under the
  // earlier enable, published before that drain, arrives in the same one.
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(20ms);
  ASSERT_NE(DriverStationGeneration(sim), before);
  PublishTarget(sim, "@NAME@", "teleop", Target{30.0}, before);
  sim.step(20ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  // The same target stamped with the current generation drives at once.
  PublishTarget(sim, "@NAME@", "teleop", Target{30.0});
  sim.step(20ms);
  EXPECT_EQ(Request(), Mode::kVelocity);
}

// REQ-RT-06: the steady-state tick (target read, request, flush, state
// publish) allocates nothing. The target is published before the window and
// stays fresh through it (60 ms < command_timeout_us).
TEST_F(@PASCAL@Test, UpdateDoesNotAllocate) {
  if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  Teleop();
  Hold(30.0, 40ms);
  PublishTarget(sim, "@NAME@", "teleop", Target{30.0});
  AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(Request(), Mode::kVelocity);
}

struct Result {
  int status;
  std::string output;
};

Result RunNode(const std::string& args) {
  const std::string command = "'" + Runfile("TALOS_TEST_NODE") + "' " + args +
                              " --config '" + Runfile("TALOS_TEST_CONFIG") +
                              "' 2>&1";
  FILE* pipe = ::popen(command.c_str(), "r");
  if (pipe == nullptr) throw std::runtime_error("popen failed");
  Result result{0, {}};
  char buffer[4096];
  while (const std::size_t n = std::fread(buffer, 1, sizeof buffer, pipe)) {
    result.output.append(buffer, n);
  }
  const int status = ::pclose(pipe);
  result.status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return result;
}

TEST(@PASCAL@Binary, LoggedSimRunReplaysWithoutDivergence) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string log =
      std::string{tmp != nullptr ? tmp : "/tmp"} + "/node_test.tlog";
  const Result run = RunNode("--sim --duration-s 1 --log '" + log + "'");
  ASSERT_EQ(run.status, 0) << run.output;
  const Result replay = RunNode("--replay '" + log + "'");
  EXPECT_EQ(replay.status, 0) << replay.output;
  EXPECT_NE(replay.output.find("diverged=0 clean_exit=1"), std::string::npos)
      << replay.output;
}

}  // namespace
}  // namespace @NAME@
"""

# Inserted into CC_TEST for a position mechanism: its fixture starts on the
# home switch, and the homing tests start off it.
CC_POSITION_FIXTURE = """\
  SimHardware hw{sim};

  // Starts on its home switch, so the motor zeroes without moving.
  @PASCAL@Test() { SetHome(true); }
  // Normally open to ground behind the input's pull-up: closed reads false.
  void SetHome(bool closed) {
    hw.set_digital_input("@NAME@", "@NAME@_home", !closed);
  }
  // Powered on off the switch, unzeroed, and moving when pushed.
  void StartOffHome() {
    SetHome(false);
    hw.set_motor("@NAME@", "@NAME@", 0.0, -1.0);
  }
};
"""

# Replaces CC_TEST's case 9 for a position mechanism.
CC_POSITION_PARAMETERS = """\
// matrix: n/a 9 (PositionMechanism's parameters, tested in talos)
"""

# The node's own logic over the mechanism: the config homes it, and the
# mechanism's phase reaches /@NAME@/state. The homing rules themselves are
# talOS/mechanisms' and tested there.
CC_POSITION_TESTS = """\
// The mechanism library's behaviour. The tests from here to the binary test
// cover what talos::mechanisms::PositionMechanism does for this node: it
// homes on the switch before the first goal, counts positions from there,
// clamps goals to min_rot..max_rot and latches FAILED when homing does not
// move. If you drive the Motor yourself instead, replace them with tests of
// your own behaviour. The tests above are the node test matrix
// (docs/TESTING.md §7 in the talos repository) that every actuator node
// keeps; they expect Motion Magic requests (Mode::kMotionMagic),
// PositionMechanism's mode, so change that if your node drives another, and
// replace the matrix's n/a 9 with BadParameterFailsConstruction once the node
// reads parameters of its own.
//
// The values are testdata/@NAME@.toml's.
constexpr double kHomingVolts = -1.0;
constexpr double kMaxRot = 100.0;

// REQ-SAFE-36
TEST_F(@PASCAL@Test, HomesBeforeTheFirstGoal) {
  StartOffHome();
  hw.ideal_motors();
  Teleop();
  Hold(30.0, 60ms);  // toward the switch, not to 30 in the power-on frame
  auto request = hw.motor_request("@NAME@", "@NAME@");
  EXPECT_EQ(request.mode, Mode::kVoltage);
  EXPECT_DOUBLE_EQ(request.demand, kHomingVolts);
  EXPECT_EQ(Latest().homing(), @PASCAL@Homing::HOMING);

  // The switch closes 0.5 rotations below where the motor powered on: that
  // is position 0, and targets go out in that frame, clamped to the travel.
  hw.set_motor("@NAME@", "@NAME@", -0.5, 0.0);
  // It zeroes after home_samples (3) consecutive closed readings.
  SetHome(true);
  Hold(30.0, 60ms);
  SetHome(false);  // the motor leaves the switch
  Hold(30.0, 20ms);
  request = hw.motor_request("@NAME@", "@NAME@");
  EXPECT_EQ(request.mode, Mode::kMotionMagic);
  EXPECT_DOUBLE_EQ(request.demand, 30.0 - 0.5);
  EXPECT_EQ(Latest().homing(), @PASCAL@Homing::HOMED);
  Hold(1000.0, 40ms);
  request = hw.motor_request("@NAME@", "@NAME@");
  EXPECT_DOUBLE_EQ(request.demand, kMaxRot - 0.5);
  Hold(-5.0, 40ms);
  request = hw.motor_request("@NAME@", "@NAME@");
  EXPECT_DOUBLE_EQ(request.demand, -0.5);  // position 0
}

// REQ-SAFE-36
TEST_F(@PASCAL@Test, HomingThatDoesNotMoveFails) {
  SetHome(false);  // velocity stays 0
  Teleop();
  Hold(30.0, 100ms);
  ASSERT_EQ(Request(), Mode::kVoltage);
  Hold(30.0, 200ms);  // past homing_stall_us (200 ms)
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Latest().homing(), @PASCAL@Homing::FAILED);
}

"""

# Replaces CC_POSITION_TESTS for --raw: the node's own homing bounds.
CC_POSITION_RAW_TESTS = """\
// The node's own homing bounds. The tests above are the node test matrix
// (docs/TESTING.md §7 in the talos repository) that every actuator node
// keeps; these guard the rules this node adds. Their times and positions
// follow the constants in
// @NAME@.h: kMaxPushTicks (50 ticks, 1 s),
// kHomeMarginRot (0.15) and kMaxRot (2.0); change them together.

// REQ-DEV-01
TEST_F(@PASCAL@Test, HomingThatNeverReachesTheSwitchFails) {
  StartOffHome();  // the switch never closes, as with a cut wire
  Teleop();
  Hold(1.0, 900ms);
  ASSERT_EQ(Request(), Mode::kVoltage);  // still pushing toward the switch
  Hold(1.0, 200ms);                      // past the 1 s bound
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Latest().homing(), @PASCAL@Homing::FAILED);
}

// REQ-DEV-01
TEST_F(@PASCAL@Test, FailureLatchesUntilDisabled) {
  StartOffHome();
  Teleop();
  Hold(1.0, 1100ms);
  ASSERT_EQ(Latest().homing(), @PASCAL@Homing::FAILED);
  sim.step(150ms);   // the lane goes stale while the button is held
  Hold(1.0, 200ms);  // live again: no second push
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Latest().homing(), @PASCAL@Homing::FAILED);
  hw.set_driver_station(DsPacket(ds::kTeleop));  // disabled
  sim.step(40ms);
  Teleop();  // enabled again: a new homing run
  Hold(1.0, 40ms);
  EXPECT_EQ(Request(), Mode::kVoltage);
}

// REQ-DEV-01
TEST_F(@PASCAL@Test, StuckSwitchStopsWithinTheMargin) {
  hw.ideal_motors();
  // Powered on near the top; the switch, stuck closed, zeroes it there.
  hw.set_motor("@NAME@", "@NAME@", 1.9, 0.0);
  Teleop();
  for (int tick = 0; tick < 10; ++tick) {
    Hold(1.5, 20ms);
    // Never more than kHomeMarginRot (0.15) above the false zero.
    EXPECT_LE(hw.motor_request("@NAME@", "@NAME@").demand,
              1.9 + 0.15 + 1e-9);
  }
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_EQ(Latest().homing(), @PASCAL@Homing::FAILED);
}

// REQ-DEV-01: off the switch, a goal past either end of the travel
// demands that end, kMaxRot (2.0) or 0.
TEST_F(@PASCAL@Test, GoalOutsideTheTravelClampsToIt) {
  hw.ideal_motors();
  Teleop();
  SetHome(false);  // zeroed on the switch, it leaves it
  Hold(5.0, 60ms);
  EXPECT_DOUBLE_EQ(
      hw.motor_request("@NAME@", "@NAME@").demand, 2.0);
  Hold(-1.0, 60ms);
  EXPECT_DOUBLE_EQ(
      hw.motor_request("@NAME@", "@NAME@").demand, 0.0);
}

// REQ-DEV-01: the switch zeroes the motor only once it moves slower than
// kRestRps (0.05).
TEST_F(@PASCAL@Test, ZeroWaitsForRest) {
  // On the switch, still moving.
  hw.set_motor("@NAME@", "@NAME@", 0.0, -1.0);
  Teleop();
  Hold(1.0, 60ms);
  EXPECT_EQ(Latest().homing(), @PASCAL@Homing::HOMING);
  EXPECT_EQ(Request(), Mode::kNeutral);  // on the switch: no push
  hw.set_motor("@NAME@", "@NAME@", 0.0, 0.0);  // at rest
  Hold(1.0, 60ms);
  EXPECT_EQ(Latest().homing(), @PASCAL@Homing::HOMED);
  EXPECT_EQ(Request(), Mode::kMotionMagic);
}

"""

CC_TESTDATA_ROBOT = """\
# Local @NAME@ fixture. Device IDs are assigned by the normal parser.
# The @NEIGHBOR@ subsystem's motor stays connected: losing the node's motor
# is one device lost, as on a robot, which only the node's own feedback
# gate stops. Losing every motor and CAN sensor is the gateway's fault.
[robot]
name = "@NAME@_test"
period_us = @PERIOD@
command_timeout_us = 100000
status_hz = 200.0
commissioned = false

[[subsystems]]
name = "@NAME@"
path = "@NAME@.toml"

[[subsystems]]
name = "@NEIGHBOR@"
path = "@NEIGHBOR@.toml"
"""

# Another subsystem's motor, which a test never disconnects.
CC_TESTDATA_NEIGHBOR = """\
[subsystem]
name = "@NEIGHBOR@"
period_us = @PERIOD@

[motors.wheel]
type = "TalonFX"
bus = "rio"
can_id = 2
"""

CC_TESTDATA_SUBSYSTEM = """\
[subsystem]
name = "@NAME@"
period_us = @PERIOD@

[motors.@NAME@_motor]
type = "TalonFX"
bus = "rio"
can_id = 1
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }
"""

CC_POSITION_TESTDATA_SUBSYSTEM = """\
[subsystem]
name = "@NAME@"
period_us = @PERIOD@

[@NAME@]
tolerance_rot = 0.05
min_rot = 0.0
max_rot = 100.0
homing = "limit_switch"
limit_switch = "@NAME@_home"
homing_v = -1.0
homing_timeout_us = 500000
homing_stall_rps = 0.1
homing_stall_us = 200000
home_margin_rot = 0.5

[motors.@NAME@]
type = "TalonFX"
bus = "rio"
can_id = 1
position_frame = "rezero"
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }

[digital_inputs.@NAME@_home]
dio = 0
"""

CC_POSITION_RAW_TESTDATA_SUBSYSTEM = """\
[subsystem]
name = "@NAME@"
period_us = @PERIOD@

[motors.@NAME@]
type = "TalonFX"
bus = "rio"
can_id = 1
position_frame = "rezero"
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }

[digital_inputs.@NAME@_home]
dio = 0
"""

PY_BUILD = """\
load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_messages", "talos_py_node")

@MESSAGES@
talos_py_node(
    name = "node",
    class_name = "@PASCAL@",
    main = "@NAME@.py",
    node_name = "@NAME@",
    deps = [":messages_py"],
)

py_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.py"],
    data = @DATA@,
    env = {"NODE_BINARY": "$(rlocationpath :node)"},
    legacy_create_init = False,
    # ctypes cannot initialize sanitizer runtimes inside the Python host.
    tags = [
        "no_asan",
        "no_tsan",
    ],
    deps = [
        ":messages_py",
        "//talOS/python:testing",
        "@rules_python//python/runfiles",
    ],
)
"""

PY_NODE = '''\
"""@PASCAL@: one motor driven at the velocity its target asks for.

Every tick the @NAME@ drives the motor at the target's velocity while a lane
of its target policy is live (robot enabled, lane active and fresh), the motor
is connected and the demand is finite, and reports what it did on
/@NAME@/state. Gains live in the robot config ([motors.@NAME@_motor] slot0);
the motor controller closes the velocity loop onboard.

Nothing asks yet. A teleop or autonomous producer writes its own lane,
/@NAME@/target/<source>, with talos.TargetLane; talos.Target picks the live
lane by the policy in [robot.targets] or the subsystem's [targets].
"""

import math

import talos
from @NAME@ import topics
from @NAME@.@PASCAL@State import @PASCAL@State
from @NAME@.@PASCAL@Target import @PASCAL@Target


class @PASCAL@:
    def __init__(self, node):
        self.motor = talos.Motor(node, "@NAME@_motor")
        self.target = talos.Target(node, @PASCAL@Target)
        self.state = node.publish(@PASCAL@State, topics.STATE)

    def update(self):
        t = self.target.get()  # None: disabled, stale or released
        drive = (t is not None and self.motor.connected
                 and math.isfinite(t.velocity_rps))
        if drive:
            # Otherwise nothing is set and the flush sends neutral.
            self.motor.set_velocity(t.velocity_rps)
        self.state.send(enabled=drive, velocity_rps=self.motor.velocity,
                        connected=self.motor.connected)
'''

PY_POSITION_NODE = '''\
"""@PASCAL@: drives the mechanism to the position its target asks for.

Reports what it did on /@NAME@/state. Producers write lanes
/@NAME@/target/<source> with talos.TargetLane; the policy comes from
[robot.targets] or the subsystem's [targets].

The control is a talos.mechanisms.PositionMechanism (the C++
talos::mechanisms::PositionMechanism), configured by the subsystem file: the
motor [motors.@NAME@] and the parameter table [@NAME@] (keys and units in
the talos repository's talOS/mechanisms/README.md). It runs Motion Magic to
the goal, clamped to min_rot..max_rot, and homes on the switch @NAME@_home:
positions count from that switch, not from where the motor powered on.
Until the switch has zeroed the motor, and again after a motor reset or a
RoboRIO reboot drops that zero, a live goal drives toward the switch at
homing_v instead. A homing timeout, a homing push that does not move, or the
switch reading closed well into the travel (stuck) latches FAILED, neutral,
until the Driver Station reports disabled.
"""

import talos
@FROM_IMPORTS@


class @PASCAL@:
    def __init__(self, node):
        # A helper, not a requirement: it homes, clamps goals to the travel
        # and latches faults. The node may drive its talos.Motor directly
        # instead (docs/FIRST_MECHANISM.md in the talos repository).
        self.mechanism = PositionMechanism(node, "@NAME@")
        self.target = talos.Target(node, @PASCAL@Target)
        self.state = node.publish(@PASCAL@State, topics.STATE)

    def update(self):
        # The mechanism drives only on this tick's request. Without a live
        # target (disabled, stale, released, another mode's lane) stop(): the
        # motor goes neutral rather than holding an old goal.
        t = self.target.get()
        if t is not None:
            self.mechanism.set_goal(t.position_rot)  # homes first
        else:
            self.mechanism.stop()
        self.mechanism.update()
        m = self.mechanism
        if m.phase == PositionPhase.FAULTED:
            homing = @PASCAL@Homing.FAILED
        elif m.homed:
            homing = @PASCAL@Homing.HOMED
        else:
            homing = @PASCAL@Homing.HOMING
        self.state.send(enabled=m.driving, position_rot=m.position_rot,
                        connected=m.connected, at_goal=m.at_goal,
                        homing=homing)
'''

# --type position --raw in Python: CC_POSITION_RAW_HEADER's control, line for
# line.
PY_POSITION_RAW_NODE = '''\
"""@PASCAL@: drives the mechanism to the position its target asks for.

Reports what it did on /@NAME@/state. Producers write lanes
/@NAME@/target/<source> with talos.TargetLane; the policy comes from
[robot.targets] or the subsystem's [targets].

Every line of the control is here, in this file: it homes on the switch
@NAME@_home, then runs Motion Magic inside a fixed travel.
Positions count from that switch, not from where the motor powered on. Until
the switch has zeroed the motor, and again after a motor reset or a RoboRIO
reboot drops that zero, a live goal pushes toward the switch at HOMING_VOLTS
instead. A push that runs past its bound, or the switch still reading closed
after the first move off it, latches FAILED, neutral, until the Driver
Station reports disabled. docs/FIRST_MECHANISM.md §6 in the talos repository
explains each rule.
"""

import math

import talos
@FROM_IMPORTS@

# PLACEHOLDER values, sized for the simulation new_subsystem's `Next:`
# describes. Set each for your mechanism before the robot drives it.
#
# Travel above the switch, in rotations: goals clamp to 0..MAX_ROT. Measure
# it on the mechanism.
MAX_ROT = 2.0
# The homing push, in volts; negative drives toward the switch. Keep it low:
# with the motor's stator_limit_a it bounds how hard the push meets the stop.
# Check its direction before the first enable.
HOMING_VOLTS = -0.5
# The push bound, in ticks of the subsystem's period_us: time a push across
# the full travel at HOMING_VOLTS and allow about twice that. 50 ticks is 1 s
# at period_us = 20000.
MAX_PUSH_TICKS = 50
# The switch must read open by here after each zero, in rotations: the
# distance from the zero at which it reliably opens, plus a little. Keep it
# small: from a wrong zero the mechanism can still move this far.
HOME_MARGIN_ROT = 0.15
# Within this of the goal, /@NAME@/state reports at_goal.
TOLERANCE_ROT = 0.05
# Slower than this the motor is at rest, and the switch may zero it.
REST_RPS = 0.05


class @PASCAL@:
    def __init__(self, node):
        self.motor = talos.Motor(node, "@NAME@")
        self.home = talos.DigitalInput(node, "@NAME@_home")
        self.ds = talos.DriverStation(node)
        self.target = talos.Target(node, @PASCAL@Target)
        self.state = node.publish(@PASCAL@State, topics.STATE)
        self.failed = False
        self.left_switch = False
        self.push_ticks = 0

    def update(self):
        motor = self.motor
        # The switch reads False when closed, True when open.
        home_connected = self.home.connected
        on_switch = home_connected and not self.home.value
        off_switch = home_connected and self.home.value
        # Zero on the switch once the motor is at rest, so the first move
        # does not start at homing speed.
        if (not motor.zeroed and on_switch
                and abs(motor.velocity) < REST_RPS):
            motor.rezero(0.0)
        zeroed = motor.zeroed
        if zeroed:
            self.push_ticks = 0  # homed: a later re-home pushes from zero
            if off_switch:
                self.left_switch = True
        else:
            self.left_switch = False  # a dropped zero homes and checks again
        # Only a Driver Station that reports disabled clears a failure: not a
        # release, a stale lane or a feedback blip, any of which would
        # otherwise start another push into the stop.
        if self.ds.connected and not self.ds.enabled:
            self.failed = False
            self.push_ticks = 0
        # Still closed after the first move off it: the switch is stuck, or
        # the zero is wrong. Stop before the move reaches HOME_MARGIN_ROT.
        if (zeroed and not self.left_switch
                and motor.position > HOME_MARGIN_ROT - TOLERANCE_ROT):
            self.failed = True
        t = self.target.get()  # None: disabled, stale or released
        connected = motor.connected
        live = (t is not None and connected
                and math.isfinite(t.position_rot) and not self.failed)
        drove = False
        goal = 0.0
        if live and zeroed:
            # Until the switch has opened, no farther than HOME_MARGIN_ROT.
            top = MAX_ROT if self.left_switch else HOME_MARGIN_ROT
            goal = min(max(t.position_rot, 0.0), top)
            motor.set_motion_magic(goal)
            drove = True
        elif live and not on_switch:
            # The push is bounded: a switch that never closes (a cut wire
            # reads open) would otherwise push into the stop while the
            # button is held.
            self.push_ticks += 1
            if self.push_ticks > MAX_PUSH_TICKS:
                # Pushed long enough to cross the travel: no switch.
                self.failed = True
            else:
                motor.set_voltage(HOMING_VOLTS)  # toward the switch
                drove = True
        # Otherwise nothing is set and the flush sends neutral.
        position = motor.position
        at_goal = drove and zeroed and abs(position - goal) < TOLERANCE_ROT
        if self.failed:
            homing = @PASCAL@Homing.FAILED
        elif zeroed:
            homing = @PASCAL@Homing.HOMED
        else:
            homing = @PASCAL@Homing.HOMING
        self.state.send(enabled=drove, position_rot=position,
                        connected=connected, at_goal=at_goal, homing=homing)
'''

PY_TEST = '''\
"""Node tests: the class on talos.testing.Sim, and the binary.

@PASCAL@Test is the node test matrix, docs/TESTING.md §7 in the talos
repository: the case names are the matrix's, which talos's
//tools:check_node_matrix looks for.
"""

import os
import subprocess
import tempfile
import unittest

import talos
@FROM_IMPORTS@

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "testdata", "robot.toml")
node_py = talos.load(os.path.join(HERE, "@NAME@.py"))


class @PASCAL@Test(unittest.TestCase):
    def setUp(self):
        self.sim = Sim(CONFIG)
        self.sim.add(node_py.@PASCAL@, "@NAME@")
        self.hw = SimHardware(self.sim)
        self.hw.set_motor("@NAME@", "@NAME@_motor", 0.0, 12.5)

    def teleop(self, enabled=True):
        """Robot enabled (or not) in teleop: the default policy reads the
        teleop lane. Steps so the mode is current before any lane."""
        self.hw.enable(enabled)
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.01)

    def send(self, demand, source="teleop", mode_since_us=None):
        """One target on a lane, stamped like a producer that has drained
        the newest Driver Station packet, or with `mode_since_us`."""
        self.sim.publish_target(@PASCAL@Target, "@NAME@", source,
                                mode_since_us=mode_since_us,
                                velocity_rps=demand)

    def hold(self, demand, seconds, source="teleop"):
        """Republishes a lane every 20 ms, as a producer does."""
        for _ in range(round(seconds / 0.02)):
            self.send(demand, source)
            self.sim.step(0.02)

    def mode(self):
        """What the node asked for (the flush sends neutral for an unset
        motor)."""
        return self.hw.motor_request("@NAME@", "@NAME@_motor").mode

    def applied(self):
        """What the gateway applies, after its enable and expiry gates."""
        return self.hw.applied_motor("@NAME@", "@NAME@_motor").mode

    def state(self):
        return self.sim.latest(@PASCAL@State, topics.STATE)

    # REQ-SAFE-07
    def test_drives_at_a_live_lane(self):
        self.teleop()
        self.hold(30.0, 0.06)
        request = self.hw.applied_motor("@NAME@", "@NAME@_motor")
        self.assertEqual(request.mode, talos.Mode.VELOCITY)
        self.assertEqual(request.demand, 30.0)
        state = self.state()
        self.assertTrue(state.enabled)
        self.assertEqual(state.velocity_rps, 12.5)
        self.assertTrue(state.connected)

    # REQ-CONTRACT-25: matrix case 2, `state` still published with
    # `enabled` false while disabled. The neutral request is the
    # devices flush's (REQ-SAFE-02, proved in talOS/devices), not this
    # node's evidence.
    def test_disabled_robot_goes_neutral(self):
        self.teleop(enabled=False)
        self.hold(30.0, 0.06)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.applied(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)  # state keeps publishing

    # REQ-CONTRACT-25: matrix case 3, the node's `state` still published,
    # `enabled` false, once its lane is stale. The neutral request is the
    # devices flush's (Target::get returns no target: REQ-SAFE-41, proved in
    # talOS/devices), not this node's evidence.
    def test_stale_lane_goes_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        self.sim.step(0.15)  # past command_timeout_us
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.applied(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)  # state keeps publishing

    # Matrix case 4: the node keeps running when its lane is released. The
    # neutral request is the devices flush's (Target::get returns no
    # target: REQ-SAFE-41, proved in talOS/devices), not this node's evidence.
    def test_released_lane_goes_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        self.sim.release_target(@PASCAL@Target, "@NAME@", "teleop")
        self.sim.step(0.04)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)

    # REQ-CONTRACT-25: matrix case 5, the node's `state` published, `enabled`
    # false, while no target was ever sent. The neutral request is the devices
    # flush's (Target::get returns no target: REQ-SAFE-41, proved in
    # talOS/devices), not this node's evidence.
    def test_never_sent_goes_neutral(self):
        self.teleop()
        self.sim.step(0.06)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)

    # Matrix case 6: the node keeps running once the Driver Station view is
    # stale. The neutral request is the devices flush's (Target::get returns no
    # target: REQ-SAFE-41, proved in talOS/devices), not this node's evidence.
    def test_driver_station_loss_goes_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        self.hw.stop_driver_station()
        self.hold(30.0, 0.16)  # past command_timeout_us: the DS is stale
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)

    # REQ-CONTRACT-25
    def test_disconnected_feedback_goes_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        self.hw.set_connected("@NAME@", "@NAME@_motor", False)
        self.hold(30.0, 0.04)
        # The neighbor's motor keeps the robot enabled (testdata/robot.toml),
        # so the node's own feedback gate is what stops it.
        self.assertTrue(talos.robot_enabled(self.sim.node("@NAME@")))
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        state = self.state()
        self.assertFalse(state.connected)
        self.assertFalse(state.enabled)

    # REQ-SAFE-04
    def test_non_finite_demand_goes_neutral(self):
        self.teleop()
        self.hold(float("nan"), 0.04)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)

    # matrix: n/a 9 (the @NAME@ reads no parameters)

    # REQ-SAFE-07: matrix case 10, the current mode's lane drives the node end
    # to end through its Target; the other mode's lane going neutral is the
    # flush's (REQ-SAFE-41).
    def test_other_modes_lane_is_ignored(self):
        self.teleop()
        self.hold(30.0, 0.04, source="auto")  # autonomous's lane, in teleop
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.sim.release_target(@PASCAL@Target, "@NAME@", "auto")
        self.hw.set_driver_station(enabled=True, autonomous=True)
        self.sim.step(0.02)
        self.hold(30.0, 0.04)  # teleop's lane, live, in autonomous
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.hold(20.0, 0.04, source="auto")
        request = self.hw.motor_request("@NAME@", "@NAME@_motor")
        self.assertEqual(request.mode, talos.Mode.VELOCITY)
        self.assertEqual(request.demand, 20.0)

    # Matrix case 11: the node drives again on a target of the current
    # generation. Refusing the earlier one is Target's and the
    # flush's (REQ-SAFE-09 in talOS/devices/target_test.cc, and REQ-SAFE-41).
    def test_earlier_generation_target_is_ignored(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        before = self.sim.driver_station_generation
        self.hw.set_driver_station(teleop=True)  # disabled
        self.sim.step(0.04)
        # Re-enabled. SimHardware publishes the packet at its next 20 ms hook,
        # which the Sim runs after the node's tick at that instant, so the
        # packet waits for the node's next drain; a target its producer
        # queued under the earlier enable, published before that drain,
        # arrives in the same one.
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.02)
        self.assertNotEqual(self.sim.driver_station_generation, before)
        self.send(30.0, mode_since_us=before)
        self.sim.step(0.02)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        # The same target stamped with the current generation drives at once.
        self.send(30.0)
        self.sim.step(0.02)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)

    # matrix: n/a 12 (a Python tick allocates in the interpreter)


class BinaryTest(unittest.TestCase):
    def run_binary(self, *argv):
        resolver = runfiles.Create()
        binary = resolver.Rlocation(os.environ["NODE_BINARY"])
        env = dict(os.environ, **resolver.EnvVars())
        env.pop("PYTHONPATH", None)
        with tempfile.TemporaryDirectory() as cwd:
            proc = subprocess.run([binary, *argv], cwd=cwd, env=env,
                                  capture_output=True, text=True, timeout=120)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        return proc.stdout

    def test_sim_run_replays_without_divergence(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, "node_test.tlog")
            self.run_binary("--sim", "--duration-s", "1", "--config", CONFIG,
                            "--log", log)
            self.run_binary("--replay", log, "--config", CONFIG)


if __name__ == "__main__":
    unittest.main()
'''

# Replaces PY_TEST's case 9 for a position mechanism.
PY_POSITION_PARAMETERS = '''\
    # matrix: n/a 9 (PositionMechanism parameters, tested in talos)
'''

# Inserted into PY_TEST for a position mechanism before BinaryTest: the
# node's own logic over the mechanism (the config homes it, and the
# mechanism's phase reaches /@NAME@/state). The homing rules themselves are
# talOS/mechanisms' and tested there.
PY_POSITION_TESTS = '''
    # The mechanism library's behaviour. The tests from here to BinaryTest
    # cover what talos.mechanisms.PositionMechanism does for this node: it
    # homes on the switch before the first goal, counts positions from
    # there, clamps goals to min_rot..max_rot and latches FAILED when homing
    # does not move. If you drive the Motor yourself instead, replace them
    # with tests of your own behaviour. The tests above are the node test
    # matrix (docs/TESTING.md §7 in the talos repository) that every actuator
    # node keeps; they expect Motion Magic requests
    # (talos.Mode.MOTION_MAGIC), PositionMechanism's mode, so change that if
    # your node drives another, and replace the matrix's n/a 9 with
    # test_bad_parameter_fails_construction once the node reads parameters of
    # its own.
    #
    # The values are testdata/robot.toml's (homing_v = -1.0,
    # max_rot = 100.0, homing_stall_us = 200000). setUp leaves the motor
    # at rest.

    def set_home(self, closed):
        """Normally open to ground behind the input's pull-up: closed reads
        False."""
        self.hw.set_digital_input("@NAME@", "@NAME@_home", not closed)

    # REQ-SAFE-36
    def test_homes_before_the_first_goal(self):
        self.set_home(False)
        self.hw.ideal_motors()
        self.teleop()
        self.hold(30.0, 0.06)  # toward the switch, not to 30 unhomed
        request = self.hw.motor_request("@NAME@", "@NAME@")
        self.assertEqual(request[:2], (talos.Mode.VOLTAGE, -1.0))
        self.assertEqual(self.state().homing, @PASCAL@Homing.HOMING)

        # The switch closes 0.5 rotations below the power-on position: that
        # is 0, and targets go out in that frame, clamped to the travel.
        self.hw.set_motor("@NAME@", "@NAME@", -0.5, 0.0)
        self.set_home(True)
        self.hold(30.0, 0.06)  # home_samples (3) consecutive closed readings
        self.set_home(False)  # the motor leaves the switch
        self.hold(30.0, 0.02)
        request = self.hw.motor_request("@NAME@", "@NAME@")
        self.assertEqual(request[:2], (talos.Mode.MOTION_MAGIC, 29.5))
        self.assertEqual(self.state().homing, @PASCAL@Homing.HOMED)
        self.hold(1000.0, 0.04)
        self.assertEqual(
            self.hw.motor_request("@NAME@", "@NAME@").demand, 99.5)
        self.hold(-5.0, 0.04)  # position 0
        self.assertEqual(
            self.hw.motor_request("@NAME@", "@NAME@").demand, -0.5)

    # REQ-SAFE-36
    def test_homing_that_does_not_move_fails(self):
        self.set_home(False)
        self.hw.set_motor("@NAME@", "@NAME@", 0.0, 0.0)
        self.teleop()
        self.hold(30.0, 0.1)
        self.assertEqual(self.mode(), talos.Mode.VOLTAGE)
        self.hold(30.0, 0.2)  # past homing_stall_us
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, @PASCAL@Homing.FAILED)
'''

# Replaces PY_POSITION_TESTS for --raw: the node's own homing bounds.
PY_POSITION_RAW_TESTS = '''
    # The node's own homing bounds. The tests above are the node test
    # matrix (docs/TESTING.md §7 in the talos repository) that every
    # actuator node keeps; these guard the rules this node adds. Their
    # times and positions follow the constants in
    # @NAME@.py: MAX_PUSH_TICKS (50 ticks, 1 s),
    # HOME_MARGIN_ROT (0.15) and MAX_ROT (2.0); change them together.
    # setUp starts the motor at rest on its switch.

    def set_home(self, closed):
        """Normally open to ground behind the input's pull-up: closed reads
        False."""
        self.hw.set_digital_input("@NAME@", "@NAME@_home",
                                  not closed)

    def start_off_home(self):
        """Powered on off the switch, unzeroed, and moving when pushed."""
        self.set_home(False)
        self.hw.set_motor("@NAME@", "@NAME@", 0.0, -1.0)

    # REQ-DEV-01
    def test_homing_that_never_reaches_the_switch_fails(self):
        self.start_off_home()  # the switch never closes, as with a cut wire
        self.teleop()
        self.hold(1.0, 0.9)
        # Still pushing toward the switch.
        self.assertEqual(self.mode(), talos.Mode.VOLTAGE)
        self.hold(1.0, 0.2)  # past the 1 s bound
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, @PASCAL@Homing.FAILED)

    # REQ-DEV-01
    def test_failure_latches_until_disabled(self):
        self.start_off_home()
        self.teleop()
        self.hold(1.0, 1.1)
        self.assertEqual(self.state().homing, @PASCAL@Homing.FAILED)
        self.sim.step(0.15)  # the lane goes stale while the button is held
        self.hold(1.0, 0.2)  # live again: no second push
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, @PASCAL@Homing.FAILED)
        self.hw.set_driver_station(teleop=True)  # disabled
        self.sim.step(0.04)
        self.teleop()  # enabled again: a new homing run
        self.hold(1.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VOLTAGE)

    # REQ-DEV-01
    def test_stuck_switch_stops_within_the_margin(self):
        self.hw.ideal_motors()
        # Powered on near the top; the switch, stuck closed, zeroes it there.
        self.hw.set_motor("@NAME@", "@NAME@", 1.9, 0.0)
        self.teleop()
        for _ in range(10):
            self.hold(1.5, 0.02)
            # Never more than HOME_MARGIN_ROT (0.15) above the false zero.
            demand = self.hw.motor_request("@NAME@", "@NAME@").demand
            self.assertLessEqual(demand, 1.9 + 0.15 + 1e-9)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, @PASCAL@Homing.FAILED)

    # REQ-DEV-01: off the switch, a goal past either end of the travel
    # demands that end, MAX_ROT (2.0) or 0.
    def test_goal_outside_the_travel_clamps_to_it(self):
        self.hw.ideal_motors()
        self.teleop()
        self.set_home(False)  # zeroed on the switch, it leaves it
        self.hold(5.0, 0.06)
        demand = self.hw.motor_request("@NAME@", "@NAME@").demand
        self.assertEqual(demand, 2.0)
        self.hold(-1.0, 0.06)
        demand = self.hw.motor_request("@NAME@", "@NAME@").demand
        self.assertEqual(demand, 0.0)

    # REQ-DEV-01: the switch zeroes the motor only once it moves slower
    # than REST_RPS (0.05).
    def test_zero_waits_for_rest(self):
        self.hw.set_motor("@NAME@", "@NAME@", 0.0, -1.0)  # on it, moving
        self.teleop()
        self.hold(1.0, 0.06)
        self.assertEqual(self.state().homing, @PASCAL@Homing.HOMING)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)  # no push on it
        self.hw.set_motor("@NAME@", "@NAME@", 0.0, 0.0)  # at rest
        self.hold(1.0, 0.06)
        self.assertEqual(self.state().homing, @PASCAL@Homing.HOMED)
        self.assertEqual(self.mode(), talos.Mode.MOTION_MAGIC)
'''

PY_POSITION_TESTDATA_ROBOT = """\
# @PASCAL@ test robot: one motor homed on one switch, and a neighbor.
# The @NEIGHBOR@ subsystem's motor stays connected: losing the node's motor
# is one device lost, as on a robot, which only the node's own feedback
# gate stops. Losing every motor and CAN sensor is the gateway's fault.
[robot]
name = "@NAME@_test"
period_us = @PERIOD@
command_timeout_us = 100000

[subsystems.@NAME@.@NAME@]
tolerance_rot = 0.05
min_rot = 0.0
max_rot = 100.0
homing = "limit_switch"
limit_switch = "@NAME@_home"
homing_v = -1.0
homing_timeout_us = 500000
homing_stall_rps = 0.1
homing_stall_us = 200000
home_margin_rot = 0.5

[subsystems.@NAME@.motors.@NAME@]
type = "TalonFX"
bus = "test"
can_id = 1
position_frame = "rezero"
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }

[subsystems.@NAME@.digital_inputs.@NAME@_home]
dio = 0

[subsystems.@NEIGHBOR@.motors.wheel]
type = "TalonFX"
bus = "test"
can_id = 2
"""

PY_POSITION_RAW_TESTDATA_ROBOT = """\
# @PASCAL@ test robot: one motor homed on one switch, and a neighbor.
# The @NEIGHBOR@ subsystem's motor stays connected: losing the node's motor
# is one device lost, as on a robot, which only the node's own feedback
# gate stops. Losing every motor and CAN sensor is the gateway's fault.
[robot]
name = "@NAME@_test"
period_us = @PERIOD@
command_timeout_us = 100000

[subsystems.@NAME@.motors.@NAME@]
type = "TalonFX"
bus = "test"
can_id = 1
position_frame = "rezero"
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }

[subsystems.@NAME@.digital_inputs.@NAME@_home]
dio = 0

[subsystems.@NEIGHBOR@.motors.wheel]
type = "TalonFX"
bus = "test"
can_id = 2
"""

PY_TESTDATA_ROBOT = """\
# @PASCAL@ test robot: one motor, and a neighbor.
# The @NEIGHBOR@ subsystem's motor stays connected: losing the node's motor
# is one device lost, as on a robot, which only the node's own feedback
# gate stops. Losing every motor and CAN sensor is the gateway's fault.
[robot]
name = "@NAME@_test"
period_us = @PERIOD@
command_timeout_us = 100000

[subsystems.@NAME@.motors.@NAME@_motor]
type = "TalonFX"
bus = "test"
can_id = 1
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }

[subsystems.@NEIGHBOR@.motors.wheel]
type = "TalonFX"
bus = "test"
can_id = 2
"""

SUBSYSTEM_TOML = """\
# @PASCAL@ subsystem, generated by //tools:new_subsystem. Parsed via the
# [[subsystems]] manifest in @CONFIG@
# and merged with the other subsystem files into one canonical view before
# validation.
[subsystem]
name = "@NAME@"
node = "//@PACKAGE@:node"
period_us = @PERIOD@

# Zero-code robot wiring: uncomment to have the teleop node write a
# Driver Station button on the /@NAME@/target/teleop lane, without editing any
# C++ or Python files in other packages. Only once slot0 is tuned on the
# mechanism: every config of this robot reads this file. A simulation-only
# binding goes in the simulation-only config new_subsystem's `Next:` names.
# [[teleop.bindings]]
# button = @BUTTON@
# while_held = { velocity_rps = 20.0 }

# Which /@NAME@/target/<source> lanes drive the @NAME@ in each Driver Station
# mode, highest priority first. Without this table [robot.targets] applies
# (teleop = ["teleop"], autonomous = ["auto"]); listed modes replace it.
# [targets]
# teleop = ["teleop"]
# autonomous = ["auto"]

# Device names are robot-wide, so the key carries the subsystem's name.
[motors.@NAME@_motor]
type = "TalonFX"
bus = "rio"
# PLACEHOLDER: the lowest id the robot config does not use yet, not the
# motor's.
# Replace it with the physical CAN id before the robot drives this motor.
can_id = @CAN_ID@
# Flip `inverted` if a positive velocity turns the mechanism the wrong way;
# `brake` holds it at stop, coast lets it spin down freely.
inverted = false
brake = true
supply_limit_a = 40.0
stator_limit_a = 80.0
max_voltage = 12.0
sensor_to_mechanism_ratio = 1.0
max_velocity_rps = 100.0
# Velocity-loop gains (slot 0). ALL ZERO means UNTUNED: the motor will not
# follow its demand. Tune v (volts per rps) until measured ~= requested, then
# p until the step response stops lagging, then s to break static friction.
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }
"""

POSITION_SUBSYSTEM_TOML = """\
# @PASCAL@ subsystem, generated by //tools:new_subsystem. Parsed via the
# [[subsystems]] manifest in @CONFIG@
# and merged with the other subsystem files into one canonical view before
# validation.
[subsystem]
name = "@NAME@"
node = "//@PACKAGE@:node"
period_us = @PERIOD@

# The @NAME@'s talos::mechanisms::PositionMechanism parameters (keys and
# units: talOS/mechanisms/README.md in the talos repository).
[@NAME@]
# /@NAME@/state reports at_goal while the motor is within this of its target.
tolerance_rot = 0.05
# Travel, in rotations from the home switch: targets clamp to
# min_rot..max_rot. PLACEHOLDER: this confines every target to within 0.001
# rotations of the switch until a human measures the travel. Nothing stops
# the mechanism past the top but this clamp; see "Overtravel" below.
min_rot = 0.0
max_rot = 0.001
# Homing drives at homing_v volts (negative: toward the switch at min_rot)
# while unzeroed and a target is live. It gives up, neutral, after
# homing_timeout_us, or once the motor has moved slower than homing_stall_rps
# for homing_stall_us. PLACEHOLDER: check the direction (`inverted`) before
# the first enable, then set a timeout a little longer than the slowest
# full-travel move at that voltage.
homing = "limit_switch"
limit_switch = "@NAME@_home"
homing_v = -0.5
homing_timeout_us = 3000000
homing_stall_rps = 0.05
homing_stall_us = 500000
# The switch reading closed with the zeroed motor more than this from it is a
# stuck or miswired switch: the @NAME@ stops, neutral.
home_margin_rot = 0.5
# The switch zeroes the motor only after reading closed on this many
# consecutive ticks, so one glitched sample mid-travel does not.
home_samples = 3

# Zero-code robot wiring: uncomment to have the teleop node write a
# Driver Station button on the /@NAME@/target/teleop lane, without editing any
# C++ or Python files in other packages. Only once slot0 is tuned on the
# mechanism: every config of this robot reads this file. A simulation-only
# binding goes in the simulation-only config new_subsystem's `Next:` names.
# [[teleop.bindings]]
# button = @BUTTON@
# while_held = { position_rot = 0.0 }

# Which /@NAME@/target/<source> lanes drive the @NAME@ in each Driver Station
# mode, highest priority first. Without this table [robot.targets] applies
# (teleop = ["teleop"], autonomous = ["auto"]); listed modes replace it.
# [targets]
# teleop = ["teleop"]
# autonomous = ["auto"]

# Device names are robot-wide, so the motor is named after the subsystem.
[motors.@NAME@]
type = "TalonFX"
bus = "rio"
# PLACEHOLDER: the lowest id the robot config does not use yet, not the
# motor's.
# Replace it with the physical CAN id before the robot drives this motor.
can_id = @CAN_ID@
# Flip `inverted` if a positive position moves the mechanism the wrong way;
# `brake` resists motion while no target drives it, coast lets it move freely.
inverted = false
brake = true
supply_limit_a = 40.0
stator_limit_a = 80.0
# homing_v may be at most half of this.
max_voltage = 12.0
# Sensor rotations per mechanism rotation: positions, the profile and the
# travel above are in mechanism rotations.
sensor_to_mechanism_ratio = 1.0
max_velocity_rps = 100.0
# The Motion Magic profile every move follows (cruise at most
# max_velocity_rps); raise it once the mechanism is tuned.
cruise_velocity_rps = 1.0
acceleration_rps2 = 2.0
# Positions count from the home switch, not from where the motor powered on:
# Motion Magic goals go out neutral until the switch zeroes the motor and it
# has come to rest, and again after a motor reset or RoboRIO reboot drops that zero. Only a remote
# CANcoder's frame may be "absolute" (usable unzeroed, surviving a reset).
position_frame = "rezero"
# No hardware soft limits: the TalonFX applies them to its power-on position
# in every control mode, so after a reset partway along the travel they sit
# in the wrong place and can stop homing; config load refuses them unless
# position_frame = "absolute". max_rot bounds targets.
# Overtravel: that clamp is the only top-of-travel protection in software. At
# the robot, wire a forward limit switch to the TalonFX's own limit input,
# which stops forward output in every control mode whatever the frame, and
# set forward_limit_switch = "NormallyOpen" (Phoenix's default) or
# "NormallyClosed" to match it. talOS cannot move soft-limit thresholds after
# a rezero.
soft_limits = false
# Position gains (slot 0). ALL ZERO means UNTUNED: the motor will not follow
# its target. Tune g to hold the mechanism against gravity (add
# gravity_type = "Arm" for a pivot), s to break static friction, v and a
# until it tracks the profile, then p until it settles within tolerance_rot.
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }

# The home switch at the reverse end of travel (position 0): normally open,
# between the DIO signal pin and ground, so the input's pull-up reads true off
# the switch and false on it. A cut or unplugged wire reads "not home": homing
# then times out or stalls instead of zeroing mid-travel. PLACEHOLDER: the
# lowest DIO channel the robot config does not use yet, not the switch's.
# Replace it with the physical channel.
[digital_inputs.@NAME@_home]
dio = @DIO@
"""

# --type position --raw: the travel, push and margin are constants in the
# node's own file, so the subsystem file has no [<name>] table.
POSITION_RAW_SUBSYSTEM_TOML = """\
# @PASCAL@ subsystem, generated by //tools:new_subsystem. Parsed via the
# [[subsystems]] manifest in @CONFIG@
# and merged with the other subsystem files into one canonical view before
# validation. The @NAME@'s travel, homing push and switch margin are
# constants in its node (kMaxRot, kHomingVolts, kMaxPushTicks,
# kHomeMarginRot in C++; MAX_ROT, ... in Python), not keys here.
[subsystem]
name = "@NAME@"
node = "//@PACKAGE@:node"
period_us = @PERIOD@

# Zero-code robot wiring: uncomment to have the teleop node write a
# Driver Station button on the /@NAME@/target/teleop lane, without editing any
# C++ or Python files in other packages. Only once slot0 is tuned on the
# mechanism: every config of this robot reads this file. A simulation-only
# binding goes in the simulation-only config new_subsystem's `Next:` names.
# [[teleop.bindings]]
# button = @BUTTON@
# while_held = { position_rot = 0.0 }

# Which /@NAME@/target/<source> lanes drive the @NAME@ in each Driver Station
# mode, highest priority first. Without this table [robot.targets] applies
# (teleop = ["teleop"], autonomous = ["auto"]); listed modes replace it.
# [targets]
# teleop = ["teleop"]
# autonomous = ["auto"]

# Device names are robot-wide, so the motor is named after the subsystem.
[motors.@NAME@]
type = "TalonFX"
bus = "rio"
# PLACEHOLDER: the lowest id the robot config does not use yet, not the
# motor's.
# Replace it with the physical CAN id before the robot drives this motor.
can_id = @CAN_ID@
# Flip `inverted` if a positive position moves the mechanism the wrong way;
# `brake` resists motion while no target drives it, coast lets it move freely.
inverted = false
brake = true
supply_limit_a = 40.0
# How hard any command, the homing push included, can drive the mechanism
# into a stop. PLACEHOLDER: a provisional low value, not the motor's 80 A
# limit; choose the real one for your mechanism.
stator_limit_a = 20.0
max_voltage = 12.0
# Sensor rotations per mechanism rotation: positions, the profile and the
# node's travel are in mechanism rotations.
sensor_to_mechanism_ratio = 1.0
max_velocity_rps = 100.0
# The Motion Magic profile every move follows (cruise at most
# max_velocity_rps); raise it once the mechanism is tuned.
cruise_velocity_rps = 1.0
acceleration_rps2 = 2.0
# Positions count from the home switch, not from where the motor powered on:
# Motion Magic goals go out neutral until the node zeroes the motor on the
# switch (Motor::rezero), and again after a motor reset or RoboRIO reboot
# drops that zero.
position_frame = "rezero"
# No hardware soft limits: the TalonFX applies them to its power-on position
# in every control mode, which a rezero does not move; config load refuses
# them unless position_frame = "absolute". The node's kMaxRot bounds goals.
# Overtravel: that clamp is the only top-of-travel protection in software. At
# the robot, wire a forward limit switch to the TalonFX's own limit input,
# which stops forward output in every control mode whatever the frame, and
# set forward_limit_switch = "NormallyOpen" (Phoenix's default) or
# "NormallyClosed" to match it.
soft_limits = false
# Position gains (slot 0). ALL ZERO means UNTUNED: the motor will not follow
# its target. Tune g to hold the mechanism against gravity (add
# gravity_type = "Arm" for a pivot), s to break static friction, v and a
# until it tracks the profile, then p until it settles at the goal.
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }

# The home switch at the reverse end of travel (position 0): normally open,
# between the DIO signal pin and ground, so the input's pull-up reads true off
# the switch and false on it. A cut or unplugged wire reads "not home": the
# homing push then fails at its bound instead of zeroing mid-travel.
# PLACEHOLDER: the lowest DIO channel the robot config does not use yet, not
# the switch's. Replace it with the physical channel.
[digital_inputs.@NAME@_home]
dio = @DIO@
"""

MANIFEST_ENTRY = """\

[[subsystems]]
name = "@NAME@"
path = "subsystems/@NAME@.toml"
"""

STANDALONE_MODULE = """\
module(
    name = "@NAME@",
    version = "0.1.0",
)

bazel_dep(name = "talos", version = "0.1.0")
@EXTRA_DEPS@"""

STANDALONE_DEFAULTS_TOML = """\
# @PASCAL@ subsystem defaults, shipped by the @@NAME@ repository.
# The robot's own subsystem file starts as a copy of this one, with the
# motor's physical can_id added; inversions and tuned slot0 gains go there.
[subsystem]
name = "@NAME@"
node = "@@NAME@//:node"
period_us = @PERIOD@

[motors.@NAME@_motor]
type = "TalonFX"
bus = "rio"
inverted = false
brake = true
supply_limit_a = 40.0
stator_limit_a = 80.0
max_voltage = 12.0
sensor_to_mechanism_ratio = 1.0
max_velocity_rps = 100.0
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }
"""

STANDALONE_POSITION_DEFAULTS_TOML = """\
# @PASCAL@ subsystem defaults, shipped by the @@NAME@ repository.
# The robot's own subsystem file starts as a copy of this one, with the
# motor's physical can_id and the home switch's dio added; inversions, the
# measured travel (max_rot), the homing voltage and tuned slot0 gains go
# there. Until then the travel confines every target to within 0.001
# rotations of the home switch. The motor requires a zero from the home switch
# (normally open, to ground) and has no hardware soft limits; see the node's
# header comment and talOS/mechanisms/README.md in the talos repository for
# the [@NAME@] keys.
[subsystem]
name = "@NAME@"
node = "@@NAME@//:node"
period_us = @PERIOD@

[@NAME@]
tolerance_rot = 0.05
min_rot = 0.0
max_rot = 0.001
homing = "limit_switch"
limit_switch = "@NAME@_home"
homing_v = -0.5
homing_timeout_us = 3000000
homing_stall_rps = 0.05
homing_stall_us = 500000
home_margin_rot = 0.5
home_samples = 3

[motors.@NAME@]
type = "TalonFX"
bus = "rio"
inverted = false
brake = true
supply_limit_a = 40.0
stator_limit_a = 80.0
max_voltage = 12.0
sensor_to_mechanism_ratio = 1.0
max_velocity_rps = 100.0
cruise_velocity_rps = 1.0
acceleration_rps2 = 2.0
position_frame = "rezero"
soft_limits = false
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }

# The home switch, normally open to ground; the robot's file adds its dio.
[digital_inputs.@NAME@_home]
"""

STANDALONE_POSITION_RAW_DEFAULTS_TOML = """\
# @PASCAL@ subsystem defaults, shipped by the @@NAME@ repository.
# The robot's own subsystem file starts as a copy of this one, with the
# motor's physical can_id and the home switch's dio added; inversions, the
# stator limit and tuned slot0 gains go there. The travel, homing push and
# switch margin are constants in the node (@NAME@.h or @NAME@.py): set them
# for the mechanism. The motor requires a zero from the home switch
# (normally open, to ground) and has no hardware soft limits; see the node's
# header comment.
[subsystem]
name = "@NAME@"
node = "@@NAME@//:node"
period_us = @PERIOD@

[motors.@NAME@]
type = "TalonFX"
bus = "rio"
inverted = false
brake = true
supply_limit_a = 40.0
stator_limit_a = 20.0
max_voltage = 12.0
sensor_to_mechanism_ratio = 1.0
max_velocity_rps = 100.0
cruise_velocity_rps = 1.0
acceleration_rps2 = 2.0
position_frame = "rezero"
soft_limits = false
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }

# The home switch, normally open to ground; the robot's file adds its dio.
[digital_inputs.@NAME@_home]
"""

# ---------------------------------------------------- estimator archetype

ESTIMATOR_MESSAGES_BUILD = """\
# The @NAME@'s topic. Its state is the shared geometry.PoseEstimate and its
# camera inputs are geometry.PoseObservation (//talOS/msgs:geometry), so the
# package declares no schema of its own. Consumers find the pose through
# config (telemetry's pose_topic), not through this package.
talos_messages(
    name = "messages",
    srcs = [],
    include_prefix = "@NAME@",
    topics = {
        "state": "/@NAME@/state",
    },
    deps = ["//talOS/msgs:geometry"],
)
"""

ESTIMATOR_DOC = """\
Reads geometry.PoseObservation from each camera node listed in `cameras`
(/<camera>/state) and publishes geometry.PoseEstimate on its own state
topic every tick. Each camera is read through a callback subscription,
which delivers every observation in sequence (a latest-value read would lose
all but the newest per tick); the callback queues at most 64 per camera (the
oldest is dropped and counted as rejected) and update() checks every queued
one. Only update() reads the queues, so the callback waits for the drain
before it (at-drain) instead of waking the node for every frame. As
generated it follows the most certain usable observation of the tick and is
valid while one arrived within observation_timeout_us; fuse odometry here
(for example wpimath's SwerveDrive4PoseEstimator or a GTSAM smoother, from
talOS's Python requirements). It drives nothing, so it runs whether the
robot is enabled or not."""

CC_ESTIMATOR_HEADER = """\
#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

@INCLUDES@

namespace @NAME@ {

// A pose estimator.
@DOC@
class @PASCAL@ {
 public:
  using State = geometry::PoseEstimate;
  using Observation = geometry::PoseObservation;
  // Observations queued per camera between two update()s: the framework's
  // per-topic, per-drain delivery cap, over a second of a 60 fps camera.
  static constexpr std::size_t kMaxPendingObservations = 64;

  // The camera callbacks capture `this`: the node stays where it was built.
  @PASCAL@(const @PASCAL@&) = delete;
  @PASCAL@& operator=(const @PASCAL@&) = delete;
  @PASCAL@(@PASCAL@&&) = delete;
  @PASCAL@& operator=(@PASCAL@&&) = delete;

  explicit @PASCAL@(talos::Node& node)
      : node_{node},
        timeout_{std::chrono::microseconds{
            node.param<std::int64_t>("observation_timeout_us", 500'000)}},
        state_{node.publish<State>(topics::kState)} {
    if (timeout_ <= talos::Duration::zero()) {
      throw std::invalid_argument(
          "@NAME@: observation_timeout_us must be positive");
    }
    const auto names = node.param<std::vector<std::string>>("cameras");
    pending_.resize(names.size());
    for (std::size_t i = 0; i < names.size(); ++i) {
      // The callback sees every observation in sequence. Only update() reads
      // the queue, so kAtDrain delivers them at the drain before update()
      // instead of waking the node for each camera frame.
      cameras_.push_back(node.subscribe<Observation>(
          "/" + names[i] + "/state",
          [this, i](const Observation& o) { pending_[i].Push(o, rejected_); },
          talos::Wake::kAtDrain, {.optional = true}));
    }
  }

  void update() {
    std::optional<Observation> best;
    for (Pending& queue : pending_) {
      for (std::size_t k = 0; k < queue.count; ++k) {  // each counts once
        const Observation& o = queue.At(k);
        if (!Usable(o)) {
          ++rejected_;
          continue;
        }
        ++accepted_;
        if (!best.has_value() ||
            o.x_std_m() + o.y_std_m() < best->x_std_m() + best->y_std_m()) {
          best = o;
        }
      }
      queue.count = 0;
    }
    if (best.has_value()) {
      pose_ = best->pose();
      sample_time_ns_ = best->sample_time_ns();
      last_accepted_ = node_.now();
    }
    const bool valid =
        last_accepted_.has_value() && node_.now() - *last_accepted_ <= timeout_;
    state_.send(State{sample_time_ns_, pose_, geometry::Twist2d{}, valid, false,
                      accepted_, rejected_});
  }

 private:
  // A fixed ring of the newest observations, oldest first: no allocation
  // after construction.
  struct Pending {
    std::array<Observation, kMaxPendingObservations> slots{};
    std::size_t head = 0;  // index of the oldest
    std::size_t count = 0;

    const Observation& At(std::size_t k) const {
      return slots[(head + k) % kMaxPendingObservations];
    }
    void Push(const Observation& o, std::uint32_t& rejected) {
      if (count == kMaxPendingObservations) {  // drop the oldest
        head = (head + 1) % kMaxPendingObservations;
        --count;
        ++rejected;
      }
      slots[(head + count) % kMaxPendingObservations] = o;
      ++count;
    }
  };

  static bool Usable(const Observation& o) {
    for (const double v : {o.pose().x_m(), o.pose().y_m(), o.pose().yaw_rad(),
                           o.x_std_m(), o.y_std_m(), o.yaw_std_rad()}) {
      if (!std::isfinite(v)) return false;
    }
    return o.valid() && o.sample_time_ns() > 0 && o.x_std_m() > 0 &&
           o.y_std_m() > 0 && o.yaw_std_rad() > 0;
  }

  talos::Node& node_;
  talos::Duration timeout_;
  std::vector<talos::Subscription<Observation>> cameras_;
  std::vector<Pending> pending_;  // per camera, in sequence
  talos::Publisher<State> state_;
  geometry::Pose2d pose_{};  // held while no usable observation arrives
  std::uint64_t sample_time_ns_{0};
  std::optional<talos::Time> last_accepted_;
  std::uint32_t accepted_{0};
  std::uint32_t rejected_{0};
};

}  // namespace @NAME@
"""

CC_ESTIMATOR_TEST = """\
#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

@INCLUDES@

namespace @NAME@ {
namespace {

using namespace std::chrono_literals;
using talos::testing::AllocationProbe;
using talos::testing::Sim;
using talos::testing::SimHardware;

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"runfile unavailable: "} + env + " " +
                             error);
  }
  return runfiles->Rlocation(location);
}

// The node test matrix for an observer, docs/TESTING.md §7 in the talos
// repository: the case names are the matrix's, which talos's
// //tools:check_node_matrix looks for.
class @PASCAL@Test : public ::testing::Test {
 protected:
  using State = @PASCAL@::State;
  using Observation = @PASCAL@::Observation;

  // A camera's observation of (x, y), taken now.
  void Observe(const char* camera, double x, double y, bool valid = true,
               double std_m = 0.1) {
    sim.publish(
        std::string{"/"} + camera + "/state",
        Observation{static_cast<std::uint64_t>(sim.now().nanos()),
                    geometry::Pose2d{x, y, 0.5}, std_m, std_m, 0.05, 2, valid});
  }
  State Latest() const {
    const auto state = sim.latest<State>(topics::kState);
    EXPECT_TRUE(state.has_value());
    return state.value_or(State{});
  }

  Sim sim{Runfile("TALOS_TEST_CONFIG")};
  @PASCAL@& node = sim.add<@PASCAL@>("@NAME@");
  SimHardware hw{sim};
};

TEST_F(@PASCAL@Test, PublishesFromALiveInput) {
  sim.step(20ms);
  Observe("front_camera", 1.0, 2.0);
  sim.step(40ms);
  const State state = Latest();
  EXPECT_TRUE(state.valid());
  EXPECT_DOUBLE_EQ(state.pose().x_m(), 1.0);
  EXPECT_DOUBLE_EQ(state.pose().y_m(), 2.0);
  EXPECT_EQ(state.observation_accepted_count(), 1u);  // counted once
}

TEST_F(@PASCAL@Test, EstimatesWhileTheRobotIsDisabled) {
  hw.enable(false);  // nothing here is actuation-gated
  sim.step(20ms);
  Observe("back_camera", 3.0, 1.0);
  sim.step(40ms);
  EXPECT_TRUE(Latest().valid());
  EXPECT_DOUBLE_EQ(Latest().pose().x_m(), 3.0);
}

// REQ-CONTRACT-25
TEST_F(@PASCAL@Test, StaleInputClearsValidity) {
  sim.step(20ms);
  Observe("front_camera", 1.0, 2.0);
  sim.step(40ms);
  ASSERT_TRUE(Latest().valid());
  sim.step(600ms);  // past observation_timeout_us
  const State state = Latest();
  EXPECT_FALSE(state.valid());
  EXPECT_DOUBLE_EQ(state.pose().x_m(), 1.0);
}

TEST_F(@PASCAL@Test, RejectsUnusableObservations) {
  sim.step(20ms);
  Observe("front_camera", 1.0, 2.0, /*valid=*/false);
  sim.step(40ms);
  Observe("front_camera", 1.0, 2.0, true,
          std::numeric_limits<double>::quiet_NaN());
  sim.step(40ms);
  const State state = Latest();
  EXPECT_FALSE(state.valid());
  EXPECT_EQ(state.observation_rejected_count(), 2u);
  EXPECT_EQ(state.observation_accepted_count(), 0u);
}

TEST_F(@PASCAL@Test, ReadsOnlyTheListedCameras) {
  sim.step(20ms);
  Observe("side_camera", 1.0, 2.0);  // not in `cameras`
  sim.step(40ms);
  EXPECT_FALSE(Latest().valid());
  EXPECT_EQ(Latest().observation_accepted_count(), 0u);
}

TEST_F(@PASCAL@Test, CountsEveryObservationBetweenTicks) {
  sim.step(20ms);
  Observe("front_camera", 1.0, 2.0);
  Observe("front_camera", 1.0, 2.0, /*valid=*/false);
  Observe("front_camera", 1.5, 2.0);
  sim.step(40ms);
  const State state = Latest();
  EXPECT_EQ(state.observation_accepted_count(), 2u);
  EXPECT_EQ(state.observation_rejected_count(), 1u);
}

// REQ-CONTRACT-18
TEST(@PASCAL@Parameters, BadParameterFailsConstruction) {
  const struct {
    const char* key;
    const char* settings;  // [subsystems.@NAME@]
  } rows[] = {
      {"cameras", "observation_timeout_us = 500000"},
      {"cameras", "cameras = 3"},
      {"observation_timeout_us",
       "cameras = ['front_camera']\\nobservation_timeout_us = 0"},
  };
  for (const auto& row : rows) {
    SCOPED_TRACE(row.settings);
    Sim sim = Sim::FromToml(
        std::string{"[robot]\\nperiod_us = 20000\\ncommand_timeout_us = 100000\\n"
                    "[subsystems.@NAME@]\\n"} +
        row.settings +
        "\\n[subsystems.chassis.motors.chassis_motor]\\n"
        "type = 'TalonFX'\\ncan_id = 1\\n");
    try {
      sim.add<@PASCAL@>("@NAME@");
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string{error.what()}.find(row.key), std::string::npos)
          << error.what();
    }
  }
}

// REQ-RT-06: a tick that drains a queued observation, picks it and publishes
// the estimate allocates nothing.
TEST_F(@PASCAL@Test, UpdateDoesNotAllocate) {
  if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  sim.step(40ms);
  Observe("front_camera", 1.0, 2.0);
  AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(Latest().observation_accepted_count(), 1u);
}

struct Result {
  int status;
  std::string output;
};

Result RunNode(const std::string& args) {
  const std::string command = "'" + Runfile("TALOS_TEST_NODE") + "' " + args +
                              " --config '" + Runfile("TALOS_TEST_CONFIG") +
                              "' 2>&1";
  FILE* pipe = ::popen(command.c_str(), "r");
  if (pipe == nullptr) throw std::runtime_error("popen failed");
  Result result{0, {}};
  char buffer[4096];
  while (const std::size_t n = std::fread(buffer, 1, sizeof buffer, pipe)) {
    result.output.append(buffer, n);
  }
  const int status = ::pclose(pipe);
  result.status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return result;
}

TEST(@PASCAL@Binary, LoggedSimRunReplaysWithoutDivergence) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string log =
      std::string{tmp != nullptr ? tmp : "/tmp"} + "/node_test.tlog";
  const Result run = RunNode("--sim --duration-s 1 --log '" + log + "'");
  ASSERT_EQ(run.status, 0) << run.output;
  const Result replay = RunNode("--replay '" + log + "'");
  EXPECT_EQ(replay.status, 0) << replay.output;
  EXPECT_NE(replay.output.find("diverged=0 clean_exit=1"), std::string::npos)
      << replay.output;
}

}  // namespace
}  // namespace @NAME@
"""

PY_ESTIMATOR_NODE = '''\
"""@PASCAL@: a pose estimator.

@DOC@
"""

import collections
import math

import talos
from @NAME@ import topics
from geometry.PoseEstimate import PoseEstimate
from geometry.PoseObservation import PoseObservation


def usable(o):
    values = (o.pose_x_m, o.pose_y_m, o.pose_yaw_rad, o.x_std_m, o.y_std_m,
              o.yaw_std_rad)
    return (o.valid and o.sample_time_ns > 0
            and all(map(math.isfinite, values))
            and min(o.x_std_m, o.y_std_m, o.yaw_std_rad) > 0)


# Observations queued per camera between two update()s: the framework's
# per-topic, per-drain delivery cap, over a second of a 60 fps camera.
MAX_PENDING_OBSERVATIONS = 64


class @PASCAL@:
    def __init__(self, node):
        self.node = node
        cameras = node.param("cameras")
        if (not isinstance(cameras, list)
                or not all(isinstance(c, str) and c for c in cameras)):
            raise ValueError(f"@NAME@: cameras must list camera node names, "
                             f"got {cameras!r}")
        # The callback sees every observation in sequence. Only update()
        # reads the queues, so AT_DRAIN delivers them at the drain before
        # update() instead of waking the node for each camera frame.
        self.pending = [collections.deque() for _ in cameras]
        for camera, queue in zip(cameras, self.pending):
            node.subscribe(PoseObservation, f"/{camera}/state",
                           callback=lambda o, q=queue: self.queue(q, o),
                           wake=talos.Wake.AT_DRAIN, optional=True)
        self.timeout_s = node.param("observation_timeout_us", 500_000) / 1e6
        if not self.timeout_s > 0:
            raise ValueError(
                "@NAME@: observation_timeout_us must be positive")
        self.state = node.publish(PoseEstimate, topics.STATE)
        self.pose = (0.0, 0.0, 0.0)  # held while no usable observation arrives
        self.sample_time_ns = 0
        self.last_accepted_s = None
        self.accepted = self.rejected = 0

    def queue(self, pending, observation):
        if len(pending) >= MAX_PENDING_OBSERVATIONS:
            pending.popleft()
            self.rejected += 1
        pending.append(observation)

    def update(self):
        best = None
        for pending in self.pending:
            for o in pending:  # each observation counts once
                if not usable(o):
                    self.rejected += 1
                    continue
                self.accepted += 1
                if best is None or (o.x_std_m + o.y_std_m
                                    < best.x_std_m + best.y_std_m):
                    best = o
            pending.clear()
        if best is not None:
            self.pose = (best.pose_x_m, best.pose_y_m, best.pose_yaw_rad)
            self.sample_time_ns = best.sample_time_ns
            self.last_accepted_s = self.node.now
        valid = (self.last_accepted_s is not None
                 and self.node.now - self.last_accepted_s <= self.timeout_s)
        x, y, yaw = self.pose
        self.state.send(sample_time_ns=self.sample_time_ns, pose_x_m=x,
                        pose_y_m=y, pose_yaw_rad=yaw, velocity_vx_mps=0.0,
                        velocity_vy_mps=0.0, velocity_omega_radps=0.0,
                        valid=valid, localized=False,
                        observation_accepted_count=self.accepted,
                        observation_rejected_count=self.rejected)
'''

PY_ESTIMATOR_TEST = '''\
"""Node tests: the class on talos.testing.Sim, and the binary.

@PASCAL@Test is the node test matrix for an observer, docs/TESTING.md §7 in
the talos repository: the case names are the matrix's, which talos's
//tools:check_node_matrix looks for.
"""

import math
import os
import subprocess
import tempfile
import unittest

import talos
@FROM_IMPORTS@

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "testdata", "robot.toml")
node_py = talos.load(os.path.join(HERE, "@NAME@.py"))


class @PASCAL@Test(unittest.TestCase):
    def setUp(self):
        self.sim = Sim(CONFIG)
        self.sim.add(node_py.@PASCAL@, "@NAME@")
        self.hw = SimHardware(self.sim)

    def observe(self, camera, x, y, valid=True, std_m=0.1):
        """A camera's observation of (x, y), taken now."""
        self.sim.publish(
            PoseObservation, f"/{camera}/state",
            sample_time_ns=self.sim.now_ns, pose_x_m=x, pose_y_m=y,
            pose_yaw_rad=0.5, x_std_m=std_m, y_std_m=std_m, yaw_std_rad=0.05,
            tag_count=2, valid=valid)

    def state(self):
        return self.sim.latest(PoseEstimate, topics.STATE)

    def test_publishes_from_a_live_input(self):
        self.sim.step(0.02)
        self.observe("front_camera", 1.0, 2.0)
        self.sim.step(0.04)
        state = self.state()
        self.assertTrue(state.valid)
        self.assertEqual((state.pose_x_m, state.pose_y_m), (1.0, 2.0))
        self.assertEqual(state.observation_accepted_count, 1)  # counted once

    def test_estimates_while_the_robot_is_disabled(self):
        self.hw.enable(False)  # nothing here is actuation-gated
        self.sim.step(0.02)
        self.observe("back_camera", 3.0, 1.0)
        self.sim.step(0.04)
        self.assertTrue(self.state().valid)
        self.assertEqual(self.state().pose_x_m, 3.0)

    # REQ-CONTRACT-25
    def test_stale_input_clears_validity(self):
        self.sim.step(0.02)
        self.observe("front_camera", 1.0, 2.0)
        self.sim.step(0.04)
        self.assertTrue(self.state().valid)
        self.sim.step(0.6)  # past observation_timeout_us
        self.assertFalse(self.state().valid)
        self.assertEqual(self.state().pose_x_m, 1.0)

    def test_rejects_unusable_observations(self):
        self.sim.step(0.02)
        self.observe("front_camera", 1.0, 2.0, valid=False)
        self.sim.step(0.04)
        self.observe("front_camera", 1.0, 2.0, std_m=math.nan)
        self.sim.step(0.04)
        state = self.state()
        self.assertFalse(state.valid)
        self.assertEqual((state.observation_accepted_count,
                          state.observation_rejected_count), (0, 2))

    def test_reads_only_the_listed_cameras(self):
        self.sim.step(0.02)
        self.observe("side_camera", 1.0, 2.0)  # not in `cameras`
        self.sim.step(0.04)
        self.assertFalse(self.state().valid)
        self.assertEqual(self.state().observation_accepted_count, 0)

    def test_counts_every_observation_between_ticks(self):
        self.sim.step(0.02)
        self.observe("front_camera", 1.0, 2.0)
        self.observe("front_camera", 1.0, 2.0, valid=False)
        self.observe("front_camera", 1.5, 2.0)
        self.sim.step(0.04)
        state = self.state()
        self.assertEqual((state.observation_accepted_count,
                          state.observation_rejected_count), (2, 1))

    # REQ-CONTRACT-18
    def test_bad_parameter_fails_construction(self):
        for key, settings in (
                ("cameras", "observation_timeout_us = 500000"),
                ("cameras", "cameras = 3"),
                ("observation_timeout_us",
                 'cameras = ["front_camera"]\\nobservation_timeout_us = 0')):
            with self.subTest(settings=settings):
                sim = Sim.from_toml(
                    "[robot]\\nperiod_us = 20000\\n"
                    "command_timeout_us = 100000\\n"
                    f"[subsystems.@NAME@]\\n{settings}\\n"
                    "[subsystems.chassis.motors.chassis_motor]\\n"
                    'type = "TalonFX"\\ncan_id = 1\\n')
                with self.assertRaisesRegex((ValueError, talos.TalosError),
                                            key):
                    sim.add(node_py.@PASCAL@, "@NAME@")

    # matrix: n/a 12 (a Python tick allocates in the interpreter)


class BinaryTest(unittest.TestCase):
    def run_binary(self, *argv):
        resolver = runfiles.Create()
        binary = resolver.Rlocation(os.environ["NODE_BINARY"])
        env = dict(os.environ, **resolver.EnvVars())
        env.pop("PYTHONPATH", None)
        with tempfile.TemporaryDirectory() as cwd:
            proc = subprocess.run([binary, *argv], cwd=cwd, env=env,
                                  capture_output=True, text=True, timeout=120)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        return proc.stdout

    def test_sim_run_replays_without_divergence(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, "node_test.tlog")
            self.run_binary("--sim", "--duration-s", "1", "--config", CONFIG,
                            "--log", log)
            self.run_binary("--replay", log, "--config", CONFIG)


if __name__ == "__main__":
    unittest.main()
'''

ESTIMATOR_TESTDATA_ROBOT = """\
# @PASCAL@ test robot: the estimator and two cameras (the tests publish
# their observations).
[robot]
name = "@NAME@_test"
period_us = @PERIOD@
command_timeout_us = 100000

[subsystems.@NAME@]
period_us = @PERIOD@
cameras = ["front_camera", "back_camera"]
observation_timeout_us = 500000

# A robot config needs one motor; this stands in for the robot the
# estimator rides on.
[subsystems.chassis.motors.chassis_motor]
type = "TalonFX"
bus = "test"
can_id = 1
"""

ESTIMATOR_SUBSYSTEM_TOML = """\
# @PASCAL@ pose estimator, generated by //tools:new_subsystem. Parsed via the
# [[subsystems]] manifest in @CONFIG@.
# It owns no devices. Point consumers at it with config only, e.g.
# [studio] pose_topic = "/@NAME@/state".
[subsystem]
name = "@NAME@"
node = "//@PACKAGE@:node"
period_us = @PERIOD@
# Camera nodes, each publishing geometry.PoseObservation on /<name>/state.
cameras = []
# The estimate is invalid once no usable observation arrived for this long.
observation_timeout_us = 500000
"""

STANDALONE_ESTIMATOR_DEFAULTS_TOML = """\
# @PASCAL@ pose estimator defaults, shipped by the @@NAME@ repository.
[subsystem]
name = "@NAME@"
node = "@@NAME@//:node"
period_us = @PERIOD@
cameras = []
observation_timeout_us = 500000
"""


def estimator_files(name, lang):
    """{relative path: content} of an estimator package."""
    values = dict(NAME=name, PASCAL=pascal(name), PERIOD=PERIOD_US)
    doc = fill(ESTIMATOR_DOC, **values)
    messages = fill(ESTIMATOR_MESSAGES_BUILD, **values)
    files = {"testdata/robot.toml": fill(ESTIMATOR_TESTDATA_ROBOT, **values)}
    if lang == "cpp":
        node_includes = [f"{name}/topics.h", "geometry/geometry_generated.h",
                         "talOS/node/node.h"]
        test_includes = ["rules_cc/cc/runfiles/runfiles.h",
                         f"{name}/{name}.h", "talOS/devices/testing.h",
                         "talOS/node/allocation_probe.h",
                         "talOS/node/testing.h"]
        build = fill(CC_BUILD, MESSAGES=messages,
                     DATA="[\n" + string_list(["testdata/robot.toml", ":node"],
                                              4), **values)
        # The node reads no devices; only its test drives SimHardware.
        build = build.replace('        "//talOS/devices",\n', "", 1)
        files.update({
            "BUILD": build,
            f"{name}.h": fill(CC_ESTIMATOR_HEADER,
                              INCLUDES=includes(node_includes),
                              DOC="\n".join("// " + line for line in
                                            doc.splitlines()), **values),
            f"{name}.cc": fill(CC_SOURCE, **values),
            "node_test.cc": fill(CC_ESTIMATOR_TEST,
                                 INCLUDES=includes(test_includes), **values),
        })
    else:
        imports = {
            name: "topics",
            "geometry.PoseEstimate": "PoseEstimate",
            "geometry.PoseObservation": "PoseObservation",
            "python.runfiles": "runfiles",
            "talos.testing": "Sim, SimHardware",
        }
        from_imports = "\n".join(
            f"from {module} import {imports[module]}"
            for module in sorted(imports, key=str.lower))
        files.update({
            "BUILD": fill(PY_BUILD, MESSAGES=messages,
                          DATA="[\n" + string_list(
                              [f"{name}.py", "testdata/robot.toml", ":node"],
                              4), **values),
            f"{name}.py": fill(PY_ESTIMATOR_NODE, DOC=doc, **values),
            "node_test.py": fill(PY_ESTIMATOR_TEST, FROM_IMPORTS=from_imports,
                                 **values),
        })
    return files


# ---------------------------------------------------------------- generation

TALOS_LABEL = re.compile(r"(?<![@\w])//talOS/")
LIST_ITEMS = re.compile(r'(?:^[ \t]+"[^"\n]*",\n)+', re.M)


def talos_labels(build, talos):
    """`build` with every talOS label (`//talOS/...`) naming the module
    `talos` (`@talos//talOS/...`), as a repository that depends on talOS
    names it, each list kept in buildifier's order. `talos` empty: in talOS
    itself, unchanged."""
    if not talos:
        return build
    build = TALOS_LABEL.sub(f"{talos}//talOS/", build)

    def resort(match):
        lines = match.group(0).splitlines(keepends=True)
        return "".join(sorted(
            lines, key=lambda line: label_order(line.strip()[1:-2])))

    return LIST_ITEMS.sub(resort, build)


def package_files(name, lang, standalone=False, kind="velocity", raw=False,
                  talos=""):
    """{relative path: content} of the new package. `raw`: a position node
    that writes out its own homing and limits instead of using
    talOS/mechanisms. `talos`: the label prefix of the talOS module, `@talos`
    in a repository that depends on it (standalone always), empty in talOS."""
    values = dict(NAME=name, PASCAL=pascal(name), PERIOD=PERIOD_US)
    files = (estimator_files(name, lang) if kind == "estimator"
             else mechanism_files(name, lang, kind, raw))
    files["BUILD"] = talos_labels(files["BUILD"],
                                  "@talos" if standalone else talos)
    if standalone:
        # The repo root is the package, so defaults.toml is exported after
        # the loads.
        loads, _, rest = files["BUILD"].partition("\n\n")
        files["BUILD"] = (loads + '\n\nexports_files(["defaults.toml"])\n\n'
                          + rest)
        extra_deps = (
            'bazel_dep(name = "rules_cc", version = "0.2.18")\n'
            'bazel_dep(name = "googletest", version = "1.17.0.bcr.2")\n'
            if lang == "cpp" else
            'bazel_dep(name = "rules_python", version = "2.0.1")\n'
        )
        files["MODULE.bazel"] = fill(
            STANDALONE_MODULE, EXTRA_DEPS=extra_deps, **values)
        files["defaults.toml"] = fill(
            STANDALONE_POSITION_RAW_DEFAULTS_TOML if raw else {
                "velocity": STANDALONE_DEFAULTS_TOML,
                "position": STANDALONE_POSITION_DEFAULTS_TOML,
                "estimator": STANDALONE_ESTIMATOR_DEFAULTS_TOML,
            }[kind], **values)
    return files


def neighbor_of(name):
    """The test robot's other subsystem: any name but the package's."""
    return "bystander" if name == "neighbor" else "neighbor"


def mechanism_files(name, lang, kind, raw=False):
    """{relative path: content} of a one-motor velocity/position package;
    `raw`: package_files'."""
    values = dict(NAME=name, PASCAL=pascal(name), PERIOD=PERIOD_US,
                  NEIGHBOR=neighbor_of(name))
    messages = fill(MESSAGES_BUILD, **values)
    fbs_template = (POSITION_RAW_FBS if raw else
                    POSITION_FBS if kind == "position" else FBS)
    files = {f"{name}_message.fbs": fill(fbs_template, **values)}
    if lang == "cpp":
        node_includes = [f"{name}/topics.h",
                         f"{name}/{name}_message_generated.h",
                         "talOS/devices/devices.h", "talOS/node/node.h"]
        test_includes = ["rules_cc/cc/runfiles/runfiles.h",
                         f"{name}/{name}.h", "talOS/devices/devices.h",
                         "talOS/devices/testing.h",
                         "talOS/node/allocation_probe.h",
                         "talOS/node/testing.h"]
        neighbor = values["NEIGHBOR"]
        data = ["testdata/robot.toml", f"testdata/{name}.toml",
                f"testdata/{neighbor}.toml", ":node"]
        build = fill(CC_BUILD, MESSAGES=messages,
                     DATA="[\n" + string_list(data, 4), **values)
        hdr_template = CC_POSITION_HEADER if kind == "position" else CC_HEADER
        cc_test = fill(CC_TEST, INCLUDES=includes(test_includes), **values)
        testdata_subsystem = CC_TESTDATA_SUBSYSTEM
        if kind == "position":
            # The motor is named after the subsystem; the fixture starts it
            # on its home switch.
            cc_test = edited(cc_test, [
                (f'"{name}_motor"', f'"{name}"'),
                ("Mode::kVelocity", "Mode::kMotionMagic"),
                ("velocity_rps()", "position_rot()"),
                ("  SimHardware hw{sim};\n};\n",
                 fill(CC_POSITION_FIXTURE, **values)),
                # Moving off a switch that still reads closed is a fault.
                ("  Teleop();\n  Hold(30.0, 60ms);\n",
                 "  Teleop();\n"
                 "  SetHome(false);  // zeroed on the switch, it leaves it\n"
                 "  Hold(30.0, 60ms);\n"),
            ])
        if kind == "position" and raw:
            # The node's own homing and limits on its Motor and home switch:
            # no talOS/mechanisms, no parameter table.
            hdr_template = CC_POSITION_RAW_HEADER
            pascal_name = values["PASCAL"]
            cc_test = edited(cc_test, [
                # A motor at rest on its home switch zeroes on its first tick.
                ("    sim.step(10ms);  // the mode is current before any lane "
                 "arrives\n",
                 "    // The mode is current before any lane arrives, and a "
                 "motor at rest on\n"
                 "    // its home switch has zeroed.\n"
                 "    sim.step(60ms);\n"),
                # Goals inside the node's travel, kMaxRot.
                ("  Hold(30.0, 60ms);\n\n", "  Hold(1.5, 60ms);\n\n"),
                ("  EXPECT_DOUBLE_EQ(request.demand, 30.0);\n",
                 "  EXPECT_DOUBLE_EQ(request.demand, 1.5);\n"),
                ("  EXPECT_DOUBLE_EQ(state.position_rot(), 30.0);\n",
                 "  EXPECT_DOUBLE_EQ(state.position_rot(), 1.5);\n"),
                (f"TEST_F({pascal_name}Test, OtherModesLaneIsIgnored) {{\n"
                 "  Teleop();\n",
                 f"TEST_F({pascal_name}Test, OtherModesLaneIsIgnored) {{\n"
                 "  Teleop();\n"
                 "  // Until the switch opens, no farther than "
                 "kHomeMarginRot.\n"
                 "  SetHome(false);\n"),
                ('  Hold(20.0, 40ms, "auto");\n',
                 '  Hold(1.0, 40ms, "auto");\n'),
                ("  EXPECT_DOUBLE_EQ(request.demand, 20.0);\n",
                 "  EXPECT_DOUBLE_EQ(request.demand, 1.0);\n"),
                ("struct Result {",
                 fill(CC_POSITION_RAW_TESTS, **values) + "struct Result {"),
            ])
            testdata_subsystem = CC_POSITION_RAW_TESTDATA_SUBSYSTEM
        elif kind == "position":
            # A thin node over talos::mechanisms::PositionMechanism, whose
            # name is both its motor and its parameter table.
            node_includes.append("talOS/mechanisms/position_mechanism.h")
            build = edited(build, [
                ('        "//talOS/devices",\n        "//talOS/node",\n',
                 '        "//talOS/devices",\n        "//talOS/mechanisms",\n'
                 '        "//talOS/node",\n'),
            ])
            cc_test = edited(cc_test, [
                # A motor on its home switch zeroes after home_samples ticks.
                ("    sim.step(10ms);  // the mode is current before any lane "
                 "arrives\n",
                 "    // The mode is current before any lane arrives, and a "
                 "motor on its home\n"
                 "    // switch has zeroed (home_samples ticks).\n"
                 "    sim.step(60ms);\n"),
                (f"// matrix: n/a 9 (the {name} reads no parameters)\n",
                 fill(CC_POSITION_PARAMETERS, **values)),
                ("struct Result {",
                 fill(CC_POSITION_TESTS, **values) + "struct Result {"),
            ])
            testdata_subsystem = CC_POSITION_TESTDATA_SUBSYSTEM
        files.update({
            "BUILD": build,
            f"{name}.h": fill(hdr_template, INCLUDES=includes(node_includes),
                              **values),
            f"{name}.cc": fill(CC_SOURCE, **values),
            "node_test.cc": cc_test,
            "testdata/robot.toml": fill(CC_TESTDATA_ROBOT, **values),
            f"testdata/{name}.toml": fill(testdata_subsystem, **values),
            f"testdata/{neighbor}.toml": fill(CC_TESTDATA_NEIGHBOR, **values),
        })
    else:
        data = [f"{name}.py", "testdata/robot.toml", ":node"]
        pascal_name = pascal(name)
        # isort order: by module, case-insensitively.
        imports = {
            name: "topics",
            f"{name}.{pascal_name}State": f"{pascal_name}State",
            f"{name}.{pascal_name}Target": f"{pascal_name}Target",
            "python.runfiles": "runfiles",
            "talos.testing": "Sim, SimHardware",
        }
        if kind == "position":
            imports[f"{name}.{pascal_name}Homing"] = f"{pascal_name}Homing"
        from_imports = "\n".join(
            f"from {module} import {imports[module]}"
            for module in sorted(imports, key=str.lower))
        build = fill(PY_BUILD, MESSAGES=messages,
                     DATA="[\n" + string_list(data, 4), **values)
        node = fill(PY_NODE, **values)
        py_test = fill(PY_TEST, FROM_IMPORTS=from_imports, **values)
        testdata_robot = PY_TESTDATA_ROBOT
        node_imports = {
            name: "topics",
            f"{name}.{pascal_name}Homing": f"{pascal_name}Homing",
            f"{name}.{pascal_name}State": f"{pascal_name}State",
            f"{name}.{pascal_name}Target": f"{pascal_name}Target",
        }
        if kind == "position":
            # The motor is named after the subsystem; setUp starts it at
            # rest on its home switch.
            py_test = edited(py_test, [
                (f'"{name}_motor"', f'"{name}"'),
                ("velocity_rps=demand", "position_rot=demand"),
                ("talos.Mode.VELOCITY", "talos.Mode.MOTION_MAGIC"),
                ("state.velocity_rps, 12.5", "state.position_rot, 0.0"),
                ('0.0, 12.5)\n',
                 '0.0, 0.0)\n'
                 '        # Starts at rest on its home switch (closed reads\n'
                 '        # False, see set_home): zeroed without moving.\n'
                 f'        self.hw.set_digital_input("{name}", '
                 f'"{name}_home", False)\n'),
            ])
        if kind == "position" and raw:
            # The node's own homing and limits on its Motor and home switch:
            # no talos.mechanisms, no parameter table.
            node = fill(PY_POSITION_RAW_NODE, FROM_IMPORTS="\n".join(
                f"from {module} import {node_imports[module]}"
                for module in sorted(node_imports, key=str.lower)), **values)
            py_test = edited(py_test, [
                # A motor at rest on its home switch zeroes on its first
                # tick.
                ("        self.sim.step(0.01)\n",
                 "        self.sim.step(0.06)  # at rest on a switch: zeroed\n"),
                # Goals inside the node's travel, MAX_ROT, off the switch.
                ("        self.teleop()\n        self.hold(30.0, 0.06)\n",
                 "        self.teleop()\n"
                 "        self.set_home(False)  # zeroed on the switch, it "
                 "leaves it\n"
                 "        self.hold(1.5, 0.06)\n"),
                ("        self.assertEqual(request.demand, 30.0)\n",
                 "        self.assertEqual(request.demand, 1.5)\n"),
                ('        self.teleop()\n'
                 '        self.hold(30.0, 0.04, source="auto")',
                 '        self.teleop()\n'
                 '        self.set_home(False)  # until it opens, no farther '
                 'than HOME_MARGIN_ROT\n'
                 '        self.hold(30.0, 0.04, source="auto")'),
                ('        self.hold(20.0, 0.04, source="auto")\n',
                 '        self.hold(1.0, 0.04, source="auto")\n'),
                ("        self.assertEqual(request.demand, 20.0)\n",
                 "        self.assertEqual(request.demand, 1.0)\n"),
                ("\n\nclass BinaryTest",
                 fill(PY_POSITION_RAW_TESTS, **values)
                 + "\n\nclass BinaryTest"),
            ])
            testdata_robot = PY_POSITION_RAW_TESTDATA_ROBOT
        elif kind == "position":
            # A thin node over talos.mechanisms.PositionMechanism, whose name
            # is both its motor and its parameter table.
            node_imports["talos.mechanisms"] = "PositionMechanism, PositionPhase"
            node = fill(PY_POSITION_NODE, FROM_IMPORTS="\n".join(
                f"from {module} import {node_imports[module]}"
                for module in sorted(node_imports, key=str.lower)), **values)
            build = edited(build, [
                ('    deps = [":messages_py"],\n',
                 '    deps = [\n        ":messages_py",\n'
                 '        "//talOS/python:mechanisms",\n    ],\n'),
                ('        ":messages_py",\n        "//talOS/python:testing",\n',
                 '        ":messages_py",\n        "//talOS/python:mechanisms",\n'
                 '        "//talOS/python:testing",\n'),
            ])
            py_test = edited(py_test, [
                (f"    # matrix: n/a 9 (the {name} reads no parameters)\n",
                 fill(PY_POSITION_PARAMETERS, **values)),
                # A motor on its home switch zeroes after home_samples ticks.
                ("        self.sim.step(0.01)\n",
                 "        self.sim.step(0.06)  # home_samples ticks on a switch\n"),
                ("\n\nclass BinaryTest",
                 fill(PY_POSITION_TESTS, **values) + "\n\nclass BinaryTest"),
            ])
            testdata_robot = PY_POSITION_TESTDATA_ROBOT
        files.update({
            "BUILD": build,
            f"{name}.py": node,
            "node_test.py": py_test,
            "testdata/robot.toml": fill(testdata_robot, **values),
        })
    return files


def edited(text, replacements):
    """`text` with each (old, new) replaced; every `old` must occur, so a
    template change cannot silently drop a position-only edit."""
    for old, new in replacements:
        if old not in text:
            raise AssertionError(f"template anchor not found: {old!r}")
        text = text.replace(old, new)
    return text


def includes(paths):
    # clang-format's SortIncludes order within one block.
    return "\n".join(f'#include "{path}"' for path in sorted(paths))


def used_can_ids(subsystems):
    """Every can_id in the effective [subsystems] view: inherited through
    `include`, layered from `defaults` files, read from custom `path`s."""
    ids = set()

    def walk(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key == "can_id" and isinstance(item, int):
                    ids.add(item)
                else:
                    walk(item)
        elif isinstance(value, list):
            for item in value:
                walk(item)

    walk(subsystems)
    return ids


def used_dio_channels(subsystems):
    """Every DIO channel (dio, dio_a, dio_b) in the effective [subsystems]
    view."""
    channels = set()

    def walk(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key in ("dio", "dio_a", "dio_b") and isinstance(item, int):
                    channels.add(item)
                else:
                    walk(item)
        elif isinstance(value, list):
            for item in value:
                walk(item)

    walk(subsystems)
    return channels


# A binding a subsystem file keeps commented until its gains are tuned, as
# this scaffold writes it: `# [[teleop.bindings]]` and its `# key = value`
# lines.
COMMENTED_BINDING = re.compile(
    r"^#[ \t]*\[\[teleop\.bindings\]\][ \t]*\n((?:#[ \t]*\w+[ \t]*=.*\n)*)",
    re.M)


class Layout(typing.NamedTuple):
    """Where a robot keeps its config and node packages, all paths relative
    to the workspace root ("" is the root itself)."""
    config: str  # the robot config the [[subsystems]] entry goes in
    packages: str  # the directory that holds the robot's node packages
    talos: str  # label prefix of talOS: "" in talOS, "@talos" outside it
    configs_root: str  # searched for other configs that include `config`

    @property
    def configuration(self):
        return posixpath.dirname(self.config)

    def package(self, name):
        return posixpath.join(self.packages, name)


def layout_of(robot):
    """`robot` as a Layout: a Layout as it is, a robot directory of talOS
    (`<robot>/main_processor/{configuration/robot.toml,<name>}`) otherwise."""
    if isinstance(robot, Layout):
        return robot
    processor = f"{robot}/main_processor"
    return Layout(f"{processor}/configuration/robot.toml", processor, "",
                  robot)


def talos_module():
    """The label prefix of talOS as the workspace being worked in names it:
    "" when talOS is the main repository, else `@<module>` (`@talos`), from
    the regenerate command sync_manifest gives the same workspace."""
    return sync_manifest.command().split()[2].partition("//")[0]


# A robot repository's config when --config is not given: the layout of
# examples/robot_repo, then a talOS robot's moved into its own repository.
DEPENDENT_CONFIGS = ("robot.toml", "main_processor/configuration/robot.toml")


def dependent_layout(root, config=None, packages=None, talos="@talos"):
    """The Layout of the robot repository at `root`, which depends on talOS
    as `talos`. `config`: the robot config (default: the first of
    DEPENDENT_CONFIGS that exists). `packages`: where node packages go
    (default: package_dir's choice)."""
    if config is None:
        config = next((c for c in DEPENDENT_CONFIGS
                       if os.path.isfile(os.path.join(root, c))), None)
        if config is None:
            raise Refused(f"no {' or '.join(DEPENDENT_CONFIGS)} in {root}; "
                          "name the robot config with --config")
    config = in_repository(root, config, "--config")
    if not os.path.isfile(os.path.join(root, config)):
        raise Refused(f"--config {config} not found")
    if packages is None:
        try:
            subsystems, _ = sync_manifest.effective(root, config)
        except sync_manifest.ManifestError as err:
            raise Refused(f"cannot read {config}: {err}")
        packages = package_dir(subsystems, config)
    packages = in_repository(root, packages, "--package-dir")
    return Layout(config, packages, talos, "")


REPOSITORY_MARKERS = ("MODULE.bazel", "WORKSPACE", "WORKSPACE.bazel")


def in_repository(root, path, flag):
    """`path` (relative to `root`, or absolute) as a path relative to the
    repository at `root` ("" for the root itself), refused naming `flag`
    when it resolves outside the repository (after `..` and symlinks) or
    into a repository nested in it, such as the talos/ submodule: its labels
    would name nothing, and its writes would land outside the robot. An
    absolute path inside the repository is taken as its relative form."""
    top = os.path.realpath(root)
    full = os.path.realpath(os.path.join(root, path))
    relative = os.path.relpath(full, top).replace(os.sep, "/")
    if relative == ".." or relative.startswith("../"):
        raise Refused(f"{flag} {path} is outside the robot repository {root}")
    relative = "" if relative == "." else relative
    parts = relative.split("/") if relative else []
    for depth in range(1, len(parts) + 1):
        directory = os.path.join(top, *parts[:depth])
        if any(os.path.isfile(os.path.join(directory, marker))
               for marker in REPOSITORY_MARKERS):
            nested = "/".join(parts[:depth])
            raise Refused(f"{flag} {path} is in {nested}/, a repository "
                          "nested in the robot repository (it holds a "
                          "MODULE.bazel or WORKSPACE), not the robot's")
    return relative


def package_dir(subsystems, config):
    """Where the robot of `config` keeps its node packages: the directory
    holding most of its own nodes' packages (`//gate:node` -> "",
    `//main_processor/shooter:node` -> "main_processor"); with none, the
    config's directory, or its parent when that is `configuration`, as in a
    talOS robot."""
    parents = collections.Counter()
    for table in subsystems.values():
        node = table.get("node") if isinstance(table, dict) else None
        if isinstance(node, str) and node.startswith("//"):
            parents[posixpath.dirname(node[2:].partition(":")[0])] += 1
    if parents:
        return parents.most_common(1)[0][0]
    directory = posixpath.dirname(config)
    if posixpath.basename(directory) == "configuration":
        return posixpath.dirname(directory)
    return directory


def teleop_button(root, robot):
    """The lowest of TELEOP_BUTTONS that no joystick 0 button of `robot` (a
    robot directory or a Layout) takes: a [[teleop.bindings]] button in the
    effective view of its config or of its simulation config, one still
    commented in a file they read, or a node's `*_button_id` parameter
    (driver's rezero_button_id)."""
    layout = layout_of(robot)
    configs = [layout.config]
    sim, sim_exists = sim_config(root, layout)
    if sim_exists:
        configs.append(sim)
    taken, files = set(), set()
    for config in configs:
        try:
            subsystems, read_files = sync_manifest.effective(root, config)
        except sync_manifest.ManifestError as err:
            raise Refused(f"cannot read {config}: {err}")
        files.update([config, *read_files])
        for table in subsystems.values():
            for binding in table.get("teleop", {}).get("bindings", []):
                if (binding.get("port", 0) == 0
                        and isinstance(binding.get("button"), int)):
                    taken.add(binding["button"])
            taken.update(value for key, value in table.items()
                         if key.endswith("_button_id")
                         and isinstance(value, int))
    binding_files = []
    for path in sorted(files):
        with open(os.path.join(root, path), encoding="utf-8") as f:
            text = f.read()
        try:
            header = tomllib.loads(text).get("subsystem", {})
        except tomllib.TOMLDecodeError:
            header = {}
        node = header.get("node")
        if isinstance(node, str) and node.endswith(TELEOP_NODE):
            continue
        if "teleop.bindings" in text or "_button_id" in text:
            binding_files.append(path)
        for match in COMMENTED_BINDING.finditer(text):
            keys = dict(re.findall(r"^#[ \t]*(\w+)[ \t]*=[ \t]*(\S+)",
                                   match.group(1), re.M))
            if keys.get("port", "0") == "0" and keys.get("button",
                                                         "").isdigit():
                taken.add(int(keys["button"]))
    button = next((b for b in TELEOP_BUTTONS if b not in taken), None)
    if button is None:
        raise Refused(
            f"every joystick 0 button {TELEOP_BUTTONS[0]}.."
            f"{TELEOP_BUTTONS[-1]} is bound (live or commented) in "
            f"{', '.join(binding_files)}; free one for the new binding")
    return button


def plan(root, robot, name, lang, kind="velocity", raw=False):
    """([(path, old content or None, new content)], placeholder CAN id, or
    None for an estimator, which owns no devices). `robot`: a robot directory
    of talOS, or the Layout of a robot repository (dependent_layout)."""
    if problem := check_name(name) or check_kind(name, lang, kind, raw):
        raise Refused(problem)
    layout = layout_of(robot)
    config = layout.config
    configuration = os.path.join(root, layout.configuration)
    robot_toml = os.path.join(root, config)
    if not os.path.isfile(robot_toml):
        raise Refused(f"{config} not found; is --robot right?")
    package = os.path.join(root, layout.package(name))
    subsystem_toml = os.path.join(configuration, "subsystems", f"{name}.toml")
    if os.path.exists(package):
        raise Refused(f"{rel(root, package)} already exists")
    if os.path.exists(subsystem_toml):
        raise Refused(f"{rel(root, subsystem_toml)} already exists")
    try:
        subsystems, _ = sync_manifest.effective(root, config)
    except sync_manifest.ManifestError as err:
        raise Refused(f"cannot read {config}: {err}")
    if name in subsystems:
        raise Refused(f"{config} already has subsystem '{name}'")

    can_id = button = None
    if kind != "estimator":
        button = teleop_button(root, layout)
        used = used_can_ids(subsystems)
        can_id = next((i for i in range(1, MAX_CAN_ID + 1) if i not in used),
                      None)
        if can_id is None:
            raise Refused(f"every CAN id 1..{MAX_CAN_ID} is taken in "
                          f"{rel(root, configuration)}")
    dio = None
    if kind == "position":
        used = used_dio_channels(subsystems)
        dio = next((i for i in range(MAX_DIO + 1) if i not in used), None)
        if dio is None:
            raise Refused(f"every DIO channel 0..{MAX_DIO} is taken in "
                          f"{rel(root, configuration)}")

    writes = [(os.path.join(package, path), None, content)
              for path, content in package_files(
                  name, lang, kind=kind, raw=raw,
                  talos=layout.talos).items()]
    subsystem = fill(
        POSITION_RAW_SUBSYSTEM_TOML if raw else
        {"velocity": SUBSYSTEM_TOML,
         "position": POSITION_SUBSYSTEM_TOML,
         "estimator": ESTIMATOR_SUBSYSTEM_TOML}[kind],
        NAME=name, PASCAL=pascal(name), CONFIG=config,
        PACKAGE=layout.package(name), PERIOD=PERIOD_US,
        CAN_ID=can_id, DIO=dio, BUTTON=button)
    configs = including_configs(root, layout)
    if kind != "estimator" and coverage_required(root, configs):
        subsystem = without_producer(subsystem, name)
    writes.append((subsystem_toml, None, subsystem))

    old = read(robot_toml)
    new_robot = old.rstrip("\n") + "\n" + fill(MANIFEST_ENTRY, NAME=name)
    writes.append((robot_toml, old, new_robot))

    # The bundle and graph test take their node and file lists from the
    # generated manifest, so regenerate it from the edited config, and every
    # other manifest in the robot whose config reads it (an `include` of
    # it), which the new entry changes too.
    overrides = {config: new_robot, rel(root, subsystem_toml): subsystem}
    for path in configs:
        try:
            manifest = sync_manifest.generate(root, path, overrides=overrides)
        except sync_manifest.ManifestError as err:
            raise Refused(f"cannot regenerate the manifest of {path}: {err}")
        manifest_file = os.path.join(root, sync_manifest.manifest_path(path))
        old = read(manifest_file) if os.path.isfile(manifest_file) else None
        writes.append((manifest_file, old, manifest))
    return writes, can_id


def including_configs(root, robot):
    """The config of `robot` (a robot directory or a Layout), then every
    other config of the robot that has a manifest and reads it (an `include`
    of it): the configs a new [[subsystems]] entry changes."""
    layout = layout_of(robot)
    configs = [layout.config]
    for other in robot_configs(root, layout.configs_root):
        if other == layout.config:
            continue
        try:
            _, files = sync_manifest.resolve(root, other)
        except sync_manifest.ManifestError as err:
            raise Refused(f"cannot regenerate the manifest of {other}: {err}")
        if layout.config in files:
            configs.append(other)
    return configs


def coverage_required(root, configs):
    """Whether any of `configs` resolves [robot] require_target_coverage
    true (talOS/configuration/config_parser.h: the key, which defaults to
    `commissioned`), with its `include` (one level) merged under it."""
    for path in configs:
        robot = robot_table(root, path)
        if robot.get("require_target_coverage",
                     robot.get("commissioned", False)) is True:
            return True
    return False


def robot_table(root, config):
    """The [robot] table of `config` with its `include` (one level) merged
    under it, as config_parser.h resolves it."""
    with open(os.path.join(root, config), "rb") as f:
        table = tomllib.load(f)
    robot = {}
    include = table.get("include")
    if isinstance(include, str):
        base = (include if posixpath.isabs(include) else posixpath.join(
            posixpath.dirname(config), include))
        with open(os.path.join(root, base), "rb") as f:
            robot.update(tomllib.load(f).get("robot", {}))
    robot.update(table.get("robot", {}))
    return robot


# The hardware bridge and simulator talOS provides, as a robot repository
# names them; used for the printed build when [robot] names neither. The
# launcher itself starts a simulator only when the config names one
# (talOS/launcher/roster.cc).
DEFAULT_PROGRAMS = {"hardware_node": "//talOS/bridge:hardware_node",
                    "sim_gateway": "//talOS/simulation:sim_gateway"}


def sim_programs(root, config, talos):
    """Labels of the hardware bridge and simulator `config` runs: its
    [robot] hardware_node and sim_gateway, else talOS's."""
    robot = robot_table(root, config)
    return [robot.get(key) if isinstance(robot.get(key), str)
            else talos + label for key, label in DEFAULT_PROGRAMS.items()]


def sim_target_config(root):
    """The config the root package's talos_sim (`//:sim`, else the only
    one) runs, workspace-relative, or None if the BUILD file has none or
    names it by anything but a string literal."""
    for build in ("BUILD.bazel", "BUILD"):
        path = os.path.join(root, build)
        if os.path.isfile(path):
            break
    else:
        return None
    try:
        with open(path, encoding="utf-8") as f:
            module = ast.parse(f.read())
    except SyntaxError:
        return None
    configs = {}
    for node in ast.walk(module):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == "talos_sim"):
            keywords = {k.arg: k.value for k in node.keywords}
            name, config = keywords.get("name"), keywords.get("config")
            if (isinstance(name, ast.Constant) and isinstance(config, ast.Constant)
                    and isinstance(config.value, str)):
                configs[name.value] = config.value
    label = configs.get("sim") or (
        next(iter(configs.values())) if len(configs) == 1 else None)
    if label is None:
        return None
    if label.startswith("//"):
        package, _, target = label[2:].partition(":")
        return posixpath.join(package, target or posixpath.basename(package))
    return label.removeprefix(":")


def without_producer(subsystem, name):
    """`subsystem` (a filled subsystem file) for a robot that requires
    target coverage: an explicit [targets] table with every mode empty. The
    launcher's coverage lint judges only a mode whose policy lists a lane,
    and with no lane talos::Target::get() is null, so the node stays
    neutral until a person lists a lane together with its producer."""
    return edited(subsystem, [
        ("# binding goes in the simulation-only config new_subsystem's "
         "`Next:` names.\n",
         "# binding goes in the simulation-only config new_subsystem's "
         "`Next:` names.\n"
         '# Uncomment it together with teleop = ["teleop"] in [targets] '
         "below.\n"),
        (f"# Which /{name}/target/<source> lanes drive the {name} in each "
         "Driver Station\n"
         "# mode, highest priority first. Without this table [robot.targets] "
         "applies\n"
         '# (teleop = ["teleop"], autonomous = ["auto"]); listed modes '
         "replace it.\n"
         "# [targets]\n"
         '# teleop = ["teleop"]\n'
         '# autonomous = ["auto"]\n',
         f"# Which /{name}/target/<source> lanes drive the {name} in each "
         "Driver Station\n"
         "# mode, highest priority first; listed modes replace "
         "[robot.targets]. The\n"
         f"# {name} has no producer yet, and this robot requires a writer for "
         "every\n"
         "# lane a mode lists ([robot] require_target_coverage), so every "
         "mode is\n"
         f"# empty and the {name} stays neutral. List a lane together with "
         "its\n"
         '# producer: "teleop" with the teleop binding above, "auto" once an\n'
         f"# autonomous node writes /{name}/target/auto.\n"
         "[targets]\n"
         "teleop = []\n"
         "autonomous = []\n"
         "test = []\n"),
    ])


def robot_configs(root, directory):
    """Workspace-relative configs under `directory` that have a manifest,
    outside any repository nested in it (a robot repository's talOS
    submodule)."""
    found = []
    top = os.path.join(root, directory)
    for current, dirs, files in os.walk(top):
        if current != top and any(
                f in files for f in ("MODULE.bazel", "WORKSPACE",
                                     "WORKSPACE.bazel")):
            dirs[:] = []
            continue
        dirs[:] = sorted(d for d in dirs if not d.startswith((".", "bazel-")))
        for f in sorted(files):
            if f.endswith("_manifest.bzl"):
                stem = f.removesuffix("_manifest.bzl")
                config = os.path.join(current, stem + ".toml")
                if os.path.isfile(config):
                    found.append(rel(root, config))
    return found


def talos_checkout(script):
    """The talOS checkout `script` (this file, through its runfiles symlink)
    lives in, or None when it is not a durable checkout: Bazel's copy of a
    fetched module, under an output base's external/ directory, is
    replaced or removed by Bazel, so nothing may point at it."""
    top = os.path.dirname(os.path.dirname(os.path.realpath(script)))
    try:
        with open(os.path.join(top, "MODULE.bazel"), encoding="utf-8") as f:
            module = f.read()
    except OSError:
        return None
    if not re.search(r'name\s*=\s*"talos"', module):
        return None
    ancestor = top
    while os.path.dirname(ancestor) != ancestor:
        parent = os.path.dirname(ancestor)
        if (os.path.basename(ancestor) == "external"
                and os.path.isdir(os.path.join(parent, "execroot"))):
            return None
        ancestor = parent
    return top


def plan_standalone(target_dir, name, lang, talos_root, kind="velocity",
                    raw=False):
    """[(path, None, content)] for a standalone Bzlmod subsystem repository
    that builds against the talOS checkout at `talos_root` as it stands."""
    if problem := check_name(name) or check_kind(name, lang, kind, raw):
        raise Refused(problem)
    if os.path.exists(target_dir) and os.listdir(target_dir):
        raise Refused(f"{target_dir} already exists and is not empty")
    files = package_files(name, lang, standalone=True, kind=kind, raw=raw)
    # Real paths, so a symlinked directory (macOS /tmp) cannot skew it.
    talos_path = os.path.relpath(os.path.realpath(talos_root),
                                 os.path.realpath(target_dir)).replace(
                                     os.sep, "/")
    files["MODULE.bazel"] += (
        "\n# The talOS checkout this repository was generated from. Replace it\n"
        "# with a registry version once talos is published to one.\n"
        "local_path_override(\n"
        '    module_name = "talos",\n'
        f'    path = "{talos_path}",\n'
        ")\n")
    # talOS's C++23 flags and Bazel version are per workspace, not carried by
    # the module, so the new workspace takes the checkout's own.
    for config in (".bazelrc", ".bazelversion"):
        files[config] = read(os.path.join(talos_root, config))
    return [(os.path.join(target_dir, path), None, content)
            for path, content in files.items()]


# Line ending of each file read, so an edit keeps a CRLF checkout's style.
NEWLINES = {}


def read(path):
    with open(path, encoding="utf-8", newline="") as f:
        text = f.read()
    NEWLINES[path] = "\r\n" if "\r\n" in text else "\n"
    return text.replace("\r\n", "\n")


def rel(root, path):
    return os.path.relpath(path, root).replace(os.sep, "/")


def show(root, writes):
    for path, old, new in writes:
        name = rel(root, path)
        if old is None:
            print(f"=== new file {name}\n{new}")
        else:
            print("".join(difflib.unified_diff(
                old.splitlines(keepends=True), new.splitlines(keepends=True),
                f"a/{name}", f"b/{name}")))


# What ./sim.sh drives a scaffold with: teleop after a second disabled, and
# joystick 0's button (button 3: bit 2) pressed from second 3 to second 6,
# after a neutral sample that arms the teleop lane (talos::OperatorArming).
SIM_DURATION_S = 8
MODE_SCHEDULE = "disabled:1,teleop:10"


def joystick_schedule(button):
    return (f"0:axes=0;buttons=0;pov=-1|3:axes=0;buttons={1 << (button - 1)};"
            "pov=-1|6:axes=0;buttons=0;pov=-1")



def sim_run(button):
    return (f"--sim-gateway --no-studio --duration-s {SIM_DURATION_S} "
            f"--mode-schedule '{MODE_SCHEDULE}' \\\n      "
            f"--joystick-schedule '{joystick_schedule(button)}'")


def launcher_sim_run(talos, config, button):
    """sim_run's schedules for the launcher itself, run from a robot
    repository on its simulation config, as talos_sim runs it (--sim,
    --start-sim-gateway, the physics simulator)."""
    return (f"bazel run {talos}//talOS/launcher -- --config $PWD/{config} "
            f"--sim --duration-s {SIM_DURATION_S} \\\n      "
            "--start-sim-gateway --sim-gateway-arg --physics \\\n      "
            f"--sim-gateway-arg --mode-schedule "
            f"--sim-gateway-arg '{MODE_SCHEDULE}' \\\n      "
            f"--sim-gateway-arg --joystick-schedule "
            f"--sim-gateway-arg '{joystick_schedule(button)}'")


def dependent_sim_run(button):
    """sim_run's schedules as a robot repository's `bazel run //:sim`
    (talos_sim: the launcher with --sim --start-sim-gateway) takes them."""
    return (f"bazel run //:sim -- --duration-s {SIM_DURATION_S} \\\n      "
            f"--sim-gateway-arg --mode-schedule "
            f"--sim-gateway-arg '{MODE_SCHEDULE}' \\\n      "
            f"--sim-gateway-arg --joystick-schedule "
            f"--sim-gateway-arg '{joystick_schedule(button)}'")


# The robot's simulation-only configs, in the order next_steps looks for
# one: `include = "robot.toml"` plus each mechanism's simulation block.
# new_robot writes robot_sim.toml with its manifest; 2026-robot's is
# robot_physics_sim.toml.
SIM_CONFIGS = ("robot_sim.toml", "robot_physics_sim.toml")

SIM_CONFIG_HEADER = """\
# Simulation only: ./sim.sh --config @CONFIG@ --sim-gateway
# robot.toml never reads this file, so nothing here reaches the robot.
include = "robot.toml"

[robot]
# The launcher and hardware_node refuse this config without --sim.
simulation_only = true
"""


def sim_config(root, robot):
    """(workspace-relative simulation config of `robot`, a robot directory or
    a Layout, whether it exists): the first of SIM_CONFIGS beside its config
    that does, else robot_sim.toml to create."""
    configuration = layout_of(robot).configuration
    for name in SIM_CONFIGS:
        if os.path.isfile(os.path.join(root, configuration, name)):
            return posixpath.join(configuration, name), True
    return posixpath.join(configuration, SIM_CONFIGS[0]), False


def simulation_block(name, kind, button=TELEOP_BUTTONS[0], raw=False):
    """The TOML a student appends to the robot's simulation config so that
    `name` moves under `./sim.sh --sim-gateway` with joystick 0's `button`
    held (teleop_button's choice). A --raw position node's travel is a
    constant in its file, so its block sets none.

    The values go there, not in the subsystem file, because the robot's own
    configs never read the simulation config: a live button binding, gains
    nobody tuned and an unmeasured travel cannot reach a real robot,
    commissioned or not. The scaffold leaves the subsystem file with its
    binding commented and every gain zero. next_steps prints the block;
    new_robot_test and scaffold_sim_test append it as printed.
    """
    lane = f"/{name}/target/teleop"
    sub = f"subsystems.{name}"
    if kind == "position":
        motor, demand = name, "position_rot = 1.0"
        measure = " and\n# measure its travel"
    else:
        motor, demand, measure = f"{name}_motor", "velocity_rps = 20.0", ""
    block = f"""\
# {name}: simulation values only. robot.toml never reads this file; before
# a real robot drives the {name}, tune slot0 on the mechanism{measure}.
# Holding joystick button {button} writes {demand} on {lane}.
[[{sub}.teleop.bindings]]
button = {button}
while_held = {{ {demand} }}
# Until an autonomous node writes /{name}/target/auto, the binding is the
# {name}'s only producer; a policy that lists "auto" with
# require_target_coverage = true refuses the graph without this. Its lane
# is listed here, since the subsystem file may list none.
[{sub}.targets]
teleop = ["teleop"]
autonomous = []
"""
    zero = "p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0"
    if kind == "position":
        gains = (zero.replace("p = 0.0", "p = 1.0")
                 .replace("d = 0.0", "d = 0.05").replace("v = 0.0", "v = 0.12"))
        travel = "" if raw else f"""\
# The simulated travel.
[{sub}.{name}]
max_rot = 2.0
"""
        push = "the homing push" if raw else "homing_v"
        return block + travel + f"""\
# The simulated home switch: it reads closed while the motor, which starts
# at 0, is at or below -0.1 rotations, where {push} drives it.
[{sub}.digital_inputs.{name}_home]
sim_motor = "{motor}"
sim_closed_at_rot = -0.1
# The simulated motor's gains, not the mechanism's.
[{sub}.motors.{motor}]
slot0 = {{ {gains} }}
"""
    return block + f"""\
# The simulated motor's feedforward (volts per rps), not the mechanism's:
# with every gain zero the physics simulation stalls the motor.
[{sub}.motors.{motor}]
slot0 = {{ {zero.replace("v = 0.0", "v = 0.12")} }}
"""


def next_steps(robot, name, can_id, kind="velocity", lang="cpp",
               golden_test=False, sim=None, button=TELEOP_BUTTONS[0],
               raw=False, coverage=False, sim_target=None,
               programs=()):
    """What to do after scaffolding `name` into `robot`. `golden_test`: the
    robot pins its hardware config in configuration/config_golden_test.cc
    (2026-robot does; a new_robot robot does not). `sim`: sim_config's
    (path, exists) for the robot; default, a robot_sim.toml that exists.
    `button`: teleop_button's choice for the robot before the scaffold.
    `raw`: a --raw position node, whose constants are in its own file
    (`lang` names it); otherwise both languages read the same keys.
    `coverage`: the robot requires target coverage, so the subsystem file
    lists no lane (without_producer) until its producer is added.
    `robot`: a robot directory of talOS, or a Layout; in a robot repository
    the commands are the ones it runs (`@talos//` tools). There
    `sim_target` is sim_target_config's answer for the repository (the
    config its root talos_sim runs, or None if it cannot tell): `bazel run
    //:sim` runs the simulation config only when that is it, otherwise the
    launcher runs it directly after `programs` (the hardware bridge and
    simulator the config names) are built."""
    layout = layout_of(robot)
    package = f"//{layout.package(name)}"
    configuration = layout.configuration
    subsystem = posixpath.join(configuration, "subsystems", f"{name}.toml")
    sim_path, sim_exists = sim or (
        posixpath.join(configuration, SIM_CONFIGS[0]), True)
    talos = layout.talos
    if talos:
        build = ("  bazel test //...    # the graph test and manifest check "
                 "of the robot repository")
        if sim_target == sim_path:
            run = f"  {dependent_sim_run(button)}"
        else:
            why = (f"//:sim runs {sim_target}, not {sim_path}"
                   if sim_target else
                   f"no root talos_sim names {sim_path} as its config")
            run = (f"  {why}, so run it with the launcher:\n"
                   f"  bazel build {' '.join(programs)}\n"
                   f"  {launcher_sim_run(talos, sim_path, button)}")
        sync = f"bazel run {talos}//tools:sync_manifest --"
        checks = "a talos_sim or graph test of it reads"
    else:
        build = (f"  bazel build //talOS/launcher //talOS/bridge:hardware_node "
                 f"//{layout.packages}/...")
        run = f"  ./sim.sh --config {sim_path} {sim_run(button)}"
        sync = "bazel run //tools:sync_manifest --"
        checks = "./sim.sh checks"
    launcher = f"{talos}//talOS/launcher"
    if can_id is None:  # an estimator
        return f"""\
Next:
  bazel test {package}:all
{build}
  bazel run {launcher} -- --config $PWD/{layout.config} --describe-only
  {subsystem}
    List the camera nodes in `cameras`; point telemetry at the pose with
    [studio] pose_topic = "/{name}/state"."""
    lines = simulation_block(name, kind, button, raw)
    if sim_exists:
        step, manifest = "append these lines to", ""
    else:
        step = "create"
        header = fill(SIM_CONFIG_HEADER, CONFIG=sim_path)
        if talos:
            header = header.replace(
                f"./sim.sh --config {sim_path} --sim-gateway",
                "the launcher runs it with --sim")
        # The include is resolved from this file's directory, which is the
        # config's (sim_config), so it is the config's file name.
        base = posixpath.basename(layout.config)
        header = header.replace('include = "robot.toml"',
                                f'include = "{base}"').replace(
                                    "# robot.toml never reads",
                                    f"# {base} never reads")
        lines = header + lines
        manifest = (
            f"\n  and generate its manifest, which {checks}:\n"
            f"  {sync} {sim_path}")
    lines = textwrap.indent(lines, "    ").rstrip("\n")
    travel = """
    The home switch's dio is a placeholder too: replace it with its physical
    channel. Set max_rot to the measured travel. Wire the switch normally
    open to ground; check that homing_v (-0.5 V) moves toward it, and set
    homing_timeout_us.
    Only the node's clamp stops the mechanism past the top: wire a forward
    limit switch to the TalonFX's limit input and set
    forward_limit_switch.""" if kind == "position" else ""
    own = ""
    if raw:
        source = (f"{layout.package(name)}/"
                  + (f"{name}.h" if lang == "cpp" else f"{name}.py"))
        constants = ("kMaxRot, kHomingVolts, kMaxPushTicks and kHomeMarginRot"
                     if lang == "cpp" else
                     "MAX_ROT, HOMING_VOLTS, MAX_PUSH_TICKS and HOME_MARGIN_ROT")
        travel = f"""
    The home switch's dio is a placeholder too: replace it with its physical
    channel. Wire the switch normally open to ground. Set stator_limit_a
    (20 A, provisional) for your mechanism.
    Only the node's clamp stops the mechanism past the top: wire a forward
    limit switch to the TalonFX's limit input and set
    forward_limit_switch."""
        own = f"""
  In {source}:
    {constants}
    are placeholders sized for the simulation: measure the travel, check
    the push moves toward the switch, time it, and set each."""
    listing = (' and listing teleop = ["teleop"] in its [targets]'
               if coverage else "")
    golden = f"""
  {configuration}/config_golden_test.cc
    pins the hardware config: update it once a human has checked the CAN id.""" \
        if golden_test else ""
    return f"""\
Next:
  bazel test {package}:all
  To see it move in simulation, {step} {sim_path},
  the simulation-only config; {subsystem}
  keeps its binding commented and its gains zero:
{lines}{manifest}
  Then check the graph and run it (button {button} held from second 3 to 6):
{build}
  bazel run {launcher} -- --config $PWD/{sim_path} --describe-only
{run}
  Before a real robot drives the motor, in {subsystem}:
    can_id = {can_id} is a placeholder: replace it with the motor's physical id.{travel}
    Tune slot0 on the mechanism before uncommenting its teleop binding{listing}.{own}{golden}"""


def standalone_next_steps(target_dir, name, kind="velocity", raw=False,
                          talos=""):
    edit = {"velocity": "add the motor's physical can_id",
            "position": "add the motor's physical can_id and the home "
                        "switch's dio, set its travel and homing_v",
            "estimator": "list its camera nodes in `cameras`"}[kind]
    if raw:  # its travel and homing push are constants in the node
        edit = "add the motor's physical can_id and the home switch's dio"
    return f"""\
Next (standalone subsystem repo at {target_dir}):
  1. Run its tests (MODULE.bazel overrides @talos with this checkout):
       cd {target_dir} && bazel test //...
  2. Consume it from a robot repo's MODULE.bazel:
       bazel_dep(name = "{name}", version = "0.1.0")
       local_path_override(module_name = "{name}", path = "{target_dir}")
  3. Copy {target_dir}/defaults.toml to the robot's
     configuration/subsystems/{name}.toml, {edit},
     list it in robot.toml ([[subsystems]] name = "{name}",
     path = "subsystems/{name}.toml") and regenerate the manifest:
       bazel run {talos}//tools:sync_manifest -- <robot.toml>"""


KINDS = {
    "velocity": "a motor held at a speed, such as a flywheel or roller",
    "position": "a motor moved to a position counted from a home switch, "
                "such as an arm, wrist or elevator",
    "estimator": "a pose estimator from camera observations, with no device",
}


# How a position node is written: printed by kind_summary, one line each.
POSITION_CONTROL = {
    False: "on talOS/mechanisms' PositionMechanism: less code, behaviour "
           "shared and tested there",
    True: "every line of homing and limits written in your own file, with "
          "its tests",
}


def kind_summary(kind, kinds=tuple(KINDS), raw=None):
    """The archetype generated and the others of `kinds` (the tool's --type
    choices), printed before `Next:`, since leaving out --type gives a
    velocity node without saying so. `raw` (new_subsystem's --raw; None for
    a tool without it): a position node also names how it was written and
    the other way."""
    others = "".join(f"\n    {other}: {KINDS[other]}"
                     for other in kinds if other != kind)
    control = ""
    if kind == "position" and raw is not None:
        control = (f"\n  Written {'with --raw' if raw else 'without --raw'}: "
                   f"{POSITION_CONTROL[raw]}.\n"
                   f"    {'without' if raw else 'with'} --raw: "
                   f"{POSITION_CONTROL[not raw]}")
    return (f"Type: {kind}, {KINDS[kind]}.{control}\n"
            f"  Other types (--type {'|'.join(kinds)}):{others}")


def shown(base, path):
    """`path` relative to `base` when inside it, else absolute."""
    relative = rel(base, path)
    return os.path.abspath(path) if relative.startswith("../") else relative


def main(argv):
    # Run from a robot repository (`bazel run @talos//tools:new_subsystem`),
    # talOS is a dependency: its labels take this prefix, and the robot is
    # that repository's (--config), not a directory of talOS (--robot).
    talos = talos_module()
    parser = argparse.ArgumentParser(
        prog=f"bazel run {talos}//tools:new_subsystem --",
        description="Scaffold a talos subsystem node package and wire it into "
        "a robot, or generate a standalone Bzlmod subsystem repository.")
    parser.add_argument("name", help="node name, e.g. wrist (talOS/NAMING.md)")
    parser.add_argument("--lang", choices=("cpp", "py"), required=True)
    parser.add_argument("--type", dest="kind",
                        choices=tuple(KINDS),
                        default="velocity",
                        help="archetype: a velocity or position mechanism, or "
                        "a hardware-free pose estimator (default: %(default)s)")
    parser.add_argument("--raw", action="store_true",
                        help="with --type position: write every line of "
                        "homing and limits in the node's own file, with its "
                        "tests, instead of using talOS/mechanisms' "
                        "PositionMechanism (less code, shared behaviour)")
    parser.add_argument("--robot",
                        help="in talOS: robot directory (default: "
                        "2026-robot)")
    parser.add_argument("--config", metavar="PATH",
                        help="in a robot repository: the robot config that "
                        "gets the [[subsystems]] entry (default: "
                        f"{' or '.join(DEPENDENT_CONFIGS)}, whichever exists)")
    parser.add_argument("--package-dir", metavar="DIR",
                        help="in a robot repository: the directory the node "
                        "package goes in (default: where the config's own "
                        "nodes are, else beside the config)")
    parser.add_argument("--standalone", metavar="DIR",
                        help="scaffold a standalone Bzlmod subsystem repo in DIR "
                        "instead of wiring into --robot")
    parser.add_argument("--talos-checkout", metavar="DIR",
                        help="with --standalone in a robot repository: the "
                        "talOS checkout the new repository builds against "
                        "(default: the one this tool runs from, when it is "
                        "a checkout such as the talos/ submodule)")
    parser.add_argument("--dry-run", action="store_true",
                        help="print every file and edit without writing")
    args = parser.parse_args(argv)
    if args.raw and args.kind != "position":
        parser.error(f"--raw writes a position node's homing and limits; "
                     f"it takes --type position, not --type {args.kind}")

    if talos and args.robot:
        parser.error("--robot names a robot directory of talOS; in a robot "
                     "repository name its config with --config")
    if args.talos_checkout and not (talos and args.standalone):
        parser.error("--talos-checkout is for --standalone in a robot "
                     "repository; in talOS the new repository builds against "
                     "this checkout")
    if not talos and (args.config or args.package_dir):
        parser.error("--config and --package-dir are for a robot repository; "
                     "in talOS name the robot directory with --robot")

    root = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
    if not root:
        print("new_subsystem: BUILD_WORKSPACE_DIRECTORY is unset; run it with "
              f"`bazel run {talos}//tools:new_subsystem -- ...`",
              file=sys.stderr)
        return 2
    robot = args.robot or "2026-robot"
    try:
        if talos and not args.standalone:
            robot = dependent_layout(root, args.config, args.package_dir,
                                     talos)
        if args.standalone:
            target_dir = os.path.join(root, args.standalone)
            # The new repository overrides @talos with a talOS checkout and
            # takes its .bazelrc: this one in talOS; from a robot
            # repository, the checkout this tool runs from (its submodule).
            talos_root = root
            if talos:
                talos_root = (os.path.join(root, args.talos_checkout)
                              if args.talos_checkout
                              else talos_checkout(__file__))
                if talos_root is None:
                    raise Refused(
                        "the talOS this tool runs from is Bazel's copy of a "
                        "fetched module, not a checkout the new repository "
                        "can point at; pass --talos-checkout DIR (the talOS "
                        "checkout, e.g. the talos/ submodule)")
            writes = plan_standalone(target_dir, args.name, args.lang,
                                     talos_root, kind=args.kind, raw=args.raw)
            can_id = None
        else:
            writes, can_id = plan(root, robot, args.name, args.lang,
                                  kind=args.kind, raw=args.raw)
            # Plan's choice, from the robot as it stands before the writes.
            button = (teleop_button(root, robot)
                      if can_id is not None else None)
    except Refused as refused:
        print(f"new_subsystem: refused: {refused}", file=sys.stderr)
        return 1

    if args.dry_run:
        show(root, writes)
    else:
        for path, _, content in writes:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8",
                      newline=NEWLINES.get(path, "\n")) as f:
                f.write(content)
            print(f"wrote {shown(root, path)}")
    print(kind_summary(args.kind, raw=args.raw))
    if args.standalone:
        print(standalone_next_steps(shown(root, target_dir), args.name,
                                    args.kind, args.raw, talos))
    else:
        golden_test = os.path.isfile(os.path.join(
            root, layout_of(robot).configuration, "config_golden_test.cc"))
        coverage = can_id is not None and coverage_required(
            root, including_configs(root, robot))
        layout = layout_of(robot)
        sim_target = programs = None
        if layout.talos:
            sim_target = sim_target_config(root)
            programs = sim_programs(root, layout.config, layout.talos)
        print(next_steps(robot, args.name, can_id, args.kind, args.lang,
                         golden_test, sim_config(root, robot), button,
                         args.raw, coverage, sim_target, programs or ()))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
