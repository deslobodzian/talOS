"""Scaffolds a talos subsystem: a node package plus its robot wiring.

    bazel run //tools:new_subsystem -- <name> --lang cpp|py \\
        [--robot 2026-robot] [--dry-run]

Writes <robot>/main_processor/<name>/ (messages, node, Sim test, testdata),
modelled on that robot's shooter (C++) and intake (Python) packages, then adds
configuration/subsystems/<name>.toml, its [[subsystems]] manifest entry, and the
node and subsystem labels in the robot's talos_bundle. Everything is checked
before anything is written, so a refused run leaves the tree untouched.
"""

import argparse
import difflib
import keyword
import os
import re
import sys
import tomllib

# Mirrors talOS/introspection/names.h (CheckSegment, CheckNodeName); keep them
# in sync. The launcher re-checks at --describe-only, so drift only costs an
# earlier error message.
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


def pascal(name):
    return "".join(part[:1].upper() + part[1:] for part in name.split("_"))


def fill(template, **values):
    for key, value in values.items():
        template = template.replace(f"@{key}@", str(value))
    return template


def buildifier_sorted(labels):
    """Buildifier's list order: files, then :local, //absolute, @external."""
    def phase(label):
        if label.startswith("@"):
            return 3
        if label.startswith("//"):
            return 2
        if label.startswith(":"):
            return 1
        return 0
    return sorted(labels, key=lambda label: (phase(label), label))


def string_list(labels, indent):
    pad = " " * indent
    return "".join(f'{pad}    "{label}",\n'
                   for label in buildifier_sorted(labels)) + f"{pad}]"


# ---------------------------------------------------------------- templates

FBS = """\
namespace @NAME@;

// What /@NAME@/target asks for. The node drives the motor only while the
// robot is enabled and this is fresh and enabled.
struct @PASCAL@Target {
  enabled:bool;
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

// What /@NAME@/target asks for. The node drives the motor with Motion Magic
// only while the robot is enabled and this is fresh and enabled.
struct @PASCAL@Target {
  enabled:bool;
  position_rot:double;
}

// What the @NAME@ is doing, published every tick.
struct @PASCAL@State {
  enabled:bool;
  position_rot:double;
  connected:bool;
  at_goal:bool;
}
"""

MESSAGES_BUILD = """\
# @PASCAL@Target / @PASCAL@State (namespace @NAME@) and the @NAME@'s topics.
# /@NAME@/target has one writer (the arbiter); teleop and autonomous producers
# publish on their own lanes (/target/teleop, /target/auto).
talos_messages(
    name = "messages",
    srcs = ["@NAME@_message.fbs"],
    include_prefix = "@NAME@",
    topics = {
        "auto_target": "/@NAME@/target/auto",
        "state": "/@NAME@/state",
        "target": "/@NAME@/target",
        "teleop_target": "/@NAME@/target/teleop",
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
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
"""

CC_HEADER = """\
#pragma once

@INCLUDES@

namespace @NAME@ {

// Drives the motor at the velocity /@NAME@/target asks for and reports what
// it did on /@NAME@/state. Gains live in the robot config, slot0 of
// [motors.@NAME@_motor]; the motor controller closes the velocity loop
// onboard.
//
// Nothing publishes /@NAME@/target yet. A teleop or autonomous producer
// publishes its own lane, /@NAME@/target/<source>, and a line in the arbiter
// picks one and republishes it on /@NAME@/target.
class @PASCAL@ {
 public:
  using Target = @PASCAL@Target;
  using State = @PASCAL@State;

  explicit @PASCAL@(talos::Node& node)
      : node_{node},
        motor_{node, "@NAME@_motor"},
        // Optional: the graph lint accepts a target nobody publishes yet.
        target_{node.subscribe<Target>(topics::kTarget, {.optional = true})},
        state_{node.publish<State>(topics::kState)} {}

  void update() {
    const bool enabled = talos::robot_enabled(node_);
    // Otherwise nothing is set and the flush sends neutral.
    if (enabled && target_.fresh() && target_->enabled())
      motor_.set_velocity(target_->velocity_rps());
    state_.send(State{enabled, motor_.velocity(), motor_.connected()});
  }

 private:
  talos::Node& node_;
  talos::Motor motor_;
  talos::Subscription<Target> target_;
  talos::Publisher<State> state_;
};

}  // namespace @NAME@
"""
CC_POSITION_HEADER = """\
#pragma once

#include <cmath>

@INCLUDES@

namespace @NAME@ {

// Drives the motor to the position /@NAME@/target asks for with Motion Magic
// and reports what it did on /@NAME@/state.
class @PASCAL@ {
 public:
  using Target = @PASCAL@Target;
  using State = @PASCAL@State;

  explicit @PASCAL@(talos::Node& node)
      : node_{node},
        motor_{node, "@NAME@_motor"},
        tolerance_rot_{node.param<double>("tolerance_rot", 0.05)},
        target_{node.subscribe<Target>(topics::kTarget, {.optional = true})},
        state_{node.publish<State>(topics::kState)} {}

  void update() {
    const bool enabled = talos::robot_enabled(node_);
    const bool active = enabled && target_.fresh() && target_->enabled();
    if (active) {
      goal_rot_ = target_->position_rot();
      motor_.set_motion_magic(goal_rot_);
    }
    const double pos = motor_.position();
    const bool at_goal = active && std::abs(pos - goal_rot_) <= tolerance_rot_;
    state_.send(State{enabled, pos, motor_.connected(), at_goal});
  }

 private:
  talos::Node& node_;
  talos::Motor motor_;
  double tolerance_rot_;
  double goal_rot_ = 0.0;
  talos::Subscription<Target> target_;
  talos::Publisher<State> state_;
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
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

@INCLUDES@

namespace @NAME@ {
namespace {

using namespace std::chrono_literals;
using talos::hardware::Mode;
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

class @PASCAL@Test : public ::testing::Test {
 protected:
  using Target = @PASCAL@::Target;
  using State = @PASCAL@::State;

  // Holds a target the way the arbiter does: republished every 20 ms.
  void Hold(double rps, std::chrono::milliseconds duration) {
    for (auto t = 0ms; t < duration; t += 20ms) {
      sim.publish(topics::kTarget, Target{true, rps});
      sim.step(20ms);
    }
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

TEST_F(@PASCAL@Test, DrivesTheMotorAtAFreshTarget) {
  hw.enable();
  hw.ideal_motors();
  Hold(30.0, 60ms);

  const auto request = hw.motor_request("@NAME@", "@NAME@_motor");
  EXPECT_EQ(request.mode, Mode::kVelocity);
  EXPECT_DOUBLE_EQ(request.demand, 30.0);
  const State state = Latest();
  EXPECT_TRUE(state.enabled());
  EXPECT_DOUBLE_EQ(state.velocity_rps(), 30.0);
  EXPECT_TRUE(state.connected());
}

TEST_F(@PASCAL@Test, DisabledRobotGoesNeutral) {
  hw.enable(false);
  Hold(30.0, 60ms);
  EXPECT_EQ(hw.motor_request("@NAME@", "@NAME@_motor").mode, Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());
}

TEST_F(@PASCAL@Test, StaleTargetGoesNeutral) {
  hw.enable();
  Hold(30.0, 40ms);
  ASSERT_EQ(hw.motor_request("@NAME@", "@NAME@_motor").mode, Mode::kVelocity);

  sim.step(150ms);  // past command_timeout_us: the target is stale
  EXPECT_EQ(hw.motor_request("@NAME@", "@NAME@_motor").mode, Mode::kNeutral);
  EXPECT_TRUE(Latest().enabled());  // state keeps publishing
}

TEST_F(@PASCAL@Test, DisabledTargetGoesNeutral) {
  hw.enable();
  sim.publish(topics::kTarget, Target{false, 30.0});
  sim.step(40ms);
  EXPECT_EQ(hw.motor_request("@NAME@", "@NAME@_motor").mode, Mode::kNeutral);
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

CC_TESTDATA_ROBOT = """\
# Local @NAME@ fixture. Device IDs are assigned by the normal parser.
[robot]
name = "@NAME@_test"
period_us = @PERIOD@
command_timeout_us = 100000
status_hz = 200.0
commissioned = false

[[subsystems]]
name = "@NAME@"
path = "@NAME@.toml"
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
    deps = [
        ":messages_py",
        "//talOS/python:testing",
        "@rules_python//python/runfiles",
    ],
)
"""

PY_NODE = '''\
"""@PASCAL@: one motor driven at the velocity /@NAME@/target asks for.

Every tick the @NAME@ drives the motor at the target's velocity while the
robot is enabled and the target is fresh and enabled, and reports what it did
on /@NAME@/state. Gains live in the robot config ([motors.@NAME@_motor]
slot0); the motor controller closes the velocity loop onboard.

Nothing publishes /@NAME@/target yet. A teleop or autonomous producer
publishes its own lane, /@NAME@/target/<source>, and a line in the arbiter
picks one and republishes it on /@NAME@/target.
"""

import talos
from @NAME@ import topics
from @NAME@.@PASCAL@State import @PASCAL@State
from @NAME@.@PASCAL@Target import @PASCAL@Target


class @PASCAL@:
    def __init__(self, node):
        self.node = node
        self.motor = talos.Motor(node, "@NAME@_motor")
        # Optional: the graph lint accepts a target nobody publishes yet.
        self.target = node.subscribe(@PASCAL@Target, topics.TARGET,
                                     optional=True)
        self.state = node.publish(@PASCAL@State, topics.STATE)

    def update(self):
        enabled = talos.robot_enabled(self.node)
        target = self.target.value  # None until the first
        if enabled and self.target.fresh() and target.enabled:
            # Otherwise nothing is set and the flush sends neutral.
            self.motor.set_velocity(target.velocity_rps)
        self.state.send(enabled=enabled, velocity_rps=self.motor.velocity,
                        connected=self.motor.connected)
'''

PY_POSITION_NODE = '''\
"""@PASCAL@: one motor driven to the position /@NAME@/target asks for."""

import talos
from @NAME@ import topics
from @NAME@.@PASCAL@State import @PASCAL@State
from @NAME@.@PASCAL@Target import @PASCAL@Target


class @PASCAL@:
    def __init__(self, node):
        self.node = node
        self.motor = talos.Motor(node, "@NAME@_motor")
        self.tolerance_rot = node.param("tolerance_rot", 0.05)
        self.goal_rot = 0.0
        self.target = node.subscribe(@PASCAL@Target, topics.TARGET,
                                     optional=True)
        self.state = node.publish(@PASCAL@State, topics.STATE)

    def update(self):
        enabled = talos.robot_enabled(self.node)
        target = self.target.value
        active = enabled and self.target.fresh() and target.enabled
        if active:
            self.goal_rot = target.position_rot
            self.motor.set_motion_magic(self.goal_rot)
        pos = self.motor.position
        at_goal = active and abs(pos - self.goal_rot) <= self.tolerance_rot
        self.state.send(enabled=enabled, position_rot=pos,
                        connected=self.motor.connected, at_goal=at_goal)
'''

PY_TEST = '''\
"""Node tests: the class on talos.testing.Sim, and the binary."""

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
    def run_node(self, enabled=True, target_enabled=True, seconds=0.05):
        """Publishes one 30 rps target at t0, runs, returns (request, state)."""
        sim = Sim(CONFIG)
        sim.add(node_py.@PASCAL@, "@NAME@")
        hw = SimHardware(sim)
        hw.enable(enabled)
        hw.set_motor("@NAME@", "@NAME@_motor", 0.0, 12.5)
        sim.publish(@PASCAL@Target, topics.TARGET, enabled=target_enabled,
                    velocity_rps=30.0)
        sim.step(seconds)
        return (hw.motor_request("@NAME@", "@NAME@_motor"),
                sim.latest(@PASCAL@State, topics.STATE))

    def test_fresh_target_drives_the_motor(self):
        request, state = self.run_node()
        self.assertEqual(request.mode, talos.Mode.VELOCITY)
        self.assertEqual(request.demand, 30.0)
        self.assertTrue(state.enabled)
        self.assertEqual(state.velocity_rps, 12.5)
        self.assertTrue(state.connected)

    def test_failed_gate_sends_neutral(self):
        cases = [
            ("disabled robot", dict(enabled=False)),
            ("disabled target", dict(target_enabled=False)),
            ("stale target", dict(seconds=0.3)),  # past command_timeout_us
        ]
        for name, inputs in cases:
            with self.subTest(name):
                request, state = self.run_node(**inputs)
                self.assertEqual(request.mode, talos.Mode.NEUTRAL)
                # State keeps publishing whatever the gate says.
                self.assertEqual(state.enabled, inputs.get("enabled", True))


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

PY_TESTDATA_ROBOT = """\
# @PASCAL@ test robot: one subsystem, one motor.
[robot]
name = "@NAME@_test"
period_us = @PERIOD@
command_timeout_us = 100000

[subsystems.@NAME@.motors.@NAME@_motor]
type = "TalonFX"
bus = "test"
can_id = 1
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }
"""

SUBSYSTEM_TOML = """\
# @PASCAL@ subsystem, generated by //tools:new_subsystem. Parsed via the
# [[subsystems]] manifest in @ROBOT@/main_processor/configuration/robot.toml
# and merged with the other subsystem files into one canonical view before
# validation.
[subsystem]
name = "@NAME@"
node = "//@PACKAGE@:node"
period_us = @PERIOD@
# Zero-code robot wiring: uncomment to arbitrate /@NAME@/target, bind a Driver
# Station button on /@NAME@/target/teleop, and stream /@NAME@/state to Studio
# without editing any C++ or Python files in other packages.
# arbitrate = true
# teleop_button = 3
# teleop_demand = 20.0
# telemetry = true
# Device names are robot-wide, so the key carries the subsystem's name.
[motors.@NAME@_motor]
type = "TalonFX"
bus = "rio"
# PLACEHOLDER: the lowest id no other subsystem file uses, not the motor's.
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

MANIFEST_ENTRY = """\

[[subsystems]]
name = "@NAME@"
path = "subsystems/@NAME@.toml"
"""

STANDALONE_MODULE = """\
module(name = "@NAME@", version = "0.1.0")

bazel_dep(name = "talos", version = "0.1.0")
@EXTRA_DEPS@"""

STANDALONE_DEFAULTS_TOML = """\
# @PASCAL@ subsystem defaults, shipped by the @@NAME@ repository.
# A robot manifest layers its own overrides (physical CAN ids, inversions, and
# tuned slot0 gains) on top of this file via `defaults = ...`.
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


# ---------------------------------------------------------------- generation

def package_files(name, lang, standalone=False, kind="velocity"):
    """{relative path: content} of the new package."""
    values = dict(NAME=name, PASCAL=pascal(name), PERIOD=PERIOD_US)
    messages = fill(MESSAGES_BUILD, **values)
    fbs_template = POSITION_FBS if kind == "position" else FBS
    files = {f"{name}_message.fbs": fill(fbs_template, **values)}
    if lang == "cpp":
        node_includes = [f"{name}/topics.h",
                         f"{name}/{name}_message_generated.h",
                         "talOS/devices/devices.h", "talOS/node/node.h"]
        test_includes = ["rules_cc/cc/runfiles/runfiles.h",
                         f"{name}/{name}.h", "talOS/devices/testing.h",
                         "talOS/node/testing.h"]
        data = ["testdata/robot.toml", f"testdata/{name}.toml", ":node"]
        build = fill(CC_BUILD, MESSAGES=messages,
                     DATA="[\n" + string_list(data, 4), **values)
        if standalone:
            build = build.replace('"//talOS/', '"@talos//talOS/')
            build = ('exports_files(["defaults.toml"])\n\n' + build)
        hdr_template = CC_POSITION_HEADER if kind == "position" else CC_HEADER
        cc_test = fill(CC_TEST, INCLUDES=includes(test_includes), **values)
        if kind == "position":
            cc_test = (cc_test
                       .replace("Mode::kVelocity", "Mode::kMotionMagic")
                       .replace("velocity_rps()", "position_rot()"))
        files.update({
            "BUILD": build,
            f"{name}.h": fill(hdr_template, INCLUDES=includes(node_includes),
                              **values),
            f"{name}.cc": fill(CC_SOURCE, **values),
            "node_test.cc": cc_test,
            "testdata/robot.toml": fill(CC_TESTDATA_ROBOT, **values),
            f"testdata/{name}.toml": fill(CC_TESTDATA_SUBSYSTEM, **values),
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
        from_imports = "\n".join(
            f"from {module} import {imports[module]}"
            for module in sorted(imports, key=str.lower))
        build = fill(PY_BUILD, MESSAGES=messages,
                     DATA="[\n" + string_list(data, 4), **values)
        if standalone:
            build = build.replace('"//talOS/', '"@talos//talOS/')
            build = ('exports_files(["defaults.toml"])\n\n' + build)
        node_template = PY_POSITION_NODE if kind == "position" else PY_NODE
        py_test = fill(PY_TEST, FROM_IMPORTS=from_imports, **values)
        if kind == "position":
            py_test = (py_test
                       .replace("velocity_rps=30.0", "position_rot=30.0")
                       .replace("talos.Mode.VELOCITY", "talos.Mode.MOTION_MAGIC")
                       .replace("state.velocity_rps, 12.5", "state.position_rot, 0.0"))
        files.update({
            "BUILD": build,
            f"{name}.py": fill(node_template, **values),
            "node_test.py": py_test,
            "testdata/robot.toml": fill(PY_TESTDATA_ROBOT, **values),
        })
    if standalone:
        extra_deps = (
            'bazel_dep(name = "rules_cc", version = "0.2.18")\n'
            'bazel_dep(name = "googletest", version = "1.17.0.bcr.2")\n'
            if lang == "cpp" else
            'bazel_dep(name = "rules_python", version = "2.0.1")\n'
        )
        files["MODULE.bazel"] = fill(
            STANDALONE_MODULE, EXTRA_DEPS=extra_deps, **values)
        files["defaults.toml"] = fill(STANDALONE_DEFAULTS_TOML, **values)
    return files


def includes(paths):
    # clang-format's SortIncludes order within one block.
    return "\n".join(f'#include "{path}"' for path in sorted(paths))


def used_can_ids(configuration):
    """Every can_id in the robot's manifest and subsystem files."""
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

    paths = [os.path.join(configuration, "robot.toml")]
    subsystems = os.path.join(configuration, "subsystems")
    if os.path.isdir(subsystems):
        paths += [os.path.join(subsystems, f)
                  for f in sorted(os.listdir(subsystems)) if f.endswith(".toml")]
    for path in paths:
        with open(path, "rb") as f:
            walk(tomllib.load(f))
    return ids


def insert_label(text, attribute, label, peers, path, rule_name="talos_bundle"):
    """`text` with `label` added to the `attribute = [...]` list of `rule_name`,
    beside the entries matching `peers` in sorted position."""
    bundle = re.search(rf"^{re.escape(rule_name)}\(\n", text, re.M)
    if not bundle:
        raise Refused(f"{path}: no {rule_name}( call to add the node to")
    match = re.compile(rf"^( +){attribute} = \[\n((?:\1 +\"[^\"\n]*\",\n)*)\1\],\n",
                       re.M).search(text, bundle.end())
    if not match:
        raise Refused(f"{path}: {rule_name} has no one-label-per-line "
                      f"`{attribute} = [...]` list; add {label} by hand")
    lines = match.group(2).splitlines(keepends=True)
    indent = (re.match(r" *", lines[0]).group(0) if lines
              else match.group(1) + "    ")
    labels = [line.strip().strip(",").strip('"') for line in lines]
    if label in labels:
        raise Refused(f"{path}: {attribute} already lists {label}")
    at = len(lines)
    matching = [i for i, existing in enumerate(labels) if peers(existing)]
    if matching:
        at = next((i for i in matching if labels[i] > label), matching[-1] + 1)
    lines.insert(at, f'{indent}"{label}",\n')
    return text[:match.start(2)] + "".join(lines) + text[match.end(2):]


def manifest_names(robot_toml):
    with open(robot_toml, "rb") as f:
        doc = tomllib.load(f)
    subsystems = doc.get("subsystems", [])
    if isinstance(subsystems, dict):  # single-file [subsystems.<name>] form
        return set(subsystems)
    return {entry.get("name") for entry in subsystems}


def plan(root, robot, name, lang, kind="velocity"):
    """([(path, old content or None, new content)], placeholder CAN id)."""
    if problem := check_name(name):
        raise Refused(problem)
    processor = os.path.join(root, robot, "main_processor")
    configuration = os.path.join(processor, "configuration")
    robot_toml = os.path.join(configuration, "robot.toml")
    bundle_build = os.path.join(processor, "BUILD")
    for path in (robot_toml, bundle_build):
        if not os.path.isfile(path):
            raise Refused(f"{rel(root, path)} not found; is --robot right?")
    package = os.path.join(processor, name)
    subsystem_toml = os.path.join(configuration, "subsystems", f"{name}.toml")
    if os.path.exists(package):
        raise Refused(f"{rel(root, package)} already exists")
    if os.path.exists(subsystem_toml):
        raise Refused(f"{rel(root, subsystem_toml)} already exists")
    if name in manifest_names(robot_toml):
        raise Refused(f"{rel(root, robot_toml)} already lists subsystem '{name}'")

    used = used_can_ids(configuration)
    can_id = next((i for i in range(1, MAX_CAN_ID + 1) if i not in used), None)
    if can_id is None:
        raise Refused(f"every CAN id 1..{MAX_CAN_ID} is taken in "
                      f"{rel(root, configuration)}")

    label_prefix = f"//{robot}/main_processor/"
    node_label = f"{label_prefix}{name}:node"
    config_label = f"{label_prefix}configuration"
    toml_label = f"{config_label}:subsystems/{name}.toml"
    writes = [(os.path.join(package, path), None, content)
              for path, content in package_files(name, lang, kind=kind).items()]
    writes.append((subsystem_toml, None, fill(
        SUBSYSTEM_TOML, NAME=name, PASCAL=pascal(name), ROBOT=robot,
        PACKAGE=f"{robot}/main_processor/{name}", PERIOD=PERIOD_US,
        CAN_ID=can_id)))

    old = read(robot_toml)
    writes.append((robot_toml, old, old.rstrip("\n") + "\n" +
                   fill(MANIFEST_ENTRY, NAME=name)))

    old = read(bundle_build)
    new = insert_label(old, "nodes", node_label,
                       lambda x: x.startswith(label_prefix) and x.endswith(":node"),
                       rel(root, bundle_build))
    new = insert_label(new, "subsystems", toml_label,
                       lambda x: x.startswith(config_label + ":subsystems/"),
                       rel(root, bundle_build))
    writes.append((bundle_build, old, new))

    # talos_bundle takes the toml as a label, so configuration/BUILD must
    # export it unless a glob already does.
    config_build = os.path.join(configuration, "BUILD")
    old = read(config_build) if os.path.isfile(config_build) else ""
    new_cfg_build = old
    exported = f'"subsystems/{name}.toml"' in new_cfg_build or re.search(
        r"exports_files\(\s*glob\(\s*\[\s*\"subsystems/\*\.toml\"", new_cfg_build)
    if not exported:
        new_cfg_build = new_cfg_build + ("\n" if new_cfg_build else "") + (
            f'exports_files(["subsystems/{name}.toml"])\n')
    if re.search(r"^talos_graph_test\(\n", new_cfg_build, re.M):
        new_cfg_build = insert_label(
            new_cfg_build, "nodes", node_label,
            lambda x: x.startswith(label_prefix) and x.endswith(":node"),
            rel(root, config_build), rule_name="talos_graph_test")
    if new_cfg_build != old:
        writes.append((config_build, old or None, new_cfg_build))
    return writes, can_id


def plan_standalone(target_dir, name, lang, kind="velocity"):
    """[(path, None, content)] for a standalone Bzlmod subsystem repository."""
    if problem := check_name(name):
        raise Refused(problem)
    if os.path.exists(target_dir) and os.listdir(target_dir):
        raise Refused(f"{target_dir} already exists and is not empty")
    return [
        (os.path.join(target_dir, path), None, content)
        for path, content in package_files(
            name, lang, standalone=True, kind=kind).items()
    ]




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


def next_steps(robot, name, can_id):
    package = f"//{robot}/main_processor/{name}"
    configuration = f"{robot}/main_processor/configuration"
    return f"""\
Next:
  bazel test {package}:all
  bazel build //talOS/launcher //talOS/bridge:hardware_node //{robot}/main_processor/...
  bazel run //talOS/launcher -- --config $PWD/{configuration}/robot.toml --describe-only
  {configuration}/subsystems/{name}.toml
    can_id = {can_id} is a placeholder: replace it with the motor's physical id.
    The gains are zero (untuned): tune slot0 there.
  {configuration}/config_golden_test.cc
    pins the hardware config: update it once a human has checked the CAN id."""


def standalone_next_steps(target_dir, name):
    return f"""\
Next (standalone subsystem repo at {target_dir}):
  1. In {target_dir}/MODULE.bazel, point @talos at your talOS checkout or registry:
       local_path_override(module_name = "talos", path = "../talOS")
  2. Run tests inside {target_dir}:
       bazel test //:all
  3. Consume from a robot repo's MODULE.bazel:
       bazel_dep(name = "{name}", version = "0.1.0")
     and layer physical CAN ids + gains over {target_dir}/defaults.toml."""


def main(argv):
    parser = argparse.ArgumentParser(
        prog="bazel run //tools:new_subsystem --",
        description="Scaffold a talos subsystem node package and wire it into "
        "a robot, or generate a standalone Bzlmod subsystem repository.")
    parser.add_argument("name", help="node name, e.g. wrist (talOS/NAMING.md)")
    parser.add_argument("--lang", choices=("cpp", "py"), required=True)
    parser.add_argument("--type", dest="kind", choices=("velocity", "position"),
                        default="velocity",
                        help="control archetype (default: %(default)s)")
    parser.add_argument("--robot", default="2026-robot",
                        help="robot directory (default: %(default)s)")
    parser.add_argument("--standalone", metavar="DIR",
                        help="scaffold a standalone Bzlmod subsystem repo in DIR "
                        "instead of wiring into --robot")
    parser.add_argument("--dry-run", action="store_true",
                        help="print every file and edit without writing")
    args = parser.parse_args(argv)

    root = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
    if not root and not args.standalone:
        print("new_subsystem: BUILD_WORKSPACE_DIRECTORY is unset; run it with "
              "`bazel run //tools:new_subsystem -- ...`", file=sys.stderr)
        return 2
    base = root or os.getcwd()
    try:
        if args.standalone:
            target_dir = (args.standalone if os.path.isabs(args.standalone)
                          else os.path.join(base, args.standalone))
            writes = plan_standalone(target_dir, args.name, args.lang,
                                     kind=args.kind)
            can_id = None
        else:
            writes, can_id = plan(root, args.robot, args.name, args.lang,
                                  kind=args.kind)
    except Refused as refused:
        print(f"new_subsystem: refused: {refused}", file=sys.stderr)
        return 1

    if args.dry_run:
        show(base, writes)
    else:
        for path, _, content in writes:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8",
                      newline=NEWLINES.get(path, "\n")) as f:
                f.write(content)
            print(f"wrote {rel(base, path)}")
    if args.standalone:
        print(standalone_next_steps(rel(base, target_dir), args.name))
    else:
        print(next_steps(args.robot, args.name, can_id))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
