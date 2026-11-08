"""Scaffolds a talos subsystem: a node package plus its robot wiring.

    bazel run //tools:new_subsystem -- <name> --lang cpp|py \\
        [--type velocity|position|estimator] [--robot 2026-robot] \\
        [--standalone DIR] [--dry-run]

Writes <robot>/main_processor/<name>/ (messages, node, Sim test, testdata),
modelled on that robot's shooter (C++) and intake (Python) packages, or for
--type estimator a hardware-free node publishing geometry.PoseEstimate from
the PoseObservations of its `cameras`, then adds
configuration/subsystems/<name>.toml and its [[subsystems]] entry in
configuration/robot.toml, and regenerates configuration/robot_manifest.bzl
(//tools:sync_manifest), which the bundle and graph test read, plus the
manifest of every other config in the robot that includes robot.toml.
Names and CAN ids already taken are read from the effective config, so a
subsystem inherited through `include`, `defaults` or a custom `path` counts.
Everything is checked before anything is written, so a refused run leaves the
tree untouched. --standalone writes the package as its own Bzlmod repository
in DIR instead, building against this checkout.
"""

import argparse
import difflib
import keyword
import os
import sys
import tomllib

from tools import sync_manifest

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

#include <cmath>

@INCLUDES@

namespace @NAME@ {

// Drives the motor to the position its target asks for with Motion Magic
// and reports what it did on /@NAME@/state. Producers write lanes
// /@NAME@/target/<source> with talos::TargetLane; the policy comes from
// [robot.targets] or the subsystem's [targets].
class @PASCAL@ {
 public:
  using Target = @PASCAL@Target;
  using State = @PASCAL@State;

  explicit @PASCAL@(talos::Node& node)
      : motor_{node, "@NAME@_motor"},
        tolerance_rot_{node.param<double>("tolerance_rot", 0.05)},
        target_{node},
        state_{node.publish<State>(topics::kState)} {}

  void update() {
    const Target* t = target_.get();  // null: disabled, stale or released
    const bool drive =
        t != nullptr && motor_.connected() && std::isfinite(t->position_rot());
    const double pos = motor_.position();
    bool at_goal = false;
    if (drive) {
      motor_.set_motion_magic(t->position_rot());
      at_goal = std::abs(pos - t->position_rot()) <= tolerance_rot_;
    }
    state_.send(State{drive, pos, motor_.connected(), at_goal});
  }

 private:
  talos::Motor motor_;
  double tolerance_rot_;
  talos::Target<Target> target_;
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
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

@INCLUDES@

namespace @NAME@ {
namespace {

using namespace std::chrono_literals;
using talos::hardware::Mode;
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
  // Holds the teleop lane the way a producer does: republished every 20 ms.
  void Hold(double demand, std::chrono::milliseconds duration) {
    for (auto t = 0ms; t < duration; t += 20ms) {
      PublishTarget(sim, "@NAME@", "teleop", Target{demand});
      sim.step(20ms);
    }
  }
  Mode Request() const {
    return hw.motor_request("@NAME@", "@NAME@_motor").mode;
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

TEST_F(@PASCAL@Test, DrivesTheMotorAtALiveLane) {
  hw.ideal_motors();
  Teleop();
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
  hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
  hw.enable(false);
  sim.step(10ms);
  Hold(30.0, 60ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());
}

TEST_F(@PASCAL@Test, StaleLaneGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVelocity);

  sim.step(150ms);  // past command_timeout_us: the lane is stale
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());  // state keeps publishing
}

TEST_F(@PASCAL@Test, ReleasedLaneGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVelocity);

  ReleaseTarget<Target>(sim, "@NAME@", "teleop");
  sim.step(40ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
}

TEST_F(@PASCAL@Test, NeverSentGoesNeutral) {
  Teleop();
  sim.step(60ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());
}

TEST_F(@PASCAL@Test, DriverStationLossGoesNeutral) {
  Teleop();
  Hold(30.0, 40ms);
  ASSERT_EQ(Request(), Mode::kVelocity);

  hw.stop_driver_station();  // the bridge stops forwarding DS packets
  Hold(30.0, 160ms);         // past command_timeout_us: the DS is stale
  EXPECT_EQ(Request(), Mode::kNeutral);
}

TEST_F(@PASCAL@Test, NonFiniteDemandGoesNeutral) {
  Teleop();
  Hold(std::numeric_limits<double>::quiet_NaN(), 40ms);
  EXPECT_EQ(Request(), Mode::kNeutral);
  EXPECT_FALSE(Latest().enabled());
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
"""@PASCAL@: one motor driven to the position its target asks for.

Producers write lanes /@NAME@/target/<source> with talos.TargetLane; the
policy comes from [robot.targets] or the subsystem's [targets].
"""

import math

import talos
from @NAME@ import topics
from @NAME@.@PASCAL@State import @PASCAL@State
from @NAME@.@PASCAL@Target import @PASCAL@Target


class @PASCAL@:
    def __init__(self, node):
        self.motor = talos.Motor(node, "@NAME@_motor")
        self.tolerance_rot = node.param("tolerance_rot", 0.05)
        self.target = talos.Target(node, @PASCAL@Target)
        self.state = node.publish(@PASCAL@State, topics.STATE)

    def update(self):
        t = self.target.get()  # None: disabled, stale or released
        drive = (t is not None and self.motor.connected
                 and math.isfinite(t.position_rot))
        pos = self.motor.position
        at_goal = False
        if drive:
            self.motor.set_motion_magic(t.position_rot)
            at_goal = abs(pos - t.position_rot) <= self.tolerance_rot
        self.state.send(enabled=drive, position_rot=pos,
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

    def hold(self, demand, seconds):
        """Republishes the teleop lane every 20 ms, as a producer does."""
        for _ in range(round(seconds / 0.02)):
            self.sim.publish_target(@PASCAL@Target, "@NAME@", "teleop",
                                    velocity_rps=demand)
            self.sim.step(0.02)

    def mode(self):
        return self.hw.motor_request("@NAME@", "@NAME@_motor").mode

    def state(self):
        return self.sim.latest(@PASCAL@State, topics.STATE)

    def test_live_lane_drives_the_motor(self):
        self.teleop()
        self.hold(30.0, 0.06)
        request = self.hw.motor_request("@NAME@", "@NAME@_motor")
        self.assertEqual(request.mode, talos.Mode.VELOCITY)
        self.assertEqual(request.demand, 30.0)
        state = self.state()
        self.assertTrue(state.enabled)
        self.assertEqual(state.velocity_rps, 12.5)
        self.assertTrue(state.connected)

    def test_disabled_robot_sends_neutral(self):
        self.teleop(enabled=False)
        self.hold(30.0, 0.06)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)

    def test_stale_lane_sends_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        self.sim.step(0.15)  # past command_timeout_us
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)  # state keeps publishing

    def test_released_lane_sends_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        self.sim.release_target(@PASCAL@Target, "@NAME@", "teleop")
        self.sim.step(0.04)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)

    def test_never_sent_sends_neutral(self):
        self.teleop()
        self.sim.step(0.06)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)

    def test_driver_station_loss_sends_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        self.hw.stop_driver_station()
        self.hold(30.0, 0.16)  # past command_timeout_us: the DS is stale
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)

    def test_non_finite_demand_sends_neutral(self):
        self.teleop()
        self.hold(float("nan"), 0.04)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)


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

# Zero-code robot wiring: uncomment to have the teleop node write a
# Driver Station button on the /@NAME@/target/teleop lane, without editing any
# C++ or Python files in other packages.
# [[teleop.bindings]]
# button = 3
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
topic every tick. As generated it follows the newest usable observation
(the most certain one when several arrive in one tick) and is valid while
one arrived within observation_timeout_us; fuse odometry here (see
2026-robot/main_processor/pose_estimator, which runs wpimath's
SwerveDrive4PoseEstimator or a GTSAM smoother). It drives nothing, so it
runs whether the robot is enabled or not."""

CC_ESTIMATOR_HEADER = """\
#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
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

  explicit @PASCAL@(talos::Node& node)
      : node_{node},
        timeout_{std::chrono::microseconds{
            node.param<std::int64_t>("observation_timeout_us", 500'000)}},
        state_{node.publish<State>(topics::kState)} {
    for (const std::string& camera :
         node.param<std::vector<std::string>>("cameras")) {
      cameras_.push_back(node.subscribe<Observation>("/" + camera + "/state",
                                                     {.optional = true}));
    }
  }

  void update() {
    const Observation* best = nullptr;
    for (const auto& camera : cameras_) {
      if (!camera.updated()) continue;  // each observation counts once
      const Observation& o = camera.value();
      if (!Usable(o)) {
        ++rejected_;
        continue;
      }
      ++accepted_;
      if (best == nullptr ||
          o.x_std_m() + o.y_std_m() < best->x_std_m() + best->y_std_m()) {
        best = &o;
      }
    }
    if (best != nullptr) {
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

TEST_F(@PASCAL@Test, FollowsAUsableObservation) {
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

TEST_F(@PASCAL@Test, StaleInputIsInvalidAndHoldsThePose) {
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

import math

from @NAME@ import topics
from geometry.PoseEstimate import PoseEstimate
from geometry.PoseObservation import PoseObservation


def usable(o):
    values = (o.pose_x_m, o.pose_y_m, o.pose_yaw_rad, o.x_std_m, o.y_std_m,
              o.yaw_std_rad)
    return (o.valid and o.sample_time_ns > 0
            and all(map(math.isfinite, values))
            and min(o.x_std_m, o.y_std_m, o.yaw_std_rad) > 0)


class @PASCAL@:
    def __init__(self, node):
        self.node = node
        cameras = node.param("cameras")
        if (not isinstance(cameras, list)
                or not all(isinstance(c, str) and c for c in cameras)):
            raise ValueError(f"@NAME@: cameras must list camera node names, "
                             f"got {cameras!r}")
        self.cameras = [
            node.subscribe(PoseObservation, f"/{camera}/state", optional=True)
            for camera in cameras]
        self.timeout_s = node.param("observation_timeout_us", 500_000) / 1e6
        self.state = node.publish(PoseEstimate, topics.STATE)
        self.pose = (0.0, 0.0, 0.0)  # held while no usable observation arrives
        self.sample_time_ns = 0
        self.last_accepted_s = None
        self.accepted = self.rejected = 0

    def update(self):
        best = None
        for camera in self.cameras:
            if not camera.updated:  # each observation counts once
                continue
            o = camera.value
            if not usable(o):
                self.rejected += 1
                continue
            self.accepted += 1
            if best is None or (o.x_std_m + o.y_std_m
                                < best.x_std_m + best.y_std_m):
                best = o
        if best is not None:
            self.pose = (best.pose_x_m, best.pose_y_m, best.pose_yaw_rad)
            self.sample_time_ns = best.sample_time_ns
            self.last_accepted_s = self.node.now
        valid = (self.last_accepted_s is not None
                 and self.node.now - self.last_accepted_s <= self.timeout_s)
        x, y, yaw = self.pose
        self.state.send(sample_time_ns=self.sample_time_ns, pose_x_m=x,
                        pose_y_m=y, pose_yaw_rad=yaw, valid=valid, localized=False,
                        observation_accepted_count=self.accepted,
                        observation_rejected_count=self.rejected)
'''

PY_ESTIMATOR_TEST = '''\
"""Node tests: the class on talos.testing.Sim, and the binary."""

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
            sample_time_ns=round(self.sim.now * 1e9), pose_x_m=x, pose_y_m=y,
            pose_yaw_rad=0.5, x_std_m=std_m, y_std_m=std_m, yaw_std_rad=0.05,
            tag_count=2, valid=valid)

    def state(self):
        return self.sim.latest(PoseEstimate, topics.STATE)

    def test_follows_a_usable_observation(self):
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

    def test_stale_input_is_invalid_and_holds_the_pose(self):
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
# [[subsystems]] manifest in @ROBOT@/main_processor/configuration/robot.toml.
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

def package_files(name, lang, standalone=False, kind="velocity"):
    """{relative path: content} of the new package."""
    values = dict(NAME=name, PASCAL=pascal(name), PERIOD=PERIOD_US)
    files = (estimator_files(name, lang) if kind == "estimator"
             else mechanism_files(name, lang, kind))
    if standalone:
        # talOS labels name the @talos module; the repo root is the package,
        # so defaults.toml is exported after the loads.
        loads, _, rest = files["BUILD"].replace(
            '"//talOS/', '"@talos//talOS/').partition("\n\n")
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
            STANDALONE_ESTIMATOR_DEFAULTS_TOML if kind == "estimator"
            else STANDALONE_DEFAULTS_TOML, **values)
    return files


def mechanism_files(name, lang, kind):
    """{relative path: content} of a one-motor velocity/position package."""
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
        node_template = PY_POSITION_NODE if kind == "position" else PY_NODE
        py_test = fill(PY_TEST, FROM_IMPORTS=from_imports, **values)
        if kind == "position":
            py_test = (py_test
                       .replace("velocity_rps=demand", "position_rot=demand")
                       .replace("talos.Mode.VELOCITY", "talos.Mode.MOTION_MAGIC")
                       .replace("state.velocity_rps, 12.5", "state.position_rot, 0.0"))
        files.update({
            "BUILD": build,
            f"{name}.py": fill(node_template, **values),
            "node_test.py": py_test,
            "testdata/robot.toml": fill(PY_TESTDATA_ROBOT, **values),
        })
    return files


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


def plan(root, robot, name, lang, kind="velocity"):
    """([(path, old content or None, new content)], placeholder CAN id, or
    None for an estimator, which owns no devices)."""
    if problem := check_name(name):
        raise Refused(problem)
    processor = os.path.join(root, robot, "main_processor")
    configuration = os.path.join(processor, "configuration")
    robot_toml = os.path.join(configuration, "robot.toml")
    if not os.path.isfile(robot_toml):
        raise Refused(f"{rel(root, robot_toml)} not found; is --robot right?")
    package = os.path.join(processor, name)
    subsystem_toml = os.path.join(configuration, "subsystems", f"{name}.toml")
    if os.path.exists(package):
        raise Refused(f"{rel(root, package)} already exists")
    if os.path.exists(subsystem_toml):
        raise Refused(f"{rel(root, subsystem_toml)} already exists")
    config = rel(root, robot_toml)
    try:
        subsystems, _ = sync_manifest.effective(root, config)
    except sync_manifest.ManifestError as err:
        raise Refused(f"cannot read {config}: {err}")
    if name in subsystems:
        raise Refused(f"{config} already has subsystem '{name}'")

    can_id = None
    if kind != "estimator":
        used = used_can_ids(subsystems)
        can_id = next((i for i in range(1, MAX_CAN_ID + 1) if i not in used),
                      None)
        if can_id is None:
            raise Refused(f"every CAN id 1..{MAX_CAN_ID} is taken in "
                          f"{rel(root, configuration)}")

    writes = [(os.path.join(package, path), None, content)
              for path, content in package_files(name, lang, kind=kind).items()]
    subsystem = fill(
        ESTIMATOR_SUBSYSTEM_TOML if kind == "estimator" else SUBSYSTEM_TOML,
        NAME=name, PASCAL=pascal(name), ROBOT=robot,
        PACKAGE=f"{robot}/main_processor/{name}", PERIOD=PERIOD_US,
        CAN_ID=can_id)
    writes.append((subsystem_toml, None, subsystem))

    old = read(robot_toml)
    new_robot = old.rstrip("\n") + "\n" + fill(MANIFEST_ENTRY, NAME=name)
    writes.append((robot_toml, old, new_robot))

    # The bundle and graph test take their node and file lists from the
    # generated manifest, so regenerate it from the edited config, and every
    # other manifest in the robot whose config reads robot.toml (an
    # `include` of it), which the new entry changes too.
    overrides = {config: new_robot, rel(root, subsystem_toml): subsystem}
    configs = [config] + [
        other for other in robot_configs(root, robot) if other != config]
    for path in configs:
        try:
            if path != config:
                _, files = sync_manifest.resolve(root, path, overrides)
                if config not in files:
                    continue
            manifest = sync_manifest.generate(root, path, overrides=overrides)
        except sync_manifest.ManifestError as err:
            raise Refused(f"cannot regenerate the manifest of {path}: {err}")
        manifest_file = os.path.join(root, sync_manifest.manifest_path(path))
        old = read(manifest_file) if os.path.isfile(manifest_file) else None
        writes.append((manifest_file, old, manifest))
    return writes, can_id


def robot_configs(root, robot):
    """Workspace-relative configs under `robot` that have a manifest."""
    found = []
    for directory, dirs, files in os.walk(os.path.join(root, robot)):
        dirs[:] = sorted(d for d in dirs if not d.startswith((".", "bazel-")))
        for f in sorted(files):
            if f.endswith("_manifest.bzl"):
                stem = f.removesuffix("_manifest.bzl")
                config = os.path.join(directory, stem + ".toml")
                if os.path.isfile(config):
                    found.append(rel(root, config))
    return found


def plan_standalone(target_dir, name, lang, talos_root, kind="velocity"):
    """[(path, None, content)] for a standalone Bzlmod subsystem repository
    that builds against the talOS checkout at `talos_root` as it stands."""
    if problem := check_name(name):
        raise Refused(problem)
    if os.path.exists(target_dir) and os.listdir(target_dir):
        raise Refused(f"{target_dir} already exists and is not empty")
    files = package_files(name, lang, standalone=True, kind=kind)
    # Real paths, so a symlinked directory (macOS /tmp) cannot skew it.
    talos_path = os.path.relpath(os.path.realpath(talos_root),
                                 os.path.realpath(target_dir)).replace(
                                     os.sep, "/")
    files["MODULE.bazel"] += (
        "\n# The talOS checkout this repository was generated from. Replace it\n"
        "# with a registry version once talos is published to one.\n"
        f'local_path_override(module_name = "talos", path = "{talos_path}")\n')
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


def next_steps(robot, name, can_id):
    package = f"//{robot}/main_processor/{name}"
    configuration = f"{robot}/main_processor/configuration"
    common = f"""\
Next:
  bazel test {package}:all
  bazel build //talOS/launcher //talOS/bridge:hardware_node //{robot}/main_processor/...
  bazel run //talOS/launcher -- --config $PWD/{configuration}/robot.toml --describe-only
  {configuration}/subsystems/{name}.toml"""
    if can_id is None:  # an estimator
        return common + f"""
    List the camera nodes in `cameras`; point telemetry at the pose with
    [studio] pose_topic = "/{name}/state"."""
    return common + f"""
    can_id = {can_id} is a placeholder: replace it with the motor's physical id.
    The gains are zero (untuned): tune slot0 there.
  {configuration}/config_golden_test.cc
    pins the hardware config: update it once a human has checked the CAN id."""


def standalone_next_steps(target_dir, name, kind="velocity"):
    edit = ("list its camera nodes in `cameras`" if kind == "estimator"
            else "add the motor's physical can_id")
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
       bazel run //tools:sync_manifest -- <robot.toml>"""


def shown(base, path):
    """`path` relative to `base` when inside it, else absolute."""
    relative = rel(base, path)
    return os.path.abspath(path) if relative.startswith("../") else relative


def main(argv):
    parser = argparse.ArgumentParser(
        prog="bazel run //tools:new_subsystem --",
        description="Scaffold a talos subsystem node package and wire it into "
        "a robot, or generate a standalone Bzlmod subsystem repository.")
    parser.add_argument("name", help="node name, e.g. wrist (talOS/NAMING.md)")
    parser.add_argument("--lang", choices=("cpp", "py"), required=True)
    parser.add_argument("--type", dest="kind",
                        choices=("velocity", "position", "estimator"),
                        default="velocity",
                        help="archetype: a velocity or position mechanism, or "
                        "a hardware-free pose estimator (default: %(default)s)")
    parser.add_argument("--robot", default="2026-robot",
                        help="robot directory (default: %(default)s)")
    parser.add_argument("--standalone", metavar="DIR",
                        help="scaffold a standalone Bzlmod subsystem repo in DIR "
                        "instead of wiring into --robot")
    parser.add_argument("--dry-run", action="store_true",
                        help="print every file and edit without writing")
    args = parser.parse_args(argv)

    root = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
    if not root:
        print("new_subsystem: BUILD_WORKSPACE_DIRECTORY is unset; run it with "
              "`bazel run //tools:new_subsystem -- ...`", file=sys.stderr)
        return 2
    try:
        if args.standalone:
            target_dir = os.path.join(root, args.standalone)
            writes = plan_standalone(target_dir, args.name, args.lang, root,
                                     kind=args.kind)
            can_id = None
        else:
            writes, can_id = plan(root, args.robot, args.name, args.lang,
                                  kind=args.kind)
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
    if args.standalone:
        print(standalone_next_steps(shown(root, target_dir), args.name,
                                    args.kind))
    else:
        print(next_steps(args.robot, args.name, can_id))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
