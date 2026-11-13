#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "config_parser.h"

namespace talos::config {
namespace {

// One valid motor, so a fixture only spells out what it is testing.
constexpr std::string_view kMotor = R"(
[subsystems.test.motors.m1]
can_id = 1
)";

RobotConfig Parse(std::string_view body) {
  return ParseRobotConfigString(std::string(kMotor) + std::string(body));
}

void ExpectRejected(std::string_view body) {
  EXPECT_THROW(Parse(body), std::invalid_argument) << body;
}

TEST(ConfigTest, RejectsNonBooleanCriticalFlag) {
  for (const auto value : {"0", "\"false\"", "[]", "{}"}) {
    EXPECT_THROW(
        ParseRobotConfigString(std::string{"[subsystems.test]\ncritical = "} +
                               value + std::string{kMotor}),
        std::invalid_argument);
  }
}

// REQ-CONTRACT-17
TEST(ConfigTest, RejectsMalformedStructuralTablesAndUnknownMotorKeys) {
  for (const auto* text :
       {"robot = 1\n[subsystems.test]\n", "[subsystems]\ntest = 1\n"}) {
    EXPECT_THROW(ParseRobotConfigString(text), std::invalid_argument);
  }
  for (const auto* section :
       {"motors", "sensors", "digital_inputs", "digital_outputs",
        "analog_inputs", "encoders", "pwm_outputs"}) {
    EXPECT_THROW(ParseRobotConfigString(std::string("[subsystems.test]\n") +
                                        section + " = 1\n"),
                 std::invalid_argument);
  }
  ExpectRejected("canid = 2\n");
  for (int slot = 0; slot < 3; ++slot) {
    ExpectRejected("slot" + std::to_string(slot) + " = 1\n");
    ExpectRejected("[subsystems.test.motors.m1.slot" + std::to_string(slot) +
                   "]\nunknown = 1\n");
  }
}

TEST(ConfigTest, RejectsInvalidFollowerAndGravityDeclarations) {
  ExpectRejected("oppose_master = false\n");
  ExpectRejected("follow = \"missing\"\n");
  ExpectRejected("[subsystems.test.motors.m1.slot0]\ngravity_type = \"bad\"\n");
  ExpectRejected(
      "follow = \"sensor\"\n[subsystems.test.sensors.sensor]\ncan_id = 2\n");
  ExpectRejected(
      "follow = \"other\"\n[subsystems.other.motors.other]\ncan_id = 2\n");
}

// An absent key keeps Phoenix's default (an enabled normally open switch), so
// a robot that never names its limit inputs behaves as before the keys existed.
TEST(ConfigTest, MotorLimitSwitchesDefaultToPhoenixAndRejectUnknownTypes) {
  using hardware::LimitSwitch;
  const auto defaults = Parse("").hardware.motors.at(0);
  EXPECT_EQ(defaults.forward_limit_switch, LimitSwitch::kNormallyOpen);
  EXPECT_EQ(defaults.reverse_limit_switch, LimitSwitch::kNormallyOpen);
  const auto set = Parse(
                       "forward_limit_switch = \"NormallyClosed\"\n"
                       "reverse_limit_switch = \"Disabled\"\n")
                       .hardware.motors.at(0);
  EXPECT_EQ(set.forward_limit_switch, LimitSwitch::kNormallyClosed);
  EXPECT_EQ(set.reverse_limit_switch, LimitSwitch::kDisabled);
  ExpectRejected("forward_limit_switch = \"normally_open\"\n");
  ExpectRejected("reverse_limit_switch = false\n");
}

TEST(ConfigTest, DeterministicOrderingIndependentOfTomlTableOrder) {
  const auto config = ParseRobotConfigString(R"(
[subsystems.sub_z.motors.z_motor]
can_id = 1
[subsystems.sub_a.motors.b_motor]
can_id = 2
[subsystems.sub_a.motors.a_motor]
can_id = 3
[subsystems.sub_a.sensors.s_sensor]
can_id = 4
)");
  ASSERT_EQ(config.hardware.motors.size(), 3u);
  const auto reordered = ParseRobotConfigString(R"(
[subsystems.sub_a.sensors.s_sensor]
can_id = 4
[subsystems.sub_a.motors.a_motor]
can_id = 3
[subsystems.sub_a.motors.b_motor]
can_id = 2
[subsystems.sub_z.motors.z_motor]
can_id = 1
)");
  EXPECT_EQ(config.hardware, reordered.hardware);
  EXPECT_EQ(config.subsystems.at("sub_a").motors,
            reordered.subsystems.at("sub_a").motors);
}

TEST(ConfigTest, RejectsParserOwnedErrors) {
  ExpectRejected("[subsystems.test.motors.m2]\nbus = \"rio\"\n");  // no can_id
  ExpectRejected("[subsystems.test.motors.m2]\ncan_id = \"2\"\n");
  ExpectRejected(
      "[subsystems.test.motors.m2]\ncan_id = 2\ntype = \"SparkMax\"\n");
  ExpectRejected(
      "[subsystems.test.sensors.s]\ncan_id = 2\ntype = \"kCANcoder\"\n");
  ExpectRejected(
      "[subsystems.test.encoders.e]\ntype = \"Quadrature\"\ndio_a = 0\ndio_b = "
      "1\n");
  ExpectRejected("[subsystems.test.encoders.e]\ndio_a = 0\n");  // no dio_b
  ExpectRejected("[subsystems.test.analog_inputs.a]\nanalog = 0\n");
  ExpectRejected("[subsystems.test.pwm_outputs.p]\npwm = 0\n");
  ExpectRejected(
      "[subsystems.test.motors.m2]\ncan_id = 2\nfeedback = \"rotor\"\n");
  ExpectRejected(
      "[subsystems.test.motors.m2]\ncan_id = 2\nfeedback = "
      "\"RemoteCANcoder\"\n");
  ExpectRejected(R"(
[subsystems.test.sensors.enc]
can_id = 2
[subsystems.test.motors.m2]
can_id = 3
feedback_sensor = "enc"
)");  // Rotor feedback naming a sensor.
  // A rotor frame restarts with the TalonFX, so it is never absolute.
  ExpectRejected(
      "[subsystems.test.motors.m2]\ncan_id = 2\nposition_frame = "
      "\"absolute\"\n");
  ExpectRejected(
      "[subsystems.test.motors.m2]\ncan_id = 2\nposition_frame = "
      "\"power_on\"\n");
  ExpectRejected(R"(
[subsystems.test.motors.steer]
can_id = 2
feedback = "RemoteCANcoder"
feedback_sensor = "ghost_encoder"
)");
  ExpectRejected("[unknown_top_level]\nfoo = 1\n");
  EXPECT_THROW(
      ParseRobotConfigString("typo_key = \"val\"\n" + std::string(kMotor)),
      std::invalid_argument);
}

TEST(ConfigTest, RejectsCrossSubsystemClaims) {
  ExpectRejected(R"(
[subsystems.subA.sensors.encA]
can_id = 20
[subsystems.subB.motors.steerB]
can_id = 2
feedback = "RemoteCANcoder"
feedback_sensor = "encA"
)");
  ExpectRejected(R"(
[subsystems.drivetrain.motors.steer]
can_id = 2
[subsystems.drivetrain.geometry]
modules = [{ name = "m1", drive = "m1", steer = "steer" }]
)");
  ExpectRejected(R"(
[subsystems.drivetrain.motors.steer]
can_id = 2
[subsystems.drivetrain.geometry]
modules = [{ name = "m1", drive = "steer", steer = "nonexistent_steer" }]
)");
}

// Ranges and exclusive ownership belong to hardware::Validate; these prove the
// parsed fields reach it.
TEST(ConfigTest, ValidateRunsOnParsedConfig) {
  ExpectRejected("[subsystems.other.motors.m2]\ncan_id = 1\n");
  ExpectRejected(R"(
[subsystems.test.digital_inputs.d]
dio = 3
[subsystems.test.encoders.e]
dio_a = 3
dio_b = 4
)");
  ExpectRejected("[robot]\nperiod_us = 500\n");
  ExpectRejected(
      "[subsystems.test.sensors.s]\ncan_id = 20\noffset_rot = 0.5\n");
  ExpectRejected(
      "[subsystems.test.motors.m2]\ncan_id = 2\nslot0 = { p = -0.5 }\n");
  ExpectRejected(R"(
[subsystems.test.sensors.imu]
type = "Pigeon2"
can_id = 30
[subsystems.test.motors.steer]
can_id = 2
feedback = "RemoteCANcoder"
feedback_sensor = "imu"
)");
  // A robot with no motor is valid (REQ-CONTRACT-35).
  EXPECT_NO_THROW(
      ParseRobotConfigString("[subsystems.test.sensors.s]\ncan_id = 1\n"));
}

// Every node's Driver Station view expires at command_timeout_us, and a packet
// can follow the previous one by 20000 us plus a gateway period: the timeout
// spans two such gaps, 2 * (20000 + period_us).
TEST(ConfigTest, CommandTimeoutSpansTwoDriverStationPacketGaps) {
  const auto robot = [](int period_us, int timeout_us) {
    return Parse("[robot]\nperiod_us = " + std::to_string(period_us) +
                 "\ncommand_timeout_us = " + std::to_string(timeout_us) + "\n");
  };
  EXPECT_THROW(robot(5000, 49999), std::invalid_argument);
  EXPECT_NO_THROW(robot(5000, 50000));
  EXPECT_THROW(robot(20000, 79999), std::invalid_argument);
  EXPECT_NO_THROW(robot(20000, 80000));
}

// The bridge holds an actuator owner's last request for at least two of its
// periods and at least half the command timeout, and the gateway refuses a
// merged command whose oldest observation is command_timeout_us old, so every
// owner's period is at most a quarter of it.
TEST(ConfigTest, ActuatorOwnerPeriodIsAtMostAQuarterOfTheCommandTimeout) {
  const auto parse = [](const std::string& subsystems) {
    return ParseRobotConfigString("[robot]\ncommand_timeout_us = 100000\n" +
                                  subsystems);
  };
  // Each output kind alone makes its subsystem an actuator owner (the config
  // needs a motor somewhere, so the others sit beside kMotor's).
  for (const std::string device :
       {"motors.m]\ncan_id = 2\n", "digital_outputs.d]\ndio = 0\n",
        "pwm_outputs.p]\nchannel = 0\n"}) {
    const auto owner = [&](int period_us) {
      return parse(
          "[subsystems.arm]\nperiod_us = " + std::to_string(period_us) +
          "\n[subsystems.arm." + device + std::string{kMotor});
    };
    EXPECT_NO_THROW(owner(25000)) << device;
    EXPECT_THROW(owner(25001), std::invalid_argument) << device;
  }
  // A node that only reads may be slow.
  EXPECT_NO_THROW(
      parse("[subsystems.camera]\nperiod_us = 200000\n"
            "[subsystems.camera.digital_inputs.d]\ndio = 0\n" +
            std::string{kMotor}));
}

// REQ-CONTRACT-17
TEST(ConfigTest, ControllerAddressOverridesAndRejectsInvalidEndpoints) {
  const auto config = Parse(R"(
[robot]
controller_address = "10.56.87.2"
controller_port = 5804
local_port = 5805
)");

  // CLI override wins over the deployment configuration.
  EXPECT_EQ(ResolveControllerAddress(config, "10.0.0.5"), "10.0.0.5");
  EXPECT_EQ(ResolveControllerAddress(config, ""), "10.56.87.2");
  EXPECT_EQ(ResolveControllerPort(config, 5900), 5900);
  EXPECT_EQ(ResolveControllerPort(config, 0), 5804);
  EXPECT_EQ(ResolveLocalPort(config, 5901), 5901);
  EXPECT_EQ(ResolveLocalPort(config, 0), 5805);

  ExpectRejected("[robot]\ncontroller_port = 0\n");
  ExpectRejected("[robot]\ncontroller_address = \"\"\n");
  // The bridge's socket takes an IPv4 literal and resolves no hostname.
  ExpectRejected("[robot]\ncontroller_address = \"roborio-5687-frc.local\"\n");
  EXPECT_THROW(ResolveControllerAddress(config, "roborio.local"),
               std::invalid_argument);
  // Port flags: a decimal port in range, never wrapped or truncated.
  EXPECT_EQ(ParsePortFlag("--remote-port", "5802"), 5802);
  for (const char* bad : {"0", "65536", "70000", "-1", "5802x", ""}) {
    EXPECT_THROW(ParsePortFlag("--remote-port", bad), std::invalid_argument)
        << bad;
  }
}

// Log retention keys of the wrong type or sign are refused, not defaulted.
TEST(ConfigTest, RejectsInvalidLogRetention) {
  EXPECT_EQ(Parse("[robot]\nlog_keep_sessions = 3\n").log_keep_sessions, 3);
  ExpectRejected("[robot]\nlog_keep_sessions = \"3\"\n");
  ExpectRejected("[robot]\nlog_keep_sessions = 0\n");
  ExpectRejected("[robot]\nlog_max_bytes = 1.5\n");
  ExpectRejected("[robot]\nlog_max_bytes = -1\n");
}

TEST(ConfigTest, RejectsNonBooleanTargetCoverage) {
  ExpectRejected("[robot]\nrequire_target_coverage = 1\n");
  ExpectRejected("[robot]\nrequire_target_coverage = \"true\"\n");
}

// REQ-CONTRACT-09: a commissioned robot requires target coverage, so a
// config that commissions it and turns coverage off contradicts itself and
// is refused, naming both keys; either key alone loads.
TEST(ConfigTest, CommissionedRefusesTargetCoverageTurnedOff) {
  try {
    Parse("[robot]\ncommissioned = true\nrequire_target_coverage = false\n");
    ADD_FAILURE() << "loaded a commissioned config without coverage";
  } catch (const std::invalid_argument& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("commissioned"), std::string::npos) << what;
    EXPECT_NE(what.find("require_target_coverage"), std::string::npos) << what;
  }
  EXPECT_NO_THROW(Parse(
      "[robot]\ncommissioned = false\nrequire_target_coverage = false\n"));
  EXPECT_NO_THROW(
      Parse("[robot]\ncommissioned = true\nrequire_target_coverage = true\n"));
}

TEST(ConfigTest, RejectsInvalidStudioOptions) {
  ExpectRejected("[studio]\nhttp_port = 70000\n");
  ExpectRejected("[studio]\nmax_clients = 0\n");
  ExpectRejected("[studio]\nmax_clients = 65\n");
  ExpectRejected("[studio]\nmax_clients = \"4\"\n");
  ExpectRejected("[studio]\nbridge = \"\"\n");
}

using Sources = std::vector<std::string>;

TEST(ConfigTest, TargetPolicyDefaultsAndOverrides) {
  const auto defaults = ParseRobotConfigString(kMotor);
  EXPECT_EQ(defaults.targets.teleop, (Sources{"teleop"}));
  EXPECT_EQ(defaults.targets.autonomous, (Sources{"auto"}));
  EXPECT_TRUE(defaults.targets.test.empty());
  EXPECT_EQ(defaults.TargetsFor("test"), defaults.targets);
  EXPECT_EQ(defaults.TargetsFor("not_configured"), defaults.targets);

  const auto cfg = Parse(R"(
[robot.targets]
autonomous = ["auto", "align"]
test = ["test"]
[subsystems.wrist]
[subsystems.wrist.targets]
teleop = ["align", "teleop"]
test = []
[subsystems.roller]
)");
  EXPECT_EQ(cfg.targets.teleop, (Sources{"teleop"}));
  EXPECT_EQ(cfg.targets.autonomous, (Sources{"auto", "align"}));
  const TargetPolicy& wrist = cfg.TargetsFor("wrist");
  EXPECT_EQ(wrist.teleop, (Sources{"align", "teleop"}));
  EXPECT_EQ(wrist.autonomous, (Sources{"auto", "align"}));  // the robot's
  EXPECT_TRUE(wrist.test.empty());
  EXPECT_EQ(wrist.Sources(), (Sources{"align", "teleop", "auto"}));
  EXPECT_EQ(cfg.TargetsFor("roller"), cfg.targets);
}

TEST(ConfigTest, RejectsBadTargetPolicies) {
  for (const char* body : {
           "[robot.targets]\nteleoperated = [\"teleop\"]\n",  // unknown mode
           "[robot.targets]\nteleop = \"teleop\"\n",          // not an array
           "[robot.targets]\nteleop = [1]\n",                 // not a name
           "[robot.targets]\nteleop = [\"Teleop\"]\n",        // not a segment
           "[robot.targets]\nteleop = [\"tgt\"]\n",           // abbreviation
           "[robot.targets]\nteleop = [\"a\", \"a\"]\n",      // twice
           "[robot]\ntargets = 3\n",
           "[subsystems.test.targets]\nauto = [\"auto\"]\n",
           "[subsystems.test]\ntargets = [\"teleop\"]\n",
       }) {
    EXPECT_THROW(
        ParseRobotConfigString(std::string(body) + std::string(kMotor)),
        std::invalid_argument)
        << body;
  }
}

class ManifestTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::path(::testing::TempDir()) /
           ::testing::UnitTest::GetInstance()->current_test_info()->name();
    std::filesystem::create_directories(dir_ / "subsystems");
  }
  void Write(const std::string& rel, std::string_view content) {
    std::ofstream(dir_ / rel) << content;
  }
  std::string Robot() const { return (dir_ / "robot.toml").string(); }

  std::filesystem::path dir_;
};

TEST_F(ManifestTest, RejectsUnknownDeviceKeysInDefaults) {
  Write("robot.toml",
        "[[subsystems]]\nname = \"test\"\npath = \"subsystems/test.toml\"\n");
  Write("subsystems/test.toml",
        "[subsystem]\nname = \"test\"\ndefaults = \"defaults.toml\"\n"
        "[sensors.encoder]\ncan_id = 2\n");
  Write("subsystems/defaults.toml", "[sensors.encoder]\noffest_rot = 0.25\n");
  EXPECT_THROW(ParseRobotConfig(Robot()), std::invalid_argument);
}

// `[target]` for `[targets]` would otherwise parse as a node parameter and
// leave the subsystem on the default target policy.
// REQ-CONTRACT-10
TEST_F(ManifestTest, RefusesASingularSectionNamingTheKeyAndFile) {
  Write("robot.toml",
        "[[subsystems]]\nname = \"arm\"\npath = \"subsystems/arm.toml\"\n");
  const auto expect_refused = [this](const std::string& file) {
    try {
      ParseRobotConfig(Robot());
      ADD_FAILURE() << "a [target] section in " << file << " was accepted";
    } catch (const std::invalid_argument& error) {
      const std::string what = error.what();
      EXPECT_NE(what.find(file), std::string::npos) << what;
      EXPECT_NE(what.find("'target'"), std::string::npos) << what;
    }
  };
  Write("subsystems/arm.toml",
        "[subsystem]\nname = \"arm\"\n[target]\nteleop = [\"align\"]\n");
  expect_refused("subsystems/arm.toml");
  // In a defaults file, the error names the defaults file, not the
  // subsystem file the key would have been merged into.
  Write("subsystems/arm.toml",
        "[subsystem]\nname = \"arm\"\ndefaults = \"base.toml\"\n");
  Write("subsystems/base.toml", "[target]\nteleop = [\"align\"]\n");
  expect_refused("subsystems/base.toml");
  // The single-file form is refused too.
  EXPECT_THROW(ParseRobotConfigString("[subsystems.arm.target]\nteleop = "
                                      "[\"align\"]\n" +
                                      std::string(kMotor)),
               std::invalid_argument);
}

// A [trajectories] entry a node could not open on the robot fails config load
// instead, naming the file that sets it and the key (REQ-CONTRACT-10).
// REQ-CONTRACT-31
TEST_F(ManifestTest, RefusesABadTrajectoryEntryNamingTheFileAndKey) {
  Write("robot.toml",
        "[[subsystems]]\nname = \"auto\"\npath = \"subsystems/auto.toml\"\n");
  const std::string missing =
      (dir_ / "trajectories" / "two_piece.traj").lexically_normal().string();
  struct Case {
    const char* name;
    const char* section;
    std::vector<std::string> named;  // what the error must contain
    bool top_level = false;  // a bare key, so written before [subsystem]
  };
  const Case cases[] = {
      {"missing file",
       "[trajectories]\ntwo_piece = \"../trajectories/"
       "two_piece.traj\"\n",
       {"trajectories.two_piece", missing}},
      {"not a string",
       "[trajectories]\ntwo_piece = 3\n",
       {"trajectories.two_piece"}},
      {"empty path",
       "[trajectories]\ntwo_piece = \"\"\n",
       {"trajectories.two_piece"}},
      {"not a segment name",
       "[trajectories]\nTwoPiece = \"a.traj\"\n",
       {"trajectories.TwoPiece"}},
      {"not a table",
       "trajectories = \"a.traj\"\n",
       {"'trajectories'"},
       /*top_level=*/true},
      {"singular section",
       "[trajectory]\ntwo_piece = \"a.traj\"\n",
       {"'trajectory'"}},
      // Merged into the subsystem's table and then replaced by a top-level
      // [trajectories], it would be resolved and bundled but never read.
      {"under the header",
       "[subsystem.trajectories]\ntwo_piece = \"a.traj\"\n",
       {"[subsystem.trajectories]"}},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.name);
    const std::string header = "[subsystem]\nname = \"auto\"\n";
    Write("subsystems/auto.toml",
          c.top_level ? c.section + header : header + c.section);
    try {
      ParseRobotConfig(Robot());
      ADD_FAILURE() << "accepted";
    } catch (const std::invalid_argument& error) {
      const std::string what = error.what();
      EXPECT_NE(what.find("subsystems/auto.toml"), std::string::npos) << what;
      for (const auto& part : c.named) {
        EXPECT_NE(what.find(part), std::string::npos) << what;
      }
    }
  }
}

// Paths resolve against the subsystem file, not robot.toml, so the node opens
// the same file the manifest lists; an undeclared name throws naming it
// (REQ-CONTRACT-19).
// REQ-CONTRACT-31
TEST_F(ManifestTest, TrajectoryPathResolvesAgainstTheSubsystemFile) {
  std::filesystem::create_directories(dir_ / "trajectories");
  Write("trajectories/two_piece.traj", "{}\n");
  Write("robot.toml",
        "[[subsystems]]\nname = \"auto\"\npath = \"subsystems/auto.toml\"\n");
  Write("subsystems/auto.toml",
        "[subsystem]\nname = \"auto\"\n"
        "[trajectories]\ntwo_piece = \"../trajectories/two_piece.traj\"\n"
        "[motors.m]\ncan_id = 1\n");
  const auto cfg = ParseRobotConfig(Robot());
  const auto path = cfg.TrajectoryPath("auto", "two_piece");
  EXPECT_TRUE(path.is_absolute()) << path;
  EXPECT_TRUE(
      std::filesystem::equivalent(path, dir_ / "trajectories/two_piece.traj"))
      << path;
  for (const auto& [subsystem, name] :
       {std::pair{"auto", "three_piece"}, std::pair{"other", "two_piece"}}) {
    try {
      cfg.TrajectoryPath(subsystem, name);
      ADD_FAILURE() << subsystem << " " << name << " was found";
    } catch (const std::invalid_argument& error) {
      const std::string what = error.what();
      EXPECT_NE(what.find(std::string("trajectories.") + name),
                std::string::npos)
          << what;
      EXPECT_NE(what.find(subsystem), std::string::npos) << what;
    }
  }
}

// The bridge and the telemetry node open the dashboard layout at the path
// config load resolved: against the file that set the key, the including
// file's setting winning, and a missing file is refused naming that file and
// the key. (Shared cases accept_studio_dashboard and
// reject_studio_dashboard_missing_file prove it is a listed dependency.)
// REQ-OPS-39
TEST_F(ManifestTest, StudioDashboardResolvesAgainstTheFileThatSetsIt) {
  std::filesystem::create_directories(dir_ / "base" / "layouts");
  Write("base/layouts/dashboard.toml", "");
  Write("subsystems/test.toml",
        "[subsystem]\nname = \"test\"\n[motors.m]\ncan_id = 1\n");
  Write(
      "base/robot.toml",
      "[studio]\ndashboard = \"layouts/dashboard.toml\"\n"
      "[[subsystems]]\nname = \"test\"\npath = \"../subsystems/test.toml\"\n");
  Write("robot.toml", "include = \"base/robot.toml\"\n");
  EXPECT_TRUE(
      std::filesystem::equivalent(ParseRobotConfig(Robot()).studio.dashboard,
                                  dir_ / "base/layouts/dashboard.toml"));

  Write("drive.toml", "");
  Write("robot.toml",
        "include = \"base/robot.toml\"\n[studio]\ndashboard = "
        "\"drive.toml\"\n");
  EXPECT_TRUE(std::filesystem::equivalent(
      ParseRobotConfig(Robot()).studio.dashboard, dir_ / "drive.toml"));

  Write("robot.toml",
        "include = \"base/robot.toml\"\n[studio]\ndashboard = "
        "\"missing.toml\"\n");
  try {
    ParseRobotConfig(Robot());
    ADD_FAILURE() << "a missing dashboard layout was accepted";
  } catch (const std::invalid_argument& error) {
    const std::string what = error.what();
    EXPECT_NE(what.find(Robot()), std::string::npos) << what;
    EXPECT_NE(what.find("[studio] dashboard"), std::string::npos) << what;
  }
}

// The TalonFX applies soft limits in its own position frame, which a reset
// restarts unless it is a remote CANcoder's absolute frame.
TEST_F(ManifestTest, RefusesSoftLimitsOutsideAnAbsolutePositionFrame) {
  Write("robot.toml",
        "[[subsystems]]\nname = \"lift\"\npath = \"subsystems/lift.toml\"\n");
  Write("subsystems/lift.toml",
        "[subsystem]\nname = \"lift\"\n[motors.winch]\ncan_id = 1\n"
        "soft_limits = true\n");
  try {
    ParseRobotConfig(Robot());
    ADD_FAILURE() << "soft limits on a rezero-frame motor were accepted";
  } catch (const std::invalid_argument& error) {
    const std::string what = error.what();
    EXPECT_NE(what.find("winch"), std::string::npos) << what;
    EXPECT_NE(what.find("position_frame = \"absolute\""), std::string::npos)
        << what;
  }
  ExpectRejected("soft_limits = true\n");
  const auto absolute = Parse(R"(
[subsystems.test.sensors.encoder]
can_id = 2
[subsystems.test.motors.arm]
can_id = 3
feedback = "RemoteCANcoder"
feedback_sensor = "encoder"
position_frame = "absolute"
soft_limits = true
)");
  const auto arm = std::find_if(
      absolute.hardware.motors.begin(), absolute.hardware.motors.end(),
      [](const auto& motor) { return motor.name == "arm"; });
  ASSERT_NE(arm, absolute.hardware.motors.end());
  EXPECT_TRUE(arm->soft_limits);
}

// The physics simulator models each motor's windings by its type, so a bench
// Kraken X44 must not stall at a Kraken X60's current. An absent key is a
// Kraken X60; any other value is refused naming the key, the motor and its
// file.
TEST_F(ManifestTest, MotorTypeSetsThePhysicsModelAndRefusesUnknownTypes) {
  using hardware::MotorType;
  EXPECT_EQ(Parse("").hardware.motors.at(0).motor_type.value,
            MotorType::kKrakenX60);
  for (const auto& [value, type] :
       {std::pair{"kraken_x60", MotorType::kKrakenX60},
        std::pair{"kraken_x44", MotorType::kKrakenX44},
        std::pair{"falcon_500", MotorType::kFalcon500}}) {
    SCOPED_TRACE(value);
    EXPECT_EQ(Parse(std::string{"motor_type = \""} + value + "\"\n")
                  .hardware.motors.at(0)
                  .motor_type.value,
              type);
  }
  ExpectRejected("motor_type = 60\n");
  Write("robot.toml",
        "[[subsystems]]\nname = \"lift\"\npath = \"subsystems/lift.toml\"\n");
  Write("subsystems/lift.toml",
        "[subsystem]\nname = \"lift\"\n[motors.winch]\ncan_id = 1\n"
        "motor_type = \"KrakenX44\"\n");
  try {
    ParseRobotConfig(Robot());
    ADD_FAILURE() << "an unknown motor_type was accepted";
  } catch (const std::invalid_argument& error) {
    const std::string what = error.what();
    for (const char* part : {"motor_type", "winch", "subsystems/lift.toml"}) {
      EXPECT_NE(what.find(part), std::string::npos) << what;
    }
  }
}

// A simulated home switch is bound in configuration, but only sim_gateway
// --physics reads it: the RoboRIO's config, and so its ConfigurationId, is
// the same with and without it, and a key that could not bind is refused
// naming the key and the input.
// REQ-DEV-10
TEST(ConfigTest, SimHomeSwitchKeysStayOffTheWireAndRefuseWhatCannotBind) {
  const std::string input = "[subsystems.test.digital_inputs.home]\ndio = 3\n";
  const auto plain = Parse(input);
  const auto with = Parse(input +
                          "sim_motor = \"m1\"\nsim_closed_at_rot = -0.1\n"
                          "sim_closed_above = true\nsim_closed_reads = true\n");
  const auto& sim = with.hardware.digital_inputs.at(0).sim_home_switch;
  EXPECT_EQ(sim.motor, "m1");
  EXPECT_EQ(sim.closed_at_rot, -0.1);
  EXPECT_TRUE(sim.closed_above);
  EXPECT_TRUE(sim.closed_reads);
  EXPECT_TRUE(
      plain.hardware.digital_inputs.at(0).sim_home_switch.motor.empty());
  EXPECT_EQ(hardware::ConfigurationId(with.hardware),
            hardware::ConfigurationId(plain.hardware));

  const auto expect_refused = [&](const std::string& body,
                                  std::initializer_list<const char*> parts) {
    SCOPED_TRACE(body);
    try {
      Parse(input + body);
      ADD_FAILURE() << "accepted";
    } catch (const std::invalid_argument& error) {
      const std::string what = error.what();
      for (const char* part : parts) {
        EXPECT_NE(what.find(part), std::string::npos) << what;
      }
    }
  };
  for (const char* key :
       {"sim_closed_at_rot", "sim_closed_above", "sim_closed_reads"}) {
    const std::string value =
        std::string_view{key} == "sim_closed_at_rot" ? "0.0" : "true";
    expect_refused(std::string{key} + " = " + value + "\n",
                   {key, "home", "sim_motor"});
  }
  expect_refused("sim_motor = \"m1\"\n", {"sim_closed_at_rot", "home"});
  for (const char* value : {"nan", "inf"}) {
    expect_refused(
        std::string{"sim_motor = \"m1\"\nsim_closed_at_rot = "} + value + "\n",
        {"sim_closed_at_rot", "home", "finite"});
  }
  // A motor of another subsystem, and a device that is no motor.
  expect_refused(
      "sim_motor = \"m2\"\nsim_closed_at_rot = 0.0\n"
      "[subsystems.other.motors.m2]\ncan_id = 2\n",
      {"sim_motor", "m2", "home"});
  expect_refused("sim_motor = \"home\"\nsim_closed_at_rot = 0.0\n",
                 {"sim_motor", "'home'"});
}

TEST_F(ManifestTest, MergesSubsystemFilesAndKeepsTopLevelTables) {
  Write("robot.toml", R"(
[robot]
period_us = 10000
[studio]
http_port = 9000
topic = "/talos/telemetry"
[[subsystems]]
name = "shooter"
path = "subsystems/shooter.toml"
[[subsystems]]
name = "intake"
path = "subsystems/intake.toml"
)");
  Write("subsystems/shooter.toml", R"(
[subsystem]
name = "shooter"
topic = "/shooter/state"
[motors.flywheel]
can_id = 2
)");
  Write("subsystems/intake.toml", R"(
[subsystem]
name = "intake"
[motors.roller]
can_id = 1
)");
  const auto cfg = ParseRobotConfig(Robot());
  EXPECT_EQ(cfg.hardware.period_us, 10000u);
  EXPECT_EQ(cfg.studio.http_port, 9000);
  EXPECT_EQ(cfg.GetDevices("intake")->motors, (std::vector<uint16_t>{1}));
  EXPECT_EQ(cfg.GetDevices("shooter")->motors, (std::vector<uint16_t>{2}));
  const auto* shooter = cfg.GetSubsystemTable("shooter");
  ASSERT_NE(shooter, nullptr);
  EXPECT_EQ((*shooter)["topic"].value_or(std::string{}), "/shooter/state");
  EXPECT_FALSE(shooter->contains("name"));
}

TEST_F(ManifestTest, EntryNodeOverridesSubsystemFileNode) {
  Write("subsystems/intake.toml", R"(
[subsystem]
name = "intake"
node = "//cpp:intake"
period_us = 20000
[motors.roller]
can_id = 1
)");
  Write("subsystems/planner.toml", R"(
[subsystem]
name = "planner"
gain = 2.0
)");
  Write("robot.toml", R"(
[[subsystems]]
name = "intake"
path = "subsystems/intake.toml"
node = "//py:intake"
[[subsystems]]
name = "planner"
path = "subsystems/planner.toml"
node = "//py:planner"
)");
  const auto cfg = ParseRobotConfig(Robot());
  const auto* intake = cfg.GetSubsystemTable("intake");
  ASSERT_NE(intake, nullptr);
  EXPECT_EQ((*intake)["node"].value_or(std::string{}), "//py:intake");
  EXPECT_EQ((*intake)["period_us"].value_or(0), 20000);
  EXPECT_EQ(cfg.GetDevices("intake")->motors, (std::vector<uint16_t>{1}));
  // The file has no `node`; the entry supplies it.
  const auto* planner = cfg.GetSubsystemTable("planner");
  ASSERT_NE(planner, nullptr);
  EXPECT_EQ((*planner)["node"].value_or(std::string{}), "//py:planner");
  EXPECT_EQ((*planner)["gain"].value_or(0.0), 2.0);

  // Without the override the file's own label stands.
  Write(
      "robot.toml",
      "[[subsystems]]\nname = \"intake\"\npath = \"subsystems/intake.toml\"\n");
  EXPECT_EQ(
      (*ParseRobotConfig(Robot()).GetSubsystemTable("intake"))["node"].value_or(
          std::string{}),
      "//cpp:intake");
}

TEST_F(ManifestTest, TopLevelIncludeMergesTablesAndSubsystemOverrides) {
  Write("subsystems/intake.toml", R"(
[subsystem]
name = "intake"
node = "//cpp:intake"
[motors.roller]
can_id = 1
)");
  Write("subsystems/elevator.toml", R"(
[subsystem]
name = "elevator"
node = "//cpp:elevator"
[motors.lift]
can_id = 2
)");
  Write("subsystems/extra.toml", R"(
[subsystem]
name = "extra"
[motors.aux]
can_id = 3
)");
  Write("robot.toml", R"(
[robot]
name = "base_robot"
period_us = 10000
controller_port = 5804

[studio]
http_port = 8080
webroot = "base/dist"
topic = "/talos/telemetry"

[[subsystems]]
name = "intake"
path = "subsystems/intake.toml"
node = "//cpp:intake"

[[subsystems]]
name = "elevator"
path = "subsystems/elevator.toml"
node = "//cpp:elevator"
)");
  Write("robot_py.toml", R"(
include = "robot.toml"

[robot]
period_us = 5000

[studio]
http_port = 9090
topic = "/talos/telemetry"

[[subsystems]]
name = "intake"
node = "//py:intake"

[[subsystems]]
name = "extra"
path = "subsystems/extra.toml"
node = "//py:extra"
)");
  const auto cfg = ParseRobotConfig((dir_ / "robot_py.toml").string());
  EXPECT_EQ(cfg.hardware.period_us, 5000u);
  EXPECT_EQ(cfg.controller_port, 5804);
  EXPECT_EQ(cfg.studio.http_port, 9090);
  EXPECT_EQ(cfg.studio.webroot, "base/dist");
  EXPECT_FALSE(cfg.toml_data.contains("include"));

  const auto* intake = cfg.GetSubsystemTable("intake");
  ASSERT_NE(intake, nullptr);
  EXPECT_EQ((*intake)["node"].value_or(std::string{}), "//py:intake");
  const auto* elevator = cfg.GetSubsystemTable("elevator");
  ASSERT_NE(elevator, nullptr);
  EXPECT_EQ((*elevator)["node"].value_or(std::string{}), "//cpp:elevator");
  const auto* extra = cfg.GetSubsystemTable("extra");
  ASSERT_NE(extra, nullptr);
  EXPECT_EQ((*extra)["node"].value_or(std::string{}), "//py:extra");

  // An empty include names no file.
  Write("bad_empty.toml", "include = \"\"\n");
  EXPECT_THROW(ParseRobotConfig((dir_ / "bad_empty.toml").string()),
               std::invalid_argument);
  Write("bad_missing.toml", "include = \"missing.toml\"\n");
  EXPECT_THROW(ParseRobotConfig((dir_ / "bad_missing.toml").string()),
               std::invalid_argument);
}

// TODO.md SIM-4: `[robot] simulation_only` reaches the config through an
// include like any [robot] key, and an including file may set it true over a
// deployable base. Once a file sets it true, a file including that one
// cannot set it back to false: refused, naming the key and both files.
// REQ-SAFE-40
TEST_F(ManifestTest, SimulationOnlyIsInheritedAndNeverUnsetByAnInclude) {
  Write("deployable.toml", "[robot]\nperiod_us = 5000\n" + std::string{kMotor});
  Write("sim.toml",
        "include = \"deployable.toml\"\n[robot]\n"
        "simulation_only = true\n");
  Write("sim_only_base.toml",
        "[robot]\nsimulation_only = true\n" + std::string{kMotor});
  Write("inherits.toml", "include = \"sim_only_base.toml\"\n");
  Write("unsets.toml",
        "include = \"sim_only_base.toml\"\n[robot]\n"
        "simulation_only = false\n");
  const auto parse = [this](std::string_view file) {
    return ParseRobotConfig((dir_ / file).string());
  };
  EXPECT_FALSE(parse("deployable.toml").simulation_only);
  EXPECT_TRUE(parse("sim.toml").simulation_only);
  EXPECT_TRUE(parse("inherits.toml").simulation_only);
  try {
    parse("unsets.toml");
    ADD_FAILURE() << "unsets.toml loaded as a deployable config";
  } catch (const std::invalid_argument& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("simulation_only"), std::string::npos) << what;
    EXPECT_NE(what.find("unsets.toml"), std::string::npos) << what;
    EXPECT_NE(what.find("sim_only_base.toml"), std::string::npos) << what;
  }
  Write("not_bool.toml",
        "[robot]\nsimulation_only = 1\n" + std::string{kMotor});
  EXPECT_THROW(parse("not_bool.toml"), std::invalid_argument);
}

TEST_F(ManifestTest, SubsystemDefaultsLayerDeviceAndSubsystemTables) {
  Write("subsystems/mech_defaults.toml", R"(
[subsystem]
period_us = 20000
feed_volts = 6.0
hold_volts = 1.5

[motors.roller]
type = "TalonFX"
supply_limit_a = 40.0
stator_limit_a = 80.0
sensor_to_mechanism_ratio = 3.0
slot0 = { p = 1.0, v = 0.2 }
)");
  // Defaults referenced inside [subsystem].
  Write("subsystems/mech.toml", R"(
[subsystem]
name = "mech"
defaults = "mech_defaults.toml"
feed_volts = 8.0

[motors.roller]
can_id = 21
supply_limit_a = 35.0
slot0 = { p = 2.5, d = 0.05 }
)");
  // Defaults referenced on the [[subsystems]] manifest entry.
  Write("robot.toml", R"(
[[subsystems]]
name = "mech"
path = "subsystems/mech.toml"
)");
  const auto cfg = ParseRobotConfig(Robot());
  ASSERT_EQ(cfg.hardware.motors.size(), 1u);
  const auto& m1 = cfg.hardware.motors[0];
  EXPECT_EQ(m1.can_id, 21);
  EXPECT_DOUBLE_EQ(m1.supply_limit_a, 35.0);
  EXPECT_DOUBLE_EQ(m1.stator_limit_a, 80.0);
  EXPECT_DOUBLE_EQ(m1.sensor_to_mechanism_ratio, 3.0);
  EXPECT_DOUBLE_EQ(m1.slots[0].p, 2.5);
  EXPECT_DOUBLE_EQ(m1.slots[0].v, 0.2);
  EXPECT_DOUBLE_EQ(m1.slots[0].d, 0.05);

  // Defaults referenced on the [[subsystems]] manifest entry.
  Write("subsystems/mech2.toml", R"(
[subsystem]
name = "mech2"

[motors.roller]
can_id = 22
)");
  Write("robot2.toml", R"(
[[subsystems]]
name = "mech2"
path = "subsystems/mech2.toml"
defaults = "subsystems/mech_defaults.toml"
)");
  const auto cfg2 = ParseRobotConfig((dir_ / "robot2.toml").string());
  ASSERT_EQ(cfg2.hardware.motors.size(), 1u);
  const auto& m2 = cfg2.hardware.motors[0];
  EXPECT_EQ(m2.can_id, 22);
  EXPECT_DOUBLE_EQ(m2.supply_limit_a, 40.0);
  EXPECT_DOUBLE_EQ(m2.sensor_to_mechanism_ratio, 3.0);
  EXPECT_DOUBLE_EQ(m2.slots[0].p, 1.0);
  const auto* mech_tbl = cfg.GetSubsystemTable("mech");
  ASSERT_NE(mech_tbl, nullptr);
  EXPECT_FALSE(mech_tbl->contains("name"));
  EXPECT_FALSE(mech_tbl->contains("defaults"));
  EXPECT_EQ((*mech_tbl)["period_us"].value_or(0), 20000);
  EXPECT_DOUBLE_EQ((*mech_tbl)["feed_volts"].value_or(0.0), 8.0);
  EXPECT_DOUBLE_EQ((*mech_tbl)["hold_volts"].value_or(0.0), 1.5);
}

TEST(ConfigTest, RejectsInvalidTeleopDefaultTypes) {
  for (const std::string value :
       {"3", "[]", "{ position_rot = [] }", "{ nested = { value = 1 } }"}) {
    ExpectRejected("[subsystems.test.teleop]\ndefaults = " + value);
  }
}

// Other binding shapes are cases in testdata/.
TEST(ConfigTest, RejectsNegativePovAndFractionalPovIndex) {
  for (const std::string binding : {"pov = -45\nwhile_held = { x = 1 }",
                                    "pov = 90\npov_index = 0.0\nwhile_held = "
                                    "{ x = 1 }"}) {
    ExpectRejected("[[subsystems.test.teleop.bindings]]\n" + binding);
  }
}

// talOS/configuration/testdata: each case is a config tree plus expect.toml
// (`config`, default "robot.toml"; `accept`; `nodes`; `files`, every file the
// config reads besides itself). //tools:sync_manifest_test runs the same
// cases, so the parser and the Bazel manifest cannot disagree untested.
namespace fs = std::filesystem;

std::vector<std::string> Strings(const toml::table& t, const char* key) {
  std::vector<std::string> out;
  if (const auto* arr = t[key].as_array()) {
    for (auto&& v : *arr) {
      const auto value = v.value<std::string>();
      EXPECT_TRUE(value.has_value());
      if (value.has_value()) {
        out.emplace_back(*value);
      }
    }
  }
  return out;
}

std::vector<std::string> Nodes(const RobotConfig& cfg) {
  std::vector<std::string> out;
  for (auto&& [name, sub] : *cfg.toml_data["subsystems"].as_table()) {
    if (auto node = (*sub.as_table())["node"].value<std::string>()) {
      out.push_back(*node);
    }
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

TEST(ConfigTest, SharedResolutionCases) {
  const fs::path testdata = "talOS/configuration/testdata";
  for (const auto& dir : fs::directory_iterator(testdata)) {
    const std::string name = dir.path().filename().string();
    SCOPED_TRACE(name);
    const toml::table expect =
        toml::parse_file((dir.path() / "expect.toml").string());
    const std::string config = expect["config"].value_or("robot.toml");
    // A private copy, so files can be deleted to prove which are read.
    const fs::path copy = fs::path(::testing::TempDir()) / "cases" / name;
    fs::remove_all(copy);
    fs::create_directories(copy);
    fs::copy(dir.path(), copy, fs::copy_options::recursive);
    const std::string path = (copy / config).string();
    if (!expect["accept"].value_or(false)) {
      EXPECT_THROW(ParseRobotConfig(path), std::invalid_argument);
      continue;
    }
    EXPECT_EQ(Nodes(ParseRobotConfig(path)), Strings(expect, "nodes"));

    const auto files = Strings(expect, "files");
    // Every listed file is read: without it the config does not load.
    for (const auto& file : files) {
      const fs::path saved = copy / (file + ".saved");
      fs::rename(copy / file, saved);
      EXPECT_THROW(ParseRobotConfig(path), std::invalid_argument) << file;
      fs::rename(saved, copy / file);
    }
    // No other file is read: the config loads with all of them gone.
    std::vector<fs::path> others;
    for (const auto& f : fs::recursive_directory_iterator(copy)) {
      const std::string rel = fs::relative(f.path(), copy).generic_string();
      if (f.is_regular_file() && rel != config && rel != "expect.toml" &&
          std::find(files.begin(), files.end(), rel) == files.end()) {
        others.push_back(f.path());
      }
    }
    for (const auto& f : others) fs::remove(f);
    EXPECT_EQ(Nodes(ParseRobotConfig(path)), Strings(expect, "nodes"));
  }
}

}  // namespace
}  // namespace talos::config
