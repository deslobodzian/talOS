#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>

#include "drivetrain/drivetrain.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/testing.h"
#include "talOS/node/testing.h"

namespace drivetrain {
namespace {

using namespace std::chrono_literals;
using talos::hardware::Mode;
using talos::testing::Sim;
using talos::testing::SimHardware;

constexpr double kTau = 2 * std::numbers::pi;

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

std::string ConfigPath() { return Runfile("TALOS_TEST_CONFIG"); }

talos::config::RobotConfig Config() {
  return talos::config::ParseRobotConfig(ConfigPath());
}

SwerveGeometry Geometry() {
  return BuildSwerveGeometry(Config(), "drivetrain");
}

// --- kinematics ----------------------------------------------------------

TEST(Swerve, TranslationAndReverseTakeTheShorterSteeringPath) {
  const SwerveGeometry geometry = Geometry();
  const double rps = 1.0 / (kTau * geometry.modules[0].wheel_radius_m);
  for (const double vx : {1.0, -1.0}) {
    const auto modules = ToModuleStates(geometry, {vx, 0, 0}, {});
    if (!modules.has_value()) {
      FAIL() << "ToModuleStates returned nullopt";
    }
    for (const ModuleState& m : *modules) {
      EXPECT_NEAR(m.drive_rps, vx * rps, 1e-12);  // reversed, not turned
      EXPECT_NEAR(m.steer_rot, 0, 1e-12);
    }
  }
  const auto sideways = ToModuleStates(geometry, {0, 1, 0}, {});
  if (!sideways.has_value()) {
    FAIL() << "ToModuleStates returned nullopt";
  }
  for (const ModuleState& m : *sideways) EXPECT_NEAR(m.steer_rot, 0.25, 1e-12);
}

TEST(Swerve, RotationDesaturationAndInvalidInput) {
  const SwerveGeometry geometry = Geometry();
  const auto spin = ToModuleStates(geometry, {0, 0, 1}, {});
  if (!spin.has_value()) {
    FAIL() << "ToModuleStates returned nullopt";
  }
  EXPECT_LT((*spin)[0].drive_rps, 0);  // back_left
  EXPECT_NEAR((*spin)[0].steer_rot, 0.125, 1e-12);
  EXPECT_GT((*spin)[1].drive_rps, 0);  // back_right
  EXPECT_NEAR((*spin)[1].steer_rot, -0.125, 1e-12);

  const auto fast = ToModuleStates(geometry, {100, 0, 0}, {});
  if (!fast.has_value()) {
    FAIL() << "ToModuleStates returned nullopt";
  }
  for (const ModuleState& m : *fast) EXPECT_NEAR(m.drive_rps, 20, 1e-12);

  EXPECT_FALSE(ToModuleStates(
      geometry, {std::numeric_limits<double>::quiet_NaN(), 0, 0}, {}));
}

TEST(Swerve, ForwardKinematicsInvertsInverseKinematics) {
  const SwerveGeometry geometry = Geometry();
  const auto modules = ToModuleStates(geometry, {1.0, -0.5, 0.8}, {});
  if (!modules.has_value()) {
    FAIL() << "ToModuleStates returned nullopt";
  }
  const ChassisSpeeds speeds = ToChassisSpeeds(geometry, *modules);
  EXPECT_NEAR(speeds.vx_mps, 1.0, 1e-9);
  EXPECT_NEAR(speeds.vy_mps, -0.5, 1e-9);
  EXPECT_NEAR(speeds.omega_radps, 0.8, 1e-9);
}

// --- geometry from config ------------------------------------------------

TEST(SwerveGeometry, NamesEachModulesDevices) {
  const SwerveGeometry geometry = Geometry();
  EXPECT_EQ(geometry.modules[2].name, "front_left");
  EXPECT_EQ(geometry.modules[2].drive, "front_left_drive");
  EXPECT_EQ(geometry.modules[2].steer, "front_left_steer");
  EXPECT_DOUBLE_EQ(geometry.modules[2].x_m, 0.3);
  EXPECT_DOUBLE_EQ(geometry.modules[2].max_drive_rps, 20.0);
}

TEST(SwerveGeometry, RejectsUnknownAndCrossedDeviceReferences) {
  const auto config = Config();
  const toml::table* geometry =
      config.GetSubsystemTable("drivetrain", "geometry");
  ASSERT_NE(geometry, nullptr);

  auto mutated = *geometry;
  mutated["modules"].as_array()->get(0)->as_table()->insert_or_assign(
      "drive", "no_such_motor");
  EXPECT_THROW(BuildSwerveGeometry(config, "drivetrain", mutated),
               std::invalid_argument);

  // Another module's encoder: the steer motor's feedback sensor no longer
  // matches, which must not be silently accepted.
  mutated = *geometry;
  mutated["modules"].as_array()->get(0)->as_table()->insert_or_assign(
      "encoder", "front_right_encoder");
  EXPECT_THROW(BuildSwerveGeometry(config, "drivetrain", mutated),
               std::invalid_argument);

  // A drive motor used as a steer motor (no continuous wrap).
  mutated = *geometry;
  mutated["modules"].as_array()->get(0)->as_table()->insert_or_assign(
      "steer", "front_left_drive");
  EXPECT_THROW(BuildSwerveGeometry(config, "drivetrain", mutated),
               std::invalid_argument);
}

TEST(SwerveGeometry, RequiresExactlyFourModulesAndAPositiveWheelRadius) {
  const auto config = Config();
  const toml::table* geometry =
      config.GetSubsystemTable("drivetrain", "geometry");
  ASSERT_NE(geometry, nullptr);

  auto short_table = *geometry;
  short_table["modules"].as_array()->pop_back();
  EXPECT_THROW(BuildSwerveGeometry(config, "drivetrain", short_table),
               std::invalid_argument);

  auto bad_radius = *geometry;
  bad_radius.insert_or_assign("wheel_radius_m", 0.0);
  EXPECT_THROW(BuildSwerveGeometry(config, "drivetrain", bad_radius),
               std::invalid_argument);
}

// --- the node ------------------------------------------------------------

// The test config with `extra` appended to the drivetrain's [subsystem]
// table, written under TEST_TMPDIR.
std::string ConfigWith(const std::string& extra) {
  const std::filesystem::path source = ConfigPath();
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::filesystem::path dir =
      std::filesystem::path{tmp != nullptr ? tmp : "/tmp"} /
      "drivetrain_config";
  std::filesystem::create_directories(dir);
  std::filesystem::copy_file(source, dir / "robot.toml",
                             std::filesystem::copy_options::overwrite_existing);
  std::stringstream text;
  text << std::ifstream{source.parent_path() / "drivetrain.toml"}.rdbuf();
  std::string toml = text.str();
  const std::string table = "[subsystem]\n";
  toml.insert(toml.find(table) + table.size(), extra + "\n");
  std::ofstream{dir / "drivetrain.toml"} << toml;
  return (dir / "robot.toml").string();
}

class DrivetrainTest : public ::testing::Test {
 protected:
  explicit DrivetrainTest(std::string config = ConfigPath())
      : sim{std::move(config)} {}

  // Holds a target the way the arbiter does: republished every 20 ms.
  void Hold(const ChassisTarget& target, std::chrono::milliseconds duration) {
    for (auto t = 0ms; t < duration; t += 20ms) {
      sim.publish(topics::kTarget, target);
      sim.step(20ms);
    }
  }
  DrivetrainState State() const {
    const auto state = sim.latest<DrivetrainState>(topics::kState);
    EXPECT_TRUE(state.has_value());
    return state.value_or(DrivetrainState{});
  }
  talos::hardware::MotorRequest Request(const std::string& device) const {
    return hw.motor_request("drivetrain", device);
  }

  Sim sim;
  Drivetrain& drivetrain = sim.add<Drivetrain>("drivetrain");
  SimHardware hw{sim};
  const SwerveGeometry geometry = Geometry();
};

TEST_F(DrivetrainTest, DrivesEveryModuleToAFreshTarget) {
  hw.enable();
  Hold(ChassisTarget{1.0, 0, 0, 0, true}, 40ms);
  const double rps = 1.0 / (kTau * geometry.modules[0].wheel_radius_m);
  for (const SwerveModule& module : geometry.modules) {
    EXPECT_EQ(Request(module.drive).mode, Mode::kVelocity) << module.name;
    EXPECT_NEAR(Request(module.drive).demand, rps, 1e-9) << module.name;
    EXPECT_EQ(Request(module.steer).mode, Mode::kPosition) << module.name;
    EXPECT_NEAR(Request(module.steer).demand, 0.0, 1e-9) << module.name;
  }
  EXPECT_TRUE(State().enabled());
}

TEST_F(DrivetrainTest, StaleTargetDisabledTargetOrDisabledRobotGoNeutral) {
  hw.enable();
  Hold(ChassisTarget{1.0, 0, 0, 0, true}, 40ms);
  ASSERT_EQ(Request("front_left_drive").mode, Mode::kVelocity);

  sim.step(150ms);  // the arbiter went quiet: the target is stale
  for (const SwerveModule& module : geometry.modules) {
    EXPECT_EQ(Request(module.drive).mode, Mode::kNeutral) << module.name;
    EXPECT_EQ(Request(module.steer).mode, Mode::kNeutral) << module.name;
  }

  Hold(ChassisTarget{1.0, 0, 0, 0, false}, 40ms);
  EXPECT_EQ(Request("front_left_drive").mode, Mode::kNeutral);

  hw.enable(false);
  Hold(ChassisTarget{1.0, 0, 0, 0, true}, 40ms);
  EXPECT_EQ(Request("front_left_drive").mode, Mode::kNeutral);
  EXPECT_FALSE(State().enabled());
}

TEST_F(DrivetrainTest, ReportsTheChassisMotionTheModulesMeasure) {
  // Every wheel pointing 90 degrees left at 2 rps.
  for (const SwerveModule& module : geometry.modules) {
    hw.set_motor("drivetrain", module.drive, 0.0, 2.0);
    hw.set_motor("drivetrain", module.steer, 0.25, 0.0);
  }
  sim.step(20ms);
  const DrivetrainState state = State();
  EXPECT_NEAR(state.vx_mps(), 0.0, 1e-9);
  EXPECT_NEAR(state.vy_mps(), 2.0 * kTau * 0.0508, 1e-9);
  EXPECT_NEAR(state.omega_radps(), 0.0, 1e-9);
}

// Pure chassis rotation at `omega` rad/s: each wheel tangential, turning at
// omega * |r|.
void SetRotating(SimHardware& hw, const SwerveGeometry& geometry,
                 double omega) {
  for (const SwerveModule& module : geometry.modules) {
    const double vx = -omega * module.y_m, vy = omega * module.x_m;
    hw.set_motor("drivetrain", module.drive, 0.0,
                 std::hypot(vx, vy) / (kTau * module.wheel_radius_m));
    hw.set_motor("drivetrain", module.steer, std::atan2(vy, vx) / kTau, 0.0);
  }
}

TEST_F(DrivetrainTest, SimulatedHeadingIntegratesTheRotationTheModulesMeasure) {
  // Under --sim (as here) the Pigeon's permanent zero is not believed.
  SetRotating(hw, geometry, 1.0);
  sim.step(200ms);
  const double expected_rot = 1.0 / kTau * 0.2;
  EXPECT_GT(State().yaw_rot(), expected_rot * 0.9);
  EXPECT_LT(State().yaw_rot(), expected_rot * 1.02);
  EXPECT_NEAR(State().yaw_rate_rps(), 1.0 / kTau, 1e-9);
}

class DrivetrainOnImuTest : public DrivetrainTest {
 protected:
  DrivetrainOnImuTest()
      : DrivetrainTest{ConfigWith("integrate_heading = false")} {}
};

TEST_F(DrivetrainOnImuTest, HeadingComesFromTheImuNotASteerEncoder) {
  SetRotating(hw, geometry, 1.0);
  hw.set_cancoder("drivetrain", "back_left_encoder", 0.375, 2.0);
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.125, 0.5);
  sim.step(200ms);
  EXPECT_NEAR(State().yaw_rot(), 0.125, 1e-12);
  EXPECT_NEAR(State().yaw_rate_rps(), 0.5, 1e-12);
}

// --- wire and binary -----------------------------------------------------

// These structs cross every chassis topic on the robot, so a layout change
// is a new contract (whole-bundle rollback), never a silent edit.
TEST(Messages, WireSizesAreFrozen) {
  EXPECT_EQ(sizeof(ChassisTarget), 40u);
  EXPECT_EQ(sizeof(DrivetrainState), 56u);
}

struct Result {
  int status;
  std::string output;
};

Result RunNode(const std::string& args) {
  const std::string command = "'" + Runfile("TALOS_TEST_NODE") + "' " + args +
                              " --config '" + ConfigPath() + "' 2>&1";
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

TEST(DrivetrainBinary, LoggedSimRunReplaysWithoutDivergence) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string log =
      std::string{tmp != nullptr ? tmp : "/tmp"} + "/drivetrain_test.tlog";
  const Result run = RunNode("--sim --duration-s 1 --log '" + log + "'");
  ASSERT_EQ(run.status, 0) << run.output;
  const Result replay = RunNode("--sim --replay '" + log + "'");
  EXPECT_EQ(replay.status, 0) << replay.output;
  EXPECT_NE(replay.output.find("diverged=0 clean_exit=1"), std::string::npos)
      << replay.output;
}

}  // namespace
}  // namespace drivetrain
