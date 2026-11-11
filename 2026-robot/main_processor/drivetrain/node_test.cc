#include <gtest/gtest.h>
#include <sys/wait.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
#include <string_view>
#include <vector>

#include "drivetrain/drivetrain.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/testing.h"
#include "talOS/introspection/describe.h"
#include "talOS/node/access.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/node/testing.h"

namespace drivetrain {
namespace {

using namespace std::chrono_literals;
using talos::hardware::Mode;
using talos::testing::Sim;
using talos::testing::SimHardware;
namespace ds = talos::driver_station;

constexpr double kTau = 2 * std::numbers::pi;
// Long enough for the bridge's clock mapping (8 hardware states at 5 ms) to
// converge, after which hardware sample times, and so the measurement
// flags, are valid.
constexpr auto kConverged = std::chrono::milliseconds{60};

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
  SwerveKinematics kinematics{geometry};
  const double rps = 1.0 / (kTau * geometry.modules[0].wheel_radius_m);
  for (const double vx : {1.0, -1.0}) {
    const auto modules = kinematics.ToModuleStates({vx, 0, 0}, {});
    if (!modules.has_value()) {
      FAIL() << "ToModuleStates returned nullopt";
    }
    for (const ModuleState& m : *modules) {
      EXPECT_NEAR(m.drive_rps, vx * rps, 1e-12);  // reversed, not turned
      EXPECT_NEAR(m.steer_rot, 0, 1e-12);
    }
  }
  const auto sideways = kinematics.ToModuleStates({0, 1, 0}, {});
  if (!sideways.has_value()) {
    FAIL() << "ToModuleStates returned nullopt";
  }
  for (const ModuleState& m : *sideways) EXPECT_NEAR(m.steer_rot, 0.25, 1e-12);
}

TEST(Swerve, SteersFromTheCurrentAzimuthAndAStoppedWheelHolds) {
  const SwerveGeometry geometry = Geometry();
  SwerveKinematics kinematics{geometry};
  // Wound up 3 turns: forward is reached without unwinding.
  std::array<ModuleState, kModuleCount> current{};
  for (ModuleState& m : current) m.steer_rot = 3.0;
  const auto forward = kinematics.ToModuleStates({1, 0, 0}, current);
  ASSERT_TRUE(forward.has_value());
  for (const ModuleState& m : *forward) {
    EXPECT_GT(m.drive_rps, 0);
    EXPECT_NEAR(m.steer_rot, 3.0, 1e-12);
  }
  // 0.4 rotations off forward: reverse and turn 0.1 rather than 0.4.
  for (ModuleState& m : current) m.steer_rot = 0.4;
  const auto reversed = kinematics.ToModuleStates({1, 0, 0}, current);
  ASSERT_TRUE(reversed.has_value());
  for (const ModuleState& m : *reversed) {
    EXPECT_LT(m.drive_rps, 0);
    EXPECT_NEAR(m.steer_rot, 0.5, 1e-12);
  }
  // No motion: every wheel stays where it points.
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    current[i].steer_rot = 0.1 * static_cast<double>(i);
  }
  const auto stopped = kinematics.ToModuleStates({0, 0, 0}, current);
  ASSERT_TRUE(stopped.has_value());
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    EXPECT_EQ((*stopped)[i].drive_rps, 0);
    EXPECT_NEAR((*stopped)[i].steer_rot, 0.1 * i, 1e-12);
  }
}

TEST(Swerve, SubThresholdRotationHoldsModuleAngles) {
  // Heading-hold noise after the driver lets go: a rotation whose wheel
  // speeds stay under kHoldFraction of each module's top speed must not
  // re-steer.
  const SwerveGeometry geometry = Geometry();
  SwerveKinematics kinematics{geometry};
  std::array<ModuleState, kModuleCount> current{};
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    current[i].steer_rot = 0.05 * static_cast<double>(i);
  }
  double radius_m = 0.0;
  double max_mps = std::numeric_limits<double>::infinity();
  for (const SwerveModule& module : geometry.modules) {
    radius_m = std::max(radius_m, std::hypot(module.x_m, module.y_m));
    max_mps = std::min(max_mps,
                       WheelSpeed(module.max_drive_rps, module.wheel_radius_m));
  }
  constexpr double kFraction = SwerveKinematics::kHoldFraction;
  const double below = 0.9 * kFraction * max_mps / radius_m;
  for (const double omega : {1e-4, below, -below}) {
    SCOPED_TRACE(omega);
    const auto held = kinematics.ToModuleStates({0, 0, omega}, current, 0.005);
    ASSERT_TRUE(held.has_value());
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      EXPECT_EQ((*held)[i].drive_rps, 0) << geometry.modules[i].name;
      EXPECT_NEAR((*held)[i].steer_rot, current[i].steer_rot, 1e-12)
          << geometry.modules[i].name;
    }
  }
  // Just above the threshold every wheel steers to its rotation angle.
  const double above = 1.1 * kFraction * max_mps / radius_m;
  const auto turning = kinematics.ToModuleStates({0, 0, above}, {}, 0.005);
  ASSERT_TRUE(turning.has_value());
  EXPECT_NEAR((*turning)[0].steer_rot, 0.125, 1e-9);  // back_left
  EXPECT_NE((*turning)[0].drive_rps, 0);
}

TEST(Swerve, RotationDesaturationAndInvalidInput) {
  const SwerveGeometry geometry = Geometry();
  SwerveKinematics kinematics{geometry};
  const auto spin = kinematics.ToModuleStates({0, 0, 1}, {});
  if (!spin.has_value()) {
    FAIL() << "ToModuleStates returned nullopt";
  }
  EXPECT_LT((*spin)[0].drive_rps, 0);  // back_left
  EXPECT_NEAR((*spin)[0].steer_rot, 0.125, 1e-12);
  EXPECT_GT((*spin)[1].drive_rps, 0);  // back_right
  EXPECT_NEAR((*spin)[1].steer_rot, -0.125, 1e-12);

  const auto fast = kinematics.ToModuleStates({100, 0, 0}, {});
  if (!fast.has_value()) {
    FAIL() << "ToModuleStates returned nullopt";
  }
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    EXPECT_NEAR((*fast)[i].drive_rps, geometry.modules[i].max_drive_rps, 1e-12);
  }

  EXPECT_FALSE(kinematics.ToModuleStates(
      {std::numeric_limits<double>::quiet_NaN(), 0, 0}, {}));
  EXPECT_FALSE(kinematics.ToModuleStates(
      {0, 0, std::numeric_limits<double>::infinity()}, {}));
}

TEST(Swerve, EachModuleIsLimitedByItsOwnDriveAndWheel) {
  // One module with a slower drive and one with a bigger wheel: the same
  // factor scales every wheel until the tightest reaches its own maximum.
  SwerveGeometry geometry = Geometry();
  geometry.modules[1].max_drive_rps = 10;
  geometry.modules[2].wheel_radius_m *= 2;
  SwerveKinematics kinematics{geometry};
  const auto fast = kinematics.ToModuleStates({100, 0, 0}, {});
  if (!fast.has_value()) {
    FAIL() << "Expected module states for valid chassis speeds";
  }
  const auto& fast_states = *fast;
  EXPECT_NEAR(fast_states[1].drive_rps, 10, 1e-9);
  EXPECT_NEAR(fast_states[0].drive_rps, 10, 1e-9);
  EXPECT_NEAR(fast_states[3].drive_rps, 10, 1e-9);
  EXPECT_NEAR(fast_states[2].drive_rps, 5, 1e-9);  // same ground speed
}

TEST(Swerve, EveryUsableSubsetRecoversTheCommandedTwist) {
  SwerveKinematics kinematics{Geometry()};
  const ChassisSpeeds target{1.0, -0.5, 0.8};
  for (unsigned mask = 0; mask < 16; ++mask) {
    SCOPED_TRACE(mask);
    const bool usable = std::popcount(mask) >= 2;
    EXPECT_EQ(kinematics.UsableSubset(mask), usable);
    const auto initial = kinematics.ToModuleStates(target, {}, 0.0, mask);
    ASSERT_EQ(initial.has_value(), usable);
    if (!usable) {
      const auto speeds = kinematics.ToChassisSpeeds({}, mask);
      EXPECT_EQ(speeds.vx_mps, 0);
      EXPECT_EQ(speeds.vy_mps, 0);
      EXPECT_EQ(speeds.omega_radps, 0);
      continue;
    }
    const auto aligned = kinematics.ToModuleStates(target, *initial, 0.0, mask);
    ASSERT_TRUE(aligned.has_value());
    const auto speeds = kinematics.ToChassisSpeeds(*aligned, mask);
    EXPECT_NEAR(speeds.vx_mps, target.vx_mps, 1e-9);
    EXPECT_NEAR(speeds.vy_mps, target.vy_mps, 1e-9);
    EXPECT_NEAR(speeds.omega_radps, target.omega_radps, 1e-9);
  }
}

TEST(Swerve, SingularAndIllConditionedSubsetsAreNotDrivenOrSolved) {
  for (double separation : {0.0, 1e-8}) {
    auto geometry = Geometry();
    geometry.modules[1].x_m = geometry.modules[0].x_m + separation;
    geometry.modules[1].y_m = geometry.modules[0].y_m;
    SwerveKinematics kinematics{geometry};
    EXPECT_FALSE(kinematics.UsableSubset(3));
    EXPECT_FALSE(kinematics.ToModuleStates({1, 0, 0}, {}, 0, 3));
    const auto speeds = kinematics.ToChassisSpeeds({}, 3);
    EXPECT_EQ(speeds.vx_mps, 0);
    EXPECT_EQ(speeds.vy_mps, 0);
    EXPECT_EQ(speeds.omega_radps, 0);
    EXPECT_TRUE(kinematics.UsableSubset(15));
  }
}

TEST(Swerve, CosineScalingSuppressesMisalignedDrive) {
  const auto geometry = Geometry();
  SwerveKinematics kinematics{geometry};
  const double full = WheelRps(1.0, geometry.modules[0].wheel_radius_m);
  for (double angle : {0.0, 0.125, 0.25}) {
    std::array<ModuleState, kModuleCount> current{};
    for (auto& module : current) module.steer_rot = angle;
    const auto states = kinematics.ToModuleStates({1, 0, 0}, current);
    if (!states.has_value()) {
      FAIL() << "Expected module states for valid chassis speeds";
    }
    const auto& module_states = *states;
    for (const auto& state : module_states) {
      EXPECT_NEAR(state.drive_rps, full * std::cos(angle * kTau), 1e-12);
    }
  }
}

TEST(Swerve, DiscretizationCompensatesCombinedTranslationAndRotation) {
  SwerveKinematics kinematics{Geometry()};
  constexpr double dt = 0.02;
  const auto initial = kinematics.ToModuleStates({1, 0.5, 2}, {}, dt);
  if (!initial.has_value()) {
    FAIL() << "Expected initial module states";
  }
  const auto aligned = kinematics.ToModuleStates({1, 0.5, 2}, *initial, dt);
  if (!aligned.has_value()) {
    FAIL() << "Expected aligned module states";
  }
  const auto actual = kinematics.ToChassisSpeeds(*aligned);
  // Independent Twist2d log oracle for (vx,vy,omega,dt)=(1,0.5,2,0.02):
  // a=0.02, c=a*cot(a), (vx',vy')=(c+0.5*a, -a+0.5*c).
  EXPECT_NEAR(actual.vx_mps, 1.0098666631109754, 1e-12);
  EXPECT_NEAR(actual.vy_mps, 0.47993333155548773, 1e-12);
  EXPECT_NEAR(actual.omega_radps, 2.0, 1e-12);
  EXPECT_GT(std::abs(actual.vy_mps - 0.5), 0.01);
}

TEST(Swerve, ForwardKinematicsHandlesOffCenterNonSquareLayout) {
  auto geometry = Geometry();
  const std::array<double, kModuleCount> xs{0.1, 0.9, 0.2, 1.1};
  const std::array<double, kModuleCount> ys{-0.2, 0.1, 0.6, 0.8};
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    geometry.modules[i].x_m = xs[i];
    geometry.modules[i].y_m = ys[i];
  }
  SwerveKinematics kinematics{geometry};
  std::array<ModuleState, kModuleCount> sideways{};
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    sideways[i] = {WheelRps(0.5, geometry.modules[i].wheel_radius_m), 0.25};
  }
  const auto translation = kinematics.ToChassisSpeeds(sideways);
  EXPECT_NEAR(translation.vx_mps, 0.0, 1e-15);
  EXPECT_NEAR(translation.vy_mps, 0.5, 1e-15);
  EXPECT_NEAR(translation.omega_radps, 0.0, 1e-15);
  for (const ChassisSpeeds requested :
       {ChassisSpeeds{0.4, -0.3, 0.2}, ChassisSpeeds{-0.2, 0.5, -0.4}}) {
    const auto initial = kinematics.ToModuleStates(requested, {});
    ASSERT_TRUE(initial.has_value());
    const auto aligned = kinematics.ToModuleStates(requested, *initial);
    ASSERT_TRUE(aligned.has_value());
    const auto recovered = kinematics.ToChassisSpeeds(*aligned);
    EXPECT_NEAR(recovered.vx_mps, requested.vx_mps, 1e-12);
    EXPECT_NEAR(recovered.vy_mps, requested.vy_mps, 1e-12);
    EXPECT_NEAR(recovered.omega_radps, requested.omega_radps, 1e-12);
  }
}

// --- geometry from config ------------------------------------------------

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

  // A steer motor on its own rotor: azimuth would be relative to boot.
  auto rotor = config;
  for (auto& motor : rotor.hardware.motors) {
    if (motor.name == "front_left_steer") {
      motor.feedback = talos::hardware::Feedback::kRotor;
    }
  }
  try {
    BuildSwerveGeometry(rotor, "drivetrain", *geometry);
    ADD_FAILURE() << "Rotor steer feedback was accepted";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string{error.what()}.find("RemoteCANcoder"),
              std::string::npos)
        << error.what();
  }

  // A steer motor whose frame is not declared absolute: the devices layer
  // would hold its positions neutral until a rezero that never comes.
  auto relative = config;
  for (auto& motor : relative.hardware.motors) {
    if (motor.name == "front_left_steer") {
      motor.absolute_position_frame.value = false;
    }
  }
  try {
    BuildSwerveGeometry(relative, "drivetrain", *geometry);
    ADD_FAILURE() << "A steer motor without an absolute frame was accepted";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string{error.what()}.find("position_frame"),
              std::string::npos)
        << error.what();
  }

  // An encoder geared to the module: its frame does not survive a reset.
  auto geared = config;
  for (auto& motor : geared.hardware.motors) {
    if (motor.name == "front_left_steer") motor.sensor_to_mechanism_ratio = 2;
  }
  try {
    BuildSwerveGeometry(geared, "drivetrain", *geometry);
    ADD_FAILURE() << "A geared steer encoder was accepted";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string{error.what()}.find("sensor_to_mechanism_ratio"),
              std::string::npos)
        << error.what();
  }

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
  // copy_file keeps the source's mode: a read-only checkout (CI) yields a
  // read-only copy that overwrite_existing could not replace next time.
  std::filesystem::remove(dir / "robot.toml");
  std::filesystem::copy_file(source, dir / "robot.toml");
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

  // An enabled robot in teleop: the policy selects /drivetrain/target/teleop.
  void Teleop() {
    hw.enable();
    hw.set_driver_station(talos::testing::DsPacket(ds::kEnabled | ds::kTeleop));
    sim.step(10ms);
  }
  // Holds a lane the way a producer does: republished every 20 ms.
  void Hold(const ChassisTarget& target, std::chrono::milliseconds duration,
            const char* source = "teleop") {
    for (auto t = 0ms; t < duration; t += 20ms) {
      talos::testing::PublishTarget(sim, "drivetrain", source, target);
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
  void ExpectNeutral() const {
    for (const SwerveModule& module : geometry.modules) {
      EXPECT_EQ(Request(module.drive).mode, Mode::kNeutral) << module.name;
      EXPECT_EQ(Request(module.steer).mode, Mode::kNeutral) << module.name;
    }
  }
  // DrivetrainState messages published so far.
  std::uint64_t StateCount() const {
    return talos::detail::SimAccess::message_count(sim, topics::kState,
                                                   sizeof(DrivetrainState));
  }
  // One state per drivetrain tick (5 ms in testdata) across `duration`.
  void ExpectPublishedEveryTick(std::uint64_t before,
                                std::chrono::milliseconds duration) const {
    EXPECT_EQ(StateCount() - before,
              static_cast<std::uint64_t>(duration / 5ms));
  }

  Sim sim;
  Drivetrain& drivetrain = sim.add<Drivetrain>("drivetrain");
  SimHardware hw{sim};
  const SwerveGeometry geometry = Geometry();
};

// The geometry is parsed from the table, not through Node::param, so the
// node declares it separately (DeclareSwerveGeometry). Every geometry key
// the config sets must be among the node's declared reads, or the launcher
// refuses the robot's own config.
TEST_F(DrivetrainTest, DeclaresEveryGeometryKeyTheConfigSets) {
  std::vector<std::string> declared;
  for (const auto& read :
       talos::detail::NodeAccess::params_read(sim.node("drivetrain"))) {
    if (read.subsystem == "drivetrain") declared.push_back(read.key);
  }
  const auto config = Config();
  const toml::table* table = config.GetSubsystemTable("drivetrain", "geometry");
  ASSERT_NE(table, nullptr);
  std::vector<std::string> set;
  for (const auto& [key, value] : *table) {
    if (key.str() != "modules") {
      set.push_back("geometry." + std::string{key.str()});
      continue;
    }
    const toml::array& modules = *value.as_array();
    for (std::size_t i = 0; i < modules.size(); ++i) {
      for (const auto& [field, _] : *modules[i].as_table()) {
        set.push_back("geometry.modules[" + std::to_string(i) + "]." +
                      std::string{field.str()});
      }
    }
  }
  ASSERT_FALSE(set.empty());
  for (const auto& key : set) {
    EXPECT_NE(std::find(declared.begin(), declared.end(), key), declared.end())
        << key;
  }
}

TEST_F(DrivetrainTest, DrivesEveryModuleToAFreshTarget) {
  Teleop();
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  const double rps = 1.0 / (kTau * geometry.modules[0].wheel_radius_m);
  for (const SwerveModule& module : geometry.modules) {
    EXPECT_EQ(Request(module.drive).mode, Mode::kVelocity) << module.name;
    EXPECT_NEAR(Request(module.drive).demand, rps, 1e-9) << module.name;
    EXPECT_EQ(Request(module.steer).mode, Mode::kPosition) << module.name;
    EXPECT_NEAR(Request(module.steer).demand, 0.0, 1e-9) << module.name;
  }
  EXPECT_TRUE(State().enabled());
}

TEST_F(DrivetrainTest, DesaturationSlowsEveryWheelByOneFactorToTheDriveLimit) {
  // Translating while turning asks the wheels on the outside of the turn for
  // more than max_velocity_rps. All four slow by one factor, so the chassis
  // keeps the commanded shape of motion.
  const ChassisSpeeds request{4.0, 0.0, 6.0};
  Teleop();
  hw.ideal_motors();  // the modules settle on their headings
  Hold(ChassisTarget{request.vx_mps, request.vy_mps, request.omega_radps},
       40ms);
  std::array<ModuleState, kModuleCount> settled{};
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    settled[i].steer_rot = Request(geometry.modules[i].steer).demand;
  }
  SwerveGeometry unlimited = geometry;
  for (SwerveModule& module : unlimited.modules) {
    module.max_drive_rps = std::numeric_limits<double>::infinity();
  }
  SwerveKinematics kinematics{unlimited};
  const auto wanted = kinematics.ToModuleStates(request, settled, 0.005);
  ASSERT_TRUE(wanted.has_value());
  double excess = 0.0;
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    excess = std::max(excess, std::abs((*wanted)[i].drive_rps) /
                                  geometry.modules[i].max_drive_rps);
  }
  ASSERT_GT(excess, 1.0);
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    const auto drive = Request(geometry.modules[i].drive);
    EXPECT_EQ(drive.mode, Mode::kVelocity) << geometry.modules[i].name;
    EXPECT_LE(std::abs(drive.demand), geometry.modules[i].max_drive_rps + 1e-9)
        << geometry.modules[i].name;
    EXPECT_NEAR(drive.demand, (*wanted)[i].drive_rps / excess, 1e-9)
        << geometry.modules[i].name;
  }
}

class DrivetrainSlewTest : public DrivetrainTest {
 protected:
  DrivetrainSlewTest()
      : DrivetrainTest{ConfigWith("max_linear_accel_mps2 = 20.0\n"
                                  "max_angular_accel_radps2 = 40.0")} {}

  void ExpectCommand(ChassisSpeeds speeds) {
    SwerveKinematics kinematics{geometry};
    const auto expected = kinematics.ToModuleStates(speeds, {}, 0.005);
    if (!expected.has_value()) {
      FAIL() << "Expected module states";
    }
    const auto& expected_states = *expected;
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      EXPECT_EQ(Request(geometry.modules[i].drive).mode, Mode::kVelocity);
      EXPECT_NEAR(Request(geometry.modules[i].drive).demand,
                  expected_states[i].drive_rps, 1e-9);
      EXPECT_NEAR(Request(geometry.modules[i].steer).demand,
                  expected_states[i].steer_rot, 1e-9);
    }
  }
};

TEST_F(DrivetrainSlewTest, TranslationUsesVectorAccelerationAndConverges) {
  Teleop();
  // A 3-4-5 diagonal: each 5 ms tick adds 0.1 m/s in magnitude,
  // not 0.1 m/s independently on each axis.
  for (int tick = 1; tick <= 7; ++tick) {
    talos::testing::PublishTarget(sim, "drivetrain", "teleop",
                                  ChassisTarget{0.3, 0.4, 0.0});
    sim.step(5ms);
    const double fraction = std::min(tick / 5.0, 1.0);
    ExpectCommand({0.3 * fraction, 0.4 * fraction, 0.0});
  }
}

TEST_F(DrivetrainSlewTest,
       AngularAccelerationLimitsBothDirectionsAndConverges) {
  Teleop();
  for (int tick = 1; tick <= 7; ++tick) {
    talos::testing::PublishTarget(sim, "drivetrain", "teleop",
                                  ChassisTarget{0.0, 0.0, 1.0});
    sim.step(5ms);
    ExpectCommand({0.0, 0.0, std::min(tick * 0.2, 1.0)});
  }
  for (int tick = 1; tick <= 12; ++tick) {
    talos::testing::PublishTarget(sim, "drivetrain", "teleop",
                                  ChassisTarget{0.0, 0.0, -1.0});
    sim.step(5ms);
    ExpectCommand({0.0, 0.0, std::max(1.0 - tick * 0.2, -1.0)});
  }
}

class DrivetrainSlewResetTest : public DrivetrainSlewTest,
                                public ::testing::WithParamInterface<int> {};

TEST_P(DrivetrainSlewResetTest, UnsafeInputNeutralizesAndRestartsFromZero) {
  Teleop();
  Hold(ChassisTarget{1.0, 0.0, 2.0}, 40ms);
  ExpectCommand({0.8, 0.0, 1.6});
  switch (GetParam()) {
    case 0:
      talos::testing::ReleaseTarget<ChassisTarget>(sim, "drivetrain", "teleop");
      break;
    case 1:
      hw.enable(false);
      break;
    case 2:
      sim.step(150ms);  // target lane expires
      break;
    case 3:
      talos::testing::PublishTarget(
          sim, "drivetrain", "teleop",
          ChassisTarget{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
      break;
    case 4:
      for (const auto& module : geometry.modules)
        hw.set_connected("drivetrain", module.steer, false);
      break;
    default:
      FAIL() << "Unknown reset scenario";
  }
  // Hardware feedback is delivered after the tick that samples it; lane
  // changes are visible directly on the next tick.
  const auto visibility = (GetParam() == 1 || GetParam() == 4) ? 10ms : 5ms;
  sim.step(visibility);
  ExpectNeutral();
  hw.enable();
  for (const auto& module : geometry.modules)
    hw.set_connected("drivetrain", module.steer, true);
  talos::testing::PublishTarget(sim, "drivetrain", "teleop",
                                ChassisTarget{1.0, 0.0, 2.0});
  sim.step(visibility);
  ExpectCommand({0.1, 0.0, 0.2});
}

INSTANTIATE_TEST_SUITE_P(SafetyGates, DrivetrainSlewResetTest,
                         ::testing::Values(0, 1, 2, 3, 4));

class DrivetrainFieldSlewTest : public DrivetrainTest {
 protected:
  DrivetrainFieldSlewTest()
      : DrivetrainTest{ConfigWith("max_linear_accel_mps2 = 4.0")} {}
};

TEST_F(DrivetrainFieldSlewTest, ConstantFieldVelocityWhileRotatingIsReached) {
  // 2 m/s along the field x axis while spinning at 5 rad/s: in the robot
  // frame the target turns at 5 rad/s, which a robot-frame limit chases at
  // v * omega = 10 m/s^2 > 4 m/s^2 and settles near 4 / 5 = 0.8 m/s. Limited
  // in the field frame the target is constant and is reached in 0.5 s.
  constexpr double kSpeedMps = 2.0;
  constexpr double kOmegaRadps = 5.0;
  Teleop();
  hw.ideal_motors();  // the modules do what they are asked; heading follows
  for (auto t = 0ms; t < 1000ms; t += 5ms) {
    const double yaw = State().yaw_rad();
    talos::testing::PublishTarget(
        sim, "drivetrain", "teleop",
        ChassisTarget{kSpeedMps * std::cos(yaw), -kSpeedMps * std::sin(yaw),
                      kOmegaRadps});
    sim.step(5ms);
  }
  const DrivetrainState state = State();
  ASSERT_TRUE(state.yaw_valid());
  EXPECT_NEAR(state.omega_radps(), kOmegaRadps, 0.02 * kOmegaRadps);
  EXPECT_GT(state.yaw_rad(), 2.0);  // it really turned
  const double c = std::cos(state.yaw_rad());
  const double s = std::sin(state.yaw_rad());
  const double field_vx = c * state.vx_mps() - s * state.vy_mps();
  const double field_vy = s * state.vx_mps() + c * state.vy_mps();
  // The measured robot-frame velocity lags the field target by a tick of
  // rotation (and SE2 discretization skew): a few hundredths of a radian.
  EXPECT_NEAR(std::hypot(field_vx, field_vy), kSpeedMps, 0.02 * kSpeedMps);
  EXPECT_NEAR(std::atan2(field_vy, field_vx), 0.0, 0.1);
}

TEST_F(DrivetrainTest, NeverSentStaysNeutralButStillReportsState) {
  Teleop();
  sim.step(60ms);
  ExpectNeutral();
  EXPECT_TRUE(State().modules_valid());
  EXPECT_TRUE(State().enabled());
}

TEST_F(DrivetrainTest, DisabledRobotGoesNeutralAndStillReportsValidModules) {
  Teleop();
  hw.enable(false);
  const std::uint64_t before = StateCount();
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  ExpectNeutral();
  ExpectPublishedEveryTick(before, 40ms);
  EXPECT_FALSE(State().enabled());
  EXPECT_TRUE(State().modules_valid());
  EXPECT_TRUE(State().yaw_valid());
}

TEST_F(DrivetrainTest, DriverStationLossOrOtherModeLaneGoesNeutral) {
  Teleop();
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  ASSERT_EQ(Request("front_left_drive").mode, Mode::kVelocity);

  // The Driver Station goes silent; its last packet ages out.
  hw.stop_driver_station();
  Hold(ChassisTarget{1.0, 0, 0}, 160ms);
  ExpectNeutral();

  // Autonomous selects the auto lane; teleop's keeps talking to nobody.
  hw.set_driver_station(
      talos::testing::DsPacket(ds::kEnabled | ds::kAutonomous));
  sim.step(10ms);
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  ExpectNeutral();
  Hold(ChassisTarget{1.0, 0, 0}, 40ms, "auto");
  EXPECT_EQ(Request("front_left_drive").mode, Mode::kVelocity);
}

TEST_F(DrivetrainTest, StaleLaneGoesNeutralOnTheFirstTickAtTheTimeout) {
  Teleop();
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  sim.step(75ms);  // last publication is 95 ms old: still fresh
  for (const SwerveModule& module : geometry.modules) {
    ASSERT_EQ(Request(module.drive).mode, Mode::kVelocity) << module.name;
  }
  sim.step(5ms);  // exactly command_timeout_us
  ExpectNeutral();
}

TEST_F(DrivetrainTest, TargetFromBeforeAReenableStaysNeutralInTheSameDrain) {
  Teleop();
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  ASSERT_EQ(Request("front_left_drive").mode, Mode::kVelocity);
  const std::uint64_t before = talos::testing::DriverStationGeneration(sim);
  hw.set_driver_station(talos::testing::DsPacket(ds::kTeleop));  // disabled
  sim.step(10ms);
  // Re-enabled off the 5 ms tick grid: the packet lands just after the
  // drivetrain's tick and waits for its next drain, together with a target
  // its producer queued under the earlier enable.
  sim.step(1ms);
  hw.set_driver_station(talos::testing::DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(4ms);
  ASSERT_NE(talos::testing::DriverStationGeneration(sim), before);
  talos::testing::PublishTarget(sim, "drivetrain", "teleop",
                                ChassisTarget{1.0, 0, 0}, before);
  sim.step(5ms);
  ExpectNeutral();
  // The same word stamped with the current generation drives at once.
  talos::testing::PublishTarget(sim, "drivetrain", "teleop",
                                ChassisTarget{1.0, 0, 0});
  sim.step(5ms);
  EXPECT_EQ(Request("front_left_drive").mode, Mode::kVelocity);
}

TEST_F(DrivetrainTest, ReportsTheChassisMotionTheModulesMeasure) {
  // Every wheel pointing 90 degrees left at 2 rps.
  for (const SwerveModule& module : geometry.modules) {
    hw.set_motor("drivetrain", module.drive, 0.0, 2.0);
    hw.set_motor("drivetrain", module.steer, 0.25, 0.0);
  }
  sim.step(kConverged);
  const DrivetrainState state = State();
  EXPECT_TRUE(state.modules_valid());
  EXPECT_NEAR(state.vx_mps(), 0.0, 1e-9);
  EXPECT_NEAR(state.vy_mps(), 2.0 * kTau * 0.0508, 1e-9);
  EXPECT_NEAR(state.omega_radps(), 0.0, 1e-9);
}

geometry::SwerveModulePosition Module(const DrivetrainState& state,
                                      std::size_t i) {
  switch (i) {
    case 0:
      return state.module0();
    case 1:
      return state.module1();
    case 2:
      return state.module2();
    default:
      return state.module3();
  }
}

TEST_F(DrivetrainTest, ModulePositionsComeFromTheDriveAndSteerMotors) {
  // Different values per module, so a module reported in another's slot or
  // a drive/steer swap shows.
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    hw.set_motor("drivetrain", geometry.modules[i].drive,
                 1.0 + static_cast<double>(i), 0.0);
    hw.set_motor("drivetrain", geometry.modules[i].steer,
                 -0.1 * static_cast<double>(i), 0.0);
  }
  sim.step(kConverged);
  const DrivetrainState state = State();
  ASSERT_TRUE(state.modules_valid());
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    const geometry::SwerveModulePosition m = Module(state, i);
    EXPECT_NEAR(m.distance_m(),
                (1.0 + i) * kTau * geometry.modules[i].wheel_radius_m, 1e-12)
        << geometry.modules[i].name;
    EXPECT_NEAR(m.angle_rad(), -0.1 * i * kTau, 1e-12)
        << geometry.modules[i].name;
  }
}

TEST_F(DrivetrainTest, DrivesBeforeTheClockMappingConvergesButReportsNoSample) {
  // Four hardware states in: the bridge's offset filter needs eight, so no
  // sample time is known yet. The modules are measured and driven anyway;
  // only the published measurement is not believed.
  Teleop();  // the lane must be published inside the teleop generation
  talos::testing::PublishTarget(sim, "drivetrain", "teleop",
                                ChassisTarget{1.0, 0, 0});
  sim.step(10ms);
  const DrivetrainState early = State();
  EXPECT_FALSE(early.modules_valid());
  EXPECT_FALSE(early.yaw_valid());
  EXPECT_EQ(early.sample_time_ns(), 0u);
  EXPECT_EQ(Request("front_left_drive").mode, Mode::kVelocity);

  Hold(ChassisTarget{1.0, 0, 0}, 60ms);
  EXPECT_TRUE(State().modules_valid());
  EXPECT_TRUE(State().yaw_valid());
}

TEST_F(DrivetrainTest, SampleTimeIsTheMappedHardwareSampleTimeNotTheTickTime) {
  // SimHardware reports the true gateway clock mapping, so a state sampled
  // at gateway time maps exactly; it arrives 4 ms later and is consumed at
  // the next 5 ms tick. A drivetrain stamping its own tick time would read
  // within a tick of now, not at least the 4 ms link latency behind it.
  hw.set_gateway_clock(std::chrono::seconds{1000}, 1ms);
  Teleop();
  sim.step(kConverged);
  ASSERT_TRUE(State().modules_valid());
  hw.set_gateway_clock(std::chrono::seconds{1000}, 4ms);
  sim.step(20ms);
  const DrivetrainState state = State();
  ASSERT_TRUE(state.modules_valid());
  const std::int64_t lag_ns =
      sim.now().nanos() - static_cast<std::int64_t>(state.sample_time_ns());
  EXPECT_GE(lag_ns, std::chrono::nanoseconds{4ms}.count());
  EXPECT_LE(lag_ns, std::chrono::nanoseconds{4ms + 5ms}.count());
}

TEST_F(DrivetrainTest, ModuleTimestampUsesOldestConsumedSignalAcquisition) {
  sim.step(kConverged);
  for (int signal = 0; signal < 3; ++signal) {
    SCOPED_TRACE(signal);
    // Unequal ages distinguish drive position, drive velocity and azimuth.
    // Steer velocity is not consumed: an age the drivetrain would reject
    // (over 50 ms) but the gateway still counts valid (under 100 ms) must
    // not invalidate this solution.
    hw.set_motor("drivetrain", geometry.modules[0].drive, 0, 0,
                 signal == 0 ? 12'000 : 2'000, signal == 1 ? 12'000 : 3'000);
    hw.set_motor("drivetrain", geometry.modules[0].steer, 0, 0,
                 signal == 2 ? 12'000 : 4'000, 60'000);
    sim.step(20ms);
    const auto state = State();
    ASSERT_TRUE(state.modules_valid());
    ASSERT_TRUE(state.yaw_valid());
    const auto lag =
        sim.now().nanos() - static_cast<std::int64_t>(state.sample_time_ns());
    EXPECT_GE(lag, std::chrono::nanoseconds{12ms}.count());
    EXPECT_LE(lag, std::chrono::nanoseconds{17ms}.count());
  }
}

// Signal times the drivetrain's timestamp policy rejects while the gateway
// still counts the feedback valid (younger than command_timeout_us, 100 ms
// here) invalidate only the published measurement: the module keeps driving
// and the safety gates are unchanged.
TEST_F(DrivetrainTest, InvalidSignalTimesDoNotChangeDriveOrSafetyGates) {
  Teleop();
  Hold(ChassisTarget{1, 0, 0}, 60ms);
  for (const std::uint32_t age : {25'000U, 60'000U, 99'999U}) {
    SCOPED_TRACE(age);
    // Respectively excessive skew, excessive age, the oldest still valid.
    for (int signal = 0; signal < 3; ++signal) {
      SCOPED_TRACE(signal);
      hw.set_motor("drivetrain", geometry.modules[0].drive, 0, 0,
                   signal == 0 ? age : 0, signal == 1 ? age : 0);
      hw.set_motor("drivetrain", geometry.modules[0].steer, 0, 0,
                   signal == 2 ? age : 0, 0);
      Hold(ChassisTarget{1, 0, 0}, 20ms);
      EXPECT_FALSE(State().modules_valid());
      EXPECT_FALSE(State().yaw_valid());
      EXPECT_EQ(State().sample_time_ns(), 0u);
      EXPECT_EQ(Request(geometry.modules[0].drive).mode, Mode::kVelocity);
    }
  }
  sim.step(150ms);
  ExpectNeutral();  // stale target still wins despite live hardware
  Hold(ChassisTarget{1, 0, 0}, 20ms);
  ASSERT_EQ(Request(geometry.modules[0].drive).mode, Mode::kVelocity);
  hw.enable(false);
  Hold(ChassisTarget{1, 0, 0}, 20ms);
  ExpectNeutral();
  hw.set_motor("drivetrain", geometry.modules[0].drive, 0, 0);
  hw.set_motor("drivetrain", geometry.modules[0].steer, 0, 0);
  sim.step(20ms);
  EXPECT_TRUE(State().modules_valid());
  EXPECT_TRUE(State().yaw_valid());
  ExpectNeutral();
}

// REQ-SAFE-22: feedback command_timeout_us old or older, or of unknown age,
// is invalid at the gateway, so the drivetrain treats that motor as it does
// a disconnected one: its module goes neutral, the others drive, and it
// drives again once its feedback is fresh.
TEST_F(DrivetrainTest, FeedbackAtTheGatewayAgeBoundNeutralizesItsModule) {
  Teleop();
  Hold(ChassisTarget{1, 0, 0}, 60ms);
  constexpr std::uint32_t kUnknown = std::numeric_limits<std::uint32_t>::max();
  for (const std::uint32_t age : {100'000U, 1'000'000U, kUnknown}) {
    SCOPED_TRACE(age);
    for (int signal = 0; signal < 3; ++signal) {
      SCOPED_TRACE(signal);
      hw.set_motor("drivetrain", geometry.modules[0].drive, 0, 0,
                   signal == 0 ? age : 0, signal == 1 ? age : 0);
      hw.set_motor("drivetrain", geometry.modules[0].steer, 0, 0,
                   signal == 2 ? age : 0, 0);
      Hold(ChassisTarget{1, 0, 0}, 20ms);
      EXPECT_FALSE(State().modules_valid());
      EXPECT_EQ(Request(geometry.modules[0].drive).mode, Mode::kNeutral);
      EXPECT_EQ(Request(geometry.modules[0].steer).mode, Mode::kNeutral);
      EXPECT_EQ(Request(geometry.modules[1].drive).mode, Mode::kVelocity);
    }
  }
  hw.set_motor("drivetrain", geometry.modules[0].drive, 0, 0);
  hw.set_motor("drivetrain", geometry.modules[0].steer, 0, 0);
  Hold(ChassisTarget{1, 0, 0}, 40ms);
  EXPECT_TRUE(State().modules_valid());
  EXPECT_EQ(Request(geometry.modules[0].drive).mode, Mode::kVelocity);
}

TEST_F(DrivetrainTest, EquallyOldSignalsAreRejectedEvenWithoutSkew) {
  for (const auto& module : geometry.modules) {
    hw.set_motor("drivetrain", module.drive, 0, 0, 60'000, 60'000);
    hw.set_motor("drivetrain", module.steer, 0, 0, 60'000, 0);
  }
  sim.step(200ms);
  EXPECT_FALSE(State().modules_valid());
  EXPECT_FALSE(State().yaw_valid());
  EXPECT_EQ(State().sample_time_ns(), 0u);
}

TEST_F(DrivetrainTest, ADisconnectedMotorNeutralizesOnlyItsModule) {
  Teleop();
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    hw.set_motor("drivetrain", geometry.modules[i].drive,
                 1.0 + static_cast<double>(i), 0.0);
  }
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  ASSERT_EQ(Request("front_left_drive").mode, Mode::kVelocity);
  const double yaw_before = State().yaw_rad();
  const DrivetrainState valid = State();

  hw.set_connected("drivetrain", "front_left_steer", false);
  const std::uint64_t before = StateCount();
  Hold(ChassisTarget{1.0, 0, 0}, 60ms);  // a fresh target the whole time
  EXPECT_EQ(Request("front_left_drive").mode, Mode::kNeutral);
  EXPECT_EQ(Request("front_left_steer").mode, Mode::kNeutral);
  EXPECT_EQ(Request("back_left_drive").mode, Mode::kVelocity);
  ExpectPublishedEveryTick(before, 60ms);
  const DrivetrainState state = State();
  EXPECT_TRUE(state.enabled());
  EXPECT_FALSE(state.modules_valid());
  // Under --sim the heading integrates from the modules, so it goes too.
  EXPECT_FALSE(state.yaw_valid());
  EXPECT_EQ(state.yaw_rad(), yaw_before);
  EXPECT_EQ(state.vx_mps(), 0.0);
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    EXPECT_EQ(Module(state, i).distance_m(), Module(valid, i).distance_m());
  }

  hw.set_connected("drivetrain", "front_left_steer", true);
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  EXPECT_TRUE(State().modules_valid());
  EXPECT_EQ(Request("front_left_drive").mode, Mode::kVelocity);
}

TEST_F(DrivetrainTest, LostCANcoderIsolatesWholeModuleAndLiveSubsetTranslates) {
  Teleop();
  hw.ideal_motors();
  Hold(ChassisTarget{1.0, 0, 0}, 100ms);
  hw.set_connected("drivetrain", "front_left_encoder", false);
  Hold(ChassisTarget{1.0, 0, 0}, 100ms);
  EXPECT_EQ(Request("front_left_drive").mode, Mode::kNeutral);
  EXPECT_EQ(Request("front_left_steer").mode, Mode::kNeutral);
  for (const auto& module : geometry.modules) {
    if (module.name == "front_left") continue;
    EXPECT_EQ(Request(module.drive).mode, Mode::kVelocity);
    EXPECT_GT(Request(module.drive).demand, 0.0);
    EXPECT_EQ(Request(module.steer).mode, Mode::kPosition);
  }
  EXPECT_NEAR(State().vx_mps(), 1.0, 1e-9);
  EXPECT_NEAR(State().vy_mps(), 0.0, 1e-9);
  EXPECT_FALSE(State().modules_valid());
  hw.set_connected("drivetrain", "front_right_drive", false);
  Hold(ChassisTarget{1.0, 0, 0}, 60ms);
  EXPECT_EQ(Request("back_left_drive").mode, Mode::kVelocity);
  EXPECT_NEAR(State().vx_mps(), 1.0, 1e-9);
  hw.set_connected("drivetrain", "back_right_drive", false);
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  ExpectNeutral();
}

TEST_F(DrivetrainTest, DegradedDriveStillHonorsDisableAndStaleTargets) {
  Teleop();
  hw.set_connected("drivetrain", "front_left_encoder", false);
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  ASSERT_EQ(Request("back_left_drive").mode, Mode::kVelocity);
  sim.step(150ms);
  ExpectNeutral();
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  ASSERT_EQ(Request("back_left_drive").mode, Mode::kVelocity);
  hw.enable(false);
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  ExpectNeutral();
}

TEST_F(DrivetrainTest, DriveDistanceKeepsTravelAcrossAGapButNotARestart) {
  hw.set_motor("drivetrain", "front_left_drive", 100.0, 0.0);
  sim.step(kConverged);
  const double before = State().module2().distance_m();
  const double travel = WheelTravel(0.1, geometry.modules[2].wheel_radius_m);
  // A count restarted at zero is not 100 rotations of travel.
  hw.set_motor("drivetrain", "front_left_drive", 0.0, 0.0);
  sim.step(20ms);
  EXPECT_NEAR(State().module2().distance_m(), before, 1e-9);
  // Travel made while the feedback was invalid for 20 ms is still travel.
  hw.set_connected("drivetrain", "front_left_drive", false);
  sim.step(20ms);
  hw.set_motor("drivetrain", "front_left_drive", 0.1, 0.0);
  hw.set_connected("drivetrain", "front_left_drive", true);
  sim.step(20ms);
  EXPECT_NEAR(State().module2().distance_m(), before + travel, 1e-9);
  // After a longer silence the count may have restarted unreported.
  hw.set_connected("drivetrain", "front_left_drive", false);
  sim.step(200ms);
  hw.set_motor("drivetrain", "front_left_drive", 0.0, 0.0);
  hw.set_connected("drivetrain", "front_left_drive", true);
  sim.step(20ms);
  EXPECT_NEAR(State().module2().distance_m(), before + travel, 1e-9);
  hw.set_motor("drivetrain", "front_left_drive", 0.1, 0.0);
  sim.step(20ms);
  EXPECT_NEAR(State().module2().distance_m(), before + 2 * travel, 1e-9);
}

TEST_F(DrivetrainTest, DrivePositionResetWhileDrivingPreservesTravel) {
  Teleop();
  hw.ideal_motors();
  Hold(ChassisTarget{1.0, 0, 0}, 500ms);
  const double before = State().module2().distance_m();
  ASSERT_GT(before, 0.3);
  hw.set_motor("drivetrain", "front_left_drive", 0.0,
               WheelRps(1.0, geometry.modules[2].wheel_radius_m));
  Hold(ChassisTarget{1.0, 0, 0}, 20ms);
  EXPECT_TRUE(State().modules_valid());
  EXPECT_GE(State().module2().distance_m(), before);
  EXPECT_LT(State().module2().distance_m() - before, 0.03);
  const double reanchored = State().module2().distance_m();
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  EXPECT_NEAR(State().module2().distance_m() - reanchored, 0.04, 1e-9);
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
  const double expected_rad = 1.0 * 0.2;
  EXPECT_TRUE(State().yaw_valid());
  EXPECT_GT(State().yaw_rad(), expected_rad * 0.9);
  EXPECT_LT(State().yaw_rad(), expected_rad * 1.02);
  EXPECT_NEAR(State().omega_radps(), 1.0, 1e-9);
}

TEST_F(DrivetrainTest, RejectedAcquisitionsDoNotAdvanceIntegratedHeading) {
  SetRotating(hw, geometry, 1.0);
  sim.step(kConverged);
  ASSERT_TRUE(State().yaw_valid());
  hw.set_motor("drivetrain", geometry.modules[0].drive, 0, 0, 0, 25'000);
  sim.step(20ms);
  EXPECT_FALSE(State().modules_valid());
  EXPECT_FALSE(State().yaw_valid());
  // The state already queued before the age change may integrate once.
  const double held = State().yaw_rad();
  sim.step(20ms);
  EXPECT_EQ(State().yaw_rad(), held);
  SetRotating(hw, geometry, 1.0);
  sim.step(20ms);
  EXPECT_TRUE(State().yaw_valid());
  EXPECT_GT(State().yaw_rad(), held);
  EXPECT_LE(State().yaw_rad() - held, 0.020 + 1e-9);
}

class DrivetrainOnImuTest : public DrivetrainTest {
 protected:
  DrivetrainOnImuTest()
      : DrivetrainTest{ConfigWith("integrate_heading = false")} {}
};

TEST_F(DrivetrainOnImuTest, HeadingIsThePigeonYawInRadians) {
  SetRotating(hw, geometry, 1.0);
  hw.set_cancoder("drivetrain", "back_left_encoder", 0.375, 2.0);
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.125, 0.5);  // rotations
  sim.step(200ms);
  EXPECT_TRUE(State().yaw_valid());
  EXPECT_NEAR(State().yaw_rad(), std::numbers::pi / 4, 1e-12);
  // The chassis rate is the modules', not the Pigeon's.
  EXPECT_NEAR(State().omega_radps(), 1.0, 1e-9);
}

TEST_F(DrivetrainOnImuTest, TimestampIncludesYawButNotUnusedYawRate) {
  hw.set_motor("drivetrain", geometry.modules[0].drive, 0, 0, 3'000, 5'000);
  // A yaw rate older than the drivetrain accepts, still valid at the gateway.
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.125, 0, 12'000, 60'000);
  sim.step(kConverged);
  ASSERT_TRUE(State().modules_valid());
  ASSERT_TRUE(State().yaw_valid());
  auto lag =
      sim.now().nanos() - static_cast<std::int64_t>(State().sample_time_ns());
  EXPECT_GE(lag, std::chrono::nanoseconds{12ms}.count());
  EXPECT_LE(lag, std::chrono::nanoseconds{17ms}.count());
  // Conversely an older module, not the gyro, owns the shared timestamp.
  hw.set_motor("drivetrain", geometry.modules[0].drive, 0, 0, 15'000, 5'000);
  sim.step(20ms);
  ASSERT_TRUE(State().modules_valid());
  ASSERT_TRUE(State().yaw_valid());
  lag = sim.now().nanos() - static_cast<std::int64_t>(State().sample_time_ns());
  EXPECT_GE(lag, std::chrono::nanoseconds{15ms}.count());
  EXPECT_LE(lag, std::chrono::nanoseconds{20ms}.count());
}

TEST_F(DrivetrainOnImuTest, InvalidYawTimingHoldsHeadingAndKeepsModulesUsable) {
  Teleop();
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.125, 0);
  sim.step(kConverged);
  const double heading = State().yaw_rad();
  // Ages the drivetrain rejects; an older one invalidates the gyro at the
  // gateway, as ADisconnectedGyroIsInvalidAndHoldsTheLastHeading covers.
  for (const std::uint32_t age : {25'000U, 60'000U, 99'999U}) {
    SCOPED_TRACE(age);
    hw.set_gyro("drivetrain", "drivetrain_imu", 0.25, 0, age, 0);
    Hold(ChassisTarget{1, 0, 0}, 20ms);
    EXPECT_TRUE(State().modules_valid());
    EXPECT_FALSE(State().yaw_valid());
    EXPECT_EQ(State().yaw_rad(), heading);
    const auto lag =
        sim.now().nanos() - static_cast<std::int64_t>(State().sample_time_ns());
    EXPECT_GE(lag, 0);
    EXPECT_LE(lag, std::chrono::nanoseconds{5ms}.count());
    EXPECT_EQ(Request(geometry.modules[0].drive).mode, Mode::kVelocity);
  }
  // A good yaw remains independently usable when a module time is rejected.
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.125, 0, 7'000, 0);
  hw.set_motor("drivetrain", geometry.modules[0].drive, 0, 0, 60'000, 0);
  sim.step(20ms);
  EXPECT_FALSE(State().modules_valid());
  EXPECT_TRUE(State().yaw_valid());
  const auto lag =
      sim.now().nanos() - static_cast<std::int64_t>(State().sample_time_ns());
  EXPECT_GE(lag, std::chrono::nanoseconds{7ms}.count());
  EXPECT_LE(lag, std::chrono::nanoseconds{12ms}.count());
  EXPECT_EQ(State().yaw_rad(), heading);
}

TEST_F(DrivetrainOnImuTest, ADisconnectedGyroIsInvalidAndHoldsTheLastHeading) {
  Teleop();
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.125, 0.0);
  sim.step(kConverged);
  ASSERT_TRUE(State().yaw_valid());

  hw.set_connected("drivetrain", "drivetrain_imu", false);
  const std::uint64_t before = StateCount();
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  ExpectPublishedEveryTick(before, 40ms);
  const DrivetrainState state = State();
  EXPECT_FALSE(state.yaw_valid());
  EXPECT_NEAR(state.yaw_rad(), std::numbers::pi / 4, 1e-12);
  // The modules do not need the heading: still valid, still driven.
  EXPECT_TRUE(state.modules_valid());
  EXPECT_EQ(Request("front_left_drive").mode, Mode::kVelocity);
}

TEST_F(DrivetrainOnImuTest, YawKeepsRotationAcrossAGapButNotAcrossAReboot) {
  // Spinning at 5 rad/s, the Pigeon advancing every 5 ms tick.
  constexpr double kRadps = 5.0;
  double yaw_rad = 0.0;
  const auto spin = [&](std::chrono::milliseconds duration) {
    for (auto t = 0ms; t < duration; t += 5ms) {
      yaw_rad += kRadps * 0.005;
      hw.set_gyro("drivetrain", "drivetrain_imu", yaw_rad / kTau,
                  kRadps / kTau);
      sim.step(5ms);
    }
  };
  sim.step(kConverged);
  spin(20ms);
  // No usable yaw for 55 ms while the robot keeps turning.
  hw.set_connected("drivetrain", "drivetrain_imu", false);
  spin(55ms);
  EXPECT_FALSE(State().yaw_valid());
  hw.set_connected("drivetrain", "drivetrain_imu", true);
  spin(20ms);
  sim.step(10ms);  // the last yaw reaches the state
  EXPECT_TRUE(State().yaw_valid());
  EXPECT_NEAR(State().yaw_rad(), yaw_rad, 1e-9);
  // A reboot: silent for longer, then counting from zero again.
  const double heading = State().yaw_rad();
  hw.set_connected("drivetrain", "drivetrain_imu", false);
  sim.step(200ms);
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.0, 0.0);
  hw.set_connected("drivetrain", "drivetrain_imu", true);
  sim.step(20ms);
  EXPECT_TRUE(State().yaw_valid());
  EXPECT_NEAR(State().yaw_rad(), heading, 1e-9);
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.01, 0.0);
  sim.step(20ms);
  EXPECT_NEAR(State().yaw_rad(), heading + ToRadians(0.01), 1e-9);
}

TEST_F(DrivetrainOnImuTest, UnannouncedPigeonResetDoesNotJumpHeading) {
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.25, 0.0);
  sim.step(kConverged);
  const double before = State().yaw_rad();
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.0, 0.0);
  sim.step(20ms);
  EXPECT_TRUE(State().yaw_valid());
  EXPECT_NEAR(State().yaw_rad(), before, 1e-9);
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.01, 0.0);
  sim.step(20ms);
  EXPECT_NEAR(State().yaw_rad(), before + ToRadians(0.01), 1e-9);
}

// The driver's field frame is this heading less the driver's own rezero, so
// a Pigeon brownout must not move it. A restart near the old count is a
// step the rate bound would take as turning; the reported reset re-anchors.
TEST_F(DrivetrainOnImuTest, AReportedPigeonResetDoesNotMoveHeading) {
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.03, 0.0);
  sim.step(kConverged);
  const double before = State().yaw_rad();
  hw.reset_device("drivetrain", "drivetrain_imu");
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.0, 0.0);
  sim.step(20ms);
  EXPECT_TRUE(State().yaw_valid());
  EXPECT_NEAR(State().yaw_rad(), before, 1e-9);
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.01, 0.0);
  sim.step(20ms);
  EXPECT_NEAR(State().yaw_rad(), before + ToRadians(0.01), 1e-9);
}

TEST_F(DrivetrainOnImuTest, AGyroThatNeverReportedReadsAnInvalidZero) {
  hw.set_connected("drivetrain", "drivetrain_imu", false);
  hw.set_gyro("drivetrain", "drivetrain_imu", 0.125, 0.0);
  sim.step(kConverged);
  EXPECT_FALSE(State().yaw_valid());
  EXPECT_EQ(State().yaw_rad(), 0.0);
  EXPECT_TRUE(State().modules_valid());
}

TEST(Drivetrain, LinkDropKeepsPublishingInvalidStateAndHoldsPositions) {
  Sim sim{ConfigPath()};
  sim.add<Drivetrain>("drivetrain");
  auto hw = std::make_unique<SimHardware>(sim);
  hw->enable();
  hw->ideal_motors();
  hw->set_driver_station(talos::testing::DsPacket(
      talos::driver_station::kEnabled | talos::driver_station::kTeleop));
  sim.step(10ms);

  const auto drive = [&](std::chrono::milliseconds duration) {
    for (auto t = 0ms; t < duration; t += 20ms) {
      talos::testing::PublishTarget(sim, "drivetrain", "teleop",
                                    ChassisTarget{1.0, 0, 1.0});
      sim.step(20ms);
    }
  };
  const auto latest = [&] {
    const auto state = sim.latest<DrivetrainState>(topics::kState);
    EXPECT_TRUE(state.has_value());
    return state.value_or(DrivetrainState{});
  };
  const auto count = [&] {
    return talos::detail::SimAccess::message_count(sim, topics::kState,
                                                   sizeof(DrivetrainState));
  };

  drive(60ms);
  const DrivetrainState before_drop = latest();
  EXPECT_TRUE(before_drop.modules_valid());
  EXPECT_TRUE(before_drop.yaw_valid());
  EXPECT_GT(before_drop.vx_mps(), 0.0);
  EXPECT_GT(before_drop.omega_radps(), 0.0);
  EXPECT_GT(before_drop.yaw_rad(), 0.0);
  EXPECT_GT(before_drop.module0().distance_m(), 0.0);

  // The bridge goes away: once /hw/state ages out, every device reads
  // disconnected.
  hw.reset();
  drive(160ms);
  const DrivetrainState at_timeout = latest();
  EXPECT_FALSE(at_timeout.modules_valid());
  EXPECT_FALSE(at_timeout.yaw_valid());
  EXPECT_EQ(at_timeout.vx_mps(), 0.0);
  EXPECT_EQ(at_timeout.omega_radps(), 0.0);

  // Still one state per tick, and nothing it holds moves.
  const std::uint64_t count_at_timeout = count();
  drive(60ms);
  EXPECT_EQ(count() - count_at_timeout, std::uint64_t{60ms / 5ms});
  const DrivetrainState after = latest();
  EXPECT_FALSE(after.modules_valid());
  EXPECT_FALSE(after.yaw_valid());
  EXPECT_EQ(after.yaw_rad(), at_timeout.yaw_rad());
  EXPECT_EQ(after.module0().distance_m(), at_timeout.module0().distance_m());
  EXPECT_EQ(after.module0().angle_rad(), at_timeout.module0().angle_rad());
}

// --- allocation ------------------------------------------------------------

TEST_F(DrivetrainTest, UpdateDoesNotAllocate) {
  if (const auto why = talos::testing::AllocationProbe::Unsupported();
      !why.empty()) {
    GTEST_SKIP() << why;
  }
  Teleop();
  SetRotating(hw, geometry, 1.0);
  Hold(ChassisTarget{1.0, -0.5, 0.8}, 40ms);
  ASSERT_EQ(Request("front_left_drive").mode, Mode::kVelocity);
  ASSERT_TRUE(State().modules_valid());
  // Both kinematics directions, the unit boundary, the target read, the
  // motor requests, the flush and the publish, on ticks the loop runs
  // (outputs are taken only inside update()). The target is published before
  // the window and stays fresh through it (60 ms < command_timeout_us), so
  // the test's own publications are not counted.
  talos::testing::PublishTarget(sim, "drivetrain", "teleop",
                                ChassisTarget{1.0, -0.5, 0.8});
  talos::testing::AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(Request("front_left_drive").mode, Mode::kVelocity);
}

// --- binary --------------------------------------------------------------

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
