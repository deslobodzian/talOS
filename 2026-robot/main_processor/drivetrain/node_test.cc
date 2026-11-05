#include <gtest/gtest.h>
#include <sys/wait.h>

#if defined(__APPLE__)
#include <dlfcn.h>
#include <execinfo.h>
#endif

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

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
  for (const ModuleState& m : *fast) EXPECT_NEAR(m.drive_rps, 20, 1e-12);

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

TEST(Swerve, ForwardKinematicsInvertsInverseKinematics) {
  const SwerveGeometry geometry = Geometry();
  SwerveKinematics kinematics{geometry};
  const auto initial = kinematics.ToModuleStates({1.0, -0.5, 0.8}, {});
  if (!initial.has_value()) {
    FAIL() << "Expected initial module states";
  }
  const auto modules = kinematics.ToModuleStates({1.0, -0.5, 0.8}, *initial);
  if (!modules.has_value()) {
    FAIL() << "ToModuleStates returned nullopt";
  }
  const ChassisSpeeds speeds = kinematics.ToChassisSpeeds(*modules);
  EXPECT_NEAR(speeds.vx_mps, 1.0, 1e-9);
  EXPECT_NEAR(speeds.vy_mps, -0.5, 1e-9);
  EXPECT_NEAR(speeds.omega_radps, 0.8, 1e-9);
}

TEST(Swerve, UnitBoundary) {
  EXPECT_NEAR(ToRadians(0.125).value(), std::numbers::pi / 4, 1e-15);
  EXPECT_NEAR(ToRotations(units::radian_t{std::numbers::pi}), 0.5, 1e-15);
  EXPECT_NEAR(WheelTravel(2.0, 0.05).value(), 2.0 * kTau * 0.05, 1e-15);
  EXPECT_NEAR(WheelSpeed(3.0, 0.05).value(), 3.0 * kTau * 0.05, 1e-15);
  EXPECT_NEAR(WheelRps(WheelSpeed(3.0, 0.05), 0.05), 3.0, 1e-12);
}

TEST(Swerve, CosineScalingSuppressesMisalignedDrive) {
  const auto geometry = Geometry();
  SwerveKinematics kinematics{geometry};
  const double full = WheelRps(units::meters_per_second_t{1},
                               geometry.modules[0].wheel_radius_m);
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
  const auto expected = frc::ChassisSpeeds::Discretize(
      frc::ChassisSpeeds{units::meters_per_second_t{1},
                         units::meters_per_second_t{0.5},
                         units::radians_per_second_t{2}},
      units::second_t{dt});
  EXPECT_NEAR(actual.vx_mps, expected.vx.value(), 1e-12);
  EXPECT_NEAR(actual.vy_mps, expected.vy.value(), 1e-12);
  EXPECT_NEAR(actual.omega_radps, expected.omega.value(), 1e-12);
  EXPECT_GT(std::abs(actual.vy_mps - 0.5), 0.01);
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

TEST_F(DrivetrainTest, NeverSentStaysNeutralButStillReportsState) {
  Teleop();
  sim.step(60ms);
  ExpectNeutral();
  EXPECT_TRUE(State().enabled());
}

TEST_F(DrivetrainTest, StaleLaneGoesNeutral) {
  Teleop();
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  ASSERT_EQ(Request("front_left_drive").mode, Mode::kVelocity);
  const std::uint64_t before = StateCount();
  sim.step(150ms);  // the producer went quiet: the lane is stale
  ExpectNeutral();
  ExpectPublishedEveryTick(before, 150ms);
  EXPECT_TRUE(State().modules_valid());
}

TEST_F(DrivetrainTest, ReleasedLaneGoesNeutralAtOnce) {
  Teleop();
  Hold(ChassisTarget{1.0, 0, 0}, 40ms);
  ASSERT_EQ(Request("front_left_drive").mode, Mode::kVelocity);
  talos::testing::ReleaseTarget<ChassisTarget>(sim, "drivetrain", "teleop");
  sim.step(20ms);
  ExpectNeutral();
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
  // 1 ms link latency while the filter converges, then 4 ms: the minimum
  // stays 1 ms for the window, so each state maps 3 ms before it arrived.
  // A drivetrain stamping its own tick time would read within a tick of now.
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
  EXPECT_GE(lag_ns, std::chrono::nanoseconds{3ms}.count());
  EXPECT_LE(lag_ns, std::chrono::nanoseconds{3ms + 5ms}.count());
}

TEST_F(DrivetrainTest, ADisconnectedMotorStillPublishesButDrivesNothing) {
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
  ExpectNeutral();                       // every module, not just front_left
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

#if defined(__APPLE__)
// Every malloc in the process, including those made inside libwpimath (whose
// operator new binds to libc++, not to this binary). libsystem_malloc calls
// malloc_logger, when set, on every allocation; each one's stack is kept
// (backtrace() does not allocate) and symbolized afterwards.
using MallocLogger = void(std::uint32_t, std::uintptr_t, std::uintptr_t,
                          std::uintptr_t, std::uintptr_t, std::uint32_t);
extern "C" MallocLogger* malloc_logger;

struct Allocation {
  void* frames[48];
  int depth;
};
constexpr std::size_t kMaxRecorded = 64;
Allocation g_recorded[kMaxRecorded];
std::atomic<std::uint64_t> g_allocations{0};

void RecordAllocation(std::uint32_t type, std::uintptr_t, std::uintptr_t,
                      std::uintptr_t, std::uintptr_t, std::uint32_t) {
  constexpr std::uint32_t kAllocate = 2;  // MALLOC_LOG_TYPE_ALLOCATE
  if ((type & kAllocate) == 0) return;
  const std::uint64_t n = g_allocations.fetch_add(1, std::memory_order_relaxed);
  if (n < kMaxRecorded) {
    Allocation& a = g_recorded[n];
    a.depth = ::backtrace(a.frames, static_cast<int>(std::size(a.frames)));
  }
}

// The Sim's in-memory topics keep every message ever published
// (talOS/events/simulated_event_loop.h), so their storage grows
// geometrically under any publisher. That is the test transport's
// allocation, not the node's; the real transport is a preallocated ring.
// SimulatedChannel::publish may be inlined into SimulatedEventLoop::send_impl,
// which does nothing else, so either frame marks the transport.
bool InSimTransport(const Allocation& a) {
  for (int i = 0; i < a.depth; ++i) {
    Dl_info info;
    if (::dladdr(a.frames[i], &info) == 0 || info.dli_sname == nullptr) {
      continue;
    }
    const std::string_view name{info.dli_sname};
    if (name.find("SimulatedChannel") != std::string_view::npos ||
        (name.find("SimulatedEventLoop") != std::string_view::npos &&
         name.find("send_impl") != std::string_view::npos)) {
      return true;
    }
  }
  return false;
}
#endif

TEST_F(DrivetrainTest, UpdateDoesNotAllocate) {
#if defined(__APPLE__)
  Teleop();
  SetRotating(hw, geometry, 1.0);
  Hold(ChassisTarget{1.0, -0.5, 0.8}, 40ms);
  ASSERT_EQ(Request("front_left_drive").mode, Mode::kVelocity);
  ASSERT_TRUE(State().modules_valid());
  // Both kinematics directions, the unit boundary, the target read, the
  // motor requests and the publish, as on every tick the node runs.
  g_allocations = 0;
  malloc_logger = RecordAllocation;
  for (int i = 0; i < 200; ++i) drivetrain.update();
  malloc_logger = nullptr;

  const std::uint64_t total = g_allocations.load();
  ASSERT_LE(total, kMaxRecorded) << "far more than transport growth";
  std::uint64_t outside_transport = 0;
  for (std::uint64_t n = 0; n < total; ++n) {
    if (InSimTransport(g_recorded[n])) continue;
    ++outside_transport;
    ::backtrace_symbols_fd(g_recorded[n].frames, g_recorded[n].depth, 2);
  }
  EXPECT_EQ(outside_transport, 0u);
  EXPECT_EQ(Request("front_left_drive").mode, Mode::kVelocity);
#else
  GTEST_SKIP() << "process-wide malloc counting uses macOS malloc_logger; "
                  "glibc has had no allocation hook since 2.34";
#endif
}

TEST_F(DrivetrainTest, TheAllocationProbeSeesAllocations) {
#if defined(__APPLE__)
  // Guards the test above against a probe that silently counts nothing.
  g_allocations = 0;
  malloc_logger = RecordAllocation;
  auto* volatile probe = new std::array<double, 64>{};
  malloc_logger = nullptr;
  delete probe;
  ASSERT_GE(g_allocations.load(), 1u);
  EXPECT_FALSE(InSimTransport(g_recorded[0]));
#else
  GTEST_SKIP() << "process-wide malloc counting uses macOS malloc_logger";
#endif
}

// --- wire and binary -----------------------------------------------------

// These structs cross every chassis topic on the robot, so a layout change
// is a new contract (whole-bundle rollback), never a silent edit.
TEST(Messages, WireSizesAreFrozen) {
  EXPECT_EQ(sizeof(ChassisTarget), 24u);
  EXPECT_EQ(sizeof(DrivetrainState), 112u);
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
