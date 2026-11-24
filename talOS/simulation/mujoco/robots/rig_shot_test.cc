// The MuJoCo rig example's shot (examples/mujoco_rig) without processes:
// its model (testdata/mujoco_rig, the example's own, which the example's
// model_twin_test holds byte for byte equal) on the 2026 field, assembled as
// the MuJoCo gateway assembles it (MujocoGatewayExtension) and ticked as
// RunSimGateway ticks it, at the example's start pose, with the commands the
// example's nodes send for its joystick schedule: disabled, then TELEOP; the
// flywheel at the shot's speed from 2 s, the feeder from 4 s.

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "sim/sim_generated.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/config_error.h"
#include "talOS/hardware/gateway.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/sim_physics.h"
#include "talOS/simulation/mujoco/gateway_extension.h"
#include "talOS/simulation/mujoco/plant.h"

namespace talos::simulation::mujoco {
namespace {

namespace ds = driver_station;
using hardware::Mode;
using hardware::MotorRequest;

// The example's numbers (examples/mujoco_rig/README.md "The shot"): its
// start pose, its teleop bindings' speeds, its schedule.
constexpr hardware::sim::PhysicsOptions kStartPose{
    .start_x_m = 1.907, .start_y_m = 5.302, .start_yaw_rad = 2.705};
constexpr double kFlywheelRps = 41.0;
constexpr double kFeederRps = 30.0;
constexpr uint64_t kTickUs = 5000;
constexpr int kTeleopFromTick = 200;  // disabled for 1 s
constexpr int kSpinFromTick = 400;    // button 3 from 2 s
constexpr int kFeedFromTick = 800;    // buttons 3 and 4 from 4 s
constexpr int kTicks = 1600;          // 8 s
constexpr unsigned kPreloads = 8;

std::string Runfile(const char* env) {
  std::string error;
  const std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* path = std::getenv(env);
  if (!runfiles || path == nullptr) {
    throw std::runtime_error("runfiles: " + error + ", " + env + " unset");
  }
  return runfiles->Rlocation(path);
}

// The example's simulated config as the gateway reads it: the two motors
// with robot_sim.toml's simulated gains, the field and its seed.
config::RobotConfig RigConfig(const std::string& field) {
  const std::filesystem::path dir =
      std::filesystem::path(::testing::TempDir()) / "rig_shot";
  std::filesystem::create_directories(dir);
  std::filesystem::copy_file(Runfile("RIG_XML"), dir / "robot.xml",
                             std::filesystem::copy_options::overwrite_existing);
  std::ofstream(dir / "robot.toml")
      << "[robot]\nname = \"rig_shot\"\nperiod_us = 5000\n"
         "command_timeout_us = 100000\n\n"
         "[subsystems.flywheel.motors.flywheel_motor]\ncan_id = 1\n"
         "supply_limit_a = 40.0\nstator_limit_a = 80.0\n"
         "brake = false\nslot0 = { p = 1.0, v = 0.12 }\n\n"
         "[subsystems.feeder.motors.feeder_motor]\ncan_id = 2\n"
         "supply_limit_a = 40.0\nstator_limit_a = 80.0\n"
         "slot0 = { v = 0.12 }\n\n"
         "[simulation.mujoco]\nrobot_model = \"robot.xml\"\nfield = \""
      << field << "\"\nseed = 1\n";
  return config::ParseRobotConfig((dir / "robot.toml").string());
}

// The program's assembly (MujocoGatewayExtension, then the gateway's
// Reconfigure), ticked as RunSimGateway ticks: BeforeTick, the gateway's
// tick, Stamp, AfterTick.
struct Rig {
  explicit Rig(const config::RobotConfig& config)
      : extension(
            {.field = [](std::string_view) { return Runfile("FIELD_XML"); },
             .wall_clock_ns = [] { return int64_t{0}; },
             .log = &log,
             .summary = &log,
             // Two rigs in one process: one writer per topic.
             .tests_only_skip_writes = true}) {
    hardware::Config hw = config.hardware;
    hw.commissioned = true;
    InstallMujocoProcessHandlers();
    backend.SetLocalMotorTypes(hw);
    extension.Configure(config, kStartPose, backend);
    gateway = std::make_unique<hardware::Gateway>(backend, 0x5eed);
    EXPECT_TRUE(gateway->Reconfigure(hw))
        << (gateway->config_error()
                ? hardware::FormatConfigError(*gateway->config_error())
                : std::string("no config error"));
    for (const auto& motor : hw.motors) {
      (motor.name == "flywheel_motor" ? flywheel : feeder) = motor.id;
    }
  }

  // One tick at `index`; returns the tick's /sim/state/match bytes.
  std::array<uint8_t, sizeof(::sim::MatchState)> Tick(int index) {
    const uint64_t now_us = 1000 + kTickUs * static_cast<uint64_t>(index + 1);
    ds::DriverStationData station;
    station.fms.flags = index < kTeleopFromTick
                            ? ds::kDsAttached
                            : ds::kDsAttached | ds::kEnabled | ds::kTeleop;
    station.fms.alliance = ds::Alliance::kBlue;
    extension.BeforeTick({.now_us = now_us,
                          .schedule_us = now_us - 1000,
                          .driver_station = &station});
    const hardware::State snap = gateway->snapshot();
    hardware::Command command{};
    command.config_id = snap.config_id;
    command.boot_id = snap.boot_id;
    command.epoch = snap.epoch;
    command.observed_time_us = snap.sample_time_us;
    // In id order, as the bridge merges them.
    const MotorRequest requests[] = {
        MotorRequest{flywheel,
                     index >= kSpinFromTick ? Mode::kVelocity : Mode::kNeutral,
                     0, kFlywheelRps, 0.0},
        MotorRequest{feeder,
                     index >= kFeedFromTick ? Mode::kVelocity : Mode::kNeutral,
                     0, kFeederRps, 0.0}};
    const bool flywheel_first = flywheel < feeder;
    command.count = 2;
    command.motors[0] = requests[flywheel_first ? 0 : 1];
    command.motors[1] = requests[flywheel_first ? 1 : 0];
    gateway->Accept(command, sequence++, last_us);
    gateway->Tick(now_us, (station.fms.flags & ds::kEnabled) != 0);
    FmsStamp stamp;
    extension.Stamp(stamp);
    extension.AfterTick(now_us);
    last_us = now_us;
    std::array<uint8_t, sizeof(::sim::MatchState)> match{};
    std::memcpy(match.data(), &extension.match_state(), match.size());
    return match;
  }

  std::ostringstream log;
  hardware::sim::PhysicsSimBackend backend{kStartPose};
  MujocoGatewayExtension extension;
  std::unique_ptr<hardware::Gateway> gateway;
  uint16_t flywheel{};
  uint16_t feeder{};
  uint64_t sequence = 1;
  uint64_t last_us = 0;
};

::sim::MatchState MatchOf(
    const std::array<uint8_t, sizeof(::sim::MatchState)>& bytes) {
  ::sim::MatchState match;
  std::memcpy(&match, bytes.data(), sizeof(match));
  return match;
}

// REQ-DEV-37, REQ-DEV-39, REQ-DEV-40, REQ-DEV-41: the rig's generated model
// on the 2026 field launches all eight preloads the game placed in its
// chute, and at least seven score in the blue HUB in TELEOP; two runs give
// the same /sim/state/match every tick.
TEST(RigShot, ScoresItsPreloadsTheSameEveryRun) {
  const config::RobotConfig config = RigConfig("rebuilt_2026");
  Rig first(config);
  Rig second(config);
  ::sim::MatchState last{};
  for (int i = 0; i < kTicks; ++i) {
    const auto a = first.Tick(i);
    ASSERT_EQ(a, second.Tick(i)) << "tick " << i;
    last = MatchOf(a);
  }
  EXPECT_EQ(last.period(), ::sim::MatchPeriod::Teleop);
  EXPECT_EQ(last.shots_count(), kPreloads);
  EXPECT_GE(last.blue_points_count(), kPreloads - 1)
      << "scored " << last.blue_scored_count() << " of " << last.shots_count()
      << " launched";
}

}  // namespace
}  // namespace talos::simulation::mujoco
