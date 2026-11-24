// REQ-DEV-37: the same model, config and tick times give bit-identical
// States. Two assemblies of the program's world (MujocoGatewayExtension's
// Configure, so the model compiles twice), each a Gateway over a
// PhysicsSimBackend stepping its own MujocoPlant, are ticked at the same
// jittered sample times with the same commands, a tick past the step cap, a
// device reset and a supply fault; their encoded States agree byte for byte
// every tick. A third, whose ticks differ from the first's by 1 us at one
// tick, must not: the comparison sees the step schedule.
//
// REQ-DEV-39, REQ-DEV-40: the same holds with the 2026 field's rules and a
// robot's scripted shooter attached, for the State and for /sim/state/match,
// over a scripted Driver Station sequence that launches and scores; the
// seed changes the match where the game draws.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
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

using hardware::Mode;
using hardware::MotorRequest;

constexpr uint64_t kBootId = 0x5eed;

// A free-root box on the flat carpet: a bound roller, a rangefinder input,
// a Pigeon2 reading the model's chassis, and a motor no actuator binds.
constexpr std::string_view kModel = R"(<mujoco><compiler angle="radian"/>
<default><geom contype="4" conaffinity="21"/></default>
<worldbody>
  <body name="box" pos="0 0 0.06">
    <freejoint name="box_root" align="false"/>
    <geom name="box_shell" type="box" size="0.2 0.2 0.05" mass="10"/>
    <site name="box_ray" pos="0.15 0 -0.04" zaxis="0 0 -1"/>
    <body name="box_roller" pos="0 0 0.08">
      <joint name="roller_joint" type="hinge" axis="0 1 0" armature="0.0016"/>
      <geom name="roller_tube" type="cylinder" size="0.025 0.15"
            zaxis="0 1 0" mass="0.3" contype="0" conaffinity="0"/>
    </body>
  </body>
</worldbody>
<actuator><motor name="roller" joint="roller_joint"/></actuator>
<sensor><rangefinder name="floor" site="box_ray" cutoff="0.2"/></sensor>
</mujoco>)";

constexpr std::string_view kConfig = R"([robot]
name = "bit_identity"
period_us = 5000
command_timeout_us = 100000

[subsystems.box.motors.roller]
can_id = 1
sensor_to_mechanism_ratio = 4.0

[subsystems.box.motors.spare]
can_id = 2

[subsystems.box.sensors.imu]
type = "Pigeon2"
can_id = 9

[subsystems.box.digital_inputs.floor_seen]
dio = 0
sim_sensor = "floor"

[simulation.mujoco]
robot_model = "model.xml"
)";

config::RobotConfig WriteConfig() {
  const std::filesystem::path dir =
      std::filesystem::path(::testing::TempDir()) / "bit_identity";
  std::filesystem::create_directories(dir);
  std::ofstream(dir / "model.xml") << kModel;
  std::ofstream(dir / "robot.toml") << kConfig;
  return config::ParseRobotConfig((dir / "robot.toml").string());
}

// The program's assembly, as the MuJoCo gateway's main and RunSimGateway make
// it: MuJoCo's process handlers, the local motor types, the extension's
// Configure, then the gateway's Reconfigure, which binds the plant.
struct Assembly {
  explicit Assembly(const config::RobotConfig& config)
      : extension({.field = [](std::string_view) -> std::string {
                     throw std::invalid_argument("no field in this test");
                   },
                   .wall_clock_ns = [] { return int64_t{0}; },
                   .log = &log,
                   .summary = &log,
                   // Two assemblies at once: one writer per topic.
                   .tests_only_skip_writes = true}) {
    hardware::Config hw = config.hardware;
    hw.commissioned = true;
    InstallMujocoProcessHandlers();
    backend.SetLocalMotorTypes(hw);
    extension.Configure(config, options, backend);
    gateway = std::make_unique<hardware::Gateway>(backend, kBootId);
    EXPECT_TRUE(gateway->Reconfigure(hw));
    for (const auto& m : hw.motors) {
      (m.name == "roller" ? roller : spare) = m.id;
    }
  }

  // Sends the tick's commands (built only from the tick's index), then ticks.
  std::vector<uint8_t> Tick(int index, uint64_t now_us) {
    const hardware::State snap = gateway->snapshot();
    hardware::Command command{};
    command.config_id = snap.config_id;
    command.boot_id = snap.boot_id;
    command.epoch = snap.epoch;
    command.observed_time_us = snap.sample_time_us;
    command.count = 2;
    command.motors[0] = MotorRequest{roller, Mode::kVoltage, 0,
                                     8.0 * std::sin(index / 15.0), 0.0};
    command.motors[1] = MotorRequest{spare, Mode::kDutyCycle, 0, 0.3, 0.0};
    gateway->Accept(command, sequence++, last_us);
    gateway->Tick(now_us, true);
    extension.AfterTick(now_us);
    last_us = now_us;
    std::array<uint8_t, 4096> buffer{};
    const std::size_t size = hardware::Encode(gateway->snapshot(), buffer);
    return {buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(size)};
  }

  std::ostringstream log;
  hardware::sim::PhysicsOptions options{.start_x_m = 2.0, .start_y_m = 2.0};
  hardware::sim::PhysicsSimBackend backend{options};
  MujocoGatewayExtension extension;
  std::unique_ptr<hardware::Gateway> gateway;
  uint16_t roller{};
  uint16_t spare{};
  uint64_t sequence = 1;
  uint64_t last_us = 0;
};

constexpr int kTicks = 200;
constexpr int kResetTick = 60;
constexpr int kSupplyFaultTick = 90;
constexpr int kSupplyRestoreTick = 130;
constexpr int kCapTick = 40;    // 40 ms late: past the 2-step cap
constexpr int kProbeTick = 20;  // where the negative control moves 1 us

// Runs `assembly` over the schedule; `shift_us` moves kProbeTick's sample
// time (and only it). Returns every tick's encoded State.
std::vector<std::vector<uint8_t>> RunSchedule(Assembly& a, int64_t shift_us) {
  std::vector<std::vector<uint8_t>> states;
  uint64_t now_us = 1000;
  uint64_t probe_us = 0;
  for (int i = 0; i < kTicks; ++i) {
    // Timer jitter of up to +-200 us around the 5 ms period.
    uint64_t next =
        now_us + 5000 + static_cast<uint64_t>((i * 7919) % 401) - 200;
    if (i == kCapTick) next = now_us + 40000;
    if (i == kProbeTick) {
      // A sample time where the nearest whole step count is about to change
      // (1 us short of the half step), so 1 us more steps once more here
      // and once less at the next tick.
      const int64_t lag = a.backend.plant_time().lag_us;
      next = static_cast<uint64_t>(static_cast<int64_t>(now_us) + 7499 - lag);
      probe_us = next;
      next = static_cast<uint64_t>(static_cast<int64_t>(next) + shift_us);
    }
    if (i == kProbeTick + 1) next = probe_us + 5000;
    if (i == kResetTick) a.backend.SimulateDeviceReset(a.roller);
    if (i == kSupplyFaultTick) a.backend.SetSupplyVoltage(9.0);
    if (i == kSupplyRestoreTick) a.backend.ClearSupplyVoltageOverride();
    states.push_back(a.Tick(i, next));
    now_us = next;
  }
  return states;
}

// REQ-DEV-37
TEST(MujocoGatewayBitIdentity, SameTicksAndCommandsGiveIdenticalStates) {
  const config::RobotConfig config = WriteConfig();
  Assembly first(config);
  Assembly second(config);
  const auto a = RunSchedule(first, 0);
  const auto b = RunSchedule(second, 0);
  // The schedule did what it says: the plant moved, the cap dropped time.
  ASSERT_GT(first.backend.plant_time().dropped_ticks, 0u);
  ASSERT_NE(first.backend.plant_time().stepped_us, 0u);
  for (int i = 0; i < kTicks; ++i) {
    ASSERT_EQ(a[i], b[i]) << "tick " << i;
  }

  Assembly shifted(config);
  const auto c = RunSchedule(shifted, 1);
  int differing = 0;
  for (int i = kProbeTick + 1; i < kTicks; ++i) differing += a[i] != c[i];
  EXPECT_GT(differing, 0)
      << "a 1 us change at tick " << kProbeTick
      << " left every later State identical: the comparison sees nothing";
}

// --- With the 2026 game ---

namespace ds = driver_station;

// A robot welded at the blue HUB, its flywheel's exit inside the HUB's
// scoring cylinder (field: blue_hub_score at z 1.3638 m) facing down, and
// one preload resting on a tray in the inlet, 4 m up, clear of the field.
// The scripted shooter launches the preload once the flywheel turns forward;
// the game scores it in the same step.
constexpr std::string_view kShooterModel = R"(<mujoco><compiler angle="radian"/>
<default><geom contype="4" conaffinity="21"/></default>
<worldbody>
  <body name="shooter_frame">
    <inertial pos="0 0 4" mass="10" diaginertia="1 1 1"/>
    <geom name="shooter_tray" type="box" size="0.15 0.15 0.01" pos="0 0 3.9"/>
    <site name="preload_0" pos="0 0 3.99"/>
    <site name="flywheel_inlet" type="box" size="0.12 0.12 0.12" pos="0 0 4"/>
    <site name="flywheel_exit" pos="0 0 1.3638" xyaxes="0 0 -1 0 1 0"/>
    <body name="shooter_wheel" pos="0.5 0 4">
      <joint name="flywheel_joint" type="hinge" axis="0 0 1" armature="0.0005"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.0025"/>
    </body>
  </body>
</worldbody>
<custom><numeric name="flywheel_shooter" data="0.5 0.0508"/></custom>
<actuator><motor name="flywheel" joint="flywheel_joint"/></actuator>
</mujoco>)";

constexpr double kHubX = 4.625594;  // blue_hub_score
constexpr double kHubY = 4.034536;

std::string FieldXml() {
  std::string error;
  const std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* path = std::getenv("FIELD_XML");
  if (!runfiles || path == nullptr) {
    throw std::runtime_error("runfiles: " + error + ", FIELD_XML unset");
  }
  return runfiles->Rlocation(path);
}

config::RobotConfig GameConfig(uint64_t seed) {
  const std::filesystem::path dir =
      std::filesystem::path(::testing::TempDir()) /
      ("game_" + std::to_string(seed));
  std::filesystem::create_directories(dir);
  std::ofstream(dir / "robot.xml") << kShooterModel;
  std::ofstream(dir / "robot.toml")
      << "[robot]\nname = \"game_identity\"\nperiod_us = 5000\n"
         "command_timeout_us = 100000\n\n"
         // The ceilings' currents (config_wire.h), which the flywheel's
         // commands (at most 2 V: 61 A at stall) never reach, so the
         // voltage, not a limit, sets the motion.
         "[subsystems.shooter.motors.flywheel]\ncan_id = 1\n"
         "supply_limit_a = 60.0\nstator_limit_a = 100.0\n\n"
         "[simulation.mujoco]\nrobot_model = \"robot.xml\"\n"
         "field = \"rebuilt_2026\"\nseed = "
      << seed << "\n";
  return config::ParseRobotConfig((dir / "robot.toml").string());
}

// The Driver Station a tick shows: disabled, then AUTO, then TELEOP.
constexpr int kAutoFromTick = 10;
constexpr int kTeleopFromTick = 110;  // after 0.5 s of AUTO
constexpr int kGameTicks = 400;       // 2 s; a HUB returns its FUEL after 1 s
constexpr int kGameProbeTick = 150;   // in TELEOP, after the launch

uint32_t FlagsAt(int tick) {
  if (tick < kAutoFromTick) return ds::kDsAttached;
  if (tick < kTeleopFromTick) {
    return ds::kDsAttached | ds::kEnabled | ds::kAutonomous;
  }
  return ds::kDsAttached | ds::kEnabled | ds::kTeleop;
}

// One tick's output: the encoded State, the MatchState's bytes, and the
// stamp Stamp wrote for the packet sent on the tick.
struct GameTick {
  std::vector<uint8_t> state;
  std::array<uint8_t, sizeof(::sim::MatchState)> match{};
  FmsStamp stamp;
};

// The program's assembly with the game, ticked as RunSimGateway ticks:
// BeforeTick, the gateway's tick, Stamp, AfterTick.
struct GameAssembly {
  explicit GameAssembly(const config::RobotConfig& config)
      : extension({.field = [](std::string_view) { return FieldXml(); },
                   .wall_clock_ns = [] { return int64_t{0}; },
                   .log = &log,
                   .summary = &log}) {
    hardware::Config hw = config.hardware;
    hw.commissioned = true;
    InstallMujocoProcessHandlers();
    backend.SetLocalMotorTypes(hw);
    extension.Configure(config, options, backend);
    gateway = std::make_unique<hardware::Gateway>(backend, kBootId);
    EXPECT_TRUE(gateway->Reconfigure(hw))
        << (gateway->config_error()
                ? hardware::FormatConfigError(*gateway->config_error())
                : std::string("no config error"));
    flywheel = hw.motors.at(0).id;
  }

  GameTick Tick(int index, uint64_t now_us, uint64_t start_us) {
    ds::DriverStationData station;
    station.fms.flags = FlagsAt(index);
    station.fms.alliance = ds::Alliance::kBlue;
    GameTick out;
    extension.BeforeTick({.now_us = now_us,
                          .schedule_us = now_us - start_us,
                          .driver_station = &station});
    const hardware::State snap = gateway->snapshot();
    hardware::Command command{};
    command.config_id = snap.config_id;
    command.boot_id = snap.boot_id;
    command.epoch = snap.epoch;
    command.observed_time_us = snap.sample_time_us;
    command.count = 1;
    // TELEOP: a varying flywheel command, so how the plant's steps fall
    // into ticks shows in the motion.
    command.motors[0] = MotorRequest{
        flywheel, Mode::kVoltage, 0,
        index >= kTeleopFromTick ? 1.5 + 0.5 * std::sin(index / 15.0) : 0.0,
        0.0};
    gateway->Accept(command, sequence++, last_us);
    gateway->Tick(now_us, (station.fms.flags & ds::kEnabled) != 0 &&
                              (station.fms.flags & ds::kEStop) == 0);
    extension.Stamp(stamp);
    out.stamp = stamp;
    extension.AfterTick(now_us);
    last_us = now_us;
    std::array<uint8_t, 4096> buffer{};
    const std::size_t size = hardware::Encode(gateway->snapshot(), buffer);
    out.state.assign(buffer.begin(),
                     buffer.begin() + static_cast<std::ptrdiff_t>(size));
    std::memcpy(out.match.data(), &extension.match_state(), out.match.size());
    return out;
  }

  std::ostringstream log;
  hardware::sim::PhysicsOptions options{.start_x_m = kHubX, .start_y_m = kHubY};
  hardware::sim::PhysicsSimBackend backend{options};
  MujocoGatewayExtension extension;
  std::unique_ptr<hardware::Gateway> gateway;
  FmsStamp stamp;
  uint16_t flywheel{};
  uint64_t sequence = 1;
  uint64_t last_us = 0;
};

// Jittered ticks as RunSchedule's; `shift_us` moves kGameProbeTick's sample
// time (and only it) across a step boundary.
std::vector<GameTick> RunGame(uint64_t seed, int64_t shift_us) {
  GameAssembly a(GameConfig(seed));
  std::vector<GameTick> ticks;
  constexpr uint64_t kStartUs = 1000;
  uint64_t now_us = kStartUs;
  uint64_t probe_us = 0;
  for (int i = 0; i < kGameTicks; ++i) {
    uint64_t next =
        now_us + 5000 + static_cast<uint64_t>((i * 7919) % 401) - 200;
    if (i == kGameProbeTick) {
      const int64_t lag = a.backend.plant_time().lag_us;
      next = static_cast<uint64_t>(static_cast<int64_t>(now_us) + 7499 - lag);
      probe_us = next;
      next = static_cast<uint64_t>(static_cast<int64_t>(next) + shift_us);
    }
    if (i == kGameProbeTick + 1) next = probe_us + 5000;
    ticks.push_back(a.Tick(i, next, kStartUs));
    now_us = next;
  }
  return ticks;
}

::sim::MatchState MatchOf(const GameTick& tick) {
  ::sim::MatchState match;
  std::memcpy(&match, tick.match.data(), sizeof(match));
  return match;
}

// REQ-DEV-37, REQ-DEV-39, REQ-DEV-40
TEST(MujocoGatewayBitIdentity, GameRunsGiveIdenticalStatesAndMatches) {
  const std::vector<GameTick> a = RunGame(1, 0);
  const std::vector<GameTick> b = RunGame(1, 0);
  for (int i = 0; i < kGameTicks; ++i) {
    ASSERT_EQ(a[i].state, b[i].state) << "tick " << i;
    ASSERT_EQ(a[i].match, b[i].match) << "tick " << i;
  }

  // The run went where it says. Periods in order, as the match input gave
  // them: None, Autonomous, Teleop.
  std::vector<::sim::MatchPeriod> periods;
  for (const GameTick& tick : a) {
    const ::sim::MatchPeriod period = MatchOf(tick).period();
    if (periods.empty() || periods.back() != period) periods.push_back(period);
  }
  EXPECT_EQ(periods,
            (std::vector<::sim::MatchPeriod>{::sim::MatchPeriod::None,
                                             ::sim::MatchPeriod::Autonomous,
                                             ::sim::MatchPeriod::Teleop}));
  // A launch, scored in the step that launched it (the shooter's rule runs
  // before the game's, which judges the FUEL at the exit).
  const ::sim::MatchState last = MatchOf(a.back());
  ASSERT_GT(last.shots_count(), 0u);
  ASSERT_GT(last.blue_scored_count(), 0u);
  for (const GameTick& tick : a) {
    const ::sim::MatchState match = MatchOf(tick);
    if (match.shots_count() == 0) continue;
    EXPECT_EQ(match.blue_scored_count(), match.shots_count())
        << "the first launch's tick";
    break;
  }
  // Each tick's stamp is that tick's match, the ticks a mode changes on
  // included: the period's time left, and from TELEOP the alliance inactive
  // first.
  for (int i = 0; i < kGameTicks; ++i) {
    const ::sim::MatchState match = MatchOf(a[i]);
    const FmsStamp& stamp = a[i].stamp;
    EXPECT_FLOAT_EQ(stamp.match_time_s,
                    static_cast<float>(match.period_remaining_us() * 1e-6))
        << "tick " << i;
    const char letter = match.period() != ::sim::MatchPeriod::Teleop ? '\0'
                        : match.blue_inactive_first()                ? 'B'
                                                                     : 'R';
    EXPECT_EQ(stamp.game_specific_message[0], letter) << "tick " << i;
  }
  EXPECT_FLOAT_EQ(a[kTeleopFromTick].stamp.match_time_s, 140.0F)
      << "the tick TELEOP starts on";

  // The seed decides TELEOP's tie (0-0 after AUTO), the game's first draw:
  // seed 1 draws 0.567 (red inactive first), seed 3 draws 0.113 (blue).
  ASSERT_TRUE(last.tie_drawn());
  const std::vector<GameTick> reseeded = RunGame(3, 0);
  EXPECT_NE(MatchOf(reseeded.back()).blue_inactive_first(),
            last.blue_inactive_first());

  // A 1 us change at one tick changes what follows.
  const std::vector<GameTick> shifted = RunGame(1, 1);
  int differing = 0;
  for (int i = kGameProbeTick + 1; i < kGameTicks; ++i) {
    differing += a[i].state != shifted[i].state;
  }
  EXPECT_GT(differing, 0) << "a 1 us change at tick " << kGameProbeTick
                          << " left every later State identical";
}

// REQ-DEV-40: the tick that first shows TELEOP is TELEOP's in its stamp and
// in /sim/state/match even when its plant time rounds to no step (1 us after
// the tick before): the rules observe the match input when the plant gets
// it, not only in a step.
TEST(MujocoGatewayBitIdentity, ModeChangeOnATickWithNoStepIsReported) {
  GameAssembly a(GameConfig(1));
  constexpr uint64_t kStartUs = 1000;
  uint64_t now_us = kStartUs;
  for (int i = 0; i < kTeleopFromTick; ++i) {
    now_us += 5000;
    a.Tick(i, now_us, kStartUs);
  }
  const uint64_t stepped_us = a.backend.plant_time().stepped_us;
  const GameTick teleop = a.Tick(kTeleopFromTick, now_us + 1, kStartUs);
  ASSERT_EQ(a.backend.plant_time().stepped_us, stepped_us)
      << "the tick ran a plant step; the case needs one that runs none";
  EXPECT_EQ(MatchOf(teleop).period(), ::sim::MatchPeriod::Teleop);
  EXPECT_FLOAT_EQ(teleop.stamp.match_time_s, 140.0F);
  EXPECT_NE(teleop.stamp.game_specific_message[0], '\0');
}

}  // namespace
}  // namespace talos::simulation::mujoco
