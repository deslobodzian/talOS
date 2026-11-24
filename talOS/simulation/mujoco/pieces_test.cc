// Component tier, REQ-DEV-43: /sim/telemetry/pieces as the MuJoCo gateway's
// extension builds it, ticked in process as RunSimGateway ticks it, against
// the model it steps. On the 2026 field, with a robot whose scripted shooter
// launches a preload that the game scores and parks, every published message
// carries each FUEL at its body's position in the MuJoCo data (the float of
// xpos, exactly), the field's 504 FUEL in FUEL order with their radius, a
// FUEL out of play exactly when the game holds it (scored and parked, or a
// reserve), and goes out on the first tick and every 10th 5 ms tick after;
// two runs give the same bytes. A free-root robot on the flat carpet is no
// piece, and a field of more than 640 pieces or of two radii is refused.

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
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

constexpr uint64_t kBootId = 0x9ce5;

// A robot welded at the blue HUB, its flywheel's exit inside the HUB's
// scoring cylinder facing down, one preload on a tray above it (as
// gateway_bit_identity_test's): the shooter launches the preload once the
// flywheel turns forward, and the game scores and parks it in that step.
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

// A config in its own directory: `robot` as robot.xml, on `field`.
config::RobotConfig WriteConfig(const std::string& name, std::string_view robot,
                                std::string_view field,
                                std::string_view motors) {
  const std::filesystem::path dir =
      std::filesystem::path(::testing::TempDir()) / name;
  std::filesystem::create_directories(dir);
  std::ofstream(dir / "robot.xml") << robot;
  std::ofstream(dir / "robot.toml")
      << "[robot]\nname = \"" << name
      << "\"\nperiod_us = 5000\ncommand_timeout_us = 100000\n\n"
      << (motors.empty() ? std::string_view{"[subsystems]\n"} : motors)
      << "\n[simulation.mujoco]\nrobot_model = \"robot.xml\"\n"
      << (field.empty() ? "" : "field = \"" + std::string(field) + "\"\n");
  return config::ParseRobotConfig((dir / "robot.toml").string());
}

constexpr std::string_view kFlywheel =
    "[subsystems.shooter.motors.flywheel]\ncan_id = 1\n"
    "supply_limit_a = 60.0\nstator_limit_a = 100.0\n";

// Disabled, then AUTO: the flywheel turns, the shooter launches.
constexpr int kAutoFromTick = 10;
constexpr int kTicks = 200;

// The program's assembly as RunSimGateway ticks it: BeforeTick, the
// gateway's tick, AfterTick. It writes no topic: two assemblies run in one
// process, and RTMS gives a topic one writer.
struct Assembly {
  explicit Assembly(
      const config::RobotConfig& config, const std::string& field = FieldXml(),
      hardware::sim::PhysicsOptions physics = {.start_x_m = kHubX,
                                               .start_y_m = kHubY})
      : options(physics),
        backend(options),
        extension({.field = [field](std::string_view) { return field; },
                   .wall_clock_ns = [] { return int64_t{0}; },
                   .log = &log,
                   .summary = &log,
                   .tests_only_skip_writes = true}) {
    hardware::Config hw = config.hardware;
    hw.commissioned = true;
    InstallMujocoProcessHandlers();
    backend.SetLocalMotorTypes(hw);
    extension.Configure(config, options, backend);
    gateway = std::make_unique<hardware::Gateway>(backend, kBootId);
    EXPECT_TRUE(gateway->Reconfigure(hw));
    has_motor = !hw.motors.empty();
    if (has_motor) flywheel = hw.motors.at(0).id;
  }

  void Tick(int index, uint64_t now_us) {
    ds::DriverStationData station;
    station.fms.flags = index < kAutoFromTick
                            ? ds::kDsAttached
                            : ds::kDsAttached | ds::kEnabled | ds::kAutonomous;
    extension.BeforeTick({.now_us = now_us,
                          .schedule_us = now_us - kStartUs,
                          .driver_station = &station});
    const hardware::State snap = gateway->snapshot();
    hardware::Command command{};
    command.config_id = snap.config_id;
    command.boot_id = snap.boot_id;
    command.epoch = snap.epoch;
    command.observed_time_us = snap.sample_time_us;
    command.count = has_motor ? 1 : 0;
    command.motors[0] = MotorRequest{flywheel, Mode::kVoltage, 0,
                                     index >= kAutoFromTick ? 1.5 : 0.0, 0.0};
    gateway->Accept(command, sequence++, last_us);
    gateway->Tick(now_us, index >= kAutoFromTick);
    extension.AfterTick(now_us);
    last_us = now_us;
  }

  static constexpr uint64_t kStartUs = 1000;
  std::ostringstream log;
  hardware::sim::PhysicsOptions options;
  hardware::sim::PhysicsSimBackend backend;
  MujocoGatewayExtension extension;
  std::unique_ptr<hardware::Gateway> gateway;
  uint16_t flywheel{};
  bool has_motor{false};
  uint64_t sequence = 1;
  uint64_t last_us = 0;
};

using Bytes = std::array<uint8_t, sizeof(::sim::GamePieces)>;

Bytes BytesOf(const ::sim::GamePieces& pieces) {
  Bytes out{};
  std::memcpy(out.data(), &pieces, out.size());
  return out;
}

// REQ-DEV-43
TEST(MujocoGatewayPieces, MessageIsTheModelsBodiesAndTheGamesPlay) {
  const config::RobotConfig config =
      WriteConfig("pieces", kShooterModel, "rebuilt_2026", kFlywheel);
  Assembly a(config);
  // A second run of the same ticks, for identical bytes.
  Assembly b(config);
  const mjModel& model = *a.extension.plant()->model();
  const mjData& data = *a.extension.plant()->data();
  // Each FUEL's body, by FUEL index (fields/rebuilt_2026/README.md).
  std::vector<int> fuel;
  for (int k = 0;; ++k) {
    std::array<char, 16> name{};
    std::snprintf(name.data(), name.size(), "fuel_%03d", k);
    const int body = mj_name2id(&model, mjOBJ_BODY, name.data());
    if (body < 0) break;
    fuel.push_back(body);
  }
  ASSERT_EQ(fuel.size(), 504u);
  EXPECT_EQ(PiecesEveryTicks(5000), 10u);

  uint64_t published_ns = 0;
  int publications = 0;
  bool scored_seen = false;
  std::vector<float> first_x;
  int moved = 0;
  for (int i = 0; i < kTicks; ++i) {
    const uint64_t now_us =
        Assembly::kStartUs + 5000 * static_cast<uint64_t>(i);
    a.Tick(i, now_us);
    b.Tick(i, now_us);
    const ::sim::GamePieces& pieces = a.extension.game_pieces();
    ASSERT_EQ(BytesOf(pieces), BytesOf(b.extension.game_pieces()))
        << "tick " << i;
    // Written on tick 0 and every 10th after; between, it holds the last.
    if (i % 10 == 0) published_ns = now_us * 1000;
    ASSERT_EQ(pieces.sample_time_ns(), published_ns) << "tick " << i;
    if (i % 10 != 0) continue;
    ++publications;
    ASSERT_EQ(pieces.pieces_count(), 504u);
    EXPECT_EQ(
        pieces.radius_m(),
        static_cast<float>(model.geom_rbound[model.body_geomadr[fuel[0]]]));
    uint32_t in_play = 0;
    for (std::size_t k = 0; k < fuel.size(); ++k) {
      const mjtNum* at = data.xpos + static_cast<std::ptrdiff_t>(3 * fuel[k]);
      const auto i_k = static_cast<flatbuffers::uoffset_t>(k);
      ASSERT_EQ(pieces.x_m()->Get(i_k), static_cast<float>(at[0])) << k;
      ASSERT_EQ(pieces.y_m()->Get(i_k), static_cast<float>(at[1])) << k;
      ASSERT_EQ(pieces.z_m()->Get(i_k), static_cast<float>(at[2])) << k;
      // The rules expose no status but InPlay, which is under test, so the
      // oracle is where the game puts a FUEL out of play: off the field's
      // y >= 0, in the reserve row (y -2.0 m) or the parking slots (y -2.5
      // m and beyond, fields/rebuilt_2026/README.md); every FUEL on the
      // field is in play.
      EXPECT_EQ(pieces.in_play()->Get(i_k), at[1] > -1.0) << "FUEL " << k;
      in_play += pieces.in_play()->Get(i_k) ? 1 : 0;
    }
    // The game's own count of FUEL in play, in the same tick.
    EXPECT_EQ(in_play, a.extension.match_state().pieces_in_play_count());
    if (a.extension.match_state().pieces_held_count() > 0) scored_seen = true;
    if (first_x.empty()) {
      for (std::size_t k = 0; k < fuel.size(); ++k) {
        first_x.push_back(
            pieces.x_m()->Get(static_cast<flatbuffers::uoffset_t>(k)));
      }
    }
  }
  for (std::size_t k = 0; k < fuel.size(); ++k) {
    moved += a.extension.game_pieces().x_m()->Get(
                 static_cast<flatbuffers::uoffset_t>(k)) != first_x[k];
  }
  EXPECT_EQ(publications, kTicks / 10);
  EXPECT_TRUE(scored_seen) << "no FUEL was scored: the case needs a held one";
  EXPECT_GT(moved, 0) << "no FUEL moved: the case needs a moving one";
  // The reserve the robot did not preload is out of play throughout.
  EXPECT_FALSE(a.extension.game_pieces().in_play()->Get(503));
}

// REQ-DEV-43: the robot is never a piece, even with a free root; the flat
// carpet has none.
TEST(MujocoGatewayPieces, AFreeRobotIsNoPiece) {
  constexpr std::string_view kBox = R"(<mujoco><worldbody>
  <body name="box" pos="0 0 0.06"><freejoint name="box_root"/>
    <geom type="box" size="0.2 0.2 0.05" mass="10" contype="4" conaffinity="21"/>
  </body></worldbody></mujoco>)";
  Assembly a(WriteConfig("pieces_free", kBox, "", ""), "",
             {.start_x_m = 2.0, .start_y_m = 2.0});
  a.Tick(0, Assembly::kStartUs);
  EXPECT_EQ(a.extension.game_pieces().pieces_count(), 0u);
  EXPECT_EQ(a.extension.game_pieces().sample_time_ns(),
            Assembly::kStartUs * 1000);
}

// A field of `count` free spheres, the first `count - 1` of radius 0.05 m
// and the last of `last_radius_m`.
std::string SphereField(const std::string& name, int count,
                        double last_radius_m) {
  const std::filesystem::path path =
      std::filesystem::path(::testing::TempDir()) / (name + ".xml");
  std::ofstream file(path);
  file << "<mujoco><worldbody>\n";
  for (int i = 0; i < count; ++i) {
    const int row = i / 40;
    file << "<body name=\"ball_" << i << "\" pos=\"" << (i % 40) * 0.2 << ' '
         << row * 0.2 << " 0.1\"><freejoint/><geom type=\"sphere\" size=\""
         << (i + 1 == count ? last_radius_m : 0.05) << "\"/></body>\n";
  }
  file << "</worldbody></mujoco>\n";
  return path.string();
}

constexpr std::string_view kWelded = R"(<mujoco><worldbody>
  <body name="post" pos="0 0 0.5">
    <geom type="box" size="0.1 0.1 0.5" contype="4" conaffinity="21"/>
  </body></worldbody></mujoco>)";

// REQ-DEV-43: Configure refuses a field /sim/telemetry/pieces cannot carry,
// naming the count, or the two bodies of different radii. 640 is accepted.
TEST(MujocoGatewayPieces, RefusesMoreThan640OrTwoRadii) {
  // field = "rebuilt_2026" is the name config load accepts; the resolver
  // hands Configure the test's own model, whose refusal comes before the
  // game's rules would refuse it.
  const config::RobotConfig config =
      WriteConfig("pieces_refused", kWelded, "rebuilt_2026", "");
  try {
    Assembly a(config, SphereField("balls_641", 641, 0.05), {});
    FAIL() << "641 pieces accepted";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("641 game pieces"), std::string::npos)
        << e.what();
  }
  try {
    Assembly a(config, SphereField("balls_two_radii", 3, 0.06), {});
    FAIL() << "two radii accepted";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("ball_0 and ball_2"),
              std::string::npos)
        << e.what();
  }
  // 640 pass the pieces' checks: what refuses them is the 2026 game's
  // rules, which this model does not meet.
  try {
    Assembly a(config, SphereField("balls_640", 640, 0.05), {});
  } catch (const std::invalid_argument& e) {
    EXPECT_EQ(std::string(e.what()).find("game pieces"), std::string::npos)
        << e.what();
  }
}

}  // namespace
}  // namespace talos::simulation::mujoco
