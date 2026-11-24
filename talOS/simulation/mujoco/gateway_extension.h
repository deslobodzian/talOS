#pragma once

// The GatewayExtension that makes RunSimGateway the MuJoCo gateway
// (README.md): at Configure it assembles the MuJoCo world from the robot
// config's [simulation.mujoco] and attaches it to the physics backend as its
// external plant; after each tick it ends the program when the plant has
// failed, reports time dilation and publishes /sim/state/match and (every
// PiecesEveryTicks ticks) /sim/telemetry/pieces; at the end it prints the
// plant's clock.
// A library so a component test drives the program's assembly without a
// process.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "sim/sim_generated.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/manifest.h"
#include "talOS/hardware/sim_physics.h"
#include "talOS/ipc/publisher.h"
#include "talOS/simulation/gateway_main.h"
#include "talOS/simulation/mujoco/game.h"
#include "talOS/simulation/mujoco/plant.h"
#include "talOS/simulation/mujoco/scripted_shooter.h"
#include "talOS/simulation/mujoco/world_rules.h"

namespace talos::simulation::mujoco {

// The path of the field model a `[simulation.mujoco] field` names; throws
// std::invalid_argument naming the field when it has none.
using FieldResolver = std::function<std::string(std::string_view field)>;

// The fields this program carries a model for, each with its game's rules.
std::span<const FieldEntry> GatewayFields();

// The most game pieces /sim/telemetry/pieces carries (sim.GamePieces's
// arrays); Configure refuses a field with more.
inline constexpr uint32_t kMaxGamePieces = 640;

// How many gateway ticks of `period_us` apart /sim/telemetry/pieces is
// written: round(50 ms / period), at least 1, so 10 at a 5 ms tick (20 Hz)
// and 16.7 to 25 Hz over the gateway's 1 to 20 ms periods.
constexpr uint32_t PiecesEveryTicks(uint32_t period_us) {
  if (period_us == 0) return 1;
  const uint32_t every = (50'000 + period_us / 2) / period_us;
  return every == 0 ? 1 : every;
}

// The match input a tick shows the world rules: the Driver Station's mode
// (its mode bits only while one is attached; test before autonomous before
// teleop), the gateway's enable (enabled and not e-stopped), the FMS
// alliance, and the schedule clock once it has started.
MatchInput MatchInputOf(const TickInput& tick);

// Finds each field model in the Bazel runfiles of the program started as
// `argv0`: those RUNFILES_DIR or RUNFILES_MANIFEST_FILE name (bazel test, a
// talos_sim, which sets them for every member it launches), else the
// `<argv0>.runfiles` tree beside the binary (bazel-bin, as the launcher
// starts it from a workspace).
FieldResolver RunfilesFieldResolver(std::string argv0);

class MujocoGatewayExtension final : public GatewayExtension {
 public:
  struct Options {
    FieldResolver field;
    // Monotonic nanoseconds. Only the dilation and overspeed reports read it,
    // to print at most one line of each per second; nothing that steps the
    // world does.
    std::function<int64_t()> wall_clock_ns;
    std::ostream* log{nullptr};      // stderr
    std::ostream* summary{nullptr};  // stdout, after the gateway's summary
    // Tests only; a program never sets it. RTMS gives a topic one writer, so
    // a test holding several assemblies in one process lets all but one skip
    // writing /sim/state/match and /sim/telemetry/pieces; match_state() and
    // game_pieces() still hold the messages. Topics() declares both either
    // way.
    bool tests_only_skip_writes{false};
  };

  explicit MujocoGatewayExtension(Options options);
  ~MujocoGatewayExtension() override;
  MujocoGatewayExtension(const MujocoGatewayExtension&) = delete;
  MujocoGatewayExtension& operator=(const MujocoGatewayExtension&) = delete;
  MujocoGatewayExtension(MujocoGatewayExtension&&) = delete;
  MujocoGatewayExtension& operator=(MujocoGatewayExtension&&) = delete;

  // Assembles the world (README.md "The MuJoCo gateway") and attaches it to
  // `backend`, which binds it at the gateway's Reconfigure. The game pieces
  // are the model's free bodies but the robot's root, in body order. Throws
  // std::invalid_argument naming the key, flag, file, element or actuator it
  // refuses, and for a field of more than kMaxGamePieces pieces, or of pieces
  // of different bounding radii, naming the count or the bodies.
  void Configure(const config::RobotConfig& config,
                 const hardware::sim::PhysicsOptions& options,
                 hardware::sim::PhysicsSimBackend& backend) override;
  // Gives the plant this tick's match input (MatchInputOf), which every rule
  // sees during the tick's steps.
  void BeforeTick(const TickInput& tick) override;
  // After the tick's steps: the game's Fms(), so a Driver Station packet
  // carries the match as of the tick it is sent on; no game, the stamp stays
  // zero.
  void Stamp(FmsStamp& fms) override;
  // Prints the binding table on the first tick. Throws std::runtime_error
  // with the plant's failure() once it has failed, after logging the plant's
  // clock. Otherwise writes /sim/state/match: the game's Report() (all zero,
  // period None, without a game), sample_time_ns the tick's node-clock time,
  // shots_count the scripted shooters' launches. On the first tick and every
  // PiecesEveryTicks(period) after it, writes /sim/telemetry/pieces: each
  // piece's body position (xpos, as float) and the game's InPlay, stamped
  // with the tick's node-clock time; pieces_count 0 on a field with none.
  void AfterTick(uint64_t now_us) override;
  // /sim/state/match and /sim/telemetry/pieces, whatever the config, so
  // --describe's probe and the run declare the same endpoints.
  event::Manifest Topics() const override;
  // /sim/telemetry/pieces's ring: hardware::sim::kPiecesSlots.
  uint32_t TopicSlots(std::size_t index) const override;
  void Counts(std::span<TopicCount> out) const override;
  void Finish() override;

  // After Configure; it outlives the backend's use of it.
  const MujocoPlant* plant() const { return plant_.get(); }
  // The last /sim/state/match message AfterTick built.
  const ::sim::MatchState& match_state() const { return match_; }
  // The last /sim/telemetry/pieces message AfterTick built, and the AfterTick
  // calls so far (the first is tick 0).
  const ::sim::GamePieces& game_pieces() const { return pieces_; }
  uint64_t ticks() const { return ticks_; }

 private:
  void ReportClock(std::string_view why);
  void PublishMatch(uint64_t now_us);
  void PublishPieces(uint64_t now_us);

  Options options_;
  // The rules the plant runs, which outlive its steps.
  std::vector<std::unique_ptr<WorldRules>> rules_;
  std::unique_ptr<GameRules> game_;
  const ScriptedShooters* shooters_{nullptr};  // in rules_
  std::unique_ptr<MujocoPlant> plant_;
  std::optional<ipc::Publisher<::sim::MatchState>> publisher_;
  ::sim::MatchState match_{};
  TopicCount match_count_;
  std::vector<int> piece_bodies_;  // the pieces' body ids, in piece order
  std::optional<ipc::Publisher<::sim::GamePieces>> pieces_publisher_;
  ::sim::GamePieces pieces_{};
  TopicCount pieces_count_;
  uint32_t pieces_every_{1};
  uint64_t ticks_{0};
  const hardware::sim::PhysicsSimBackend* backend_{nullptr};
  uint64_t reported_dropped_us_{0};
  uint64_t reported_excursions_{0};
  int64_t dilation_logged_ns_{0};
  int64_t overspeed_logged_ns_{0};
  bool dilation_logged_{false};
  bool overspeed_logged_{false};
  bool table_printed_{false};
};

}  // namespace talos::simulation::mujoco
