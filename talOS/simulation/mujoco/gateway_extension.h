#pragma once

// The GatewayExtension that makes RunSimGateway `mujoco_gateway`
// (README.md): at Configure it assembles the MuJoCo world from the robot
// config's [simulation.mujoco] and attaches it to the physics backend as its
// external plant; after each tick it ends the program when the plant has
// failed and reports time dilation; at the end it prints the plant's clock.
// A library so a component test drives the program's assembly without a
// process.

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
    // writing /sim/state/match; match_state() still holds each tick's
    // message. Topics() declares the topic either way.
    bool tests_only_skip_match_writes{false};
  };

  explicit MujocoGatewayExtension(Options options);
  ~MujocoGatewayExtension() override;
  MujocoGatewayExtension(const MujocoGatewayExtension&) = delete;
  MujocoGatewayExtension& operator=(const MujocoGatewayExtension&) = delete;
  MujocoGatewayExtension(MujocoGatewayExtension&&) = delete;
  MujocoGatewayExtension& operator=(MujocoGatewayExtension&&) = delete;

  // Assembles the world (README.md "mujoco_gateway") and attaches it to
  // `backend`, which binds it at the gateway's Reconfigure. Throws
  // std::invalid_argument naming the key, flag, file, element or actuator it
  // refuses.
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
  // shots_count the scripted shooters' launches.
  void AfterTick(uint64_t now_us) override;
  // /sim/state/match, whatever the config, so --describe's probe and the run
  // declare the same endpoints.
  event::Manifest Topics() const override;
  void Counts(std::span<TopicCount> out) const override;
  void Finish() override;

  // After Configure; it outlives the backend's use of it.
  const MujocoPlant* plant() const { return plant_.get(); }
  // The last /sim/state/match message AfterTick built.
  const ::sim::MatchState& match_state() const { return match_; }

 private:
  void ReportClock(std::string_view why);
  void PublishMatch(uint64_t now_us);

  Options options_;
  // The rules the plant runs, which outlive its steps.
  std::vector<std::unique_ptr<WorldRules>> rules_;
  std::unique_ptr<GameRules> game_;
  const ScriptedShooters* shooters_{nullptr};  // in rules_
  std::unique_ptr<MujocoPlant> plant_;
  std::optional<ipc::Publisher<::sim::MatchState>> publisher_;
  ::sim::MatchState match_{};
  TopicCount match_count_;
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
