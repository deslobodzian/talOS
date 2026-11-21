#pragma once

// What a launch runs: its options, the bundle it may come from, and the
// roster of members its robot config names, each resolved to a binary.

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/names.h"
#include "talOS/introspection/registry.h"
#include "talOS/launcher/session.h"
#include "talOS/launcher/session_lock.h"
#include "talOS/launcher/session_retention.h"
#include "talOS/launcher/supervision_policy.h"
#include "talOS/process/durable_sync.h"

namespace talos::launcher {

// The launcher works in the vocabulary of the naming protocol, not its own.
namespace naming = introspect::naming;

struct NodeSpec {
  std::string name;
  std::string target;  // e.g. "//bench/main_processor/swerve_module:node"
  std::string binary_path;
  // Forwarded unchanged to both --describe and runtime: the hardware
  // bridge's ControllerArgs, the sim gateway's SimulatorArgs.
  std::vector<std::string> extra_args;
  bool critical{true};
  bool owns_actuators{false};
  // The launcher's own Studio bridge, which DiscoverNodes adds under
  // --start-studio-bridge: an observer (StudioBridge), never a subsystem.
  // Set nowhere else, so a robot's subsystem named kStudioBridgeName is an
  // ordinary node.
  bool studio_bridge{false};
  // The launcher's own simulation gateway, which DiscoverNodes adds under
  // --sim --start-sim-gateway: started with --config and no event log, and
  // never replayed. Set nowhere else, so a robot's subsystem named
  // kSimGatewayName is an ordinary node.
  bool sim_gateway{false};
};

struct LauncherOptions {
  // No default: the framework names no robot, so the caller must say which
  // config to launch. Empty means missing.
  std::string config_path;
  std::string output_dir;
  uint64_t session_id{0};
  bool simulation{false};
  int duration_s{0};
  bool start_sim_gateway{false};
  std::vector<std::string> sim_gateway_args;
  // Run the Studio viewer bridge as a supervised member of the session, with
  // or without --sim (see "Studio bridge" in studio_bridge.h and [studio] in
  // the robot config).
  bool start_studio_bridge{false};

  // On a phase-1 refusal (when not --describe-only), if start_studio_bridge is
  // also set, keep the Studio bridge running in refusal mode so /drive shows
  // LAUNCH REFUSED with the reason instead of BRIDGE LINK, until a stop signal
  // (or duration_s > 0) arrives or the bridge exits, then still return 1.
  bool serve_refusal{false};

  // Controller addressing. Empty/zero means "take it from [robot]".
  std::string remote;
  std::string local_address;
  uint16_t remote_port{0};
  uint16_t local_port{0};

  // Studio HTTP port override. Unset means "take [studio] http_port"; 0 asks
  // the OS for an ephemeral port.
  std::optional<uint16_t> studio_http_port;

  // Restart budget per node in restart_window_ms; unset takes
  // robot.max_restarts.
  std::optional<int> max_restarts;
  int restart_window_ms{60000};
  int heartbeat_periods{10};
  int heartbeat_startup_grace_ms{5000};
  // Internal supervision seam; production uses the fixed registry deadline.
  std::chrono::milliseconds construction_timeout{
      introspect::kConstructionTimeout};
  // Internal supervision seam: the next N spawns of the named roster member
  // fail as fork() does under EAGAIN. Production leaves it empty.
  std::map<std::string, int> spawn_failures;
  // Internal describe seam: the first N spawn attempts of each --describe
  // probe of the named roster member fail as the guardian fork does under
  // EAGAIN (ProbeDescribe). Production leaves it empty.
  std::map<std::string, int> probe_spawn_failures;
  // Internal supervision seam: every time the session's supervision reads
  // (each pass, a run's spawn, the session deadline); steady_clock in
  // production.
  SupervisionClock supervision_clock{
      [] { return std::chrono::steady_clock::now(); }};
  // Internal supervision seam: syncs an open session file or directory to
  // disk; process::DurableSync in production. The supervision loop never
  // calls it: the manifest writer's thread syncs the versions the loop
  // hands it (PersistManifest).
  SyncFunction sync_file{process::DurableSync};
  // Internal retention seam: the bytes still free on the filesystem holding
  // the log root, or nullopt when unknown; AvailableBytes in production.
  std::function<std::optional<uint64_t>(const std::filesystem::path&)>
      available_bytes{AvailableBytes};
  // Internal session seam: SessionLock::Acquire.
  std::function<std::optional<SessionLock>(std::string& error, bool* no_room)>
      acquire_session_lock{SessionLock::Acquire};
  // Internal session seam: the flock(2) every SessionDirectoryLock of the
  // launch (its own, retention's) is taken with; ::flock in production.
  FlockFunction lock_session_directory{::flock};
  // Internal retention seam: creates the session directory and its parents,
  // setting the error when it cannot; std::filesystem::create_directories in
  // production.
  std::function<void(const std::filesystem::path&, std::error_code&)>
      create_session_directory{
          [](const std::filesystem::path& path, std::error_code& error) {
            std::filesystem::create_directories(path, error);
          }};
  // Internal retention seam: removes a moved session or leftover directory
  // tree, setting the error when it cannot; std::filesystem::remove_all in
  // production.
  std::function<std::uintmax_t(const std::filesystem::path&, std::error_code&)>
      remove_all{[](const std::filesystem::path& path, std::error_code& error) {
        return std::filesystem::remove_all(path, error);
      }};
  // Internal start-up seam: called on the launcher's thread once the
  // session's topic segments exist, before the first manifest and any spawn.
  // Empty in production.
  std::function<void()> session_segments_created;
  // Internal start-up seam: called on the launcher's thread with a roster
  // member's name right after its first run was spawned, before the next
  // member's. Empty in production.
  std::function<void(const std::string& member)> member_spawned;

  // Launch a graph the linter called broken anyway. The default is refusal:
  // two spellings of one topic are silent at runtime. Only with `simulation`:
  // Launcher::Run refuses it otherwise, before anything is probed.
  bool allow_graph_errors{false};

  // Probe and lint without spawning anything, as a pre-flight check.
  bool describe_only{false};
  // With `describe_only`: also make the refusals a launch with the same
  // options makes before it probes, those that need no lock, shared memory,
  // socket or process. Launcher::RunSession makes them, and refuses this
  // without `describe_only`.
  bool describe_as_launch{false};

  // Input replay: run only `only`, fed the topics the session in this
  // directory logged (talOS/events/input_replay.h), and nothing else.
  std::string replay_inputs;
  std::string only;
  // With `replay_inputs`: allow replaying logs capped by [robot]
  // log_max_bytes (`<log>.capped` or manifest `log_capped`) from the start of
  // their kept window instead of refusing them (REQ-OPS-14). Refused without
  // `replay_inputs`.
  bool allow_capped_replay{false};

  // How long one binary gets to answer --describe. A Python node importing
  // native wheels (robotpy-wpimath) needs several seconds on its first run
  // from a freshly copied bundle, while macOS checks every new library.
  int describe_timeout_ms{15000};

  // Bundle directory assembled by talos_bundle.
  std::string bundle_dir;

  // The config identity hash from the bundle manifest, carried into the
  // session manifest.
  std::string bundle_config_sha256;
  // The bundle's source revision and dirty flag (bundle_manifest.toml
  // git_revision, git_dirty), present when it was built with --stamp.
  std::string bundle_git_revision;
  bool bundle_git_dirty{false};
  // The bundle's Studio (bundle_manifest.toml [studio], written when its
  // talos_bundle sets studio_bridge and studio_webroot): whether it carries
  // one, and the absolute path of its webroot.
  bool bundle_has_studio{false};
  std::string bundle_studio_webroot;

  // Target or node name -> binary path. Filled from the bundle manifest, or
  // directly by tests.
  std::map<std::string, std::string> binary_overrides;

  // Main-repository apparent -> canonical repo name map, parsed from Bazel's
  // `_repo_mapping`. When unset, MakeNodeSpec loads it from runfiles.
  std::optional<std::map<std::string, std::string>> repo_mapping;
};

// ---------------------------------------------------------------------------
// Repository mapping
// ---------------------------------------------------------------------------

// Main-repository apparent repo name -> canonical repo name (from entries of
// Bazel's `_repo_mapping` whose source canonical name is empty).
using RepoMapping = std::map<std::string, std::string>;

// Parses Bazel's 3-column CSV `_repo_mapping` content
// (`source_canonical,target_apparent,target_canonical`), returning the
// `target_apparent -> target_canonical` map for `source_canonical == ""`.
RepoMapping ParseRepoMapping(std::string_view content);

// Reads Bazel's `_repo_mapping` from `<argv0>.runfiles/_repo_mapping` (or
// `<argv0>.repo_mapping`) when `argv0` is non-empty, else from
// `RUNFILES_DIR/_repo_mapping` when `RUNFILES_DIR` is set, else beside the
// running executable (`<exe>.runfiles/_repo_mapping` / `<exe>.repo_mapping`).
// Returns an empty map when no mapping file exists.
RepoMapping LoadRepoMapping(std::string_view argv0 = {});

// ---------------------------------------------------------------------------
// Bundles
// ---------------------------------------------------------------------------

inline constexpr std::string_view kBundleManifest = "bundle_manifest.toml";

// Reads `bundle_manifest.toml` (written by talOS/build/bundle_builder.py) from
// options.bundle_dir, or from the working directory when that holds one. A
// [studio] table registers the bundled Studio bridge under its labels and
// kStudioBridgeName, and its webroot.
bool LoadBundle(LauncherOptions& options, std::string& error);

// ---------------------------------------------------------------------------
// Roster
// ---------------------------------------------------------------------------

inline constexpr std::string_view kHardwareNodeName =
    naming::kHardwareBridgeNode;
inline constexpr std::string_view kSimGatewayName = "sim_gateway";
inline constexpr std::string_view kStudioBridgeName = "studio_bridge";
// File in the session directory where the launcher writes the node-clock
// timestamp (decimal nanoseconds) that starts sim_gateway's schedules once
// every non-observer member's loop has started (PrepareLaunch, Supervise).
inline constexpr std::string_view kSimScheduleStartFile = "sim_schedule_start";

// Resolves a node target to an executable path: an override (bundle manifest
// or test) wins, then the target's bazel-bin output relative to the
// workspace. Anything that is not a `//pkg:target` or `@repo//pkg:target`
// label is taken as a path.
//
// One node has two label spellings: apparent `@repo//pkg:rule` (as a robot
// config spells it) and canonical `@@canon//pkg:rule` (as a baked
// TALOS_NODE_TARGET and the bundle manifest spell it). When Bazel's
// `_repo_mapping` covers the repo, `@apparent` resolves through
// `apparent -> canonical` so custom `repo_name != module_name` and both
// Bazel 8 (`+`) and Bazel 7 (`~`) canonical names compare equal. When no
// mapping covers the repo, fallback strips a `+`/`~` canonical suffix so
// `@wrist`, `@@wrist+`, `@@wrist~` and `@@wrist` still compare equal, while
// `//pkg:rule`, paths and node names compare literally.
std::string NormalizeTargetKey(std::string_view target);
std::string NormalizeTargetKey(std::string_view target,
                               const RepoMapping& repo_mapping);

std::string ResolveBinary(
    const std::string& target_or_path,
    const std::map<std::string, std::string>& overrides = {});
std::string ResolveBinary(const std::string& target_or_path,
                          const std::map<std::string, std::string>& overrides,
                          const RepoMapping& repo_mapping);

// ResolveBinary, falling back to an override keyed by the node's name when
// the target does not resolve to an executable. `studio_bridge` marks the
// launcher's own Studio bridge (NodeSpec::studio_bridge); only it falls
// back to the override named kStudioBridgeName, the bundled bridge
// (LoadBundle), which a subsystem of that name never runs.
NodeSpec MakeNodeSpec(std::string_view name, const std::string& target,
                      const LauncherOptions& options,
                      bool studio_bridge = false);

// The controller addressing the hardware bridge is described and started
// with: `--remote`, `--remote-port`, `--local-port` and, when non-empty,
// `--local-address`, each the effective value (a flag wins over the config).
// DiscoverNodes puts them in the bridge's NodeSpec::extra_args, so its
// --describe probe and every run get the same ones: hardware_node refuses
// --sim without the loopback remote, and a probe that fell back to the
// config's address would refuse `--sim --remote 127.0.0.1` on a config naming
// the RoboRIO. Throws as config::ResolveControllerAddress /
// config::ResolveLocalAddress do on an invalid --remote or --local-address.
std::vector<std::string> ControllerArgs(const config::RobotConfig& config,
                                        const LauncherOptions& options);

// The simulator's arguments: `--port` (the bridge's effective
// `--remote-port`, which the simulator listens on) and `--remote-port` (the
// bridge's effective `--local-port`, which it sends to), then the
// --sim-gateway-arg values. The simulator stands in for the controller the
// bridge addresses, so the two always agree on the ports (REQ-DEV-31).
std::vector<std::string> SimulatorArgs(const config::RobotConfig& config,
                                       const LauncherOptions& options);

// A --sim-gateway-arg value Launcher::Run refuses: a port flag of the
// simulator's, which SimulatorArgs already passes.
bool IsSimulatorPortArg(std::string_view arg);

// Whether `args` (--sim-gateway-arg values) already passes
// --schedule-start-file: when a caller supplies its own, the launcher leaves
// schedule coordination to that caller instead of passing and writing
// kSimScheduleStartFile.
bool HasSimulatorScheduleStartArg(const std::vector<std::string>& args);

// The roster: the hardware bridge (with ControllerArgs), the subsystems, the
// sim gateway under --sim-gateway (with SimulatorArgs), and the Studio bridge
// (NodeSpec::studio_bridge) under --start-studio-bridge unless
// BundleWithoutStudio(options). A subsystem may share a name with a member
// the launcher adds; Launcher::Run refuses such a roster.
std::vector<NodeSpec> DiscoverNodes(const config::RobotConfig& robot_config,
                                    const LauncherOptions& options);

// Launched from a bundle built without Studio (its talos_bundle sets no
// studio_bridge and studio_webroot): there is no bridge to start or probe.
bool BundleWithoutStudio(const LauncherOptions& options);

// Effective endpoints for a launch: a CLI flag wins, otherwise the config.
std::string EffectiveRemoteAddress(const config::RobotConfig& config,
                                   const LauncherOptions& options);

// Effective local interface bind address: CLI `--local-address` over `[robot]
// local_address`, or empty when neither is set (wildcard `0.0.0.0`).
std::string EffectiveLocalAddress(const config::RobotConfig& config,
                                  const LauncherOptions& options);

uint16_t EffectiveRemotePort(const config::RobotConfig& config,
                             const LauncherOptions& options);

uint16_t EffectiveLocalPort(const config::RobotConfig& config,
                            const LauncherOptions& options);

uint16_t EffectiveStudioHttpPort(const config::RobotConfig& config,
                                 const LauncherOptions& options);

// The directory the Studio bridge serves: `[studio] webroot` when it is an
// absolute path or the bundle carries no Studio; otherwise the bundle's
// webroot, since a relative webroot names a checkout's build output.
std::string EffectiveStudioWebroot(const config::RobotConfig& config,
                                   const LauncherOptions& options);

}  // namespace talos::launcher
