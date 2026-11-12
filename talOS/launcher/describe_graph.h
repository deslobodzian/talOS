#pragma once

// Phase one of a launch: every roster member's --describe probe, the graph
// they declare, its lint (LintGraph plus the parameter checks), the session's
// topic segments, and the declared graph's record on disk.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/launcher/roster.h"
#include "talOS/rtms/rtms.h"

namespace talos::launcher {

// What one binary answered when it was asked what it is.
struct NodeDescription {
  std::string roster_name;  // The `[subsystems.<name>]` key from the config.
  std::string target;
  std::string binary_path;
  bool answered{false};
  // No file at binary_path: not built, or not in the bundle. Such a node is
  // not probed, and DescribeGraph reports every one in a single finding.
  bool missing{false};
  std::string error;  // Why not, when it did not answer.
  introspect::Description declared;
};

// Runs `args` (a binary and --describe) and parses the manifest from its
// stdout, killing it after `timeout`: a node that blocks in its constructor
// costs the launch that much and no more. stderr is left alone so a node that
// throws says so. A binary that cannot be run says why (SpawnProcess).
//
// A spawn refused for want of a process slot or memory
// (IsTransientSpawnFailure) is retried after a backoff that doubles from
// kProbeSpawnRetryFirst to kProbeSpawnRetryMax, within the same `timeout`;
// past it, `error` names the last failure and the attempts made, and a probe
// that spawns after a retry says so on stderr. A probe is asked once per
// launch, so a fork refused while the parallel probes briefly hold every
// process the user may run would otherwise refuse a critical node's launch.
// Every other spawn failure is reported at once.
// `injected_spawn_failures` is a test seam: the first that many attempts fail
// as the guardian fork does under EAGAIN, without spawning.
inline constexpr std::chrono::milliseconds kProbeSpawnRetryFirst{5};
inline constexpr std::chrono::milliseconds kProbeSpawnRetryMax{200};

bool ProbeDescribe(const std::vector<std::string>& args,
                   std::chrono::milliseconds timeout,
                   introspect::Description& out, std::string& error,
                   int injected_spawn_failures = 0);

// Every subsystem's target sources, for the lane checks in LintGraph.
naming::TargetSources TargetSourcesOf(const config::RobotConfig& robot_config);

naming::TargetModeSources TargetModeSourcesOf(
    const config::RobotConfig& robot_config);

// METHODOLOGY §6: every numeric parameter key ends in a unit suffix. One
// warning per subsystem whose [subsystems.<name>] table has numeric keys (a
// number, or an array of numbers) without one, listing them. Nested tables
// (motors, targets) are device and policy config, not parameters.
std::vector<naming::Diagnostic> LintParameterUnits(
    const config::RobotConfig& robot_config);

// Every key of a [subsystems.<name>] table that is neither a framework key
// (config::kFrameworkSubsystemKeys) nor read by any node's constructor
// (introspect::Description::params, from its --describe probe) is an error
// naming the subsystem, the key, the files that set it (when `config_path`
// is given) and the nearest key a node read there. A key is read when a node
// read it, an element of it when it is an array ("names[2]" reads "names"),
// or a key inside it when it is an empty table; a read under a scalar
// ("recording.enabled" where `recording = false`) reads nothing the config
// set. A subsystem whose own node did not answer, or answered without a
// params field (a binary built before it), is not checked. Only the latter
// turns findings in other tables into warnings, since such a binary may read
// them with param_of; a failed probe is already a finding of its own, so a
// key nobody else reads stays an error that names it. Roster members that
// are not subsystems (the bridge, the sim gateway, Studio's bridge) read no
// parameters.
std::vector<naming::Diagnostic> LintParameterKeys(
    const config::RobotConfig& robot_config,
    const std::vector<NodeDescription>& described,
    const std::string& config_path = {});

// Where the launcher writes the declared graph that graph-reading nodes get
// as `--graph` (introspect::DeclaredGraph): beside graph.json in the session
// directory.
std::string DeclaredGraphPath(const LauncherOptions& options);

// Phase one of a launch: ask every binary in the roster what it is, assemble
// the graph they describe, and lint it. The binaries, not the config, are the
// source: a second declaration of the topology is a thing to forget to update.
//
// Two passes. A framework node whose topics follow from the other nodes'
// (telemetry observes every state topic, teleop writes each bound lane with
// its consumer's schema) asks for the declared graph in its constructor and
// says so in its description (reads_graph). Every node is described once
// without a graph; then the descriptions of the nodes that do not read it
// are written to DeclaredGraphPath, and each graph reader is described
// again with `--graph` that file, which it is also started with. A graph
// reader therefore sees every node but the graph readers. Each pass probes
// its binaries in parallel (ForEachInParallel), so a pass takes about as long
// as its slowest probe rather than the sum of them.
// Throws when that file cannot be written: a graph reader is never described,
// or started, without the graph.
//
// A roster member with no file at its binary path is not probed. All such
// members make one finding (MissingBinariesFinding) instead of one "did not
// answer" per node: before a build that is every node, and the cure is one
// command. A binary that exists but does not answer keeps its own finding.
std::vector<naming::Diagnostic> DescribeGraph(
    const std::vector<NodeSpec>& specs, const LauncherOptions& options,
    std::vector<NodeDescription>& out,
    const naming::TargetSources& target_sources = {},
    const naming::TargetModeSources& target_mode_sources = {},
    bool require_target_coverage = false);

// The one finding for roster members whose binary is missing, or nullopt when
// none is. Its subject lists their names. Outside a bundle its message ends
// with the command that builds them, `bazel build` of their labels (members
// named by a path, not a label, are listed but not in the command); in a
// bundle (`bundle_dir` nonempty) it says the bundle lacks them. An error when
// any of them is critical or owns actuators, as a failed probe would be;
// otherwise a warning.
std::optional<naming::Diagnostic> MissingBinariesFinding(
    const std::vector<NodeSpec>& missing, const std::string& bundle_dir);

// How a run's live endpoints differ from what its --describe probe declared,
// or empty when they agree. An endpoint is a topic end: its kind (SENDER,
// WATCHER, ...), topic, message size, alignment and the graph lint's
// attributes (external, optional, observer). Timers are not endpoints, and
// kSourceFlagLatestOnly, which a probe reports, is the loop's delivery
// detail, not an endpoint attribute. The schema is not compared because the
// registry does not carry it: a declared topic's session segment was created
// with its declared schema, so a live end with another one already failed at
// attach, and an undeclared topic differs here by name. Ends are compared as
// multisets. `live_complete` is false when the registry row held only part of
// the node's sources (kMaxSourcesPerNode): only live-only ends are reported
// then.
std::string EndpointMismatch(const std::vector<naming::SourceShape>& declared,
                             const std::vector<naming::SourceShape>& live,
                             bool live_complete = true);

// Distinct topics in a declared graph. Timers are not topics.
std::size_t CountDeclaredTopics(const std::vector<NodeDescription>& described);

// Phase two's first step: the session owns every declared topic. Each
// segment is created session-owned (rtms.h "Segment lifetime") before any
// node starts, replacing whatever the name held, and unlinked when the
// returned queues are destroyed at the end of the session. Nodes only
// attach, so a node that is killed and restarted, or that exits cleanly,
// leaves its topics -- and the subscribers reading them -- in place. A
// topic of a node that did not answer --describe is not declared, so its
// first process creates it as outside a session. Throws, naming the topic,
// if a segment cannot be created.
std::vector<RTMSQueue> CreateSessionSegments(
    const std::vector<NodeDescription>& described);

// The declared graph on disk, beside the session's manifest. uint64 values are
// decimal strings: a JSON number is a double, and an id past 2^53 would change.
std::string GraphToJson(uint64_t session_id, const std::string& config_path,
                        const std::vector<NodeDescription>& described,
                        const std::vector<naming::Diagnostic>& diagnostics,
                        const std::string& config_sha256 = "");

}  // namespace talos::launcher
