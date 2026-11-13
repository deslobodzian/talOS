#pragma once

// The naming protocol, as code. talOS/NAMING.md is the prose; this header is
// the authority.
//
// Topics are shared-memory objects addressed by exact string, so two spellings
// of one idea (`/hw/req/drive` vs `/hw/req/drivetrain`) produce a publisher and
// a subscriber that both work and never meet. The launcher runs LintGraph over
// every node's `--describe` before it spawns anything; registry.h runs the
// per-name checks when a node publishes itself.

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/hardware/config.h"
#include "talOS/hardware/packet.h"
#include "talOS/rtms/rtms.h"

namespace talos::introspect::naming {

// A topic name is `/owner/role[/qualifier[/qualifier]]`.
inline constexpr std::size_t kMinSegments = 2;
inline constexpr std::size_t kMaxSegments = 4;

// The transport's limit: macOS caps a shm name at PSHMNAMLEN, which
// `rtms::ValidatePath` enforces as 30 characters after the slash. When a name
// does not fit, shorten the qualifier, not the owner or the role.
inline constexpr std::size_t kMaxTopicBytes = 31;
static_assert(kMaxTopicBytes <= event::MAX_SOURCE_NAME,
              "a name the transport accepts must also fit a registry record");

inline constexpr std::size_t kMaxSegmentBytes = 24;

// The second segment, from a closed set (see NAMING.md).
//
// The owner segment names the subject. `state`-like roles flow out of it, so
// the owner must publish them; `target`-like roles flow into it, so the owner
// must not.
inline constexpr std::string_view kOwnerPublishedRoles[] = {
    "state", "status", "event", "telemetry"};
inline constexpr std::string_view kOwnerConsumedRoles[] = {"target", "request",
                                                           "command"};

// Owners that are not nodes: the hardware bridge (`hw`), framework feeds
// (`talos`), simulation, replay, and tests (refused in a session).
inline constexpr std::string_view kReservedOwners[] = {"hw", "talos", "sim",
                                                       "replay", "test"};

struct Abbreviation {
  std::string_view wrong;
  std::string_view right;
};
inline constexpr Abbreviation kAbbreviations[] = {
    {"tgt", "target"},        {"cmd", "command"},      {"req", "request"},
    {"ds", "driver_station"}, {"st", "state"},         {"stat", "status"},
    {"pos", "position"},      {"vel", "velocity"},     {"acc", "acceleration"},
    {"dt", "drivetrain"},     {"drive", "drivetrain"}, {"tlm", "telemetry"},
    {"odom", "odometry"},     {"msg", "message"},      {"pkt", "packet"},
    {"cfg", "config"},        {"idx", "index"},        {"cnt", "count"},
};

// Direction belongs to an endpoint, not to a topic.
inline constexpr std::string_view kDirectionSuffixes[] = {
    "_in", "_out", "_pub", "_sub", "_tx", "_rx", "_topic", "_send", "_recv"};

// How a source's far end is expected to behave (SourceRecord::flags), so the
// lint can tell a designed dead end from a fault: `/hw/command` is consumed
// over UDP (external); `/drivetrain/target/auto` has no publisher yet
// (optional). An observer is a framework node reading a topic it found in the
// declared graph (telemetry): it is not a consumer, so a topic only an
// observer reads is still "published but nothing subscribes to it".
inline constexpr std::uint32_t kSourceFlagExternal = 1u << 0;
inline constexpr std::uint32_t kSourceFlagOptional = 1u << 1;
inline constexpr std::uint32_t kSourceFlagObserver = 1u << 2;
// Not about the far end: a watcher that delivers only the newest message
// (event::EventLoopBase::latest_only), whose dropped count therefore includes
// the messages it skipped by design, and whose log records at most one
// message per drain. Set from the loop (introspect::LoopSourceFlags) by the
// Reporter and by `--describe` (`"latest_only": true`), never declared by a
// node.
inline constexpr std::uint32_t kSourceFlagLatestOnly = 1u << 3;
// Not about the far end either: the publisher the devices layer's flush writes
// its node's hardware request on (detail::NodeAccess::add_hardware_request),
// set by the node from that registration, never declared by node code. The
// graph lint refuses a `/hw/request` writer without it (RULES.md 2). A bit of
// the existing flags word: an older registry reader ignores it, and an older
// `--describe`, which never wrote it, reads as "not the flush".
inline constexpr std::uint32_t kSourceFlagHardwareRequest = 1u << 4;

// The hardware bridge's roster name: the launcher lists the bridge under it
// (launcher kHardwareNodeName), whatever binary `[robot] hardware_node` names,
// and the stock bridge describes itself by it (talOS/bridge kNodeName; both
// are held to this one). The graph lint knows the bridge by its roster slot
// (NodeShape::hardware_bridge), not by the name a binary describes.
inline constexpr std::string_view kHardwareBridgeNode = "hardware_node";

inline bool IsSender(event::SourceKind kind) {
  return kind == event::SourceKind::SENDER;
}
inline bool IsReader(event::SourceKind kind) {
  return kind == event::SourceKind::WATCHER ||
         kind == event::SourceKind::FETCHER;
}

namespace detail {

template <std::size_t N>
bool OneOf(const std::string_view (&set)[N], std::string_view value) {
  return std::find(std::begin(set), std::end(set), value) != std::end(set);
}

inline std::vector<std::string_view> Segments(std::string_view topic) {
  std::vector<std::string_view> out;
  if (topic.empty() || topic.front() != '/') return out;
  std::size_t at = 1;
  for (;;) {
    const std::size_t end = topic.find('/', at);
    out.push_back(topic.substr(at, end - at));
    if (end == std::string_view::npos) return out;
    at = end + 1;
  }
}

// A lowercase identifier: a letter, then letters, digits and single
// underscores; no abbreviation, no direction suffix.
inline std::optional<std::string> CheckSegment(std::string_view segment,
                                               std::string_view what) {
  const std::string quoted =
      std::string{what} + " '" + std::string{segment} + "'";
  if (segment.empty()) return std::string{what} + " is empty";
  if (segment.size() > kMaxSegmentBytes)
    return quoted + " is " + std::to_string(segment.size()) +
           " characters; the limit is " + std::to_string(kMaxSegmentBytes);
  if (segment.front() < 'a' || segment.front() > 'z')
    return quoted + " must start with a lowercase letter";
  for (const char c : segment) {
    if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'))
      return quoted + " contains '" + std::string{c} +
             "'; segments are lowercase letters, digits and underscores";
  }
  if (segment.find("__") != std::string_view::npos)
    return quoted + " has a doubled underscore";
  if (segment.back() == '_') return quoted + " ends with an underscore";
  for (const auto& [wrong, right] : kAbbreviations) {
    if (segment == wrong)
      return quoted + " is an abbreviation; use '" + std::string{right} + "'";
  }
  for (const auto suffix : kDirectionSuffixes) {
    if (segment.size() > suffix.size() && segment.ends_with(suffix))
      return quoted + " encodes direction in '" + std::string{suffix} +
             "'; direction belongs to the endpoint, not the name";
  }
  return std::nullopt;
}

}  // namespace detail

// One segment of a topic name, e.g. a target lane's source
// (`/drivetrain/target/<source>`); `what` names it in the message.
inline std::optional<std::string> CheckSegmentName(std::string_view segment,
                                                   std::string_view what) {
  return detail::CheckSegment(segment, what);
}

// A node's name: its `[subsystems.<name>]` key, log stem and registry row.
inline std::optional<std::string> CheckNodeName(std::string_view name) {
  if (auto problem = detail::CheckSegment(name, "node name")) return problem;
  if (detail::OneOf(kReservedOwners, name))
    return "node name '" + std::string{name} +
           "' is a reserved topic namespace";
  return std::nullopt;
}

inline std::optional<std::string> CheckTopicName(std::string_view topic) {
  const std::string quoted = "topic '" + std::string{topic} + "'";
  if (topic.empty()) return std::string{"topic name is empty"};
  if (topic.front() != '/') return quoted + " must start with '/'";
  if (topic.size() > kMaxTopicBytes)
    return quoted + " is " + std::to_string(topic.size()) +
           " characters; the limit is " + std::to_string(kMaxTopicBytes);
  if (topic.back() == '/') return quoted + " ends with '/'";
  if (topic.find("//") != std::string_view::npos)
    return quoted + " has an empty segment";

  const auto segments = detail::Segments(topic);
  if (segments.size() < kMinSegments || segments.size() > kMaxSegments)
    return quoted + " has " + std::to_string(segments.size()) +
           " segments; the form is /owner/role[/qualifier[/qualifier]]";

  static constexpr std::string_view kWhat[] = {"owner", "role", "qualifier",
                                               "qualifier"};
  for (std::size_t i = 0; i < segments.size(); ++i) {
    if (auto problem = detail::CheckSegment(segments[i], kWhat[i]))
      return quoted + ": " + *problem;
  }

  if (!detail::OneOf(kOwnerPublishedRoles, segments[1]) &&
      !detail::OneOf(kOwnerConsumedRoles, segments[1])) {
    std::string allowed;
    for (const auto role : kOwnerPublishedRoles)
      allowed += (allowed.empty() ? "" : ", ") + std::string{role};
    for (const auto role : kOwnerConsumedRoles)
      allowed += ", " + std::string{role};
    return quoted + ": role '" + std::string{segments[1]} + "' is not one of " +
           allowed;
  }
  return std::nullopt;
}

// Alerts: a bool field of a node's own `status` message named
// `alert_<level>_<name>` is an alert, active while true (NAMING.md, the
// `status` role). ParseAlertField is the only reader of that grammar: the
// graph lint checks fields with it, the telemetry dictionary tags channels
// with its level, and every viewer reads the level from there.
inline constexpr std::string_view kAlertPrefix = "alert_";

// Ordered by severity; the values are telemetry.fbs's AlertLevel.
enum class AlertLevel : std::uint8_t { kNone, kInfo, kWarning, kError };

struct AlertField {
  AlertLevel level{AlertLevel::kNone};
  std::string_view name;  // the `<name>` part, a view into the field name
};

// `field` (a field name, not a path) as an alert: `alert_error_<name>`,
// `alert_warning_<name>` or `alert_info_<name>`, where `<name>` is a
// lowercase letter, then lowercase letters, digits and single underscores,
// not ending in one. Anything else, including an `alert_` field of another
// level or with no name, is nullopt.
inline std::optional<AlertField> ParseAlertField(std::string_view field) {
  if (!field.starts_with(kAlertPrefix)) return std::nullopt;
  field.remove_prefix(kAlertPrefix.size());
  struct Spelling {
    std::string_view word;
    AlertLevel level;
  };
  static constexpr Spelling kLevels[] = {{"error_", AlertLevel::kError},
                                         {"warning_", AlertLevel::kWarning},
                                         {"info_", AlertLevel::kInfo}};
  for (const auto& [word, level] : kLevels) {
    if (!field.starts_with(word)) continue;
    const std::string_view name = field.substr(word.size());
    if (name.empty() || name.front() < 'a' || name.front() > 'z' ||
        name.back() == '_' || name.find("__") != std::string_view::npos) {
      return std::nullopt;
    }
    for (const char c : name) {
      if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'))
        return std::nullopt;
    }
    return AlertField{level, name};
  }
  return std::nullopt;
}

namespace detail {
// One field of a message whose name starts with kAlertPrefix.
struct AlertPrefixedField {
  std::string path;     // field names from the root struct, joined by '.'
  std::string name;     // the field's own name
  bool is_bool{false};  // a bool scalar (not an array, not a struct)
};

// Every field whose name starts with kAlertPrefix in struct `root` of
// `bfbs` (a reflection.Schema), nested structs and arrays of structs
// included, each struct type visited once. Empty when `bfbs` does not
// verify or does not hold `root`. In names.cc: it reads reflection.
std::vector<AlertPrefixedField> AlertPrefixedFields(std::string_view bfbs,
                                                    std::string_view root);
}  // namespace detail

// The node name must match the target's package leaf
// (`//2026-robot/main_processor/odometry:node`), its repository when at the
// repository root (`@wrist//:node`), or its rule name
// (`//talOS/bridge:hardware_node`), so a name in a viewer traces to source.
inline std::optional<std::string> CheckNodeTarget(std::string_view name,
                                                  std::string_view target) {
  const std::string context = "target '" + std::string{target} +
                              "' for node '" + std::string{name} + "'";
  std::string_view repo;
  std::string_view rest = target;
  if (rest.starts_with("@")) {
    const std::size_t at_count = rest.starts_with("@@") ? 2 : 1;
    const std::size_t slashes = rest.find("//", at_count);
    if (slashes == std::string_view::npos)
      return context +
             " must be an absolute Bazel label (//package:rule or "
             "@repo//package:rule)";
    repo = rest.substr(at_count, slashes - at_count);
    while (!repo.empty() && repo.back() == '+') repo.remove_suffix(1);
    if (repo.empty() || repo.find_first_of("@/:") != std::string_view::npos)
      return context +
             " must be an absolute Bazel label (//package:rule or "
             "@repo//package:rule)";
    rest.remove_prefix(slashes);
  } else if (!rest.starts_with("//")) {
    return context +
           " must be an absolute Bazel label (//package:rule or "
           "@repo//package:rule)";
  }
  const std::size_t colon = rest.find(':');
  if (colon == std::string_view::npos)
    return context +
           " must name a rule, as //package:rule or @repo//package:rule";

  const std::string_view package = rest.substr(2, colon - 2);
  const std::string_view rule = rest.substr(colon + 1);
  const std::size_t slash = package.rfind('/');
  const std::string_view leaf = package.empty() ? repo
                                : slash == std::string_view::npos
                                    ? package
                                    : package.substr(slash + 1);
  if (leaf == name || rule == name) return std::nullopt;
  const char* what =
      package.empty() && !repo.empty() ? "repository" : "package leaf";
  return "node '" + std::string{name} + "' is built by '" +
         std::string{target} + "'; the name must match either the " + what +
         " ('" + std::string{leaf} + "') or the rule ('" + std::string{rule} +
         "')";
}

// One end of one topic, as its node declared it. A `--describe` and a running
// registry are checked through the same shape. The schema and layout fields
// are what a `--describe` reports (zero/empty from the registry, which does
// not carry them): the topic's schema identity (talOS/schema/identity.h) and
// its ring layout, which the launcher creates the segment with.
struct SourceShape {
  event::SourceKind kind{event::SourceKind::SENDER};
  std::string name;
  std::uint32_t message_bytes{0};
  std::uint32_t flags{0};
  std::string schema;
  std::uint64_t schema_hash{0};
  std::uint32_t alignment{0};
  std::uint32_t slots{0};
  // The schema's reflection.Schema bytes (schema::Identity::bfbs): empty for
  // a layout without one, and never in a registry snapshot.
  std::string bfbs;

  bool external() const { return (flags & kSourceFlagExternal) != 0; }
  bool optional() const { return (flags & kSourceFlagOptional) != 0; }
  bool observer() const { return (flags & kSourceFlagObserver) != 0; }
};

struct NodeShape {
  std::string name;
  std::string target;
  std::vector<SourceShape> sources;
  // The roster's hardware bridge (the launcher marks the node in the
  // kHardwareBridgeNode slot, whatever its binary calls itself): the one node
  // that writes the `hw` namespace, except each node's own request.
  bool hardware_bridge{false};
};

enum class Severity { WARNING, ERROR };

struct Diagnostic {
  Severity severity{Severity::ERROR};
  std::string subject;  // The topic or node the finding is about.
  std::string message;

  std::string ToString() const {
    return std::string{severity == Severity::ERROR ? "error" : "warning"} +
           ": " + subject + ": " + message;
  }
};

// Advisory hardware limits, not configuration errors: the estimate counts
// status frames on the roboRIO CAN bus only, not CANivore traffic.
inline std::vector<Diagnostic> CheckHardwareConfig(
    const hardware::Config& config) {
  std::vector<Diagnostic> out;
  std::size_t rio_frames_per_update = 0;
  for (const auto& motor : config.motors) {
    const double sensor_velocity =
        motor.max_velocity_rps * motor.sensor_to_mechanism_ratio;
    if (sensor_velocity > 105.0) {
      out.push_back(
          {Severity::WARNING, motor.name,
           "max_velocity_rps * sensor_to_mechanism_ratio is " +
               std::to_string(sensor_velocity) +
               " rps, exceeding the recommended 105 rps; check gearing and "
               "the configured maximum velocity"});
    }
    if (motor.bus == "rio") rio_frames_per_update += 4;
  }
  for (const auto& sensor : config.sensors) {
    if (sensor.bus == "rio") rio_frames_per_update += 2;
  }
  const double rio_frames_per_second =
      static_cast<double>(rio_frames_per_update) * config.status_hz;
  if (rio_frames_per_second > 15000.0) {
    out.push_back(
        {Severity::WARNING, "rio",
         "estimated CAN status traffic (4 * motors + 2 * sensors) * "
         "status_hz is " +
             std::to_string(rio_frames_per_second) +
             " frames/s, exceeding the recommended 15000 frames/s; reduce "
             "status_hz or move devices to another CAN bus"});
  }
  return out;
}

// Each subsystem's target sources: every `<source>` its target policy lists
// in any mode ([robot.targets] plus its own [targets]), keyed by subsystem.
using TargetSources =
    std::map<std::string, std::vector<std::string>, std::less<>>;

// Per-mode policy sources, kept separate from the union used for lane naming.
using TargetModeSources =
    std::map<std::string,
             std::map<std::string, std::vector<std::string>, std::less<>>,
             std::less<>>;

// A target lane is `/<subsystem>/target/<source>`: a valid topic name of
// exactly three segments whose role is `target`. C++ recognises a lane only
// through IsTargetLane, and producers and test helpers build one only through
// TargetLaneTopic; a consumer (talos::Target) subscribes to the relative
// `target/<source>`, which its node resolves (and may remap). session_report.py
// and contract.bzl mirror the grammar.
inline bool IsTargetLane(std::string_view topic) {
  if (CheckTopicName(topic)) return false;
  const auto segments = detail::Segments(topic);
  return segments.size() == 3 && segments[1] == "target";
}

namespace detail {
// Unchecked: LintGraph names a policy lane even when it would be malformed.
inline std::string JoinTargetLane(std::string_view subsystem,
                                  std::string_view source) {
  return "/" + std::string{subsystem} + "/target/" + std::string{source};
}
}  // namespace detail

// Whether `topic` is in the hardware request namespace: `/hw/request` or any
// topic below it, well formed or not. Only a node's devices-layer flush
// writes there (detail::NodeAccess::add_hardware_request); the generic
// publishers refuse every such topic, however it was spelled or remapped.
inline bool IsHardwareRequest(std::string_view topic) {
  const auto segments = detail::Segments(topic);
  return segments.size() >= 2 && segments[0] == "hw" &&
         segments[1] == "request";
}

// The hardware request of node `node`: hardware::RequestTopic, the one
// spelling the bridge also reads.
inline std::string HardwareRequestTopic(std::string_view node) {
  return hardware::RequestTopic(node);
}
static_assert(std::string_view{hardware::kRequestTopicPrefix} == "/hw/request/",
              "IsHardwareRequest recognises the prefix the bridge reads");

// Why `/<subsystem>/target/<source>` is not a target lane, or nullopt.
inline std::optional<std::string> CheckTargetLane(std::string_view subsystem,
                                                  std::string_view source) {
  if (auto problem = CheckSegmentName(subsystem, "target lane subsystem"))
    return problem;
  if (auto problem = CheckSegmentName(source, "target lane source"))
    return problem;
  return CheckTopicName(detail::JoinTargetLane(subsystem, source));
}

// The lane `/<subsystem>/target/<source>`; std::invalid_argument with
// CheckTargetLane's reason when the parts do not make one.
inline std::string TargetLaneTopic(std::string_view subsystem,
                                   std::string_view source) {
  if (auto problem = CheckTargetLane(subsystem, source))
    throw std::invalid_argument(*problem);
  return detail::JoinTargetLane(subsystem, source);
}

namespace detail {
// Why `node` may not write `source`, a topic of the `hw` namespace, or
// nullopt (RULES.md 2). The bridge (NodeShape::hardware_bridge) writes the
// namespace, except the requests:
// `/hw/request/<node>` is written only by <node>'s devices-layer flush, which
// gates every output on robot_enabled and update() and applies the motor
// limits and position frames before it sends; a plain publisher would skip
// all of that.
inline std::optional<std::string> HardwareWriterProblem(
    const NodeShape& node, const SourceShape& source) {
  const std::string writer = "is written by '" + node.name + "'";
  if (!IsHardwareRequest(source.name)) {
    if (node.hardware_bridge) return std::nullopt;
    return writer + "; the hw namespace is written only by the hardware " +
           "bridge (the roster's '" + std::string{kHardwareBridgeNode} +
           "'), except each node's own " + HardwareRequestTopic("<node>");
  }
  if (source.name != HardwareRequestTopic(node.name))
    return writer + ", but the only hardware request a node writes is its " +
           "own, " + HardwareRequestTopic(node.name);
  if ((source.flags & kSourceFlagHardwareRequest) == 0)
    return writer + " outside its devices layer; a hardware request " +
           "leaves only through the devices-layer flush after update(), " +
           "which gates it on robot_enabled and applies the motor limits " +
           "(command the node's devices instead of publishing a request), " +
           "or the binary predates the hardware_request attribute; rebuild " +
           "it";
  return std::nullopt;
}
}  // namespace detail

// Everything checkable about a whole graph, errors first.
//
// Errors cannot be correct in any configuration (two writers, a reader with no
// writer, two schemas or message sizes or ring layouts on one topic, a
// malformed name, a lane its subsystem never reads, an `hw` topic written by
// anything but the roster's bridge or, for `/hw/request/<node>`, <node>'s
// devices-layer flush, an `alert_` field that is not a bool or not
// ParseAlertField's grammar). Warnings are normal during development or in a
// partial launch (an unread topic, an owner not in this session, a policy
// lane nobody writes yet, an alert outside a `status` topic). Required coverage
// additionally rejects a target-consuming subsystem with no producer for a
// nonempty mode policy. Timers are skipped: their names are labels, not topics.
// A subsystem absent from target_sources is not lane-checked.
inline std::vector<Diagnostic> LintGraph(
    const std::vector<NodeShape>& nodes,
    const TargetSources& target_sources = {},
    const TargetModeSources& target_mode_sources = {},
    bool require_target_coverage = false) {
  std::vector<Diagnostic> out;
  const auto add = [&out](Severity severity, std::string subject,
                          std::string message) {
    out.push_back({severity, std::move(subject), std::move(message)});
  };
  const auto join = [](const std::vector<std::string>& names) {
    std::string joined;
    for (const auto& name : names)
      joined += (joined.empty() ? "" : ", ") + name;
    return joined;
  };

  struct TopicFacts {
    std::string name;
    std::vector<std::string> writers;
    std::vector<std::string> readers;
    std::size_t reader_count{0};  // Observers also consume transport slots.
    bool exempt{false};           // Any end declared external or optional.
    std::uint32_t message_bytes{0};
    std::string first_size_from;
    std::string schema;  // the first end's that declared one
    std::uint64_t schema_hash{0};
    std::string first_schema_from;
    std::uint32_t alignment{0};
    std::uint32_t slots{0};
    std::string first_layout_from;
  };
  std::vector<TopicFacts> topics;  // Declaration order, for stable output.
  const auto facts = [&topics](const std::string& name) -> TopicFacts& {
    for (auto& topic : topics)
      if (topic.name == name) return topic;
    return topics.emplace_back(TopicFacts{.name = name});
  };

  std::vector<std::string> seen_names;
  for (const auto& node : nodes) {
    if (auto problem = CheckNodeName(node.name))
      add(Severity::ERROR, node.name.empty() ? "<unnamed node>" : node.name,
          *problem);
    if (!node.target.empty()) {
      if (auto problem = CheckNodeTarget(node.name, node.target))
        add(Severity::ERROR, node.name, *problem);
    }
    if (std::find(seen_names.begin(), seen_names.end(), node.name) !=
        seen_names.end())
      add(Severity::ERROR, node.name,
          "two nodes are registered under this name; a name identifies one "
          "process");
    seen_names.push_back(node.name);

    for (const auto& source : node.sources) {
      if (!IsSender(source.kind) && !IsReader(source.kind)) continue;

      if (auto problem = CheckTopicName(source.name)) {
        add(Severity::ERROR,
            source.name.empty() ? "<unnamed topic>" : source.name,
            *problem + " (declared by " + node.name + ")");
        continue;
      }

      auto& topic = facts(source.name);
      topic.exempt = topic.exempt || source.external() || source.optional();
      if (IsSender(source.kind)) {
        topic.writers.push_back(node.name);
        // Only the writing end makes a claim about the subject.
        const auto segments = detail::Segments(source.name);
        const std::string owner{segments[0]};
        const std::string role{segments[1]};
        if (owner == "hw") {
          if (auto problem = detail::HardwareWriterProblem(node, source))
            add(Severity::ERROR, source.name, *problem);
        }
        // Alert fields, from the writer's schema (a shape without one, a
        // registry snapshot, has no fields to check).
        if (!source.bfbs.empty()) {
          for (const auto& field :
               detail::AlertPrefixedFields(source.bfbs, source.schema)) {
            const auto alert = ParseAlertField(field.name);
            if (!alert) {
              add(Severity::ERROR, source.name,
                  "field '" + field.path + "' (written by " + node.name +
                      ") starts with 'alert_' but is not "
                      "alert_<error|warning|info>_<name>");
            } else if (!field.is_bool) {
              add(Severity::ERROR, source.name,
                  "alert field '" + field.path + "' (written by " + node.name +
                      ") is not a bool; an alert is active "
                      "while true");
            } else if (role != "status") {
              add(Severity::WARNING, source.name,
                  "alert field '" + field.path + "' (written by " + node.name +
                      ") is in a '" + role +
                      "' topic; an alert belongs to its node's 'status' "
                      "topic, the health it reports");
            }
          }
        }
        if (!detail::OneOf(kReservedOwners, owner)) {
          if (detail::OneOf(kOwnerPublishedRoles, role) && owner != node.name)
            add(Severity::ERROR, source.name,
                "carries the state of '" + owner + "' but is published by '" +
                    node.name + "'; a '" + role +
                    "' topic is published by its own subject");
          if (detail::OneOf(kOwnerConsumedRoles, role) && owner == node.name)
            add(Severity::ERROR, source.name,
                "is published by '" + node.name +
                    "', which is also its subject; a '" + role +
                    "' topic is written by whoever is asking, not by the "
                    "subsystem being asked");
          if (IsTargetLane(source.name) && owner != node.name) {
            const std::string lane_source{segments[2]};
            const auto policy = target_sources.find(owner);
            if (policy != target_sources.end() &&
                std::find(policy->second.begin(), policy->second.end(),
                          lane_source) == policy->second.end())
              add(Severity::ERROR, source.name,
                  "is written by '" + node.name + "', but '" + lane_source +
                      "' is in no mode of " + owner + "'s target policy, so " +
                      owner +
                      " never reads it; list it in [robot.targets] or " +
                      owner + "'s [targets]");
          }
        }
      } else if (!source.observer()) {
        topic.readers.push_back(node.name);
      }
      if (IsReader(source.kind)) ++topic.reader_count;

      // Two types on one topic: named by schema identity, which also catches
      // two same-size types a size check cannot tell apart. A shape without
      // a schema (a registry snapshot) falls back to its size alone.
      bool schema_conflict = false;
      if (!source.schema.empty()) {
        if (topic.schema.empty()) {
          topic.schema = source.schema;
          topic.schema_hash = source.schema_hash;
          topic.first_schema_from = node.name;
        } else if (topic.schema != source.schema ||
                   topic.schema_hash != source.schema_hash) {
          schema_conflict = true;
          add(Severity::ERROR, source.name,
              "used with two schemas: " +
                  schema::Describe(topic.schema, topic.schema_hash) + " by " +
                  topic.first_schema_from + ", " +
                  schema::Describe(source.schema, source.schema_hash) + " by " +
                  node.name);
        }
      }
      if (source.message_bytes != 0) {
        if (topic.message_bytes == 0) {
          topic.message_bytes = source.message_bytes;
          topic.first_size_from = node.name;
        } else if (topic.message_bytes != source.message_bytes &&
                   !schema_conflict) {
          add(Severity::ERROR, source.name,
              "used with two message sizes: " +
                  std::to_string(topic.message_bytes) + " bytes by " +
                  topic.first_size_from + ", " +
                  std::to_string(source.message_bytes) + " bytes by " +
                  node.name);
        }
      }
      // Every end opens the segment the launcher creates with the first
      // end's ring layout, so another layout could not attach to it.
      if (source.slots != 0 && !schema_conflict) {
        if (topic.slots == 0) {
          topic.alignment = source.alignment;
          topic.slots = source.slots;
          topic.first_layout_from = node.name;
        } else if (topic.slots != source.slots ||
                   topic.alignment != source.alignment) {
          add(Severity::ERROR, source.name,
              "used with two ring layouts: " + std::to_string(topic.slots) +
                  " slots aligned to " + std::to_string(topic.alignment) +
                  " by " + topic.first_layout_from + ", " +
                  std::to_string(source.slots) + " slots aligned to " +
                  std::to_string(source.alignment) + " by " + node.name);
        }
      }
    }
  }

  for (const auto& topic : topics) {
    const std::string owner{detail::Segments(topic.name)[0]};
    if (!detail::OneOf(kReservedOwners, owner) &&
        std::find(seen_names.begin(), seen_names.end(), owner) ==
            seen_names.end())
      add(Severity::WARNING, topic.name,
          "the owner segment names '" + owner +
              "', which is not a node in this session and not a reserved "
              "namespace");
    if (owner == "test")
      add(Severity::ERROR, topic.name,
          "the 'test' namespace is for tests; a node in a session must not "
          "use it");

    if (topic.reader_count > MAX_READERS) {
      add(Severity::ERROR, topic.name,
          "has " + std::to_string(topic.reader_count) +
              " readers; RTMS reader limit is " + std::to_string(MAX_READERS) +
              " (including observers)");
    }
    if (topic.writers.size() > 1) {
      add(Severity::ERROR, topic.name,
          "has " + std::to_string(topic.writers.size()) + " writers (" +
              join(topic.writers) +
              "); one writer per topic is what keeps the dispatch log "
              "replayable");
    } else if (topic.writers.empty() && !topic.exempt) {
      add(Severity::ERROR, topic.name,
          "is read by " + join(topic.readers) +
              " but nothing publishes it; either the writer is missing or the "
              "two ends are spelled differently");
    } else if (topic.writers.empty() && !topic.readers.empty() &&
               IsTargetLane(topic.name)) {
      add(Severity::WARNING, topic.name,
          "is in the target policy of " + join(topic.readers) +
              " but nothing writes it yet");
    }
    if (topic.readers.empty() && !topic.exempt)
      add(Severity::WARNING, topic.name,
          "is published but nothing subscribes to it");
  }

  if (require_target_coverage) {
    for (const auto& node : nodes) {
      const bool consumes_targets =
          std::any_of(node.sources.begin(), node.sources.end(),
                      [&node](const SourceShape& source) {
                        return IsReader(source.kind) && !source.observer() &&
                               IsTargetLane(source.name) &&
                               detail::Segments(source.name)[0] == node.name;
                      });
      if (!consumes_targets) continue;
      const auto policy = target_mode_sources.find(node.name);
      if (policy == target_mode_sources.end()) continue;
      for (const auto& [mode, sources] : policy->second) {
        if (sources.empty()) continue;
        bool has_writer = false;
        std::vector<std::string> lanes;
        for (const auto& source : sources) {
          lanes.push_back(detail::JoinTargetLane(node.name, source));
          const auto topic = std::find_if(topics.begin(), topics.end(),
                                          [&lanes](const TopicFacts& facts) {
                                            return facts.name == lanes.back();
                                          });
          has_writer =
              has_writer || (topic != topics.end() && !topic->writers.empty());
        }
        if (!has_writer) {
          add(Severity::ERROR, node.name,
              "target coverage required for mode '" + mode +
                  "', but none of its policy lanes has a writer: " +
                  join(lanes));
        }
      }
    }
  }

  std::stable_partition(out.begin(), out.end(), [](const Diagnostic& d) {
    return d.severity == Severity::ERROR;
  });
  return out;
}

inline bool HasError(const std::vector<Diagnostic>& diagnostics) {
  return std::any_of(
      diagnostics.begin(), diagnostics.end(),
      [](const Diagnostic& d) { return d.severity == Severity::ERROR; });
}

}  // namespace talos::introspect::naming
