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
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/events/manifest.h"

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
// (optional).
inline constexpr std::uint32_t kSourceFlagExternal = 1u << 0;
inline constexpr std::uint32_t kSourceFlagOptional = 1u << 1;

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

// The node name must match the target's package leaf
// (`//2026-robot/main_processor/odometry:node`) or its rule name
// (`//talOS/bridge:hardware_node`), so a name in a viewer traces to source.
inline std::optional<std::string> CheckNodeTarget(std::string_view name,
                                                  std::string_view target) {
  const std::string context = "target '" + std::string{target} +
                              "' for node '" + std::string{name} + "'";
  if (!target.starts_with("//"))
    return context + " must be an absolute Bazel label";
  const std::size_t colon = target.find(':');
  if (colon == std::string_view::npos)
    return context + " must name a rule, as //package:rule";

  const std::string_view package = target.substr(2, colon - 2);
  const std::string_view rule = target.substr(colon + 1);
  const std::size_t slash = package.rfind('/');
  const std::string_view leaf =
      slash == std::string_view::npos ? package : package.substr(slash + 1);
  if (leaf == name || rule == name) return std::nullopt;
  return "node '" + std::string{name} + "' is built by '" +
         std::string{target} +
         "'; the name must match either the package leaf ('" +
         std::string{leaf} + "') or the rule ('" + std::string{rule} + "')";
}

// One end of one topic, as its node declared it. A `--describe` and a running
// registry are checked through the same shape.
struct SourceShape {
  event::SourceKind kind{event::SourceKind::SENDER};
  std::string name;
  std::uint32_t message_bytes{0};
  std::uint32_t flags{0};

  bool external() const { return (flags & kSourceFlagExternal) != 0; }
  bool optional() const { return (flags & kSourceFlagOptional) != 0; }
};

struct NodeShape {
  std::string name;
  std::string target;
  std::vector<SourceShape> sources;
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

// Everything checkable about a whole graph, errors first.
//
// Errors cannot be correct in any configuration (two writers, a reader with no
// writer, two message sizes, a malformed name). Warnings are normal during
// development or in a partial launch (an unread topic, an owner not in this
// session). Timers are skipped: their names are labels, not topics.
inline std::vector<Diagnostic> LintGraph(const std::vector<NodeShape>& nodes) {
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
    bool exempt{false};  // Any end declared external or optional.
    std::uint32_t message_bytes{0};
    std::string first_size_from;
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
        }
      } else {
        topic.readers.push_back(node.name);
      }

      if (source.message_bytes != 0) {
        if (topic.message_bytes == 0) {
          topic.message_bytes = source.message_bytes;
          topic.first_size_from = node.name;
        } else if (topic.message_bytes != source.message_bytes) {
          add(Severity::ERROR, source.name,
              "used with two message sizes: " +
                  std::to_string(topic.message_bytes) + " bytes by " +
                  topic.first_size_from + ", " +
                  std::to_string(source.message_bytes) + " bytes by " +
                  node.name);
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
    }
    if (topic.readers.empty() && !topic.exempt)
      add(Severity::WARNING, topic.name,
          "is published but nothing subscribes to it");
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
