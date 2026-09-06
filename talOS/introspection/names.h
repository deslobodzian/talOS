#pragma once

/*
 * The naming protocol, as code.
 *
 * talOS topics are POSIX shared-memory objects addressed by exact string. There
 * is no registry to resolve a name against and no type check at the transport,
 * so two spellings of one idea do not fail loudly -- they produce a publisher
 * and a subscriber that both work perfectly and never meet. That has happened
 * here more than once: `/hw/req/drive` against `/hw/req/drivetrain`, kept alive
 * by a special case in the bridge; `/hw/cmd` against `/hw/cmd_in`, which reads
 * as a topic nobody consumes.
 *
 * A convention in a document does not stop that, because nothing reads the
 * document. So the convention lives here, where the launcher checks it before
 * it spawns anything and a test checks it on every build. talOS/NAMING.md is
 * the prose; this header is the authority.
 *
 * Deliberately dependency-light -- names, no transport -- so the launcher can
 * validate a graph that does not exist yet and the registry can validate one
 * that does.
 */

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/manifest.h"

namespace talos::introspect::naming {

// A topic name is `/owner/role[/qualifier[/qualifier]]`.
inline constexpr std::size_t kMinSegments = 2;
inline constexpr std::size_t kMaxSegments = 4;

// The transport's limit, not the event loop's.
//
// A topic is a POSIX shared-memory object, and macOS caps a shm name at
// PSHMNAMLEN; `rtms::ValidatePath` enforces 30 characters after the leading
// slash and throws otherwise. The event loop would take 63
// (`event::MAX_SOURCE_NAME`), and an earlier version of this file used that
// number -- which made the check worse than useless: it accepted names that
// pass every review and then throw inside a node's constructor at startup. A
// validator has to enforce the tightest limit in the chain, not its own.
//
// The budget is genuinely tight, and it is the one rule here a person is
// likely to hit by accident. `/operator_interface/target/teleop` is 33
// characters and does not fit. When a name will not fit, shorten the
// qualifier, not the owner or the role -- those two are what make the graph
// readable.
inline constexpr std::size_t kMaxTopicBytes = 31;
static_assert(kMaxTopicBytes <= event::MAX_SOURCE_NAME,
              "a name the transport accepts must also fit a registry record");

// Bounded so one segment cannot spend the whole budget and leave a name that
// technically parses but says nothing.
inline constexpr std::size_t kMaxSegmentBytes = 24;

// The second segment, from a closed set.
//
// Closed on purpose. The point of a role is that a reader who has never seen a
// topic before still knows what is on it and which way it flows; a vocabulary
// anyone may extend cannot promise that. Adding one is a deliberate edit here
// and in NAMING.md, not an accident at a call site.
//
//   state     what a thing is now, published by the thing itself
//   target    what a thing is being asked to become
//   command   an actuator-level instruction, already resolved
//   request   an actuator-level instruction awaiting arbitration
//   status    health and liveness about a thing, not its physics
//   event     something that happened once, not a level
//   telemetry an observation feed, for recording and display only
inline constexpr std::string_view kRoles[] = {
    "state", "target", "command", "request", "status", "event", "telemetry"};

// Which side of the topic the owner segment sits on.
//
// This is the part that took a wrong turn first. "The owner is the publishing
// node" is the obvious rule and it is false here: the arbiter publishes
// `/drivetrain/target` and the drivetrain consumes it, which is the whole point
// of arbitration. The owner segment names the *subject* -- the subsystem the
// topic is about -- and the role says which way the data moves relative to it.
//
// So `state` and its relatives flow outward from the owner, and the owner must
// be the publisher: a node reporting another node's state is either misnamed or
// reaching into something that is not its business. `target`, `request` and
// `command` flow inward, and the owner must NOT be the publisher: a node
// publishing its own target is talking to itself, which means the name is wrong
// or the arbitration is confused.
inline constexpr std::string_view kOwnerPublishedRoles[] = {
    "state", "status", "event", "telemetry"};
inline constexpr std::string_view kOwnerConsumedRoles[] = {"target", "request",
                                                           "command"};

// Owners that are not nodes.
//
//   hw       the hardware bridge's namespace: the shm side of the controller
//   talos    framework-internal feeds, owned by no subsystem
//   sim      simulation-only topics, never present on a real robot
//   replay   topics fed from a log rather than from a peer
//   test     tests only; the launcher refuses to spawn a node that uses one
inline constexpr std::string_view kReservedOwners[] = {"hw", "talos", "sim",
                                                       "replay", "test"};

// Abbreviations seen in this repo, and what to write instead. The check exists
// because the cost of an abbreviation is not brevity, it is that the long form
// is still available to whoever writes the other end.
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

// Suffixes that encode direction into a name.
//
// Direction is a property of an endpoint, not of a topic: the same topic is
// outbound to its publisher and inbound to every subscriber. A name that picks
// one side is wrong from the other, and it invites a second topic for the
// other direction when what was wanted was one topic with two ends.
inline constexpr std::string_view kDirectionSuffixes[] = {
    "_in", "_out", "_pub", "_sub", "_tx", "_rx", "_topic", "_send", "_recv"};

// How a source's far end is expected to behave. Stored in SourceRecord::flags,
// which was a reserved word, so this costs no layout change.
//
// Without these the graph cannot tell a fault from a design: `/hw/command` is
// consumed by the RoboRIO over UDP and will never have a shm subscriber, and
// `/drivetrain/target/auto` has no publisher because autonomous is not written
// yet. Reporting both as broken trains people to ignore the report, which is
// worse than not having one.
inline constexpr std::uint32_t kSourceFlagExternal = 1u << 0;
inline constexpr std::uint32_t kSourceFlagOptional = 1u << 1;

static_assert(std::size(kRoles) == std::size(kOwnerPublishedRoles) +
                                       std::size(kOwnerConsumedRoles),
              "every role must sit on one side of its owner or the other, or "
              "adding a role would silently opt it out of the owner check");

inline bool IsSender(event::SourceKind kind) {
  return kind == event::SourceKind::SENDER;
}
inline bool IsReader(event::SourceKind kind) {
  return kind == event::SourceKind::WATCHER ||
         kind == event::SourceKind::FETCHER;
}

inline std::vector<std::string_view> Segments(std::string_view topic) {
  std::vector<std::string_view> out;
  if (topic.empty() || topic.front() != '/') return out;
  std::size_t at = 1;
  while (at <= topic.size()) {
    const std::size_t end = topic.find('/', at);
    if (end == std::string_view::npos) {
      out.push_back(topic.substr(at));
      break;
    }
    out.push_back(topic.substr(at, end - at));
    at = end + 1;
  }
  return out;
}

// `/drivetrain/target/teleop` -> `drivetrain`. Empty when the name does not
// parse, so callers can check the name first and then read the owner.
inline std::string_view TopicOwner(std::string_view topic) {
  const auto segments = Segments(topic);
  return segments.empty() ? std::string_view{} : segments.front();
}

inline std::string_view TopicRole(std::string_view topic) {
  const auto segments = Segments(topic);
  return segments.size() < 2 ? std::string_view{} : segments[1];
}

inline bool OwnerPublishes(std::string_view role) {
  return std::find(std::begin(kOwnerPublishedRoles),
                   std::end(kOwnerPublishedRoles),
                   role) != std::end(kOwnerPublishedRoles);
}

inline bool OwnerConsumes(std::string_view role) {
  return std::find(std::begin(kOwnerConsumedRoles),
                   std::end(kOwnerConsumedRoles),
                   role) != std::end(kOwnerConsumedRoles);
}

inline bool IsReservedOwner(std::string_view owner) {
  return std::find(std::begin(kReservedOwners), std::end(kReservedOwners),
                   owner) != std::end(kReservedOwners);
}

// A lowercase identifier: starts with a letter, then letters, digits and
// underscores. No hyphens, no camelCase, no leading digit.
inline std::optional<std::string> CheckSegment(std::string_view segment,
                                               std::string_view what) {
  const std::string label{what};
  if (segment.empty()) return label + " is empty";
  if (segment.size() > kMaxSegmentBytes)
    return label + " '" + std::string{segment} + "' is " +
           std::to_string(segment.size()) + " characters; the limit is " +
           std::to_string(kMaxSegmentBytes);
  if (segment.front() < 'a' || segment.front() > 'z')
    return label + " '" + std::string{segment} +
           "' must start with a lowercase letter";
  for (const char c : segment) {
    const bool ok =
        (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
    if (!ok)
      return label + " '" + std::string{segment} + "' contains '" +
             std::string{c} +
             "'; segments are lowercase letters, digits and underscores";
  }
  if (segment.find("__") != std::string_view::npos)
    return label + " '" + std::string{segment} + "' has a doubled underscore";
  if (segment.back() == '_')
    return label + " '" + std::string{segment} + "' ends with an underscore";
  for (const auto& [wrong, right] : kAbbreviations) {
    if (segment == wrong)
      return label + " '" + std::string{segment} +
             "' is an abbreviation; use '" + std::string{right} + "'";
  }
  for (const auto suffix : kDirectionSuffixes) {
    if (segment.size() > suffix.size() && segment.ends_with(suffix))
      return label + " '" + std::string{segment} + "' encodes direction in '" +
             std::string{suffix} +
             "'; direction belongs to the endpoint, not the name";
  }
  return std::nullopt;
}

// A node's name: the `[subsystems.<name>]` key, the Reporter name, the log file
// stem and the registry row, all one string.
inline std::optional<std::string> CheckNodeName(std::string_view name) {
  if (auto problem = CheckSegment(name, "node name")) return problem;
  if (IsReservedOwner(name))
    return "node name '" + std::string{name} +
           "' is a reserved topic namespace";
  return std::nullopt;
}

inline std::optional<std::string> CheckTopicName(std::string_view topic) {
  if (topic.empty()) return std::string{"topic name is empty"};
  if (topic.front() != '/')
    return "topic '" + std::string{topic} + "' must start with '/'";
  if (topic.size() > kMaxTopicBytes)
    return "topic '" + std::string{topic} + "' is " +
           std::to_string(topic.size()) + " characters; the limit is " +
           std::to_string(kMaxTopicBytes);
  if (topic.back() == '/')
    return "topic '" + std::string{topic} + "' ends with '/'";
  if (topic.find("//") != std::string_view::npos)
    return "topic '" + std::string{topic} + "' has an empty segment";

  const auto segments = Segments(topic);
  if (segments.size() < kMinSegments || segments.size() > kMaxSegments)
    return "topic '" + std::string{topic} + "' has " +
           std::to_string(segments.size()) + " segments; the form is " +
           "/owner/role[/qualifier[/qualifier]]";

  static constexpr std::string_view kWhat[] = {"owner", "role", "qualifier",
                                               "qualifier"};
  for (std::size_t i = 0; i < segments.size(); ++i) {
    if (auto problem = CheckSegment(segments[i], kWhat[i]))
      return "topic '" + std::string{topic} + "': " + *problem;
  }

  if (std::find(std::begin(kRoles), std::end(kRoles), segments[1]) ==
      std::end(kRoles)) {
    std::string allowed;
    for (const auto role : kRoles) {
      if (!allowed.empty()) allowed += ", ";
      allowed += std::string{role};
    }
    return "topic '" + std::string{topic} + "': role '" +
           std::string{segments[1]} + "' is not one of " + allowed;
  }
  return std::nullopt;
}

// The Bazel target and the node name must agree, because the launcher resolves
// one from the config and reports the other, and a viewer that shows a name no
// build target produces cannot be traced back to source.
//
// A robot's node lives at `//<robot>/<processor>/<name>:node`, so the package's
// last segment carries the name. A framework node has no subsystem package, so
// there the rule is the other one available: the rule name after the colon.
inline std::optional<std::string> CheckNodeTarget(std::string_view name,
                                                  std::string_view target) {
  if (!target.starts_with("//"))
    return "target '" + std::string{target} + "' for node '" +
           std::string{name} + "' must be an absolute Bazel label";
  const std::size_t colon = target.find(':');
  if (colon == std::string_view::npos)
    return "target '" + std::string{target} + "' for node '" +
           std::string{name} + "' must name a rule, as //package:rule";

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

// One end of one topic, as its owning node declared it. The shape the linter
// works on, so a graph that has not started yet and a graph that is running are
// checked by the same code.
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

// Everything checkable about a whole graph rather than one name.
//
// Ordered errors first so a caller printing the list leads with what will stop
// a launch. Errors are things that cannot be correct in any configuration; a
// topic with no reader is a warning because a feed nobody has subscribed to yet
// is a normal state during development, whereas two writers on one topic is not
// a state at all -- it makes the dispatch log unreplayable.
inline std::vector<Diagnostic> LintGraph(const std::vector<NodeShape>& nodes) {
  std::vector<Diagnostic> out;
  const auto add = [&out](Severity severity, std::string subject,
                          std::string message) {
    out.push_back({severity, std::move(subject), std::move(message)});
  };

  struct TopicFacts {
    std::vector<std::string> writers;
    std::vector<std::string> readers;
    bool external{false};
    bool optional{false};
    std::uint32_t message_bytes{0};
    std::string first_size_from;
  };
  std::vector<std::pair<std::string, TopicFacts>> topics;
  const auto facts = [&topics](const std::string& name) -> TopicFacts& {
    for (auto& [key, value] : topics)
      if (key == name) return value;
    topics.push_back({name, TopicFacts{}});
    return topics.back().second;
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
      // A timer's name is a label, not an address. `swerve`, `shooter` and
      // `telemetry` are real timer names in this robot, and checking them as
      // topics reported three errors on a graph that was correct -- which is
      // the exact failure mode this linter exists to avoid, pointed at itself.
      // Only the three kinds that name a transport endpoint are checked.
      if (!IsSender(source.kind) && !IsReader(source.kind)) continue;

      if (auto problem = CheckTopicName(source.name)) {
        add(Severity::ERROR,
            source.name.empty() ? "<unnamed topic>" : source.name,
            *problem + " (declared by " + node.name + ")");
        continue;
      }

      auto& topic = facts(source.name);
      topic.external = topic.external || source.external();
      topic.optional = topic.optional || source.optional();
      if (IsSender(source.kind)) {
        topic.writers.push_back(node.name);
        // Checked on the writing end only: a subscriber has no claim on the
        // name of what it reads.
        const std::string_view owner = TopicOwner(source.name);
        const std::string_view role = TopicRole(source.name);
        if (!IsReservedOwner(owner)) {
          if (OwnerPublishes(role) && owner != node.name)
            add(Severity::ERROR, source.name,
                "carries the state of '" + std::string{owner} +
                    "' but is published by '" + node.name + "'; a '" +
                    std::string{role} +
                    "' topic is published by its own subject");
          if (OwnerConsumes(role) && owner == node.name)
            add(Severity::ERROR, source.name,
                "is published by '" + node.name +
                    "', which is also its subject; a '" + std::string{role} +
                    "' topic is written by whoever is asking, not by the "
                    "subsystem being asked");
        }
      } else if (IsReader(source.kind)) {
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

  for (const auto& [name, topic] : topics) {
    // An owner that names no node in the session is the shape a typo takes:
    // `/drivetraim/state`, or a name left behind by a rename. A warning rather
    // than an error, because a partial launch -- one subsystem disabled in
    // robot.toml, one node run on its own -- is a legitimate session in which
    // the owner really is absent.
    const std::string_view owner = TopicOwner(name);
    if (!IsReservedOwner(owner) &&
        std::find(seen_names.begin(), seen_names.end(), owner) ==
            seen_names.end())
      add(Severity::WARNING, name,
          "the owner segment names '" + std::string{owner} +
              "', which is not a node in this session and not a reserved "
              "namespace");

    if (TopicOwner(name) == "test")
      add(Severity::ERROR, name,
          "the 'test' namespace is for tests; a node in a session must not "
          "use it");

    if (topic.writers.size() > 1) {
      std::string writers;
      for (const auto& writer : topic.writers) {
        if (!writers.empty()) writers += ", ";
        writers += writer;
      }
      add(Severity::ERROR, name,
          "has " + std::to_string(topic.writers.size()) + " writers (" +
              writers +
              "); one writer per topic is what keeps the dispatch log "
              "replayable");
    } else if (topic.writers.empty() && !topic.external && !topic.optional) {
      std::string readers;
      for (const auto& reader : topic.readers) {
        if (!readers.empty()) readers += ", ";
        readers += reader;
      }
      add(Severity::ERROR, name,
          "is read by " + readers +
              " but nothing publishes it; either the writer is missing or the "
              "two ends are spelled differently");
    }

    if (topic.readers.empty() && !topic.external && !topic.optional)
      add(Severity::WARNING, name, "is published but nothing subscribes to it");

    // No rule here about prefix families -- /drivetrain/target beside
    // /drivetrain/target/teleop. That is the sanctioned arbitration shape, and
    // it needs no constraint: RTMS lookups are exact string matches, so a
    // parent name never captures a child's traffic. An earlier draft checked
    // that a family shared one owner, which cannot fail -- the owner *is* the
    // first segment, so a family shares it by construction. A check that can
    // never fire is worse than no check, because it implies coverage that does
    // not exist.
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
