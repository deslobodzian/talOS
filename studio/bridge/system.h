#pragma once

/*
 * The system graph: every node, every topic, and who is on which end.
 *
 * Telemetry frames are FlatBuffers because they are high rate and fixed shape.
 * This is the opposite: a few times a second, structure-heavy, and read by
 * people and agents as much as by the viewer. So it is JSON -- no generated
 * code, no hand-written offset verifier on the receiving end, and `curl
 * localhost:5800/system.json` answers "what is running" without a build step.
 *
 * uint64 values are decimal strings, the same rule the telemetry timestamps
 * follow: JSON numbers are doubles, and a sequence counter or a nanosecond
 * timestamp silently loses precision past 2^53. Values that cannot approach
 * that -- byte counts, capacities, client counts -- stay numbers.
 */

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/introspection/names.h"
#include "talOS/introspection/registry.h"

namespace studio {

namespace event = talos::event;
namespace introspect = talos::introspect;
namespace naming = talos::introspect::naming;

// A deadline that never comes: nothing is due.
inline constexpr std::int64_t kNeverNs =
    std::numeric_limits<std::int64_t>::max();

// Bumped for the per-source `external`/`optional` booleans and the per-topic
// `health` string. A version-1 reader would ignore the new fields and keep
// classifying topics itself, which is exactly the wrong outcome: it would go on
// calling `/hw/command` orphaned. The bump is what lets such a reader refuse
// the document instead of drawing a fault that is not there.
inline constexpr int kSystemGraphVersion = 2;

// What the bridge itself is doing. It is a participant in the system, not a
// neutral observer: it holds a reader slot on the telemetry topic and it is
// where frames go missing when a client cannot keep up.
struct BridgeStats {
  std::string topic;
  std::size_t clients{0};
  std::uint64_t frames_forwarded{0};
  std::uint64_t invalid{0};
  std::uint64_t ws_drops{0};
  std::uint64_t refused{0};
  std::uint64_t egress_bytes{0};
};

inline void AppendEscaped(std::string& out, std::string_view value) {
  out += '"';
  for (const char c : value) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        // Control characters have no literal form in JSON, and a topic name
        // should never contain one; emit the escape rather than invalid JSON.
        if (static_cast<unsigned char>(c) < 0x20) {
          static constexpr char kHex[] = "0123456789abcdef";
          out += "\\u00";
          out += kHex[(static_cast<unsigned char>(c) >> 4) & 0xF];
          out += kHex[static_cast<unsigned char>(c) & 0xF];
        } else {
          out += c;
        }
    }
  }
  out += '"';
}

// uint64 and int64 leave as quoted decimals; see the note at the top.
inline void AppendU64(std::string& out, std::uint64_t value) {
  out += '"';
  out += std::to_string(value);
  out += '"';
}

inline void AppendI64(std::string& out, std::int64_t value) {
  out += '"';
  out += std::to_string(value);
  out += '"';
}

// Where the document's fast values sit: the counters, heartbeats and verdicts
// that change from one refresh to the next. Everything else is the graph's
// structure (see SystemGraphDocument).
using FastSpans = std::vector<std::pair<std::size_t, std::size_t>>;

// Appends one fast value with `append`, recording where it landed.
template <class Append>
void AppendFast(std::string& out, FastSpans* fast, Append&& append) {
  const std::size_t begin = out.size();
  append();
  if (fast != nullptr) fast->emplace_back(begin, out.size());
}

inline const char* KindName(event::SourceKind kind) {
  return event::to_string(kind);
}

inline bool IsSubscriber(event::SourceKind kind) {
  return kind == event::SourceKind::WATCHER ||
         kind == event::SourceKind::FETCHER;
}

// One row per topic, gathered across every node. This is the view that answers
// the questions worth asking: is anything publishing this, is anyone reading
// it, and is the traffic moving.
struct TopicRow {
  std::string name;
  std::uint32_t message_bytes{0};
  std::vector<std::string> publishers;
  std::vector<std::string> subscribers;
  std::vector<std::string> observers;
  std::uint64_t published{0};
  std::uint64_t received{0};

  // Messages lost: the dropped counts of every end except latest-only
  // watchers (naming::kSourceFlagLatestOnly). A latest-only watcher's count
  // mixes its skips by design with its torn and slot-lost reads in one
  // counter, so none of it is loss this row can claim; its own source entry
  // still carries the whole count.
  std::uint64_t dropped{0};

  // True when any end of this topic said so, which is how naming::LintGraph
  // aggregates the same flags. A flag describes one endpoint's expectation
  // about its far side, and a topic only needs one end to make the claim: the
  // sender of `/hw/command` is the only party that knows its subscriber lives
  // across a UDP link.
  bool external{false};
  bool optional{false};
};

// What is worth flagging about a topic, in the order a person cares about it.
//
// BRIDGED and UNCONNECTED exist so this list can be honest. `/hw/command` is
// consumed by the RoboRIO over UDP and will never have a shared-memory
// subscriber, and an optional end may be absent by design. Reporting a designed
// dead-end as a fault trains people to ignore the report -- which is worse than
// not having one.
enum class TopicHealth {
  OK,
  ORPHANED,     // Subscribed, and nothing in this system publishes it.
  UNREAD,       // Published, and nothing in this system reads it.
  BRIDGED,      // The absent end is outside talOS; there is nothing to find.
  UNCONNECTED,  // The absent end is declared optional; absence is the design.
  // A non-latest-only end lost messages (TopicRow::dropped).
  LOSSY,
  IDLE,
};

inline const char* HealthName(TopicHealth health) {
  switch (health) {
    case TopicHealth::ORPHANED:
      return "orphaned";
    case TopicHealth::UNREAD:
      return "unread";
    case TopicHealth::BRIDGED:
      return "bridged";
    case TopicHealth::UNCONNECTED:
      return "unconnected";
    case TopicHealth::LOSSY:
      return "lossy";
    case TopicHealth::IDLE:
      return "idle";
    case TopicHealth::OK:
      break;
  }
  return "ok";
}

inline TopicHealth HealthOf(const TopicRow& row) {
  if (row.publishers.empty() || row.subscribers.empty()) {
    // A missing end is a fault only when both ends were meant to be here.
    // EXTERNAL is checked first because it is the more specific statement: it
    // says where the far end actually is, not merely that it may be absent.
    if (row.external) return TopicHealth::BRIDGED;
    if (row.optional) return TopicHealth::UNCONNECTED;
    return row.publishers.empty() ? TopicHealth::ORPHANED : TopicHealth::UNREAD;
  }
  if (row.dropped != 0) return TopicHealth::LOSSY;
  if (row.published == 0) return TopicHealth::IDLE;
  return TopicHealth::OK;
}

inline std::vector<TopicRow> TopicsOf(
    const std::vector<introspect::NodeSnapshot>& nodes) {
  std::map<std::string, TopicRow> rows;

  for (const introspect::NodeSnapshot& node : nodes) {
    for (const introspect::SourceSnapshot& source : node.sources) {
      // A timer is not a topic: it has a name, but nothing is on the other end.
      if (source.kind == event::SourceKind::TIMER) {
        continue;
      }

      TopicRow& row = rows[source.name];
      row.name = source.name;
      row.message_bytes = std::max(row.message_bytes, source.message_bytes);
      if (!source.latest_only()) row.dropped += source.dropped;
      row.external = row.external || source.external();
      row.optional = row.optional || source.optional();

      if (source.kind == event::SourceKind::SENDER) {
        row.publishers.push_back(node.name);
        row.published += source.events;
      } else if (source.observer()) {
        row.observers.push_back(node.name);
        row.received += source.events;
      } else {
        row.subscribers.push_back(node.name);
        row.received += source.events;
      }
    }
  }

  std::vector<TopicRow> out;
  out.reserve(rows.size());
  for (auto& [name, row] : rows) {
    out.push_back(std::move(row));
  }
  return out;
}

inline void AppendSource(std::string& out,
                         const introspect::SourceSnapshot& source,
                         FastSpans* fast = nullptr) {
  out += "{\"id\":";
  out += std::to_string(source.id);
  out += ",\"kind\":";
  AppendEscaped(out, KindName(source.kind));
  out += ",\"name\":";
  AppendEscaped(out, source.name);
  out += ",\"message_bytes\":";
  out += std::to_string(source.message_bytes);
  out += ",\"alignment\":";
  out += std::to_string(source.alignment);
  out += ",\"external\":";
  out += source.external() ? "true" : "false";
  out += ",\"optional\":";
  out += source.optional() ? "true" : "false";
  out += ",\"observer\":";
  out += source.observer() ? "true" : "false";
  // Not a version bump: a reader that predates it ignores it. Its absence
  // means an older bridge, whose topic `dropped` still counts latest-only
  // skips.
  out += ",\"latest_only\":";
  out += source.latest_only() ? "true" : "false";
  out += ",\"period_ns\":";
  AppendI64(out, source.period_ns);
  out += ",\"events\":";
  AppendFast(out, fast, [&] { AppendU64(out, source.events); });
  out += ",\"dropped\":";
  AppendFast(out, fast, [&] { AppendU64(out, source.dropped); });
  out += ",\"sequence\":";
  AppendFast(out, fast, [&] { AppendU64(out, source.sequence); });
  out += ",\"last_monotonic_ns\":";
  AppendFast(out, fast, [&] { AppendI64(out, source.last_monotonic_ns); });
  out += ",\"last_latency_ns\":";
  AppendFast(out, fast, [&] { AppendI64(out, source.last_latency_ns); });
  out += ",\"max_latency_ns\":";
  AppendFast(out, fast, [&] { AppendI64(out, source.max_latency_ns); });
  out += '}';
}

inline void AppendNames(std::string& out,
                        const std::vector<std::string>& names) {
  out += '[';
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (i != 0) out += ',';
    AppendEscaped(out, names[i]);
  }
  out += ']';
}

// One Target of a node: its policy's sources and who drives, `null` when
// nobody does. Not a version bump: a reader that predates it ignores the
// field, and one that knows it reads its absence as "no Targets", which is
// what an older bridge could say about any node.
inline void AppendSelection(std::string& out,
                            const introspect::SelectionSnapshot& selection,
                            FastSpans* fast = nullptr) {
  out += "{\"sources\":";
  AppendNames(out, selection.sources);
  out += ",\"selected\":";
  AppendFast(out, fast, [&] {
    if (const auto source = selection.selected_source()) {
      AppendEscaped(out, *source);
    } else {
      out += "null";
    }
  });
  out += '}';
}

inline void AppendNode(std::string& out, const introspect::NodeSnapshot& node,
                       std::int64_t wall_ns, std::int64_t monotonic_ns,
                       FastSpans* fast = nullptr) {
  out += "{\"name\":";
  AppendEscaped(out, node.name);
  out += ",\"target\":";
  AppendEscaped(out, node.target);
  out += ",\"slot\":";
  out += std::to_string(node.slot);
  out += ",\"pid\":";
  AppendU64(out, node.pid);
  out += ",\"session_id\":";
  AppendU64(out, node.session_id);
  out += ",\"generation\":";
  AppendU64(out, node.generation);
  out += ",\"start_wall_ns\":";
  AppendI64(out, node.start_wall_ns);
  out += ",\"heartbeat_wall_ns\":";
  // Keep the public JSON field in the wall epoch. Liveness itself is decided
  // from monotonic time in the registry, never from this display timestamp.
  AppendFast(out, fast, [&] {
    AppendI64(out, wall_ns - (monotonic_ns - node.heartbeat_monotonic_ns));
  });
  out += ",\"dispatch_count\":";
  AppendFast(out, fast, [&] { AppendU64(out, node.dispatch_count); });
  out += ",\"declared_sources\":";
  out += std::to_string(node.declared_source_count);
  out += ",\"alive\":";
  AppendFast(out, fast, [&] { out += node.alive ? "true" : "false"; });
  // What the node lost of what it should have been woken for or delivered,
  // each normally 0 (introspect::DeliveryLoss). Not a version bump: a reader
  // that predates it ignores it, and one that knows it reads its absence as
  // "not reported", what an older bridge could say.
  const introspect::DeliveryLoss loss = node.delivery_loss();
  out += ",\"delivery\":{";
  const char* separator = "";
  for (const auto& [key, value] :
       {std::pair{"wake_failures", loss.wake_failures},
        std::pair{"stalled_arms", loss.stalled_arms},
        std::pair{"wake_fallbacks", loss.wake_fallbacks},
        std::pair{"requests_skipped", loss.requests_skipped}}) {
    out += separator;
    out += '"';
    out += key;
    out += "\":";
    AppendFast(out, fast, [&] { AppendU64(out, value); });
    separator = ",";
  }
  out += '}';
  out += ",\"simulation\":";
  out += (node.flags & introspect::kFlagSimulation) ? "true" : "false";
  out += ",\"replay\":";
  out += (node.flags & introspect::kFlagReplay) ? "true" : "false";
  out += ",\"sources\":[";
  for (std::size_t i = 0; i < node.sources.size(); ++i) {
    if (i != 0) out += ',';
    AppendSource(out, node.sources[i], fast);
  }
  out += ']';
  out += ",\"selections\":[";
  for (std::size_t i = 0; i < node.selections.size(); ++i) {
    if (i != 0) out += ',';
    AppendSelection(out, node.selections[i], fast);
  }
  out += "]}";
}

inline void AppendTopic(std::string& out, const TopicRow& row,
                        FastSpans* fast = nullptr) {
  out += "{\"name\":";
  AppendEscaped(out, row.name);
  out += ",\"message_bytes\":";
  out += std::to_string(row.message_bytes);
  out += ",\"publishers\":";
  AppendNames(out, row.publishers);
  out += ",\"subscribers\":";
  AppendNames(out, row.subscribers);
  out += ",\"observers\":";
  AppendNames(out, row.observers);
  out += ",\"published\":";
  AppendFast(out, fast, [&] { AppendU64(out, row.published); });
  out += ",\"received\":";
  AppendFast(out, fast, [&] { AppendU64(out, row.received); });
  out += ",\"dropped\":";
  AppendFast(out, fast, [&] { AppendU64(out, row.dropped); });
  out += ",\"external\":";
  out += row.external ? "true" : "false";
  out += ",\"optional\":";
  out += row.optional ? "true" : "false";
  // Derived, and sent anyway: every consumer -- viewer, agent, `curl` -- would
  // otherwise reimplement the ordering, and a second implementation is a second
  // chance to call a bridged topic broken.
  out += ",\"health\":";
  AppendFast(out, fast, [&] { AppendEscaped(out, HealthName(HealthOf(row))); });
  out += '}';
}

struct SystemGraphInput {
  std::vector<introspect::NodeSnapshot> nodes;
  BridgeStats bridge;
  bool registry_available{false};
  std::uint32_t node_capacity{0};
  std::uint32_t source_capacity{0};
  std::int64_t wall_ns{0};
  std::int64_t monotonic_ns{0};
};

// One refresh of the system graph, split so a client can be kept current
// without being re-sent the whole document. `structure` identifies everything
// but the fast values (names, endpoints, pids, capacities: what changes only
// when a node starts, stops or restarts); `values` is the fast values (wall
// time, counters, heartbeats, liveness, selected sources, topic verdicts) in
// document order, as a JSON array of the same JSON values the document holds.
// A client holding a document of the same structure becomes current by
// writing `values` over its fast fields in that order.
struct SystemGraphDocument {
  std::string json;       // the whole document, with its "structure" id
  std::string structure;  // 16 hex digits
  std::string values;     // "[...]"
};

inline SystemGraphDocument BuildSystemGraph(const SystemGraphInput& input) {
  std::string out;
  out.reserve(4096);
  FastSpans fast;

  out += "{\"kind\":\"talos.system_graph\",\"version\":";
  out += std::to_string(kSystemGraphVersion);
  out += ",\"wall_ns\":";
  AppendFast(out, &fast, [&] { AppendI64(out, input.wall_ns); });

  out += ",\"registry\":{\"available\":";
  out += input.registry_available ? "true" : "false";
  out += ",\"node_capacity\":";
  out += std::to_string(input.node_capacity);
  out += ",\"source_capacity\":";
  out += std::to_string(input.source_capacity);
  out += ",\"liveness_timeout_ns\":";
  AppendI64(out, std::chrono::duration_cast<std::chrono::nanoseconds>(
                     introspect::kLivenessTimeout)
                     .count());
  out += '}';

  out += ",\"bridge\":{\"topic\":";
  AppendEscaped(out, input.bridge.topic);
  out += ",\"clients\":";
  AppendFast(out, &fast, [&] { out += std::to_string(input.bridge.clients); });
  out += ",\"frames_forwarded\":";
  AppendFast(out, &fast,
             [&] { AppendU64(out, input.bridge.frames_forwarded); });
  out += ",\"invalid\":";
  AppendFast(out, &fast, [&] { AppendU64(out, input.bridge.invalid); });
  out += ",\"ws_drops\":";
  AppendFast(out, &fast, [&] { AppendU64(out, input.bridge.ws_drops); });
  out += ",\"refused\":";
  AppendFast(out, &fast, [&] { AppendU64(out, input.bridge.refused); });
  out += ",\"egress_bytes\":";
  AppendFast(out, &fast, [&] { AppendU64(out, input.bridge.egress_bytes); });
  out += '}';

  out += ",\"nodes\":[";
  for (std::size_t i = 0; i < input.nodes.size(); ++i) {
    if (i != 0) out += ',';
    AppendNode(out, input.nodes[i], input.wall_ns, input.monotonic_ns, &fast);
  }
  out += ']';

  const std::vector<TopicRow> topics = TopicsOf(input.nodes);
  out += ",\"topics\":[";
  for (std::size_t i = 0; i < topics.size(); ++i) {
    if (i != 0) out += ',';
    AppendTopic(out, topics[i], &fast);
  }
  out += ']';

  // FNV-1a 64 over the document with every fast value cut out; a marker
  // byte stands in for each, so moving a value cannot alias its neighbours.
  std::uint64_t hash = 14695981039346656037ull;
  const auto mix = [&hash](unsigned char byte) {
    hash = (hash ^ byte) * 1099511628211ull;
  };
  SystemGraphDocument document;
  document.values = "[";
  std::size_t at = 0;
  for (const auto& [begin, end] : fast) {
    for (; at < begin; ++at) mix(static_cast<unsigned char>(out[at]));
    mix(0xff);
    if (document.values.size() > 1) document.values += ',';
    document.values.append(out, begin, end - begin);
    at = end;
  }
  for (; at < out.size(); ++at) mix(static_cast<unsigned char>(out[at]));
  document.values += ']';

  char id[17];
  std::snprintf(id, sizeof(id), "%016llx",
                static_cast<unsigned long long>(hash));
  document.structure = id;
  // Not a version bump: a reader that predates it ignores it.
  out += ",\"structure\":\"";
  out += document.structure;
  out += "\"}";
  document.json = std::move(out);
  return document;
}

inline std::string SystemGraphJson(const SystemGraphInput& input) {
  return BuildSystemGraph(input).json;
}

// What one client has of the system graph, so it is sent the whole document
// only on connect and when the structure changes, and otherwise only the fast
// values, at most every kUpdateNs and only when they changed. A whole
// document goes at most every kResendNs after the first: a node restarting
// faster than that (a crash loop) would otherwise keep the client's budget
// on resending the whole graph; the structure changes in between coalesce
// into the next one. A message the budget cannot afford waits and coalesces:
// the next one carries the latest values. While the client's graph is behind
// and it has heard nothing for kNoticeNs, a small notice tells it its graph
// is deferred, not lost, so it does not mark it stale.
class GraphFeed {
 public:
  static constexpr std::int64_t kUpdateNs = 500'000'000;
  static constexpr std::int64_t kResendNs = 2'000'000'000;
  // Past the update interval, so an update only briefly held (the usual case
  // beside telemetry) arrives without a notice; under Studio's one-second
  // GRAPH_STALE_AFTER_MS, so a held graph never reads as stale.
  static constexpr std::int64_t kNoticeNs = 750'000'000;

  // `afford(bytes)` says whether a message of that many payload bytes may be
  // sent now; `send(text)` sends one and returns false if it was not sent.
  // A message over `max_bytes` can never be afforded: it is not sent, the
  // client gets no notice, and its graph goes stale, which is the truth.
  // Returns true while a message waits for budget, so the caller holds the
  // client's lower-priority traffic and the budget refills for this one.
  template <class Afford, class Send>
  bool Pump(const SystemGraphDocument& document, std::int64_t now_ns,
            std::size_t max_bytes, Afford&& afford, Send&& send) {
    if (document.json.empty()) return false;
    std::string update;
    std::string_view message;
    const bool whole = structure_ != document.structure;
    if (whole) {
      if (!structure_.empty() && now_ns - whole_ns_ < kResendNs) {
        Notice(now_ns, afford, send);
        return false;
      }
      message = document.json;
    } else if (values_ != document.values &&
               now_ns - updated_ns_ >= kUpdateNs) {
      update = "{\"kind\":\"talos.system_graph.update\",\"structure\":\"";
      update += document.structure;
      update += "\",\"values\":";
      update += document.values;
      update += '}';
      message = update;
    } else {
      return false;
    }
    if (message.size() > max_bytes) return false;
    if (afford(message.size())) {
      if (!send(message)) {
        // Whatever the client holds now is unknown: start it over.
        structure_.clear();
        return false;
      }
      structure_ = document.structure;
      values_ = document.values;
      updated_ns_ = sent_ns_ = now_ns;
      if (whole) whole_ns_ = now_ns;
      return false;
    }
    Notice(now_ns, afford, send);
    return true;
  }

  // The earliest instant Pump would send something `document` holds that it
  // has not, once a waiting interval ends (whole resend, update, notice);
  // kNeverNs when the client holds it. A message deferred only for budget
  // is due when the budget refills, which the caller tracks.
  std::int64_t NextDueNs(const SystemGraphDocument& document) const {
    if (document.json.empty() || structure_.empty()) return kNeverNs;
    const bool whole = structure_ != document.structure;
    if (!whole && values_ == document.values) return kNeverNs;
    return std::min(whole ? whole_ns_ + kResendNs : updated_ns_ + kUpdateNs,
                    sent_ns_ + kNoticeNs);
  }

 private:
  template <class Afford, class Send>
  void Notice(std::int64_t now_ns, Afford& afford, Send& send) {
    if (structure_.empty() || now_ns - sent_ns_ < kNoticeNs) return;
    const std::string notice =
        "{\"kind\":\"talos.system_graph.deferred\",\"structure\":\"" +
        structure_ + "\"}";
    if (afford(notice.size()) && send(notice)) sent_ns_ = now_ns;
  }

  std::string structure_;  // empty until the client holds a document
  std::string values_;
  std::int64_t updated_ns_{0};
  std::int64_t sent_ns_{0};
  std::int64_t whole_ns_{0};
};

}  // namespace studio
