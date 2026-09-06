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
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/introspection/names.h"
#include "talOS/introspection/registry.h"

namespace studio {

namespace event = talos::event;
namespace introspect = talos::introspect;
namespace naming = talos::introspect::naming;

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
  std::uint64_t udp_drops{0};
  std::uint64_t ws_drops{0};
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
  std::uint64_t published{0};
  std::uint64_t received{0};
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
// subscriber; `/drivetrain/target/auto` has no publisher because autonomous is
// not written yet. Both used to land in ORPHANED/UNREAD beside real faults, and
// reporting a designed dead-end as a fault trains people to ignore the report
// -- which is worse than not having one. The five original names still mean
// exactly what they meant, so a viewer's existing filters keep working.
enum class TopicHealth {
  OK,
  ORPHANED,     // Subscribed, and nothing in this system publishes it.
  UNREAD,       // Published, and nothing in this system reads it.
  BRIDGED,      // The absent end is outside talOS; there is nothing to find.
  UNCONNECTED,  // The absent end is declared optional; absence is the design.
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
      row.dropped += source.dropped;
      row.external = row.external || source.external();
      row.optional = row.optional || source.optional();

      if (source.kind == event::SourceKind::SENDER) {
        row.publishers.push_back(node.name);
        row.published += source.events;
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
                         const introspect::SourceSnapshot& source) {
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
  out += ",\"period_ns\":";
  AppendI64(out, source.period_ns);
  out += ",\"events\":";
  AppendU64(out, source.events);
  out += ",\"dropped\":";
  AppendU64(out, source.dropped);
  out += ",\"sequence\":";
  AppendU64(out, source.sequence);
  out += ",\"last_monotonic_ns\":";
  AppendI64(out, source.last_monotonic_ns);
  out += ",\"last_latency_ns\":";
  AppendI64(out, source.last_latency_ns);
  out += ",\"max_latency_ns\":";
  AppendI64(out, source.max_latency_ns);
  out += '}';
}

inline void AppendNode(std::string& out, const introspect::NodeSnapshot& node) {
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
  AppendI64(out, node.heartbeat_wall_ns);
  out += ",\"dispatch_count\":";
  AppendU64(out, node.dispatch_count);
  out += ",\"declared_sources\":";
  out += std::to_string(node.declared_source_count);
  out += ",\"alive\":";
  out += node.alive ? "true" : "false";
  out += ",\"simulation\":";
  out += (node.flags & introspect::kFlagSimulation) ? "true" : "false";
  out += ",\"replay\":";
  out += (node.flags & introspect::kFlagReplay) ? "true" : "false";
  out += ",\"sources\":[";
  for (std::size_t i = 0; i < node.sources.size(); ++i) {
    if (i != 0) out += ',';
    AppendSource(out, node.sources[i]);
  }
  out += "]}";
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

inline void AppendTopic(std::string& out, const TopicRow& row) {
  out += "{\"name\":";
  AppendEscaped(out, row.name);
  out += ",\"message_bytes\":";
  out += std::to_string(row.message_bytes);
  out += ",\"publishers\":";
  AppendNames(out, row.publishers);
  out += ",\"subscribers\":";
  AppendNames(out, row.subscribers);
  out += ",\"published\":";
  AppendU64(out, row.published);
  out += ",\"received\":";
  AppendU64(out, row.received);
  out += ",\"dropped\":";
  AppendU64(out, row.dropped);
  out += ",\"external\":";
  out += row.external ? "true" : "false";
  out += ",\"optional\":";
  out += row.optional ? "true" : "false";
  // Derived, and sent anyway: every consumer -- viewer, agent, `curl` -- would
  // otherwise reimplement the ordering, and a second implementation is a second
  // chance to call a bridged topic broken.
  out += ",\"health\":";
  AppendEscaped(out, HealthName(HealthOf(row)));
  out += '}';
}

struct SystemGraphInput {
  std::vector<introspect::NodeSnapshot> nodes;
  BridgeStats bridge;
  bool registry_available{false};
  std::uint32_t node_capacity{0};
  std::uint32_t source_capacity{0};
  std::int64_t wall_ns{0};
};

inline std::string SystemGraphJson(const SystemGraphInput& input) {
  std::string out;
  out.reserve(4096);

  out += "{\"kind\":\"talos.system_graph\",\"version\":";
  out += std::to_string(kSystemGraphVersion);
  out += ",\"wall_ns\":";
  AppendI64(out, input.wall_ns);

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
  out += std::to_string(input.bridge.clients);
  out += ",\"frames_forwarded\":";
  AppendU64(out, input.bridge.frames_forwarded);
  out += ",\"invalid\":";
  AppendU64(out, input.bridge.invalid);
  out += ",\"udp_drops\":";
  AppendU64(out, input.bridge.udp_drops);
  out += ",\"ws_drops\":";
  AppendU64(out, input.bridge.ws_drops);
  out += '}';

  out += ",\"nodes\":[";
  for (std::size_t i = 0; i < input.nodes.size(); ++i) {
    if (i != 0) out += ',';
    AppendNode(out, input.nodes[i]);
  }
  out += ']';

  const std::vector<TopicRow> topics = TopicsOf(input.nodes);
  out += ",\"topics\":[";
  for (std::size_t i = 0; i < topics.size(); ++i) {
    if (i != 0) out += ',';
    AppendTopic(out, topics[i]);
  }
  out += "]}";

  return out;
}

// --- datagram framing ------------------------------------------------------
//
// The UDP path carries both message families, so a datagram has to say which
// one it is. Telemetry frames start with a FlatBuffers size prefix, so system
// graphs get an explicit tag instead of relying on the shape of the bytes.
//
// It also has to be chunked. macOS caps a UDP datagram at net.inet.udp.maxdgram
// (9216 bytes by default, and raising it needs root), while a graph for a real
// robot runs to tens of kilobytes -- seven nodes already produce 11 KB, and the
// endpoint attributes added since cost roughly 60 bytes per source. Sending it
// whole fails with EMSGSIZE, which is silent from the desktop app's side: the
// System tab would simply stay empty forever while the WebSocket client worked
// fine. So the document is split, and the receiver reassembles.
inline constexpr std::string_view kSystemDatagramTag = "TSYS";

// tag(4) + document id(2, little-endian) + chunk index(1) + chunk count(1).
inline constexpr std::size_t kSystemHeaderBytes = 8;

// Comfortably inside the 9216-byte cap with room for the header and for a
// smaller limit on some other host.
inline constexpr std::size_t kSystemChunkBytes = 8000;

// One byte of header addresses the chunk index, so this is a hard ceiling, not
// a tuning knob. A registry filled to its own capacity -- 32 nodes of 64
// sources, longest names, saturated counters -- produces 1.4 MB and 180 chunks,
// which system_test asserts on and prints: the margin is real but it is not
// large, and widening a row is now a decision that has to look at that number.
inline constexpr std::size_t kMaxSystemChunks = 255;

// Splits `json` into datagrams. Returns empty when the document needs more
// chunks than the framing can address, which the registry's own capacities put
// far out of reach: reporting nothing is better than sending a document the
// receiver would silently assemble wrong.
inline std::vector<std::string> SystemDatagrams(std::string_view json,
                                                std::uint16_t document_id) {
  const std::size_t chunks =
      json.empty() ? 1
                   : (json.size() + kSystemChunkBytes - 1) / kSystemChunkBytes;
  if (chunks > kMaxSystemChunks) {
    return {};
  }

  std::vector<std::string> out;
  out.reserve(chunks);
  for (std::size_t i = 0; i < chunks; ++i) {
    std::string datagram;
    datagram.reserve(kSystemHeaderBytes + kSystemChunkBytes);
    datagram += kSystemDatagramTag;
    datagram += static_cast<char>(document_id & 0xFF);
    datagram += static_cast<char>((document_id >> 8) & 0xFF);
    datagram += static_cast<char>(i);
    datagram += static_cast<char>(chunks);
    datagram += json.substr(i * kSystemChunkBytes, kSystemChunkBytes);
    out.push_back(std::move(datagram));
  }
  return out;
}

}  // namespace studio
