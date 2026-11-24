#pragma once

/*
 * What a node's shape is, before the node runs.
 *
 * The live registry answers "what is running". This answers the question one
 * step earlier: "what would run, and would it wire up". Every node accepts
 * `--describe`, builds itself on a simulated loop -- which touches no shared
 * memory, no hardware and no network -- prints the manifest its constructor
 * registered, and exits. The launcher runs that against every binary in the
 * config before it spawns anything, assembles the declared graph, and refuses
 * to start a robot whose topics do not meet.
 *
 * The point is that this cannot drift from the code. A second declaration of
 * the topology -- a YAML file, a comment, a diagram -- is a thing to forget to
 * update, and the failure it causes is the one it was written to prevent. Here
 * the answer comes from the same constructor that builds the real loop, so
 * "declared" and "actual" differ only when the node behaves differently, which
 * is the interesting case rather than a bookkeeping error.
 *
 * "Touches nothing external" means no socket, no shared-memory segment and no
 * hardware. It does not mean no filesystem: the hardware bridge's topology is
 * derived from `robot.toml` -- one request topic per subsystem -- so describing
 * it requires reading the config, which is why the launcher passes `--config`
 * to a probe exactly as it does to a real run. A node whose topology is fixed
 * in its constructor needs no file, and most are.
 *
 * The format is JSON so that `--describe` is useful on its own -- pipe it to
 * `jq`, hand it to an agent -- and the reader below is a purpose-built parser
 * for exactly this shape rather than a general one. That is a deliberate
 * trade: pulling in a JSON library for a self-describing round trip is not
 * worth the dependency, and `describe_test` proves the pair agrees.
 */

#include <cctype>
#include <charconv>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/introspection/names.h"

namespace talos::introspect {

inline constexpr int kDescribeVersion = 1;

// One parameter a node read while it was built (Node::param, or param_of
// for another subsystem's table): `key` as the node named it
// ("geometry.modules[0].x_m") in `subsystem`'s table.
struct ParamRead {
  std::string subsystem;
  std::string key;
  bool operator==(const ParamRead&) const = default;
};

// A node as `--describe` reports it: identity, plus every end it will hold.
struct Description {
  std::string name;
  std::string target;
  std::vector<naming::SourceShape> sources;

  // Sources the loop registered, plus anything the node owns outside it. Both
  // reach the registry, so both belong here or the declared graph would flag a
  // topic the running graph shows as fine.

  // The node's constructor asked for the declared graph (NodeAccess::
  // declared_graph): its topics depend on the other nodes', so the launcher
  // describes it again with the graph in hand (see DeclaredGraph below).
  bool reads_graph = false;

  // The binary may be started with process::kHeldStopSignals blocked: its
  // run calls process::InstallStopHandlers, which unblocks them, before
  // anything that can wait. A stop sent before then stays pending instead of
  // meeting the runtime's own handling (CPython's KeyboardInterrupt) or its
  // default action, and is taken once the handlers are in. False for a
  // binary that does not say so.
  bool holds_stop = false;

  // The node's update() is a callback through the C ABI
  // (talos_node_set_update): a Python node's, whose every tick runs the
  // interpreter. False for a C++ update(), a talos.cpp_node's included, for a
  // node without one, and for a binary that does not say so. The launcher
  // holds such a node to a slower period when it owns actuators
  // (LintPythonActuatorPeriods).
  bool abi_update = false;

  // Every parameter key the node's constructor read, in first-read order:
  // what the node declares it reads, so the launcher can refuse a subsystem
  // key no node reads. nullopt when the describe did not report them (a
  // hand-written describe, or one from a binary built before the field), which
  // the launcher cannot judge.
  std::optional<std::vector<ParamRead>> params;
};

// The declared graph a framework node reads (telemetry observes every state
// and status topic in it, teleop finds each lane's schema in it): the
// descriptions of every node in the session that does not read the graph
// itself, plus the Pass-1 static non-`telemetry`-role `SENDER` endpoints of
// other graph-reading nodes (each reader strips its own entry when loading
// `--graph`, so a reader never sees itself), schema bytes included. The
// launcher writes it as DeclaredGraphToJson and passes its path as `--graph`;
// testing::Sim builds the same view from the nodes added so far.
using DeclaredGraph = std::vector<Description>;

inline constexpr std::string_view kBase64Alphabet =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// RFC 4648 base64 with padding: how describe carries a schema's bfbs bytes.
inline std::string Base64(std::string_view bytes) {
  std::string out;
  out.reserve((bytes.size() + 2) / 3 * 4);
  std::size_t i = 0;
  for (; i + 2 < bytes.size(); i += 3) {
    const std::uint32_t n = (static_cast<std::uint8_t>(bytes[i]) << 16) |
                            (static_cast<std::uint8_t>(bytes[i + 1]) << 8) |
                            static_cast<std::uint8_t>(bytes[i + 2]);
    out += kBase64Alphabet[(n >> 18) & 63];
    out += kBase64Alphabet[(n >> 12) & 63];
    out += kBase64Alphabet[(n >> 6) & 63];
    out += kBase64Alphabet[n & 63];
  }
  if (i < bytes.size()) {
    std::uint32_t n = static_cast<std::uint8_t>(bytes[i]) << 16;
    if (i + 1 < bytes.size()) n |= static_cast<std::uint8_t>(bytes[i + 1]) << 8;
    out += kBase64Alphabet[(n >> 18) & 63];
    out += kBase64Alphabet[(n >> 12) & 63];
    out += i + 1 < bytes.size() ? kBase64Alphabet[(n >> 6) & 63] : '=';
    out += '=';
  }
  return out;
}

// The inverse of Base64; false on anything it did not write.
inline bool FromBase64(std::string_view text, std::string& out) {
  out.clear();
  if (text.size() % 4 != 0) return false;
  out.reserve(text.size() / 4 * 3);
  for (std::size_t i = 0; i < text.size(); i += 4) {
    std::uint32_t n = 0;
    int padding = 0;
    for (std::size_t j = 0; j < 4; ++j) {
      const char c = text[i + j];
      std::uint32_t value = 0;
      if (c == '=' && i + 4 == text.size() && j >= 2) {
        ++padding;
      } else {
        if (padding != 0) return false;
        const std::size_t at = kBase64Alphabet.find(c);
        if (at == std::string_view::npos) return false;
        value = static_cast<std::uint32_t>(at);
      }
      n = (n << 6) | value;
    }
    out += static_cast<char>((n >> 16) & 0xFF);
    if (padding < 2) out += static_cast<char>((n >> 8) & 0xFF);
    if (padding < 1) out += static_cast<char>(n & 0xFF);
  }
  return true;
}

namespace detail {

// Length of a well-formed RFC 3629 UTF-8 sequence starting at `text[0]` (a
// non-ASCII byte), or 0 if invalid.
inline std::size_t Utf8Length(std::string_view text) {
  if (text.empty()) return 0;
  const auto at = [&](std::size_t i) {
    return static_cast<unsigned char>(text[i]);
  };
  const unsigned char lead = at(0);
  std::size_t length = 0;
  unsigned char low = 0x80;
  unsigned char high = 0xBF;
  if (lead >= 0xC2 && lead <= 0xDF) {
    length = 2;
  } else if (lead >= 0xE0 && lead <= 0xEF) {
    length = 3;
    if (lead == 0xE0) low = 0xA0;
    if (lead == 0xED) high = 0x9F;
  } else if (lead >= 0xF0 && lead <= 0xF4) {
    length = 4;
    if (lead == 0xF0) low = 0x90;
    if (lead == 0xF4) high = 0x8F;
  } else {
    return 0;
  }
  if (text.size() < length) return 0;
  if (at(1) < low || at(1) > high) return 0;
  for (std::size_t i = 2; i < length; ++i) {
    if (at(i) < 0x80 || at(i) > 0xBF) return 0;
  }
  return length;
}

}  // namespace detail

inline void AppendJsonString(std::string& out, std::string_view value) {
  static constexpr char kHex[] = "0123456789abcdef";
  const auto escape = [&](unsigned char c) {
    out += "\\u00";
    out += kHex[(c >> 4) & 0xF];
    out += kHex[c & 0xF];
  };
  out += '"';
  for (std::size_t i = 0; i < value.size();) {
    const auto c = static_cast<unsigned char>(value[i]);
    if (c < 0x80) {
      if (c == '"' || c == '\\') {
        out += '\\';
        out += static_cast<char>(c);
      } else if (c < 0x20) {
        escape(c);
      } else {
        out += static_cast<char>(c);
      }
      ++i;
      continue;
    }
    const std::size_t length = detail::Utf8Length(value.substr(i));
    if (length == 0) {
      escape(c);
      ++i;
    } else {
      out += value.substr(i, length);
      i += length;
    }
  }
  out += '"';
}

// "0x" and 16 lowercase hex digits: how describe writes a schema hash.
inline std::string HexHash(std::uint64_t hash) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out = "0x";
  for (int shift = 60; shift >= 0; shift -= 4)
    out += kHex[(hash >> shift) & 0xF];
  return out;
}

inline std::string DescribeToJson(const Description& description) {
  std::string out;
  out += "{\n  \"describe_version\": " + std::to_string(kDescribeVersion) +
         ",\n  \"name\": ";
  AppendJsonString(out, description.name);
  out += ",\n  \"target\": ";
  AppendJsonString(out, description.target);
  out += ",\n  \"reads_graph\": ";
  out += description.reads_graph ? "true" : "false";
  out += ",\n  \"holds_stop\": ";
  out += description.holds_stop ? "true" : "false";
  out += ",\n  \"abi_update\": ";
  out += description.abi_update ? "true" : "false";
  // Before the sources, whose reader looks only past them.
  if (description.params) {
    out += ",\n  \"params\": [";
    for (std::size_t i = 0; i < description.params->size(); ++i) {
      const ParamRead& read = (*description.params)[i];
      out += i ? ",\n    {" : "\n    {";
      out += "\"subsystem\": ";
      AppendJsonString(out, read.subsystem);
      out += ", \"key\": ";
      AppendJsonString(out, read.key);
      out += "}";
    }
    out += description.params->empty() ? "]" : "\n  ]";
  }
  out += ",\n  \"sources\": [";
  for (std::size_t i = 0; i < description.sources.size(); ++i) {
    const auto& source = description.sources[i];
    out += i ? ",\n    {" : "\n    {";
    out += "\"kind\": ";
    AppendJsonString(out, event::to_string(source.kind));
    out += ", \"name\": ";
    AppendJsonString(out, source.name);
    out += ", \"message_bytes\": " + std::to_string(source.message_bytes);
    out += ", \"external\": ";
    out += source.external() ? "true" : "false";
    out += ", \"optional\": ";
    out += source.optional() ? "true" : "false";
    out += ", \"observer\": ";
    out += source.observer() ? "true" : "false";
    // Only on the devices layer's request publisher; absent means false.
    if ((source.flags & naming::kSourceFlagHardwareRequest) != 0)
      out += ", \"hardware_request\": true";
    // Only on a request publisher that lifted the live-target gate.
    if ((source.flags & naming::kSourceFlagUntargetedOutputs) != 0)
      out += ", \"untargeted_outputs\": true";
    // Only on a target-lane publisher opened with OperatorArming; absent means
    // unarmed.
    if ((source.flags & naming::kSourceFlagOperatorArming) != 0)
      out += ", \"operator_arming\": true";
    // Only on `/hw/state/driver_station` subscribed solely by internal
    // Target/TargetLane/OperatorArming plumbing; absent means explicit reader.
    if ((source.flags & naming::kSourceFlagInternalDriverStation) != 0)
      out += ", \"internal_driver_station\": true";
    // Only on a latest-only watcher; absent means a sequence watcher.
    if ((source.flags & naming::kSourceFlagLatestOnly) != 0)
      out += ", \"latest_only\": true";
    if (source.kind != event::SourceKind::TIMER) {
      // The hash as a hex string: a JSON number is a double.
      out += ", \"schema\": ";
      AppendJsonString(out, source.schema);
      out += ", \"schema_hash\": \"" + HexHash(source.schema_hash) + "\"";
      out += ", \"alignment\": " + std::to_string(source.alignment);
      out += ", \"slots\": " + std::to_string(source.slots);
      // Last, so the short fields stay readable at the start of the line.
      out += ", \"bfbs\": \"" + Base64(source.bfbs) + "\"";
    }
    out += "}";
  }
  out += description.sources.empty() ? "]\n}\n" : "\n  ]\n}\n";
  return out;
}

// The declared graph as the launcher hands it to a graph-reading node: a JSON
// array of describe documents.
inline std::string DeclaredGraphToJson(const DeclaredGraph& graph) {
  std::string out = "[\n";
  for (std::size_t i = 0; i < graph.size(); ++i) {
    if (i != 0) out += ",\n";
    out += DescribeToJson(graph[i]);
  }
  out += "]\n";
  return out;
}

// Builds a description from a loop's manifest, whose topics are rings of
// `slots` slots. Endpoint attributes are not something the loop knows --
// `/hw/command` looks like any other sender to it -- so the node supplies
// them by topic name. `source_flags`, indexed by registration id, carries
// what only the loop knows (LoopSourceFlags); shorter than the manifest
// means none for the rest.
struct EndpointAttribute {
  std::string_view topic;
  std::uint32_t flags{0};
  // When set, applies only to the manifest registration with this source id
  // rather than every source sharing `topic`. Leaves topic-keyed callers
  // (nullopt) unchanged while preventing flag smearing across distinct
  // endpoints on the same topic.
  std::optional<std::uint32_t> source_id{std::nullopt};
};

// What only `loop` knows about its own sources, indexed by source id:
// naming::kSourceFlagLatestOnly on each watcher that delivers only the newest
// message (event::EventLoopBase::latest_only). The Reporter publishes it to
// the registry and `--describe` writes it, so the launcher can tell a watcher
// that logs at most one message per drain from one that logs every message.
template <class Loop>
std::vector<std::uint32_t> LoopSourceFlags(const Loop& loop) {
  std::vector<std::uint32_t> flags(loop.manifest().size(), 0u);
  for (std::size_t id = 0; id < flags.size(); ++id) {
    if (loop.latest_only(static_cast<std::uint16_t>(id))) {
      flags[id] = naming::kSourceFlagLatestOnly;
    }
  }
  return flags;
}

inline Description DescribeManifest(
    std::string name, std::string target, const event::Manifest& manifest,
    std::uint32_t slots, const std::vector<EndpointAttribute>& attributes = {},
    const std::vector<std::uint32_t>& source_flags = {}) {
  Description description{std::move(name), std::move(target), {}};
  for (const auto& registration : manifest) {
    std::uint32_t flags = 0;
    for (const auto& attribute : attributes) {
      if (attribute.source_id.has_value()) {
        if (*attribute.source_id == registration.id) flags |= attribute.flags;
      } else if (attribute.topic == registration.name) {
        flags |= attribute.flags;
      }
    }
    if (registration.id < source_flags.size()) {
      flags |= source_flags[registration.id];
    }
    naming::SourceShape source{registration.kind, registration.name,
                               registration.message_bytes, flags};
    if (registration.kind != event::SourceKind::TIMER) {
      source.schema = registration.schema.name;
      source.schema_hash = registration.schema.hash;
      source.bfbs = registration.schema.bfbs;
      source.alignment = registration.alignment;
      source.slots = slots;
    }
    description.sources.push_back(std::move(source));
  }
  return description;
}

// Reads what DescribeToJson wrote.
//
// Scans for the fields it needs rather than parsing a general document: the
// only producer is the function above, and a describe output that does not
// contain what this looks for is a bug in the pair, not an input to tolerate.
// Returns false and sets `error` rather than throwing, because the launcher's
// caller is a child process whose output may be anything at all -- a stack
// trace, a usage message, nothing.
class DescribeReader {
 public:
  static bool Parse(std::string_view text, Description& out,
                    std::string& error) {
    out = Description{};
    std::size_t at = 0;

    if (!Field(text, "describe_version", at)) {
      error = "no describe_version field; the binary did not answer --describe";
      return false;
    }
    std::int64_t version = 0;
    if (!Number(text, at, version)) {
      error = "invalid describe_version";
      return false;
    }
    if (version != kDescribeVersion) {
      error = "describe_version " + std::to_string(version) + ", expected " +
              std::to_string(kDescribeVersion);
      return false;
    }
    if (!Field(text, "name", at) || !String(text, at, out.name)) {
      error = "no name field";
      return false;
    }
    if (!Field(text, "target", at) || !String(text, at, out.target)) {
      error = "no target field";
      return false;
    }
    const std::size_t header_end = at;
    if (!Field(text, "sources", at)) {
      error = "no sources field";
      return false;
    }
    // Optional, so a hand-written describe (the Studio bridge's) need not
    // say it; looked up only between the target and the sources.
    {
      const std::string_view header = text.substr(header_end, at - header_end);
      std::size_t flag = 0;
      out.reads_graph =
          Field(header, "reads_graph", flag) && Boolean(header, flag);
      flag = 0;
      out.holds_stop =
          Field(header, "holds_stop", flag) && Boolean(header, flag);
      flag = 0;
      out.abi_update =
          Field(header, "abi_update", flag) && Boolean(header, flag);
      std::size_t params = 0;
      if (Field(header, "params", params) &&
          !Params(header, params, out.params.emplace(), error)) {
        return false;
      }
    }

    // Each source is parsed inside its own object.
    //
    // An earlier version searched forward from a running cursor for each key,
    // which meant an entry missing its "name" quietly took the *next* entry's
    // name and reported success -- exactly the class of silent misread this
    // whole mechanism exists to avoid. Bounding every lookup to one object's
    // braces is what makes a missing field a missing field. The objects have no
    // nested braces by construction, which is why finding the next '}' is
    // enough.
    const std::size_t array_begin = FindOutsideString(text, at, '[');
    if (array_begin == std::string_view::npos) {
      error = "sources is not an array";
      return false;
    }
    const std::size_t array_end = FindOutsideString(text, array_begin + 1, ']');
    if (array_end == std::string_view::npos) {
      error = "sources array is not terminated";
      return false;
    }

    std::size_t cursor = array_begin + 1;
    for (;;) {
      const std::size_t open = FindOutsideString(text, cursor, '{');
      if (open == std::string_view::npos || open >= array_end) break;
      const std::size_t close = FindOutsideString(text, open + 1, '}');
      if (close == std::string_view::npos || close >= array_end) {
        error = "source object is not terminated";
        return false;
      }
      const std::string_view entry = text.substr(open, close - open + 1);
      cursor = close + 1;

      naming::SourceShape source;
      std::size_t field = 0;
      std::string kind;
      if (!Field(entry, "kind", field) || !String(entry, field, kind)) {
        error = "source has no kind";
        return false;
      }
      if (!KindFromString(kind, source.kind)) {
        error = "unknown source kind '" + kind + "'";
        return false;
      }
      if (!Field(entry, "name", field) || !String(entry, field, source.name)) {
        error = "source of kind " + kind + " has no name";
        return false;
      }
      if (!Field(entry, "message_bytes", field)) {
        error = "source '" + source.name + "' has no message_bytes";
        return false;
      }
      if (!Uint32(entry, field, source.message_bytes)) {
        error = "source '" + source.name + "' has invalid message_bytes";
        return false;
      }

      // Absent attributes mean "neither".
      std::size_t attribute = 0;
      if (Field(entry, "external", attribute) && Boolean(entry, attribute))
        source.flags |= naming::kSourceFlagExternal;
      attribute = 0;
      if (Field(entry, "optional", attribute) && Boolean(entry, attribute))
        source.flags |= naming::kSourceFlagOptional;
      attribute = 0;
      if (Field(entry, "observer", attribute) && Boolean(entry, attribute))
        source.flags |= naming::kSourceFlagObserver;
      attribute = 0;
      if (Field(entry, "hardware_request", attribute) &&
          Boolean(entry, attribute))
        source.flags |= naming::kSourceFlagHardwareRequest;
      attribute = 0;
      if (Field(entry, "untargeted_outputs", attribute) &&
          Boolean(entry, attribute))
        source.flags |= naming::kSourceFlagUntargetedOutputs;
      attribute = 0;
      if (Field(entry, "operator_arming", attribute) &&
          Boolean(entry, attribute))
        source.flags |= naming::kSourceFlagOperatorArming;
      attribute = 0;
      if (Field(entry, "internal_driver_station", attribute) &&
          Boolean(entry, attribute))
        source.flags |= naming::kSourceFlagInternalDriverStation;
      attribute = 0;
      if (Field(entry, "latest_only", attribute) && Boolean(entry, attribute))
        source.flags |= naming::kSourceFlagLatestOnly;

      // Every topic end names its schema and ring layout: the launcher
      // creates the segment from them.
      if (source.kind != event::SourceKind::TIMER) {
        std::size_t at_schema = 0;
        if (!Field(entry, "schema", at_schema) ||
            !String(entry, at_schema, source.schema) || source.schema.empty()) {
          error = "source '" + source.name + "' has no schema";
          return false;
        }
        std::string hash;
        std::size_t at_hash = 0;
        if (!Field(entry, "schema_hash", at_hash) ||
            !String(entry, at_hash, hash) ||
            !ParseHex(hash, source.schema_hash)) {
          error = "source '" + source.name + "' has no schema_hash";
          return false;
        }
        std::size_t at_alignment = 0;
        std::size_t at_slots = 0;
        if (!Field(entry, "alignment", at_alignment) ||
            !Field(entry, "slots", at_slots)) {
          error = "source '" + source.name + "' has no ring layout";
          return false;
        }
        if (!Uint32(entry, at_alignment, source.alignment) ||
            !Uint32(entry, at_slots, source.slots)) {
          error = "source '" + source.name + "' has invalid ring layout";
          return false;
        }
        // Absent means a layout without reflection bytes.
        std::size_t at_bfbs = 0;
        if (Field(entry, "bfbs", at_bfbs)) {
          std::string encoded;
          if (!String(entry, at_bfbs, encoded) ||
              !FromBase64(encoded, source.bfbs)) {
            error = "source '" + source.name + "' has malformed bfbs";
            return false;
          }
        }
      }

      out.sources.push_back(std::move(source));
    }
    return true;
  }

  // Reads what DeclaredGraphToJson wrote: every describe document in `text`,
  // each starting at its "describe_version" key.
  static bool ParseGraph(std::string_view text, DeclaredGraph& out,
                         std::string& error) {
    out.clear();
    static constexpr std::string_view kStart = "\"describe_version\"";
    // The key, not a string value that happens to spell it.
    const auto start = [text](std::size_t from) {
      for (std::size_t found = text.find(kStart, from);
           found != std::string_view::npos;
           found = text.find(kStart, found + kStart.size())) {
        std::size_t cursor = found + kStart.size();
        if (Expect(text, cursor, ':')) return found;
      }
      return std::string_view::npos;
    };
    std::size_t begin = start(0);
    while (begin != std::string_view::npos) {
      const std::size_t next = start(begin + kStart.size());
      const std::string_view document = text.substr(
          begin, next == std::string_view::npos ? next : next - begin);
      Description description;
      if (!Parse(document, description, error)) {
        error = "node " + std::to_string(out.size()) + ": " + error;
        return false;
      }
      out.push_back(std::move(description));
      begin = next;
    }
    if (out.empty()) {
      std::size_t at = 0;
      if (!Expect(text, at, '[') || !Expect(text, at, ']')) {
        error = "not a declared graph (expected a JSON array of descriptions)";
        return false;
      }
      SkipSpace(text, at);
      if (at != text.size()) {
        error =
            "not a declared graph (unexpected content after empty JSON array)";
        return false;
      }
    }
    return true;
  }

  // "0x" and 16 lowercase hex digits, as HexHash writes.
  static bool ParseHex(std::string_view text, std::uint64_t& out) {
    if (text.size() != 18 || !text.starts_with("0x")) return false;
    out = 0;
    for (const char c : text.substr(2)) {
      out <<= 4;
      if (c >= '0' && c <= '9') {
        out |= static_cast<std::uint64_t>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        out |= static_cast<std::uint64_t>(c - 'a' + 10);
      } else {
        return false;
      }
    }
    return true;
  }

 private:
  static void SkipSpace(std::string_view text, std::size_t& at) {
    while (at < text.size() &&
           (std::isspace(static_cast<unsigned char>(text[at])) != 0))
      ++at;
  }

  // Consumes `c` (after any whitespace); false when the next character is
  // anything else.
  static bool Expect(std::string_view text, std::size_t& at, char c) {
    SkipSpace(text, at);
    if (at >= text.size() || text[at] != c) return false;
    ++at;
    return true;
  }

  // The params array at `at`, read token by token in the order
  // DescribeToJson writes: a key may hold any character, brackets and braces
  // included, so finding the next ']' would cut one short.
  static bool Params(std::string_view text, std::size_t at,
                     std::vector<ParamRead>& out, std::string& error) {
    error = "malformed params";
    if (!Expect(text, at, '[')) return false;
    if (Expect(text, at, ']')) return true;
    for (;;) {
      ParamRead read;
      std::string label;
      if (!Expect(text, at, '{')) return false;
      SkipSpace(text, at);
      if (!String(text, at, label) || label != "subsystem" ||
          !Expect(text, at, ':')) {
        return false;
      }
      SkipSpace(text, at);
      if (!String(text, at, read.subsystem) || !Expect(text, at, ',')) {
        return false;
      }
      SkipSpace(text, at);
      if (!String(text, at, label) || label != "key" ||
          !Expect(text, at, ':')) {
        return false;
      }
      SkipSpace(text, at);
      if (!String(text, at, read.key) || !Expect(text, at, '}')) return false;
      out.push_back(std::move(read));
      if (Expect(text, at, ']')) break;
      if (!Expect(text, at, ',')) return false;
    }
    error.clear();
    return true;
  }

  // Advances `at` past `"key":` and any following whitespace. An occurrence
  // not followed by ':' is a string value (a parameter named "sources"), not
  // the key, so the search goes on past it.
  static bool Field(std::string_view text, std::string_view key,
                    std::size_t& at) {
    const std::string needle = "\"" + std::string{key} + "\"";
    for (std::size_t found = text.find(needle, at);
         found != std::string_view::npos;
         found = text.find(needle, found + needle.size())) {
      std::size_t cursor = found + needle.size();
      if (!Expect(text, cursor, ':')) continue;
      SkipSpace(text, cursor);
      at = cursor;
      return true;
    }
    return false;
  }

  static int HexDigit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  }

  // Finds the next occurrence of `target` in `text` at or after `from` outside
  // any double-quoted JSON string literal (respecting `\` escapes). Returns
  // npos on an unterminated string or if `target` is absent.
  static std::size_t FindOutsideString(std::string_view text, std::size_t from,
                                       char target) {
    bool in_string = false;
    for (std::size_t i = from; i < text.size(); ++i) {
      const char c = text[i];
      if (in_string) {
        if (c == '\\') {
          if (i + 1 >= text.size()) return std::string_view::npos;
          ++i;
        } else if (c == '"') {
          in_string = false;
        }
        continue;
      }
      if (c == '"') {
        in_string = true;
      } else if (c == target) {
        return i;
      }
    }
    return std::string_view::npos;
  }

  static bool String(std::string_view text, std::size_t& at, std::string& out) {
    if (at >= text.size() || text[at] != '"') return false;
    out.clear();
    std::size_t cursor = at + 1;
    while (cursor < text.size() && text[cursor] != '"') {
      if (text[cursor] != '\\') {
        out += text[cursor++];
        continue;
      }
      ++cursor;
      if (cursor >= text.size()) return false;
      const char esc = text[cursor];
      switch (esc) {
        case '"':
        case '\\':
        case '/':
          out += esc;
          ++cursor;
          break;
        case 'b':
          out += '\b';
          ++cursor;
          break;
        case 'f':
          out += '\f';
          ++cursor;
          break;
        case 'n':
          out += '\n';
          ++cursor;
          break;
        case 'r':
          out += '\r';
          ++cursor;
          break;
        case 't':
          out += '\t';
          ++cursor;
          break;
        case 'u': {
          if (cursor + 4 >= text.size()) return false;
          const int h0 = HexDigit(text[cursor + 1]);
          const int h1 = HexDigit(text[cursor + 2]);
          const int h2 = HexDigit(text[cursor + 3]);
          const int h3 = HexDigit(text[cursor + 4]);
          if (h0 != 0 || h1 != 0 || h2 < 0 || h3 < 0) return false;
          out += static_cast<char>((h2 << 4) | h3);
          cursor += 5;
          break;
        }
        default:
          return false;
      }
    }
    if (cursor >= text.size()) return false;
    at = cursor + 1;
    return true;
  }

  static bool Number(std::string_view text, std::size_t& at,
                     std::int64_t& out) {
    std::size_t cursor = at;
    if (cursor < text.size() && text[cursor] == '-') ++cursor;
    const std::size_t digits_start = cursor;
    while (cursor < text.size() &&
           (std::isdigit(static_cast<unsigned char>(text[cursor])) != 0)) {
      ++cursor;
    }
    if (cursor == digits_start) return false;
    if (cursor < text.size()) {
      const auto next = static_cast<unsigned char>(text[cursor]);
      if (std::isspace(next) == 0 && next != ',' && next != '}' &&
          next != ']') {
        return false;
      }
    }
    std::int64_t value = 0;
    const auto [end, ec] =
        std::from_chars(text.data() + at, text.data() + cursor, value);
    if (ec != std::errc{} || end != text.data() + cursor) return false;
    out = value;
    at = cursor;
    return true;
  }

  static bool Uint32(std::string_view text, std::size_t& at,
                     std::uint32_t& out) {
    std::int64_t value = 0;
    if (!Number(text, at, value) || value < 0 ||
        value > std::numeric_limits<std::uint32_t>::max()) {
      return false;
    }
    out = static_cast<std::uint32_t>(value);
    return true;
  }

  // By value: nothing reads the cursor afterwards, and a reference here would
  // advertise an advance that callers do not use.
  static bool Boolean(std::string_view text, std::size_t at) {
    return text.compare(at, 4, "true") == 0;
  }

  static bool KindFromString(std::string_view text, event::SourceKind& out) {
    if (text == "TIMER")
      out = event::SourceKind::TIMER;
    else if (text == "WATCHER")
      out = event::SourceKind::WATCHER;
    else if (text == "FETCHER")
      out = event::SourceKind::FETCHER;
    else if (text == "SENDER")
      out = event::SourceKind::SENDER;
    else
      return false;
    return true;
  }
};

}  // namespace talos::introspect
