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
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/introspection/names.h"

namespace talos::introspect {

inline constexpr int kDescribeVersion = 1;

// A node as `--describe` reports it: identity, plus every end it will hold.
struct Description {
  std::string name;
  std::string target;
  std::vector<naming::SourceShape> sources;

  // Sources the loop registered, plus anything the node owns outside it. Both
  // reach the registry, so both belong here or the declared graph would flag a
  // topic the running graph shows as fine.
};

inline void AppendJsonString(std::string& out, std::string_view value) {
  out += '"';
  for (const char c : value) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      static constexpr char kHex[] = "0123456789abcdef";
      out += "\\u00";
      out += kHex[(static_cast<unsigned char>(c) >> 4) & 0xF];
      out += kHex[static_cast<unsigned char>(c) & 0xF];
    } else {
      out += c;
    }
  }
  out += '"';
}

// "0x" and 16 lowercase hex digits: how describe writes a schema hash.
inline std::string HexHash(std::uint64_t hash) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out = "0x";
  for (int shift = 60; shift >= 0; shift -= 4) out += kHex[(hash >> shift) & 0xF];
  return out;
}

inline std::string DescribeToJson(const Description& description) {
  std::string out;
  out += "{\n  \"describe_version\": " + std::to_string(kDescribeVersion) +
         ",\n  \"name\": ";
  AppendJsonString(out, description.name);
  out += ",\n  \"target\": ";
  AppendJsonString(out, description.target);
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
    if (source.kind != event::SourceKind::TIMER) {
      // The hash as a hex string: a JSON number is a double.
      out += ", \"schema\": ";
      AppendJsonString(out, source.schema);
      out += ", \"schema_hash\": \"" + HexHash(source.schema_hash) + "\"";
      out += ", \"alignment\": " + std::to_string(source.alignment);
      out += ", \"slots\": " + std::to_string(source.slots);
    }
    out += "}";
  }
  out += description.sources.empty() ? "]\n}\n" : "\n  ]\n}\n";
  return out;
}

// Builds a description from a loop's manifest, whose topics are rings of
// `slots` slots. Endpoint attributes are not something the loop knows --
// `/hw/command` looks like any other sender to it -- so the node supplies
// them by topic name.
struct EndpointAttribute {
  std::string_view topic;
  std::uint32_t flags{0};
};

inline Description DescribeManifest(
    std::string name, std::string target, const event::Manifest& manifest,
    std::uint32_t slots,
    const std::vector<EndpointAttribute>& attributes = {}) {
  Description description{std::move(name), std::move(target), {}};
  for (const auto& registration : manifest) {
    std::uint32_t flags = 0;
    for (const auto& attribute : attributes) {
      if (attribute.topic == registration.name) flags |= attribute.flags;
    }
    naming::SourceShape source{registration.kind, registration.name,
                               registration.message_bytes, flags};
    if (registration.kind != event::SourceKind::TIMER) {
      source.schema = registration.schema.name;
      source.schema_hash = registration.schema.hash;
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
    const std::int64_t version = Number(text, at);
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
    if (!Field(text, "sources", at)) {
      error = "no sources field";
      return false;
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
    const std::size_t array_begin = text.find('[', at);
    if (array_begin == std::string_view::npos) {
      error = "sources is not an array";
      return false;
    }
    const std::size_t array_end = text.find(']', array_begin);
    if (array_end == std::string_view::npos) {
      error = "sources array is not terminated";
      return false;
    }

    std::size_t cursor = array_begin + 1;
    for (;;) {
      const std::size_t open = text.find('{', cursor);
      if (open == std::string_view::npos || open > array_end) break;
      const std::size_t close = text.find('}', open);
      if (close == std::string_view::npos || close > array_end) {
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
      source.message_bytes = static_cast<std::uint32_t>(Number(entry, field));

      // Absent attributes mean "neither".
      std::size_t attribute = 0;
      if (Field(entry, "external", attribute) && Boolean(entry, attribute))
        source.flags |= naming::kSourceFlagExternal;
      attribute = 0;
      if (Field(entry, "optional", attribute) && Boolean(entry, attribute))
        source.flags |= naming::kSourceFlagOptional;

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
            !String(entry, at_hash, hash) || !ParseHex(hash, source.schema_hash)) {
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
        source.alignment =
            static_cast<std::uint32_t>(Number(entry, at_alignment));
        source.slots = static_cast<std::uint32_t>(Number(entry, at_slots));
      }

      out.sources.push_back(std::move(source));
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
  // Advances `at` past `"key":` and any following whitespace.
  static bool Field(std::string_view text, std::string_view key,
                    std::size_t& at) {
    const std::string needle = "\"" + std::string{key} + "\"";
    const std::size_t found = text.find(needle, at);
    if (found == std::string_view::npos) return false;
    std::size_t cursor = found + needle.size();
    while (cursor < text.size() &&
           (std::isspace(static_cast<unsigned char>(text[cursor])) != 0))
      ++cursor;
    if (cursor >= text.size() || text[cursor] != ':') return false;
    ++cursor;
    while (cursor < text.size() &&
           (std::isspace(static_cast<unsigned char>(text[cursor])) != 0))
      ++cursor;
    at = cursor;
    return true;
  }

  static bool String(std::string_view text, std::size_t& at, std::string& out) {
    if (at >= text.size() || text[at] != '"') return false;
    out.clear();
    std::size_t cursor = at + 1;
    while (cursor < text.size() && text[cursor] != '"') {
      if (text[cursor] == '\\' && cursor + 1 < text.size()) ++cursor;
      out += text[cursor++];
    }
    if (cursor >= text.size()) return false;
    at = cursor + 1;
    return true;
  }

  static std::int64_t Number(std::string_view text, std::size_t& at) {
    std::int64_t value = 0;
    bool negative = false;
    if (at < text.size() && text[at] == '-') {
      negative = true;
      ++at;
    }
    while (at < text.size() &&
           (std::isdigit(static_cast<unsigned char>(text[at])) != 0)) {
      value = value * 10 + (text[at++] - '0');
    }
    return negative ? -value : value;
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
