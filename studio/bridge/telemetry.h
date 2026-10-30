#pragma once

/*
 * The telemetry JSON views: the latest v2 frame and its dictionary, as
 * self-describing JSON documents.
 *
 * The wire feed (talOS/telemetry/README.md) is FlatBuffers because it is
 * 100 Hz and size-capped. These views are the opposite: read on demand by
 * people and agents, so `curl localhost:5800/telemetry.json` answers "what
 * is the robot doing" with names, units and enum labels joined in, and no
 * FlatBuffers tooling. Same rule as system.h: uint64 values leave as
 * decimal strings, because a JSON number is a double and a nanosecond
 * timestamp or sequence counter past 2^53 would come back quietly wrong.
 * NaN -- the wire's absence marker -- leaves as null.
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "studio/bridge/system.h"
#include "talOS/telemetry/telemetry_generated.h"

namespace studio {

inline constexpr int kTelemetryVersion = 2;

struct TelemetryChannelInfo {
  std::string name;
  std::string unit;
  std::vector<std::string> labels;  // non-empty: enum channel
};

struct TelemetryPose {
  double x{0}, y{0}, z{0}, roll{0}, pitch{0}, yaw{0};
};

// The bridge's decoded view of the feed: the latest dictionary and the
// latest frame that aligns with it, plus the raw dictionary frame to hand
// to a WebSocket client that connects mid-stream.
struct TelemetryState {
  bool has_dictionary{false};
  std::vector<TelemetryChannelInfo> dictionary;
  std::uint32_t dictionary_id{0};
  std::uint64_t stream_id{0};
  std::string dictionary_frame;  // size-prefixed bytes, as received

  bool has_frame{false};
  std::uint64_t sequence_id{0};
  std::uint64_t timestamp_ns{0};
  TelemetryPose chassis{};
  std::vector<double> values;
  std::int64_t received_wall_ns{0};
};

// Decodes one size-prefixed v2 payload (as telemetry_wire::payload returns
// it) into the state. A frame carrying a dictionary replaces the cached
// dictionary; a frame's values become the latest frame only when the frame
// belongs to the cached (stream_id, dictionary_id) -- its own dictionary
// counts. Returns false for bytes that are not a verifiable v2 Frame.
inline bool IngestTelemetry(TelemetryState& state, std::string_view payload,
                            std::int64_t wall_ns) {
  flatbuffers::Verifier verifier{
      reinterpret_cast<const std::uint8_t*>(payload.data()), payload.size()};
  if (!Talos::Telemetry::VerifySizePrefixedFrameBuffer(verifier)) return false;
  const auto* frame = Talos::Telemetry::GetSizePrefixedFrame(payload.data());
  if (frame == nullptr) return false;

  if (frame->dictionary() != nullptr) {
    std::vector<TelemetryChannelInfo> dictionary;
    dictionary.reserve(frame->dictionary()->size());
    for (const auto* info : *frame->dictionary()) {
      TelemetryChannelInfo channel;
      channel.name = info->name() != nullptr ? info->name()->str() : "";
      channel.unit = info->unit() != nullptr ? info->unit()->str() : "";
      if (info->labels() != nullptr) {
        for (const auto* label : *info->labels()) {
          channel.labels.push_back(label != nullptr ? label->str() : "");
        }
      }
      dictionary.push_back(std::move(channel));
    }
    state.dictionary = std::move(dictionary);
    state.dictionary_id = frame->dictionary_id();
    state.stream_id = frame->stream_id();
    state.dictionary_frame.assign(payload);
    state.has_dictionary = true;
  }

  const bool aligns = state.has_dictionary &&
                      frame->stream_id() == state.stream_id &&
                      frame->dictionary_id() == state.dictionary_id &&
                      frame->values() != nullptr &&
                      frame->values()->size() == state.dictionary.size();
  if (!aligns) return true;  // Well-formed; just not usable yet.

  state.sequence_id = frame->sequence_id();
  state.timestamp_ns = frame->timestamp_ns();
  if (frame->chassis() != nullptr) {
    state.chassis = {frame->chassis()->x(),     frame->chassis()->y(),
                     frame->chassis()->z(),     frame->chassis()->roll(),
                     frame->chassis()->pitch(), frame->chassis()->yaw()};
  }
  state.values.assign(frame->values()->begin(), frame->values()->end());
  state.received_wall_ns = wall_ns;
  state.has_frame = true;
  return true;
}

// A finite double in shortest round-trip form; anything else is null, so a
// JSON reader never has to special-case the wire's NaN absence marker.
inline void AppendTelemetryDouble(std::string& out, double value) {
  if (!std::isfinite(value)) {
    out += "null";
    return;
  }
  char buffer[32];
  std::snprintf(buffer, sizeof buffer, "%.17g", value);
  out += buffer;
}

inline void AppendPose(std::string& out, const TelemetryPose& pose) {
  out += "{\"x\":";
  AppendTelemetryDouble(out, pose.x);
  out += ",\"y\":";
  AppendTelemetryDouble(out, pose.y);
  out += ",\"z\":";
  AppendTelemetryDouble(out, pose.z);
  out += ",\"roll\":";
  AppendTelemetryDouble(out, pose.roll);
  out += ",\"pitch\":";
  AppendTelemetryDouble(out, pose.pitch);
  out += ",\"yaw\":";
  AppendTelemetryDouble(out, pose.yaw);
  out += '}';
}

// GET /telemetry.json. Precondition: has_dictionary && has_frame (the
// route answers 503 until then).
inline std::string TelemetryJson(const TelemetryState& state,
                                 std::int64_t now_wall_ns) {
  std::string out;
  out.reserve(2048);
  out += "{\"kind\":\"talos.telemetry\",\"version\":";
  out += std::to_string(kTelemetryVersion);
  out += ",\"stream_id\":";
  AppendU64(out, state.stream_id);
  out += ",\"sequence_id\":";
  AppendU64(out, state.sequence_id);
  out += ",\"timestamp_ns\":";
  AppendU64(out, state.timestamp_ns);
  out += ",\"dictionary_id\":";
  out += std::to_string(state.dictionary_id);
  out += ",\"age_ms\":";
  const std::int64_t age_ms =
      now_wall_ns > state.received_wall_ns
          ? (now_wall_ns - state.received_wall_ns) / 1'000'000
          : 0;
  out += std::to_string(age_ms);
  out += ",\"chassis\":";
  AppendPose(out, state.chassis);
  out += ",\"channels\":[";
  for (std::size_t i = 0; i < state.dictionary.size(); ++i) {
    if (i != 0) out += ',';
    const TelemetryChannelInfo& channel = state.dictionary[i];
    const double value = i < state.values.size()
                             ? state.values[i]
                             : std::numeric_limits<double>::quiet_NaN();
    out += "{\"name\":";
    AppendEscaped(out, channel.name);
    out += ",\"value\":";
    AppendTelemetryDouble(out, value);
    if (!channel.unit.empty()) {
      out += ",\"unit\":";
      AppendEscaped(out, channel.unit);
    }
    // Enum channels name their value, but only in range: an out-of-range
    // code is a number, not a label waiting to be invented.
    if (!channel.labels.empty() && std::isfinite(value) &&
        value == std::floor(value) && value >= 0 &&
        value < static_cast<double>(channel.labels.size())) {
      out += ",\"label\":";
      AppendEscaped(out, channel.labels[static_cast<std::size_t>(value)]);
    }
    out += '}';
  }
  out += "]}";
  return out;
}

// GET /channels.json: the dictionary document on its own, so a client can
// learn the channel set without waiting for values it may not need.
// Precondition: has_dictionary (the route answers 503 until then).
inline std::string ChannelsJson(const TelemetryState& state) {
  std::string out;
  out.reserve(2048);
  out += "{\"kind\":\"talos.telemetry.channels\",\"version\":";
  out += std::to_string(kTelemetryVersion);
  out += ",\"stream_id\":";
  AppendU64(out, state.stream_id);
  out += ",\"dictionary_id\":";
  out += std::to_string(state.dictionary_id);
  out += ",\"channels\":[";
  for (std::size_t i = 0; i < state.dictionary.size(); ++i) {
    if (i != 0) out += ',';
    const TelemetryChannelInfo& channel = state.dictionary[i];
    out += "{\"name\":";
    AppendEscaped(out, channel.name);
    out += ",\"unit\":";
    AppendEscaped(out, channel.unit);
    out += ",\"labels\":[";
    for (std::size_t j = 0; j < channel.labels.size(); ++j) {
      if (j != 0) out += ',';
      AppendEscaped(out, channel.labels[j]);
    }
    out += "]}";
  }
  out += "]}";
  return out;
}

}  // namespace studio
