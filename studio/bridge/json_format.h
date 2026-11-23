#pragma once

/*
 * Pure JSON formatting helpers shared by the live Studio bridge (system.h,
 * telemetry.h) and the offline session reviewer (studio/review). Kept free of
 * registry, RTMS, shared-memory and transport dependencies so studio_review
 * never links live-robot plumbing (REQ-SAFE-35).
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>

#include "talOS/telemetry/frames.h"
#include "talOS/telemetry/json_string.h"
#include "talOS/telemetry/telemetry_generated.h"

namespace studio {

inline constexpr int kTelemetryVersion = 3;

using talos::telemetry::AlertLevel;
using talos::telemetry::AppendEscaped;
using talos::telemetry::ChannelKind;

// uint64 and int64 leave as quoted decimals: JSON numbers are doubles and
// silently lose precision past 2^53.
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

// A finite double in round-trip form; NaN and the infinities, which JSON
// numbers cannot spell, as the strings "NaN", "Infinity" and "-Infinity".
inline void AppendTelemetryDouble(std::string& out, double value) {
  if (std::isnan(value)) {
    out += "\"NaN\"";
    return;
  }
  if (std::isinf(value)) {
    out += value > 0 ? "\"Infinity\"" : "\"-Infinity\"";
    return;
  }
  char buffer[32];
  std::snprintf(buffer, sizeof buffer, "%.17g", value);
  out += buffer;
}

// One channel sample as JSON: a number, or for an integer past 2^53 (which a
// JSON number, read as a double, would round) its exact decimal string.
inline void AppendSample(std::string& out, ChannelKind kind,
                         std::uint64_t raw) {
  constexpr std::uint64_t kExact = std::uint64_t{1} << 53;
  switch (kind) {
    case Talos::Telemetry::ChannelKind_Int: {
      const auto value = static_cast<std::int64_t>(raw);
      const bool exact = value >= -static_cast<std::int64_t>(kExact) &&
                         value <= static_cast<std::int64_t>(kExact);
      if (!exact) out += '"';
      out += std::to_string(value);
      if (!exact) out += '"';
      return;
    }
    case Talos::Telemetry::ChannelKind_UInt:
    case Talos::Telemetry::ChannelKind_Bool:
      if (raw > kExact) out += '"';
      out += std::to_string(raw);
      if (raw > kExact) out += '"';
      return;
    default:
      AppendTelemetryDouble(out, talos::telemetry::DoubleFromRaw(raw));
  }
}

inline std::string_view ChannelKindName(ChannelKind kind) {
  switch (kind) {
    case Talos::Telemetry::ChannelKind_Int:
      return "int";
    case Talos::Telemetry::ChannelKind_UInt:
      return "uint";
    case Talos::Telemetry::ChannelKind_Bool:
      return "bool";
    default:
      return "double";
  }
}

// The level as the alert grammar spells it (`alert_<level>_<name>`), "none"
// for a channel that is not an alert.
inline std::string_view AlertLevelName(AlertLevel level) {
  switch (level) {
    case Talos::Telemetry::AlertLevel_Info:
      return "info";
    case Talos::Telemetry::AlertLevel_Warning:
      return "warning";
    case Talos::Telemetry::AlertLevel_Error:
      return "error";
    default:
      return "none";
  }
}

}  // namespace studio
