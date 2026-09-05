#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace talos::event {

// Maximum characters in a source name (topic path or timer label). Fixed so
// that manifest entries are fixed-size records on disk.
inline constexpr std::size_t MAX_SOURCE_NAME = 63;

enum class SourceKind : std::uint16_t {
  TIMER = 1,
  WATCHER = 2,
  FETCHER = 3,
  SENDER = 4,
};

constexpr const char* to_string(SourceKind kind) {
  switch (kind) {
    case SourceKind::TIMER:
      return "TIMER";
    case SourceKind::WATCHER:
      return "WATCHER";
    case SourceKind::FETCHER:
      return "FETCHER";
    case SourceKind::SENDER:
      return "SENDER";
  }
  return "UNKNOWN";
}

// One registered event source. Ids are handed out in registration order, which
// is what makes a log meaningful across runs: the same program registering the
// same sources in the same order produces the same ids.
struct Registration {
  std::uint16_t id{0};
  SourceKind kind{SourceKind::TIMER};
  std::string name;
  std::uint32_t message_bytes{0};
  std::uint32_t alignment{0};
  std::int64_t period_ns{0};
  std::int64_t offset_ns{0};

  bool operator==(const Registration&) const = default;
};

using Manifest = std::vector<Registration>;

// Returns a description of the first difference, or nullopt when the two
// manifests describe the same program. Replay refuses to run on a mismatch:
// feeding a log into a differently-shaped program would silently misroute
// every record.
inline std::optional<std::string> compare_manifests(const Manifest& recorded,
                                                    const Manifest& current) {
  if (recorded.size() != current.size()) {
    return "source count differs: log has " + std::to_string(recorded.size()) +
           ", program registered " + std::to_string(current.size());
  }

  for (std::size_t i = 0; i < recorded.size(); ++i) {
    if (recorded[i] == current[i]) {
      continue;
    }

    const auto describe = [](const Registration& r) {
      return std::string{to_string(r.kind)} + " '" + r.name + "' (" +
             std::to_string(r.message_bytes) + " bytes, period " +
             std::to_string(r.period_ns) + "ns)";
    };

    return "source " + std::to_string(i) + " differs: log has " +
           describe(recorded[i]) + ", program registered " +
           describe(current[i]);
  }

  return std::nullopt;
}

}  // namespace talos::event
