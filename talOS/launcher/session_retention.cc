#include "talOS/launcher/session_retention.h"

#include <sys/statvfs.h>

#include <algorithm>
#include <charconv>
#include <cstring>
#include <exception>
#include <fstream>
#include <iterator>
#include <map>
#include <span>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "talOS/driver_station/driver_station.h"
#include "talOS/events/context.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/manifest.h"
#include "talOS/hardware/packet.h"

namespace talos::launcher {
namespace {

namespace fs = std::filesystem;
namespace tlog = event::log;

std::string ReadSmallFile(const fs::path& path) {
  std::ifstream in{path, std::ios::binary};
  return {std::istreambuf_iterator<char>{in}, std::istreambuf_iterator<char>{}};
}

// The value after `"key": ` in the launcher's own manifest JSON, which writes
// one key per line (SessionManifest::ToJson); empty when absent.
std::string_view JsonValue(std::string_view json, std::string_view key) {
  const std::string quoted = "\"" + std::string{key} + "\": ";
  const auto at = json.find(quoted);
  if (at == std::string_view::npos) return {};
  json.remove_prefix(at + quoted.size());
  return json.substr(0, json.find_first_of(",\n}"));
}

// What this build can read: an unknown verdict holds until it changes.
std::string ReaderIdentity() {
  return "log" + std::to_string(tlog::FORMAT_VERSION) + ".packet" +
         std::to_string(sizeof(hardware::Packet)) + ".ds" +
         std::to_string(driver_station::kFixedEncodedSize) + "-" +
         std::to_string(driver_station::kMaxEncodedSize);
}

enum class Seen : std::uint8_t { kNothing, kDriverStation, kUnreadable, kFms };

// The most the event log at `path` shows of the FMS, streamed with
// tlog::VisitLog. Only a watcher or fetcher of /hw/state/driver_station
// receives packets (its publisher logs none), so a log without one is
// passed over at its manifest.
Seen ScanLog(const std::string& path) {
  std::vector<std::uint16_t> watched;
  Seen seen = Seen::kNothing;
  const auto manifest = [&watched](std::span<const tlog::ManifestEntry> all) {
    for (const auto& entry : all) {
      const auto kind = static_cast<event::SourceKind>(entry.kind);
      if ((kind == event::SourceKind::WATCHER ||
           kind == event::SourceKind::FETCHER) &&
          tlog::read_name(entry.name) == hardware::kDriverStationTopic)
        watched.push_back(entry.id);
    }
    return !watched.empty();
  };
  const auto record = [&](const tlog::LogReader::Record& r) {
    if (static_cast<event::EventKind>(r.header.kind) !=
            event::EventKind::MESSAGE ||
        std::ranges::find(watched, r.header.source_id) == watched.end())
      return true;
    hardware::Packet packet{};
    driver_station::DriverStationData data;
    if (r.payload.size() != sizeof packet) {
      seen = std::max(seen, Seen::kUnreadable);
      return true;
    }
    std::memcpy(&packet, r.payload.data(), sizeof packet);
    if (!driver_station::Decode(packet.bytes(), data)) {
      seen = std::max(seen, Seen::kUnreadable);
    } else if ((data.fms.flags & driver_station::kFmsAttached) != 0) {
      seen = Seen::kFms;
      return false;
    } else {
      seen = std::max(seen, Seen::kDriverStation);
    }
    return true;
  };
  try {
    tlog::VisitLog(path, {.manifest = manifest, .record = record});
  } catch (const tlog::IncompleteLogHeader&) {
    return Seen::kNothing;  // Cut while it was being created: no records.
  } catch (const std::exception&) {
    return Seen::kUnreadable;  // Another format version, or not a log.
  }
  return seen;
}

}  // namespace

SessionFacts InspectSession(const fs::path& dir) {
  SessionFacts facts;
  std::error_code error;
  if (!fs::is_regular_file(dir / "manifest.json", error)) return facts;
  const std::string manifest = ReadSmallFile(dir / "manifest.json");
  const std::string_view start = JsonValue(manifest, "start_wall_ns");
  std::int64_t start_ns = 0;
  const auto [end, parsed] =
      std::from_chars(start.data(), start.data() + start.size(), start_ns);
  if (parsed == std::errc{} && end == start.data() + start.size() &&
      start_ns > 0)
    facts.start_wall_ns = start_ns;
  if (JsonValue(manifest, "simulation") != "true")
    facts.verdict = RecordedMatch(dir);
  return facts;
}

MatchVerdict RecordedMatch(const fs::path& dir) {
  struct Log {
    std::string path;
    std::uintmax_t bytes;
  };
  std::vector<Log> logs;
  std::uintmax_t log_bytes = 0;  // Every event log's: the cache's key.
  std::error_code error;
  for (fs::directory_iterator file{dir, error};
       !error && file != fs::directory_iterator{}; file.increment(error)) {
    std::error_code unreadable;
    if (file->path().extension() != ".tlog") continue;
    const fs::file_type type = file->symlink_status(unreadable).type();
    const std::uintmax_t bytes =
        type == fs::file_type::regular ? file->file_size(unreadable) : 0;
    if (unreadable) return MatchVerdict::kUnknown;
    if (type != fs::file_type::regular) continue;
    log_bytes += bytes;
    logs.push_back({file->path().string(), bytes});
  }
  if (error) return MatchVerdict::kUnknown;

  const fs::path cache = dir / std::string{kSessionClassFile};
  const std::string reader = ReaderIdentity();
  {
    std::istringstream cached{ReadSmallFile(cache)};
    std::string verdict;
    std::string identity;
    std::uintmax_t bytes = 0;
    if (cached >> verdict >> bytes) {
      if (verdict == "match") return MatchVerdict::kMatch;
      if (verdict == "other" && bytes == log_bytes) return MatchVerdict::kOther;
      if (verdict == "unknown" && bytes == log_bytes && cached >> identity &&
          identity == reader)
        return MatchVerdict::kUnknown;
    }
  }

  // Each node's logs (`<name>.tlog`, `<name>.restart<k>.tlog`,
  // `<name>.segment<k>.tlog`: a node name has no dot), smallest node first.
  using Node = std::pair<std::uintmax_t, std::vector<std::string>>;
  std::map<std::string, Node> nodes;
  for (const auto& log : logs) {
    const std::string name = fs::path{log.path}.filename().string();
    Node& node = nodes[name.substr(0, name.find('.'))];
    node.first += log.bytes;
    node.second.push_back(log.path);
  }
  std::vector<const Node*> order;
  order.reserve(nodes.size());
  for (const auto& entry : nodes) order.push_back(&entry.second);
  std::sort(order.begin(), order.end(),
            [](const Node* a, const Node* b) { return a->first < b->first; });
  Seen seen = Seen::kNothing;
  for (const Node* node : order) {
    for (const auto& path : node->second) {
      seen = std::max(seen, ScanLog(path));
      if (seen == Seen::kFms) break;
    }
    if (seen == Seen::kFms) break;
  }
  const MatchVerdict verdict = seen == Seen::kFms ? MatchVerdict::kMatch
                               : seen == Seen::kUnreadable
                                   ? MatchVerdict::kUnknown
                                   : MatchVerdict::kOther;
  // A cache that cannot be written costs a rescan at the next launch.
  std::ofstream out{cache};
  switch (verdict) {
    case MatchVerdict::kMatch:
      out << "match " << log_bytes << "\n";
      break;
    case MatchVerdict::kOther:
      out << "other " << log_bytes << "\n";
      break;
    case MatchVerdict::kUnknown:
      out << "unknown " << log_bytes << " " << reader << "\n";
      break;
  }
  return verdict;
}

std::optional<std::uint64_t> AvailableBytes(const fs::path& path) {
  struct statvfs stats {};
  if (::statvfs(path.c_str(), &stats) != 0) return std::nullopt;
  return static_cast<std::uint64_t>(stats.f_bavail) *
         static_cast<std::uint64_t>(stats.f_frsize);
}

}  // namespace talos::launcher
