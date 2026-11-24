#include "studio/review/session.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>

#include "nlohmann/json.hpp"
#include "studio/bridge/static_files.h"
#include "studio/review/hardware_channels.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/launcher/log_merge.h"
#include "talOS/schema/identity.h"
#include "talOS/schema/unit_suffix.h"

namespace studio::review {
namespace {

namespace fs = std::filesystem;
namespace tlog = talos::event::log;
using talos::event::EventKind;
using talos::launcher::HardwareRecord;
using talos::telemetry::Channel;
using talos::telemetry::ChannelKind;

constexpr std::int64_t kNanosPerSecond = 1'000'000'000;

std::int64_t FloorSecond(std::int64_t ns) {
  std::int64_t second = ns / kNanosPerSecond;
  if (ns % kNanosPerSecond < 0) --second;
  return second;
}

std::uint64_t SourceBit(std::uint16_t source_id) {
  return std::uint64_t{1} << std::min<std::uint16_t>(source_id, 63);
}

// A value a reader received: a MESSAGE, or a SEND its transport accepted
// (a failed send is logged too, and reached no reader).
bool IsValueRecord(const tlog::RecordHeader& header) {
  return header.kind == static_cast<std::uint16_t>(EventKind::MESSAGE) ||
         tlog::IsAcceptedSend(header);
}

void SaturatingAdd(std::uint64_t& dst, std::uint64_t add) noexcept {
  constexpr std::uint64_t kMax = std::numeric_limits<std::uint64_t>::max();
  dst = add > kMax - dst ? kMax : dst + add;
}

std::int64_t ClampedSubI64(std::int64_t a, std::int64_t b,
                           std::int64_t min_val) noexcept {
  const __int128 diff = static_cast<__int128>(a) - static_cast<__int128>(b);
  constexpr __int128 kMax = std::numeric_limits<std::int64_t>::max();
  if (diff < static_cast<__int128>(min_val)) return min_val;
  if (diff > kMax) return std::numeric_limits<std::int64_t>::max();
  return static_cast<std::int64_t>(diff);
}

// Truncates `text` to at most `max_bytes` bytes without splitting a multi-byte
// UTF-8 sequence.
std::string TruncateUtf8(std::string_view text, std::size_t max_bytes) {
  if (text.size() <= max_bytes) return std::string{text};
  std::size_t len = max_bytes;
  while (len > 0 && (static_cast<unsigned char>(text[len]) & 0xC0u) == 0x80u) {
    --len;
  }
  return std::string{text.substr(0, len)};
}

struct RawAlertInterval {
  std::int64_t start_ns{0};
  std::int64_t end_ns{0};
};
static_assert(sizeof(RawAlertInterval) == 16,
              "REQ-OPS-42: 16 B per transient alert interval");

struct AlertChannelState {
  bool active{false};
  std::int64_t start_ns{0};
  std::vector<RawAlertInterval> intervals;
  std::uint64_t dropped{0};
};

struct AlertStream {
  std::vector<AlertChannelState> channels;
  std::int64_t last_publish_ns{std::numeric_limits<std::int64_t>::min()};
};

struct ModeStream {
  std::vector<ModeInterval> intervals;
  std::uint64_t dropped{0};
  std::int64_t last_publish_ns{std::numeric_limits<std::int64_t>::min()};
  ModeKind last_mode{ModeKind::kDisabled};
  bool last_fms{false};
  bool closed_last{false};
};

// Single canonical stream tracked during Open(): SEND beats MESSAGE; lowest
// file index wins among SENDs, lexicographically smallest path wins among
// MESSAGEs (matching post-scan topic selection).
struct CanonicalStream {
  std::optional<std::size_t> file;
  bool is_send{false};
};

enum class StreamAction : std::uint8_t {
  kSkip,
  kContinue,
  kReset,
};

StreamAction SelectCanonicalStream(CanonicalStream& stream,
                                   const std::vector<LogFile>& files,
                                   std::size_t file, bool is_send) {
  const LogFile& log_file = files[file];
  if (!stream.file.has_value()) {
    stream.file = file;
    stream.is_send = is_send;
    return StreamAction::kReset;
  }
  const std::size_t cur_file = *stream.file;
  const LogFile& cur = files[cur_file];
  if (stream.is_send) {
    if (!is_send) return StreamAction::kSkip;
    if (log_file.process == cur.process) {
      if (file < cur_file) stream.file = file;
      return StreamAction::kContinue;
    }
    if (file < cur_file) {
      stream.file = file;
      return StreamAction::kReset;
    }
    return StreamAction::kSkip;
  }
  if (is_send) {
    stream.file = file;
    stream.is_send = true;
    return StreamAction::kReset;
  }
  if (log_file.process == cur.process) {
    if (log_file.path < cur.path) stream.file = file;
    return StreamAction::kContinue;
  }
  if (log_file.path < cur.path) {
    stream.file = file;
    return StreamAction::kReset;
  }
  return StreamAction::kSkip;
}

constexpr std::uint32_t HardwareDeviceKey(std::uint8_t tag,
                                          std::uint16_t id) noexcept {
  return (static_cast<std::uint32_t>(tag) << 16) | id;
}

template <typename Array>
bool AnyUnseenDevice(const std::unordered_set<std::uint32_t>& seen,
                     std::uint8_t tag, const Array& items, std::size_t count) {
  const std::size_t n = std::min(count, items.size());
  for (std::size_t i = 0; i < n; ++i) {
    if (!seen.contains(HardwareDeviceKey(tag, items[i].id))) return true;
  }
  return false;
}

template <typename Array>
void RecordDevices(std::unordered_set<std::uint32_t>& seen, std::uint8_t tag,
                   const Array& items, std::size_t count) {
  const std::size_t n = std::min(count, items.size());
  for (std::size_t i = 0; i < n; ++i) {
    seen.insert(HardwareDeviceKey(tag, items[i].id));
  }
}

bool HasUnseenHardwareDevice(HardwareRecord kind,
                             const talos::launcher::HardwarePacket& packet,
                             const std::unordered_set<std::uint32_t>& seen) {
  if (kind == HardwareRecord::kState) {
    const auto& s = packet.state;
    return AnyUnseenDevice(seen, 0, s.motors, s.motor_count) ||
           AnyUnseenDevice(seen, 1, s.sensors, s.sensor_count) ||
           AnyUnseenDevice(seen, 2, s.digital_inputs, s.digital_input_count) ||
           AnyUnseenDevice(seen, 3, s.digital_outputs,
                           s.digital_output_count) ||
           AnyUnseenDevice(seen, 4, s.analog_inputs, s.analog_input_count) ||
           AnyUnseenDevice(seen, 5, s.encoders, s.encoder_count) ||
           AnyUnseenDevice(seen, 6, s.pwm_outputs, s.pwm_output_count);
  }
  if (kind == HardwareRecord::kCommand) {
    const auto& c = packet.command;
    return AnyUnseenDevice(seen, 0, c.motors, c.count) ||
           AnyUnseenDevice(seen, 3, c.digital_outputs,
                           c.digital_output_count) ||
           AnyUnseenDevice(seen, 6, c.pwm_outputs, c.pwm_output_count);
  }
  return false;
}

void RecordHardwareDevices(HardwareRecord kind,
                           const talos::launcher::HardwarePacket& packet,
                           std::unordered_set<std::uint32_t>& seen) {
  if (kind == HardwareRecord::kState) {
    const auto& s = packet.state;
    RecordDevices(seen, 0, s.motors, s.motor_count);
    RecordDevices(seen, 1, s.sensors, s.sensor_count);
    RecordDevices(seen, 2, s.digital_inputs, s.digital_input_count);
    RecordDevices(seen, 3, s.digital_outputs, s.digital_output_count);
    RecordDevices(seen, 4, s.analog_inputs, s.analog_input_count);
    RecordDevices(seen, 5, s.encoders, s.encoder_count);
    RecordDevices(seen, 6, s.pwm_outputs, s.pwm_output_count);
  } else if (kind == HardwareRecord::kCommand) {
    const auto& c = packet.command;
    RecordDevices(seen, 0, c.motors, c.count);
    RecordDevices(seen, 3, c.digital_outputs, c.digital_output_count);
    RecordDevices(seen, 6, c.pwm_outputs, c.pwm_output_count);
  }
}

std::size_t SessionChannelBytes(const Channel& channel) {
  std::size_t bytes = channel.name.size() + channel.unit.size();
  for (const std::string& label : channel.labels) {
    bytes += label.size();
  }
  return bytes;
}

std::size_t HardwareChannelBytes(std::string_view name, ChannelKind kind) {
  std::size_t bytes = name.size();
  if (kind != Talos::Telemetry::ChannelKind_Bool) {
    const std::size_t slash = name.rfind('/');
    const std::string_view leaf =
        slash == std::string_view::npos ? name : name.substr(slash + 1);
    bytes += talos::schema::unit_for_field(leaf).size();
  }
  return bytes;
}

// A topic as the index pass learns it.
struct TopicBuild {
  std::string name;
  Decoder decoder{Decoder::kStruct};
  HardwareRecord hardware{HardwareRecord::kNone};
  std::uint64_t schema_hash{0};
  std::vector<Channel> channels;  // struct topics
  // Indices into `channels` whose alert level is not None.
  std::vector<std::uint32_t> alert_channels;
  CanonicalStream alert_stream_canon;
  AlertStream alert_stream;
  CanonicalStream mode_stream_canon;
  ModeStream mode_stream;
  // Hardware topics: every channel name seen on the canonical stream, with its
  // kind.
  CanonicalStream hardware_stream;
  std::map<std::string, ChannelKind> hardware_channels;
  std::size_t hardware_channel_bytes{0};
  bool hardware_initialized{false};
  std::unordered_set<std::uint32_t> seen_hardware_devices;
  std::unordered_set<std::string> pending_null_channels;
};

constexpr std::uintmax_t kMaxManifestBytes = std::uintmax_t{4} << 20;
constexpr std::uintmax_t kMaxConfigBytes = std::uintmax_t{1} << 20;

// Resolves `target` strictly within `session_dir` (`canonical_root` is
// fs::canonical(session_dir)), rejecting any symlink leaf, any symlink
// directory component inside `session_dir`, non-regular files, and any path
// escaping `canonical_root` (REQ-OPS-42, REQ-SAFE-35).
std::optional<fs::path> ConfinedRegularFile(const fs::path& session_dir,
                                            const fs::path& canonical_root,
                                            const fs::path& target) {
  std::error_code ec;
  const fs::path abs_dir = fs::absolute(session_dir, ec).lexically_normal();
  if (ec) return std::nullopt;
  const fs::path path = target.is_absolute() ? target : abs_dir / target;
  const fs::file_status leaf_st = fs::symlink_status(path, ec);
  if (ec || fs::is_symlink(leaf_st) || !fs::is_regular_file(leaf_st)) {
    return std::nullopt;
  }
  fs::path canonical_path = fs::canonical(path, ec);
  if (ec) return std::nullopt;
  const fs::path rel = canonical_path.lexically_relative(canonical_root);
  if (rel.empty() || rel == "." || rel.is_absolute() || *rel.begin() == "..") {
    return std::nullopt;
  }
  // Reject any symlink component inside the session directory (while allowing
  // OS-level symlink ancestors above `session_dir`, e.g. /var -> /private/var).
  const fs::path norm = path.lexically_normal();
  fs::path check_base;
  fs::path check_rel = norm.lexically_relative(abs_dir);
  if (!check_rel.empty() && !check_rel.is_absolute() &&
      *check_rel.begin() != "..") {
    check_base = abs_dir;
  } else {
    check_rel = norm.lexically_relative(canonical_root);
    if (check_rel.empty() || check_rel.is_absolute() ||
        *check_rel.begin() == "..") {
      return std::nullopt;
    }
    check_base = canonical_root;
  }
  fs::path cur = check_base;
  for (const fs::path& part : check_rel) {
    cur /= part;
    const fs::file_status st = fs::symlink_status(cur, ec);
    if (ec || fs::is_symlink(st)) return std::nullopt;
  }
  return canonical_path;
}

// Reads a confined, non-symlink regular TOML file up to kMaxConfigBytes
// without hanging on FIFOs or device nodes or allocating unbounded memory
// (REQ-OPS-42, REQ-SAFE-35).
std::optional<toml::table> ReadBoundedToml(const fs::path& path) {
  const auto content = ReadBoundedRegularFile(path, kMaxConfigBytes);
  if (!content) return std::nullopt;
  try {
    return toml::parse(*content, path.string());
  } catch (const toml::parse_error&) {
    return std::nullopt;
  }
}

// Extracts `[studio] pose_topic` from `config_path` confined within
// `session_dir` (following at most one direct top-level `include`, also
// confined within `session_dir`, without following symlinks or expanding
// subsystem manifests). Returns std::nullopt on any missing, symlinked,
// out-of-session, non-regular, oversized, or unparseable file.
std::optional<std::string> ReadPoseTopicFromConfig(
    const fs::path& session_dir, const fs::path& config_path) {
  const auto extract =
      [](const toml::table& tbl,
         bool& explicit_invalid) -> std::optional<std::string> {
    if (!tbl.contains("studio")) return std::nullopt;
    const auto* studio = tbl["studio"].as_table();
    if (studio == nullptr) {
      explicit_invalid = true;
      return std::nullopt;
    }
    if (!studio->contains("pose_topic")) return std::nullopt;
    auto val = (*studio)["pose_topic"].value_exact<std::string>();
    if (!val || val->empty()) {
      explicit_invalid = true;
      return std::nullopt;
    }
    return val;
  };

  std::error_code ec;
  const fs::path canonical_root = fs::canonical(session_dir, ec);
  if (ec) return std::nullopt;
  const auto confined_root =
      ConfinedRegularFile(session_dir, canonical_root, config_path);
  if (!confined_root) return std::nullopt;
  const auto root = ReadBoundedToml(*confined_root);
  if (!root) return std::nullopt;
  bool explicit_invalid = false;
  if (auto pose = extract(*root, explicit_invalid)) return pose;
  if (explicit_invalid) return std::nullopt;

  if (root->contains("include")) {
    const auto inc = (*root)["include"].value_exact<std::string>();
    if (!inc || inc->empty()) return std::nullopt;
    const fs::path base_dir = confined_root->parent_path();
    const fs::path inc_rel{*inc};
    const fs::path inc_path = inc_rel.is_absolute() || base_dir.empty()
                                  ? inc_rel
                                  : base_dir / inc_rel;
    const auto confined_inc =
        ConfinedRegularFile(session_dir, canonical_root, inc_path);
    if (!confined_inc) return std::nullopt;
    const auto base = ReadBoundedToml(*confined_inc);
    if (!base) return std::nullopt;
    return extract(*base, explicit_invalid);
  }
  return std::nullopt;
}

ModeKind ModeFromDriverStation(
    const std::array<std::uint64_t,
                     talos::telemetry::kDriverStationChannelCount>& raw) {
  if (raw[5] != 0) return ModeKind::kEstop;
  if (raw[0] == 0 || raw[1] == 0) return ModeKind::kDisabled;
  if (raw[2] != 0) return ModeKind::kAuto;
  if (raw[3] != 0) return ModeKind::kTeleop;
  if (raw[4] != 0) return ModeKind::kTest;
  return ModeKind::kDisabled;
}

bool ReadManifestInt(const nlohmann::json& object, std::string_view key,
                     std::int64_t& out) {
  const auto it = object.find(key);
  if (it == object.end()) return false;
  if (it->is_number_integer()) {
    if (it->is_number_unsigned() &&
        it->get<std::uint64_t>() >
            static_cast<std::uint64_t>(
                std::numeric_limits<std::int64_t>::max())) {
      return false;
    }
    out = it->get<std::int64_t>();
    return true;
  }
  if (it->is_string()) {
    const std::string& text = it->get_ref<const std::string&>();
    const auto [end, ec] =
        std::from_chars(text.data(), text.data() + text.size(), out);
    return ec == std::errc{} && end == text.data() + text.size() &&
           !text.empty();
  }
  return false;
}

bool RunLess(const RunInterval& a, const RunInterval& b) {
  const std::int64_t a_end =
      a.end_ns.value_or(std::numeric_limits<std::int64_t>::max());
  const std::int64_t b_end =
      b.end_ns.value_or(std::numeric_limits<std::int64_t>::max());
  return std::tie(a.start_ns, a.node, a_end, a.reason) <
         std::tie(b.start_ns, b.node, b_end, b.reason);
}

void ReadManifestRuns(const nlohmann::json& manifest,
                      std::int64_t origin_wall_ns,
                      std::vector<RunInterval>& runs,
                      std::uint64_t& runs_dropped) {
  if (!manifest.is_object()) return;
  const auto nodes = manifest.find("nodes");
  if (nodes == manifest.end() || !nodes->is_array()) return;
  for (const auto& node : *nodes) {
    if (!node.is_object()) continue;
    const auto name_it = node.find("name");
    if (name_it == node.end() || !name_it->is_string()) continue;
    const std::string node_name = TruncateUtf8(
        name_it->get_ref<const std::string&>(), kMaxRunStringBytes);
    std::int64_t node_dropped = 0;
    if (ReadManifestInt(node, "runs_dropped", node_dropped) &&
        node_dropped > 0) {
      SaturatingAdd(runs_dropped, static_cast<std::uint64_t>(node_dropped));
    }
    const auto node_runs = node.find("runs");
    if (node_runs == node.end() || !node_runs->is_array()) continue;
    for (const auto& run : *node_runs) {
      if (!run.is_object()) continue;
      std::int64_t start_wall_ns = 0;
      if (!ReadManifestInt(run, "start_wall_ns", start_wall_ns) ||
          start_wall_ns <= 0) {
        continue;
      }
      const auto reason_it = run.find("reason");
      if (reason_it == run.end() || !reason_it->is_string()) continue;
      std::int64_t end_wall_ns = 0;
      (void)ReadManifestInt(run, "end_wall_ns", end_wall_ns);
      const std::int64_t start_ns =
          ClampedSubI64(start_wall_ns, origin_wall_ns, 0);
      std::optional<std::int64_t> end_ns;
      if (end_wall_ns > 0) {
        end_ns =
            std::max(start_ns, ClampedSubI64(end_wall_ns, origin_wall_ns, 0));
      }
      RunInterval item{node_name, start_ns, end_ns,
                       TruncateUtf8(reason_it->get_ref<const std::string&>(),
                                    kMaxRunStringBytes)};
      if (runs.size() < kMaxTimelineRuns) {
        runs.push_back(std::move(item));
        if (runs.size() == kMaxTimelineRuns) {
          std::make_heap(runs.begin(), runs.end(), RunLess);
        }
      } else {
        SaturatingAdd(runs_dropped, 1);
        if (RunLess(item, runs.front())) {
          std::pop_heap(runs.begin(), runs.end(), RunLess);
          runs.back() = std::move(item);
          std::push_heap(runs.begin(), runs.end(), RunLess);
        }
      }
    }
  }
  std::sort(runs.begin(), runs.end(), RunLess);
  runs.shrink_to_fit();
}

// Lists regular, non-symlink `.tlog` files directly inside non-symlink `dir`,
// verified with O_NOFOLLOW | O_NONBLOCK and fstat(S_ISREG) so symlinks and
// FIFOs never escape confinement or hang Session::Open (REQ-OPS-41, REQ-OPS-42,
// REQ-SAFE-35).
std::vector<std::string> SessionLogFiles(const fs::path& dir) {
  std::error_code ec;
  const fs::file_status dir_st = fs::symlink_status(dir, ec);
  if (ec || fs::is_symlink(dir_st) || !fs::is_directory(dir_st)) {
    return {};
  }
  std::vector<std::string> paths;
  for (fs::directory_iterator it{dir, ec};
       !ec && it != fs::directory_iterator{}; it.increment(ec)) {
    if (it->path().extension() != ".tlog") continue;
    std::error_code entry_ec;
    const fs::file_status st = it->symlink_status(entry_ec);
    if (entry_ec || fs::is_symlink(st) || !fs::is_regular_file(st)) continue;
    const int fd = ::open(it->path().c_str(),
                          O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) continue;
    struct stat sb {};
    const bool regular = (::fstat(fd, &sb) == 0) && S_ISREG(sb.st_mode);
    ::close(fd);
    if (!regular) continue;
    paths.push_back(it->path().string());
  }
  std::sort(paths.begin(), paths.end());
  return paths;
}

constexpr std::int64_t kBridgeStatusSearchNs = 1'000'000'000;
constexpr std::string_view kHardwareStatusTopic = "/hw/status";

// The protocol_version_id of the first /hw/status SEND in `path` within its
// first second of records, matching hardware_record.cc's
// RecordedProtocolVersion without scanning the directory again.
std::optional<std::uint16_t> FirstBridgeStatusVersion(const std::string& path) {
  try {
    tlog::WindowedLogReader reader{path};
    std::optional<std::uint16_t> id;
    std::optional<talos::launcher::ProtocolVersionField> version_field;
    for (const auto& source : reader.manifest()) {
      if (source.kind == talos::event::SourceKind::SENDER &&
          source.name == kHardwareStatusTopic) {
        version_field = talos::launcher::ResolveProtocolVersionField(
            source.schema.bfbs, source.schema.name, source.message_bytes);
        if (version_field) id = source.id;
        break;
      }
    }
    if (!id || !version_field) return std::nullopt;
    tlog::LogReader::Record record;
    std::optional<std::int64_t> first_ns;
    while (reader.next(record)) {
      if (!first_ns) first_ns = record.header.event_time_ns;
      if (record.header.event_time_ns > *first_ns &&
          static_cast<std::uint64_t>(record.header.event_time_ns) -
                  static_cast<std::uint64_t>(*first_ns) >
              static_cast<std::uint64_t>(kBridgeStatusSearchNs)) {
        return std::nullopt;
      }
      if (record.header.source_id != *id ||
          record.header.kind != static_cast<std::uint16_t>(EventKind::SEND)) {
        continue;
      }
      return talos::launcher::ReadProtocolVersion(*version_field,
                                                  record.payload);
    }
  } catch (const std::exception&) {
    return std::nullopt;
  }
  return std::nullopt;
}

// A log file as the index pass learns it: per source id, its topic (-1 for
// none, kUnresolved before its first value record) and which value record
// kinds it holds.
constexpr std::int32_t kUnresolved = -2;
struct FileBuild {
  std::vector<std::int32_t> topic;
  std::vector<std::uint8_t> kinds;  // bit 0 SEND, bit 1 MESSAGE
  // Transient map from source_id (>= 63) to dense bit index in
  // LogFile::overflow_source_ids.
  std::unordered_map<std::uint16_t, std::size_t> overflow_bit;
};

constexpr std::size_t kMaxOverflowWordsPerFile =
    kMaxOverflowBytesPerFile / sizeof(std::uint64_t);

void RecordOverflowSource(LogFile& log_file, FileBuild& build,
                          std::size_t cp_idx, std::uint16_t source_id,
                          std::size_t& session_overflow_bytes) {
  const auto it = build.overflow_bit.find(source_id);
  std::size_t bit_idx = 0;
  if (it == build.overflow_bit.end()) {
    if (log_file.overflow_source_ids.size() >= kMaxOverflowSources) {
      log_file.overflow_capped = true;
      return;
    }
    const std::size_t next_bit = log_file.overflow_source_ids.size();
    const std::size_t needed_words = (next_bit + 64) / 64;
    if (needed_words > log_file.overflow_words_per_cp) {
      const std::size_t old_words = log_file.overflow_words_per_cp;
      const std::size_t new_words =
          std::min(kMaxOverflowWordsPerCheckpoint,
                   std::max(needed_words,
                            old_words == 0 ? std::size_t{1} : old_words * 2));
      const std::size_t old_cps =
          old_words == 0 ? 0 : log_file.overflow_bits.size() / old_words;
      const std::size_t target_cps = std::max(old_cps, cp_idx + 1);
      if (target_cps > kMaxOverflowWordsPerFile / new_words) {
        log_file.overflow_capped = true;
        return;
      }
      const std::size_t target_words = target_cps * new_words;
      const std::size_t old_bytes =
          log_file.overflow_bits.size() * sizeof(std::uint64_t);
      const std::size_t new_bytes = target_words * sizeof(std::uint64_t);
      if (new_bytes > old_bytes &&
          new_bytes - old_bytes >
              kMaxSessionOverflowBytes - session_overflow_bytes) {
        throw std::runtime_error("session exceeds overflow bitmap cap (" +
                                 std::to_string(kMaxSessionOverflowBytes) +
                                 " bytes)");
      }
      std::vector<std::uint64_t> grown(target_words, 0);
      for (std::size_t c = 0; c < old_cps; ++c) {
        for (std::size_t w = 0; w < old_words; ++w) {
          grown[c * new_words + w] = log_file.overflow_bits[c * old_words + w];
        }
      }
      log_file.overflow_bits = std::move(grown);
      log_file.overflow_words_per_cp = new_words;
      if (new_bytes > old_bytes) {
        session_overflow_bytes += new_bytes - old_bytes;
      }
    } else if (cp_idx + 1 >
               kMaxOverflowWordsPerFile / log_file.overflow_words_per_cp) {
      log_file.overflow_capped = true;
      return;
    }
    build.overflow_bit.emplace(source_id, next_bit);
    log_file.overflow_source_ids.push_back(source_id);
    bit_idx = next_bit;
  } else {
    bit_idx = it->second;
    if (cp_idx + 1 >
        kMaxOverflowWordsPerFile / log_file.overflow_words_per_cp) {
      log_file.overflow_capped = true;
      return;
    }
  }
  const std::size_t required_size =
      (cp_idx + 1) * log_file.overflow_words_per_cp;
  if (log_file.overflow_bits.size() < required_size) {
    const std::size_t delta_bytes =
        (required_size - log_file.overflow_bits.size()) * sizeof(std::uint64_t);
    if (delta_bytes > kMaxSessionOverflowBytes - session_overflow_bytes) {
      throw std::runtime_error("session exceeds overflow bitmap cap (" +
                               std::to_string(kMaxSessionOverflowBytes) +
                               " bytes)");
    }
    if (log_file.overflow_bits.capacity() < required_size) {
      const std::size_t doubled = log_file.overflow_bits.capacity() * 2;
      log_file.overflow_bits.reserve(
          std::min(kMaxOverflowWordsPerFile, std::max(required_size, doubled)));
    }
    log_file.overflow_bits.resize(required_size, 0);
    session_overflow_bytes += delta_bytes;
  }
  log_file.overflow_bits[cp_idx * log_file.overflow_words_per_cp +
                         (bit_idx / 64)] |=
      (std::uint64_t{1} << (bit_idx % 64));
}

class Indexer {
 public:
  std::vector<TopicBuild> topics;
  std::size_t total_channels{0};
  std::size_t total_channel_bytes{0};

  // The topic `source` carries, created on first sight; -1 for one that
  // holds no channel review reads, or whose schema disagrees with the one
  // the topic was first seen with. Throws std::runtime_error when adding
  // the topic would exceed kMaxSessionChannels or kMaxSessionChannelBytes
  // across the session.
  std::int32_t Resolve(const talos::event::Registration& source,
                       const std::string& path) {
    const auto& schema = source.schema;
    if (schema.name.empty() || schema.name == talos::schema::kBytesName) {
      return -1;
    }
    const auto found = by_name_.find(source.name);
    if (found != by_name_.end()) {
      if (found->second >= 0 &&
          topics[static_cast<std::size_t>(found->second)].schema_hash !=
              schema.hash) {
        std::cerr << "studio_review: " << path << " records " << source.name
                  << " as " << schema.name
                  << ", another log as another type; its records are "
                     "skipped\n";
        return -1;
      }
      return found->second;
    }
    TopicBuild topic;
    topic.name = source.name;
    topic.schema_hash = schema.hash;
    topic.hardware =
        talos::launcher::HardwareRecordOf(source.name, schema.name);
    std::int32_t index = -1;
    bool resolved = false;
    try {
      if (topic.hardware == HardwareRecord::kDriverStation) {
        topic.decoder = Decoder::kDriverStation;
        topic.channels = talos::telemetry::DriverStationChannels();
      } else if (topic.hardware != HardwareRecord::kNone) {
        topic.decoder = Decoder::kHardware;
      } else if (schema.name == talos::schema::kPacketName ||
                 schema.bfbs.empty()) {
        topic.decoder = Decoder::kStruct;  // nothing to decode
      } else if (const auto inner = talos::schema::EnvelopeInner(schema.name);
                 !inner.empty()) {
        topic.channels =
            talos::telemetry::EnvelopeChannels(source.name, inner, schema.bfbs);
      } else {
        topic.channels = talos::telemetry::FlattenTopic(
            source.name, schema.name, schema.bfbs);
      }
      resolved = topic.decoder == Decoder::kHardware || !topic.channels.empty();
    } catch (const std::exception& e) {
      std::cerr << "studio_review: " << source.name << " in " << path
                << " is not reviewed: " << e.what() << '\n';
    }
    if (resolved) {
      if (topics.size() >= kMaxSessionChannels ||
          topic.channels.size() > kMaxSessionChannels - total_channels) {
        throw std::runtime_error("session exceeds channel cap (" +
                                 std::to_string(kMaxSessionChannels) +
                                 ") at topic " + source.name);
      }
      std::size_t topic_bytes = 0;
      for (const Channel& ch : topic.channels) {
        const std::size_t ch_bytes = SessionChannelBytes(ch);
        if (ch_bytes >
            kMaxSessionChannelBytes - total_channel_bytes - topic_bytes) {
          throw std::runtime_error("session exceeds channel byte cap (" +
                                   std::to_string(kMaxSessionChannelBytes) +
                                   ") at topic " + source.name);
        }
        topic_bytes += ch_bytes;
      }
      total_channels += topic.channels.size();
      total_channel_bytes += topic_bytes;
      for (std::uint32_t c = 0; c < topic.channels.size(); ++c) {
        if (topic.channels[c].alert != Talos::Telemetry::AlertLevel_None) {
          topic.alert_channels.push_back(c);
        }
      }
      index = static_cast<std::int32_t>(topics.size());
      topics.push_back(std::move(topic));
    }
    by_name_.emplace(source.name, index);
    return index;
  }

 private:
  std::unordered_map<std::string, std::int32_t> by_name_;
};

}  // namespace

std::optional<std::size_t> LogFile::OverflowBit(std::uint16_t source_id) const {
  if (source_id < 63 || overflow_source_ids.empty()) return std::nullopt;
  const auto it = std::find(overflow_source_ids.begin(),
                            overflow_source_ids.end(), source_id);
  if (it == overflow_source_ids.end()) return std::nullopt;
  return static_cast<std::size_t>(it - overflow_source_ids.begin());
}

bool LogFile::HasSource(std::size_t cp_idx, std::uint16_t source_id,
                        std::optional<std::size_t> overflow_bit) const {
  const std::uint64_t mask = checkpoints[cp_idx].sources;
  if (source_id < 63) return (mask & (std::uint64_t{1} << source_id)) != 0;
  if ((mask & (std::uint64_t{1} << 63)) == 0) return false;
  if (!overflow_bit) return overflow_capped;
  if (overflow_words_per_cp == 0) return true;
  const std::size_t word_idx =
      cp_idx * overflow_words_per_cp + (*overflow_bit / 64);
  if (word_idx >= overflow_bits.size()) return true;
  return (overflow_bits[word_idx] &
          (std::uint64_t{1} << (*overflow_bit % 64))) != 0;
}

std::string_view ModeKindName(ModeKind mode) {
  switch (mode) {
    case ModeKind::kDisabled:
      return "disabled";
    case ModeKind::kTeleop:
      return "teleop";
    case ModeKind::kAuto:
      return "auto";
    case ModeKind::kTest:
      return "test";
    case ModeKind::kEstop:
      return "estop";
  }
  return "disabled";
}

double ValueOf(ChannelKind kind, std::uint64_t raw) {
  switch (kind) {
    case Talos::Telemetry::ChannelKind_Bool:
      return raw != 0 ? 1.0 : 0.0;
    case Talos::Telemetry::ChannelKind_Int:
      return static_cast<double>(static_cast<std::int64_t>(raw));
    case Talos::Telemetry::ChannelKind_UInt:
      return static_cast<double>(raw);
    default:
      return talos::telemetry::DoubleFromRaw(raw);
  }
}

std::unique_ptr<Session> Session::Open(const fs::path& dir) {
  const std::vector<std::string> paths = SessionLogFiles(dir);
  if (paths.empty()) {
    throw std::runtime_error("no .tlog files in " + dir.string());
  }
  std::optional<nlohmann::json> manifest;
  if (const auto manifest_text =
          ReadBoundedRegularFile(dir / "manifest.json", kMaxManifestBytes)) {
    auto parsed = nlohmann::json::parse(*manifest_text, nullptr,
                                        /*allow_exceptions=*/false);
    if (parsed.is_object()) manifest = std::move(parsed);
  }
  talos::launcher::RecordedProtocol recorded;
  if (manifest) {
    const auto ver_it = manifest->find("hardware_protocol_version");
    if (ver_it != manifest->end() && ver_it->is_number_integer()) {
      if (ver_it->is_number_unsigned()) {
        const auto ver = ver_it->get<std::uint64_t>();
        if (ver >= 1 && ver <= 0xFFFFu) {
          recorded.manifest = static_cast<std::uint16_t>(ver);
        }
      } else {
        const auto ver = ver_it->get<std::int64_t>();
        if (ver >= 1 && ver <= 0xFFFF) {
          recorded.manifest = static_cast<std::uint16_t>(ver);
        }
      }
    }
  }
  for (const std::string& log_path : paths) {
    const auto version = FirstBridgeStatusVersion(log_path);
    if (version && std::find(recorded.bridge.begin(), recorded.bridge.end(),
                             *version) == recorded.bridge.end()) {
      recorded.bridge.push_back(*version);
    }
  }
  if (const std::string line = recorded.Disagreement(); !line.empty()) {
    std::cerr << "studio_review: " << dir.string() << ": " << line << '\n';
  }
  talos::launcher::HardwareProtocolCheck protocol{std::move(recorded)};

  std::unique_ptr<Session> session{new Session};
  Indexer indexer;
  std::vector<FileBuild> builds;
  std::unordered_map<const talos::launcher::MergedLog*, std::size_t> file_of;
  const talos::launcher::MergedLog* last_log = nullptr;
  std::size_t file = 0;
  std::size_t session_overflow_bytes = 0;
  bool any = false;
  std::vector<HardwareSample> samples;

  const auto visit = [&](const talos::launcher::MergedRecord& r) {
    const auto* merged = r.log.get();
    if (merged != last_log) {
      const auto [it, inserted] =
          file_of.emplace(merged, session->files_.size());
      if (inserted) {
        session->files_.push_back({merged->path, merged->process_name, {}});
        builds.emplace_back();
      }
      last_log = merged;
      file = it->second;
    }
    LogFile& log_file = session->files_[file];
    if (const auto gap = tlog::GapOf(r.header, r.payload)) {
      // No checkpoint and no value: only its span, in the bounds too.
      session->gaps_.push_back(
          {gap->start_ns, gap->resumed_ns, gap->detail.records,
           gap->detail.dispatches, gap->detail.sends,
           static_cast<std::uint32_t>(file),
           (gap->detail.flags & tlog::GAP_CLOSED_AT_FLUSH) != 0});
      const std::int64_t low = std::min(gap->start_ns, gap->resumed_ns);
      const std::int64_t high = std::max(gap->start_ns, gap->resumed_ns);
      if (!any || low < session->start_ns_) session->start_ns_ = low;
      if (!any || high > session->end_ns_) session->end_ns_ = high;
      any = true;
      return true;
    }
    const std::int64_t event_ns = r.header.event_time_ns;
    const std::int64_t second = FloorSecond(event_ns);
    if (log_file.checkpoints.empty() ||
        second > log_file.checkpoints.back().second) {
      log_file.checkpoints.push_back({second, r.offset, 0});
    }
    if (!any || event_ns < session->start_ns_) session->start_ns_ = event_ns;
    if (!any || event_ns > session->end_ns_) session->end_ns_ = event_ns;
    any = true;
    if (r.header.kind == static_cast<std::uint16_t>(EventKind::EXIT)) {
      return true;
    }
    if (!IsValueRecord(r.header)) return true;
    const std::size_t cp_idx = log_file.checkpoints.size() - 1;
    const std::int64_t cp_second = log_file.checkpoints[cp_idx].second;
    const std::int64_t publish_second = FloorSecond(r.header.publish_time_ns);
    if (cp_second > publish_second) {
      log_file.max_lead_s =
          std::max(log_file.max_lead_s, cp_second - publish_second);
    } else if (publish_second > cp_second) {
      log_file.max_lag_s =
          std::max(log_file.max_lag_s, publish_second - cp_second);
    }
    if (r.source == nullptr || r.payload.empty()) return true;
    FileBuild& build = builds[file];
    const std::uint16_t id = r.header.source_id;
    if (id >= build.topic.size()) {
      build.topic.resize(id + 1u, kUnresolved);
      build.kinds.resize(id + 1u, 0);
    }
    if (build.topic[id] == kUnresolved) {
      build.topic[id] = indexer.Resolve(*r.source, merged->path);
    }
    const talos::launcher::HardwarePacket* decoded_hw = nullptr;
    if (const auto kind = talos::launcher::HardwareRecordOf(
            r.source->name, r.source->schema.name);
        kind != HardwareRecord::kNone) {
      decoded_hw = protocol.Add(
          kind, r.source->name,
          r.header.kind != static_cast<std::uint16_t>(EventKind::SEND),
          r.payload);
    }
    if (build.topic[id] < 0) return true;
    log_file.checkpoints[cp_idx].sources |= SourceBit(id);
    if (id >= 63) {
      RecordOverflowSource(log_file, build, cp_idx, id, session_overflow_bytes);
    }
    const bool is_send =
        r.header.kind == static_cast<std::uint16_t>(EventKind::SEND);
    build.kinds[id] |= is_send ? 1 : 2;
    const auto topic_idx = static_cast<std::size_t>(build.topic[id]);
    TopicBuild& topic = indexer.topics[topic_idx];
    // A hardware topic's channels are what its canonical stream's packets
    // hold: re-flattened only on first packet, while any previously-null
    // channel is still pending, or when an unseen device id appears. Never
    // decode foreign-protocol hardware or Driver Station packets (REQ-OPS-51).
    if (topic.decoder == Decoder::kHardware) {
      if (!protocol.foreign()) {
        const StreamAction action = SelectCanonicalStream(
            topic.hardware_stream, session->files_, file, is_send);
        if (action != StreamAction::kSkip) {
          if (action == StreamAction::kReset) {
            indexer.total_channels -= topic.hardware_channels.size();
            indexer.total_channel_bytes -= topic.hardware_channel_bytes;
            topic.hardware_channels.clear();
            topic.hardware_channel_bytes = 0;
            topic.seen_hardware_devices.clear();
            topic.pending_null_channels.clear();
            topic.hardware_initialized = false;
          }
          const bool needs_flatten =
              !topic.hardware_initialized ||
              !topic.pending_null_channels.empty() || decoded_hw == nullptr ||
              HasUnseenHardwareDevice(topic.hardware, *decoded_hw,
                                      topic.seen_hardware_devices);
          if (needs_flatten) {
            FlattenHardware(topic.name, topic.hardware, r.payload, samples);
            if (!samples.empty()) {
              topic.hardware_initialized = true;
              if (decoded_hw != nullptr) {
                RecordHardwareDevices(topic.hardware, *decoded_hw,
                                      topic.seen_hardware_devices);
              }
              for (const HardwareSample& sample : samples) {
                if (sample.null) {
                  if (!topic.hardware_channels.contains(sample.name) &&
                      topic.pending_null_channels.size() <
                          talos::telemetry::kMaxTopicChannels) {
                    topic.pending_null_channels.insert(sample.name);
                  }
                  continue;
                }
                topic.pending_null_channels.erase(sample.name);
                if (topic.hardware_channels.contains(sample.name)) continue;
                const std::size_t ch_bytes =
                    HardwareChannelBytes(sample.name, sample.kind);
                if (topic.hardware_channels.size() >=
                    talos::telemetry::kMaxTopicChannels) {
                  throw std::runtime_error(
                      "topic " + topic.name + " exceeds channel cap (" +
                      std::to_string(talos::telemetry::kMaxTopicChannels) +
                      ")");
                }
                if (indexer.total_channels >= kMaxSessionChannels) {
                  throw std::runtime_error("session exceeds channel cap (" +
                                           std::to_string(kMaxSessionChannels) +
                                           ") at topic " + topic.name);
                }
                if (ch_bytes >
                    kMaxSessionChannelBytes - indexer.total_channel_bytes) {
                  throw std::runtime_error(
                      "session exceeds channel byte cap (" +
                      std::to_string(kMaxSessionChannelBytes) + ") at topic " +
                      topic.name);
                }
                topic.hardware_channels.emplace(sample.name, sample.kind);
                topic.hardware_channel_bytes += ch_bytes;
                ++indexer.total_channels;
                indexer.total_channel_bytes += ch_bytes;
              }
            }
          }
        }
      }
    } else if (topic.decoder == Decoder::kDriverStation) {
      if (!protocol.foreign()) {
        const StreamAction action = SelectCanonicalStream(
            topic.mode_stream_canon, session->files_, file, is_send);
        if (action != StreamAction::kSkip) {
          if (action == StreamAction::kReset) {
            topic.mode_stream = ModeStream{};
          }
          std::array<std::uint64_t,
                     talos::telemetry::kDriverStationChannelCount>
              values{};
          if (talos::telemetry::ReadDriverStation(r.payload, values)) {
            const std::int64_t publish_ns = r.header.publish_time_ns;
            ModeStream& stream = topic.mode_stream;
            if (publish_ns >= stream.last_publish_ns) {
              stream.last_publish_ns = publish_ns;
              const ModeKind mode = ModeFromDriverStation(values);
              const bool fms = values[6] != 0;
              if (stream.intervals.empty()) {
                stream.intervals.push_back({publish_ns, publish_ns, mode, fms});
                stream.last_mode = mode;
                stream.last_fms = fms;
              } else if (mode != stream.last_mode || fms != stream.last_fms) {
                if (!stream.closed_last) {
                  stream.intervals.back().end_ns = publish_ns;
                  if (stream.intervals.size() < kMaxTimelineModes) {
                    stream.intervals.push_back(
                        {publish_ns, publish_ns, mode, fms});
                  } else {
                    stream.closed_last = true;
                    SaturatingAdd(stream.dropped, 1);
                  }
                } else {
                  SaturatingAdd(stream.dropped, 1);
                }
                stream.last_mode = mode;
                stream.last_fms = fms;
              }
            }
          }
        }
      }
    } else if (topic.decoder == Decoder::kStruct &&
               !topic.alert_channels.empty()) {
      const StreamAction action = SelectCanonicalStream(
          topic.alert_stream_canon, session->files_, file, is_send);
      if (action != StreamAction::kSkip) {
        if (action == StreamAction::kReset) {
          topic.alert_stream = AlertStream{};
          topic.alert_stream.channels.resize(topic.alert_channels.size());
        }
        const std::int64_t publish_ns = r.header.publish_time_ns;
        AlertStream& stream = topic.alert_stream;
        if (publish_ns >= stream.last_publish_ns) {
          stream.last_publish_ns = publish_ns;
          for (std::size_t i = 0; i < topic.alert_channels.size(); ++i) {
            const std::uint32_t c = topic.alert_channels[i];
            std::uint64_t raw = 0;
            if (!talos::telemetry::ReadChannel(topic.channels[c], r.payload,
                                               raw)) {
              continue;
            }
            const bool active = raw != 0;
            AlertChannelState& st = stream.channels[i];
            if (active && !st.active) {
              st.active = true;
              st.start_ns = publish_ns;
            } else if (!active && st.active) {
              st.active = false;
              if (st.intervals.size() < kMaxTimelineAlerts) {
                st.intervals.push_back({st.start_ns, publish_ns});
              } else {
                SaturatingAdd(st.dropped, 1);
              }
            }
          }
        }
      }
    }
    return true;
  };
  std::string error;
  if (!talos::launcher::MergeLogs(paths, visit, error)) {
    throw std::runtime_error(error);
  }
  if (!any) throw std::runtime_error("no records in " + dir.string());
  for (LogFile& log_file : session->files_) {
    const std::size_t used_words =
        (log_file.overflow_source_ids.size() + 63) / 64;
    if (used_words > 0 && used_words < log_file.overflow_words_per_cp) {
      const std::size_t old_words = log_file.overflow_words_per_cp;
      const std::size_t cps = log_file.overflow_bits.size() / old_words;
      std::vector<std::uint64_t> compact(cps * used_words, 0);
      for (std::size_t c = 0; c < cps; ++c) {
        for (std::size_t w = 0; w < used_words; ++w) {
          compact[c * used_words + w] =
              log_file.overflow_bits[c * old_words + w];
        }
      }
      log_file.overflow_bits = std::move(compact);
      log_file.overflow_words_per_cp = used_words;
    }
    log_file.overflow_bits.shrink_to_fit();
    log_file.overflow_source_ids.shrink_to_fit();
  }
  std::stable_sort(session->gaps_.begin(), session->gaps_.end(),
                   [&files = session->files_](const Gap& a, const Gap& b) {
                     if (a.start_ns != b.start_ns) {
                       return a.start_ns < b.start_ns;
                     }
                     return files[a.file].process < files[b.file].process;
                   });
  session->gaps_.shrink_to_fit();
  if (protocol.foreign()) {
    constexpr std::string_view kWarning = "warning: ";
    session->hardware_not_decoded_ = protocol.Warning().substr(kWarning.size());
    std::cerr << "studio_review: " << dir.string() << ": "
              << session->hardware_not_decoded_ << '\n';
  }

  // Each topic's process: the one that sent it, else the one whose log
  // path sorts first among those that received it.
  std::vector<std::size_t> topic_build_index;
  for (std::size_t t = 0; t < indexer.topics.size(); ++t) {
    TopicBuild& build = indexer.topics[t];
    std::optional<std::size_t> sender;
    std::optional<std::size_t> receiver;
    for (std::size_t f = 0; f < builds.size(); ++f) {
      const FileBuild& file_build = builds[f];
      for (std::size_t id = 0; id < file_build.topic.size(); ++id) {
        if (file_build.topic[id] != static_cast<std::int32_t>(t)) continue;
        if ((file_build.kinds[id] & 1) != 0 && !sender) sender = f;
        // The reader whose log path sorts first, as RecordedSession orders
        // logs: not the first in merge order, which depends on record times.
        if ((file_build.kinds[id] & 2) != 0 &&
            (!receiver ||
             session->files_[f].path < session->files_[*receiver].path)) {
          receiver = f;
        }
      }
    }
    if (!sender && !receiver) continue;
    if (protocol.foreign() && build.hardware != HardwareRecord::kNone) {
      continue;
    }
    Topic topic;
    topic.name = build.name;
    topic.decoder = build.decoder;
    topic.hardware = build.hardware;
    topic.kind = sender ? EventKind::SEND : EventKind::MESSAGE;
    topic.process = session->files_[sender ? *sender : *receiver].process;
    const std::uint8_t want_kind = sender ? 1 : 2;
    for (std::size_t f = 0; f < builds.size(); ++f) {
      if (session->files_[f].process != topic.process) continue;
      const FileBuild& file_build = builds[f];
      for (std::size_t id = 0; id < file_build.topic.size(); ++id) {
        if (file_build.topic[id] == static_cast<std::int32_t>(t) &&
            (file_build.kinds[id] & want_kind) != 0) {
          topic.sources.push_back({f, static_cast<std::uint16_t>(id)});
        }
      }
    }
    std::unordered_map<std::string, std::uint32_t> hardware_index;
    if (build.decoder == Decoder::kHardware) {
      for (const auto& [name, kind] : build.hardware_channels) {
        Channel channel;
        channel.name = name;
        const std::string_view leaf =
            std::string_view{name}.substr(name.rfind('/') + 1);
        if (kind != Talos::Telemetry::ChannelKind_Bool) {
          channel.unit = std::string{talos::schema::unit_for_field(leaf)};
        }
        channel.kind = kind;
        hardware_index.emplace(
            name, static_cast<std::uint32_t>(topic.channels.size()));
        topic.channels.push_back(std::move(channel));
      }
    } else {
      topic.channels = std::move(build.channels);
    }
    // A hardware topic none of whose packets decoded has nothing to serve.
    if (topic.channels.empty()) continue;
    session->topics_.push_back(std::move(topic));
    session->hardware_index_.push_back(std::move(hardware_index));
    topic_build_index.push_back(t);
  }
  // Topics by name, so channels.json lists them in a stable order.
  std::vector<std::size_t> order(session->topics_.size());
  for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return session->topics_[a].name < session->topics_[b].name;
  });
  std::vector<Topic> topics;
  std::vector<std::unordered_map<std::string, std::uint32_t>> hardware_index;
  std::vector<std::size_t> sorted_build_index;
  for (const std::size_t i : order) {
    topics.push_back(std::move(session->topics_[i]));
    hardware_index.push_back(std::move(session->hardware_index_[i]));
    sorted_build_index.push_back(topic_build_index[i]);
  }
  session->topics_ = std::move(topics);
  session->hardware_index_ = std::move(hardware_index);
  for (std::uint32_t t = 0; t < session->topics_.size(); ++t) {
    const auto& channels = session->topics_[t].channels;
    for (std::uint32_t c = 0; c < channels.size(); ++c) {
      session->by_name_.emplace(channels[c].name, ChannelRef{t, c});
    }
  }

  // Finalize timeline modes and alerts from each topic's canonical stream.
  std::vector<AlertInterval> combined_alerts;
  const auto sort_alerts = [&combined_alerts, &session] {
    std::sort(combined_alerts.begin(), combined_alerts.end(),
              [&session](const AlertInterval& a, const AlertInterval& b) {
                if (a.start_ns != b.start_ns) return a.start_ns < b.start_ns;
                if (a.end_ns != b.end_ns) return a.end_ns < b.end_ns;
                return session->channel(a.channel()).name <
                       session->channel(b.channel()).name;
              });
  };
  for (std::uint32_t t = 0; t < session->topics_.size(); ++t) {
    const Topic& topic = session->topics_[t];
    const bool is_send = topic.kind == EventKind::SEND;
    if (!protocol.foreign() && topic.decoder == Decoder::kDriverStation) {
      const std::size_t build_idx = sorted_build_index[t];
      TopicBuild& build = indexer.topics[build_idx];
      if (build.mode_stream_canon.file.has_value() &&
          build.mode_stream_canon.is_send == is_send &&
          session->files_[*build.mode_stream_canon.file].process ==
              topic.process) {
        session->modes_ = std::move(build.mode_stream.intervals);
        session->modes_dropped_ = build.mode_stream.dropped;
        if (!session->modes_.empty() && !build.mode_stream.closed_last) {
          session->modes_.back().end_ns =
              std::max(session->modes_.back().start_ns, session->end_ns_);
        }
        session->modes_.shrink_to_fit();
        build.mode_stream = ModeStream{};
      }
    } else if (topic.decoder == Decoder::kStruct) {
      const std::size_t build_idx = sorted_build_index[t];
      TopicBuild& build = indexer.topics[build_idx];
      if (build.alert_stream_canon.file.has_value() &&
          build.alert_stream_canon.is_send == is_send &&
          session->files_[*build.alert_stream_canon.file].process ==
              topic.process) {
        AlertStream& stream = build.alert_stream;
        const auto& alert_channels = build.alert_channels;
        for (std::size_t i = 0; i < alert_channels.size(); ++i) {
          AlertChannelState& st = stream.channels[i];
          const ChannelRef ref{t, alert_channels[i]};
          const std::size_t closed_count = st.intervals.size();
          for (const RawAlertInterval& iv : st.intervals) {
            combined_alerts.push_back(AlertInterval::Make(
                iv.start_ns, iv.end_ns, ref, /*active_at_end=*/false));
          }
          std::vector<RawAlertInterval>{}.swap(st.intervals);
          if (st.active) {
            if (closed_count < kMaxTimelineAlerts) {
              combined_alerts.push_back(AlertInterval::Make(
                  st.start_ns, std::max(st.start_ns, session->end_ns_), ref,
                  /*active_at_end=*/true));
            } else {
              SaturatingAdd(st.dropped, 1);
            }
          }
          SaturatingAdd(session->alerts_dropped_, st.dropped);
          if (combined_alerts.size() > kMaxTimelineAlerts) {
            sort_alerts();
            SaturatingAdd(session->alerts_dropped_,
                          combined_alerts.size() - kMaxTimelineAlerts);
            combined_alerts.resize(kMaxTimelineAlerts);
          }
        }
        stream = AlertStream{};
      }
    }
  }
  sort_alerts();
  session->alerts_ = std::move(combined_alerts);
  session->alerts_.shrink_to_fit();

  std::optional<std::pair<std::int64_t, std::int64_t>> origin;
  for (const LogFile& log_file : session->files_) {
    const tlog::WindowedLogReader reader{log_file.path};
    const std::int64_t monotonic_ns = reader.start_time().nanos();
    const std::int64_t wall_ns = reader.start_wall_ns();
    if (!origin || monotonic_ns < origin->first) {
      origin = {monotonic_ns, wall_ns};
    }
  }
  if (origin) {
    session->origin_wall_ns_ =
        ClampedSubI64(origin->second, origin->first,
                      std::numeric_limits<std::int64_t>::min());
  }
  if (manifest) {
    ReadManifestRuns(*manifest, session->origin_wall_ns_, session->runs_,
                     session->runs_dropped_);
    const auto cfg_it = manifest->find("config_path");
    if (cfg_it != manifest->end() && cfg_it->is_string()) {
      const std::string& cfg = cfg_it->get_ref<const std::string&>();
      if (!cfg.empty()) {
        if (auto pose = ReadPoseTopicFromConfig(dir, cfg)) {
          session->pose_topic_ = std::move(*pose);
        }
      }
    }
  }
  return session;
}

std::optional<ChannelRef> Session::Find(std::string_view name) const {
  const auto it = by_name_.find(std::string{name});
  if (it == by_name_.end()) return std::nullopt;
  return it->second;
}

std::size_t Session::log_seconds() const {
  std::size_t seconds = 0;
  for (const LogFile& file : files_) seconds += file.checkpoints.size();
  return seconds;
}

std::size_t Session::index_bytes() const {
  std::size_t bytes =
      log_seconds() * sizeof(Checkpoint) + gaps_.size() * sizeof(Gap);
  for (const LogFile& file : files_) {
    bytes += file.overflow_bits.size() * sizeof(std::uint64_t) +
             file.overflow_source_ids.size() * sizeof(std::uint16_t);
  }
  return bytes;
}

bool Session::Decode(const Topic& topic, std::span<const std::byte> payload,
                     std::vector<std::uint64_t>& raw,
                     std::vector<std::uint8_t>& present) const {
  const std::size_t count = topic.channels.size();
  raw.assign(count, 0);
  present.assign(count, 0);
  switch (topic.decoder) {
    case Decoder::kStruct: {
      bool any = false;
      for (std::size_t i = 0; i < count; ++i) {
        present[i] =
            talos::telemetry::ReadChannel(topic.channels[i], payload, raw[i])
                ? 1
                : 0;
        any = any || present[i] != 0;
      }
      return any;
    }
    case Decoder::kDriverStation: {
      std::array<std::uint64_t, talos::telemetry::kDriverStationChannelCount>
          values{};
      if (!talos::telemetry::ReadDriverStation(payload, values)) return false;
      for (std::size_t i = 0; i < count && i < values.size(); ++i) {
        raw[i] = values[i];
        present[i] = 1;
      }
      return true;
    }
    case Decoder::kHardware: {
      const auto& index =
          hardware_index_[static_cast<std::size_t>(&topic - topics_.data())];
      std::vector<HardwareSample> samples;
      FlattenHardware(topic.name, topic.hardware, payload, samples);
      for (const HardwareSample& sample : samples) {
        const auto it = index.find(sample.name);
        if (it == index.end()) continue;
        const Channel& channel = topic.channels[it->second];
        // A null is a number with no finite reading: no reading at all.
        if (sample.null || sample.kind != channel.kind) continue;
        raw[it->second] = sample.raw;
        present[it->second] = 1;
      }
      return !samples.empty();
    }
  }
  return false;
}

namespace {

// The sources of the requested topics, per file: (source id, topic index).
using FileSources =
    std::map<std::size_t, std::vector<std::pair<std::uint16_t, std::uint32_t>>>;

void Count(const tlog::LogReader::Record& record, ReadStats* stats) {
  if (stats == nullptr) return;
  ++stats->records;
  stats->bytes += sizeof(tlog::RecordHeader) + record.header.stored_bytes;
}

FileSources SourcesByFile(const std::vector<Topic>& topics,
                          const std::vector<std::uint32_t>& wanted) {
  FileSources by_file;
  for (const std::uint32_t t : wanted) {
    for (const Topic::Source& source : topics[t].sources) {
      by_file[source.file].emplace_back(source.source_id, t);
    }
  }
  return by_file;
}

}  // namespace

std::vector<Series> Session::ReadSeries(std::span<const ChannelRef> channels,
                                        std::int64_t start_ns,
                                        std::int64_t end_ns,
                                        std::uint32_t points,
                                        ReadStats* stats) const {
  if (end_ns <= start_ns || points == 0) {
    throw std::invalid_argument(
        "a series needs end_ns > start_ns and points > 0");
  }
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  std::vector<Series> out(channels.size());
  std::vector<std::vector<std::int64_t>> last_ns(channels.size());
  // Per topic: which outputs want which of its channels.
  std::map<std::uint32_t, std::vector<std::pair<std::size_t, std::uint32_t>>>
      wants;
  for (std::size_t i = 0; i < channels.size(); ++i) {
    Series& series = out[i];
    series.t_ns.assign(points, kNaN);
    series.min.assign(points, std::numeric_limits<double>::infinity());
    series.max.assign(points, -std::numeric_limits<double>::infinity());
    series.last.assign(points, kNaN);
    series.count.assign(points, 0);
    last_ns[i].assign(points, std::numeric_limits<std::int64_t>::min());
    wants[channels[i].topic].emplace_back(i, channels[i].channel);
  }
  std::vector<std::uint32_t> wanted_topics;
  wanted_topics.reserve(wants.size());
  for (const auto& [topic, unused] : wants) wanted_topics.push_back(topic);

  const std::int64_t start_second = FloorSecond(start_ns);
  const std::int64_t end_second = FloorSecond(end_ns);
  const auto span_ns = static_cast<__int128>(end_ns) - start_ns;
  std::vector<std::uint64_t> raw;
  std::vector<std::uint8_t> present;
  for (const auto& [file_index, sources] :
       SourcesByFile(topics_, wanted_topics)) {
    const LogFile& file = files_[file_index];
    const auto& checkpoints = file.checkpoints;
    const std::int64_t first_second = start_second - file.max_lag_s;
    const std::int64_t last_second = end_second + file.max_lead_s;
    const auto first = std::lower_bound(
        checkpoints.begin(), checkpoints.end(), first_second,
        [](const Checkpoint& c, std::int64_t s) { return c.second < s; });
    const auto end_it = std::upper_bound(
        first, checkpoints.end(), last_second,
        [](std::int64_t s, const Checkpoint& c) { return s < c.second; });
    std::uint64_t low_mask = 0;
    bool fallback_overflow = false;
    std::vector<std::pair<std::size_t, std::uint64_t>> overflow_checks;
    for (const auto& [id, topic] : sources) {
      if (id < 63) {
        low_mask |= (std::uint64_t{1} << id);
      } else if (const auto bit = file.OverflowBit(id)) {
        overflow_checks.emplace_back(*bit / 64,
                                     std::uint64_t{1} << (*bit % 64));
      } else if (file.overflow_capped) {
        fallback_overflow = true;
      }
    }
    const auto cp_holds = [&](std::size_t cp_idx) {
      const std::uint64_t mask = checkpoints[cp_idx].sources;
      if ((mask & low_mask) != 0) return true;
      if ((mask & (std::uint64_t{1} << 63)) == 0) return false;
      if (fallback_overflow) return true;
      if (overflow_checks.empty()) return false;
      if (file.overflow_words_per_cp == 0) return true;
      const std::size_t base = cp_idx * file.overflow_words_per_cp;
      if (base >= file.overflow_bits.size()) return true;
      for (const auto& [word_off, bit_mask] : overflow_checks) {
        const std::size_t idx = base + word_off;
        if (idx >= file.overflow_bits.size() ||
            (file.overflow_bits[idx] & bit_mask) != 0) {
          return true;
        }
      }
      return false;
    };
    const auto first_idx =
        static_cast<std::size_t>(first - checkpoints.begin());
    const auto end_idx = static_cast<std::size_t>(end_it - checkpoints.begin());
    std::optional<std::size_t> first_match;
    std::size_t last_match = 0;
    for (std::size_t cp = first_idx; cp < end_idx; ++cp) {
      if (!cp_holds(cp)) continue;
      if (!first_match) first_match = cp;
      last_match = cp;
    }
    if (!first_match) continue;
    const std::uint64_t stop_offset =
        last_match + 1 < checkpoints.size()
            ? checkpoints[last_match + 1].offset
            : std::numeric_limits<std::uint64_t>::max();
    if (stats != nullptr) ++stats->seeks;
    tlog::LogVisitor visitor;
    visitor.manifest = [](std::span<const tlog::ManifestEntry>) {
      return true;
    };
    visitor.record = [&](const tlog::LogReader::Record& record) {
      if (record.offset >= stop_offset) return false;
      const auto& header = record.header;
      Count(record, stats);
      const auto source = std::find_if(
          sources.begin(), sources.end(),
          [&](const auto& s) { return s.first == header.source_id; });
      if (source == sources.end()) return true;
      const Topic& topic = topics_[source->second];
      if (header.kind != static_cast<std::uint16_t>(topic.kind) ||
          !IsValueRecord(header)) {
        return true;
      }
      const std::int64_t publish_ns = header.publish_time_ns;
      if (publish_ns < start_ns || publish_ns >= end_ns) return true;
      if (!Decode(topic, record.payload, raw, present)) return true;
      const auto bucket = static_cast<std::size_t>(
          (static_cast<__int128>(publish_ns) - start_ns) * points / span_ns);
      for (const auto& [output, channel] : wants.at(source->second)) {
        if (present[channel] == 0) continue;
        const double value =
            ValueOf(topic.channels[channel].kind, raw[channel]);
        Series& series = out[output];
        series.min[bucket] = std::fmin(series.min[bucket], value);
        series.max[bucket] = std::fmax(series.max[bucket], value);
        ++series.count[bucket];
        if (publish_ns >= last_ns[output][bucket]) {
          last_ns[output][bucket] = publish_ns;
          series.last[bucket] = value;
          series.t_ns[bucket] = static_cast<double>(publish_ns);
        }
      }
      return true;
    };
    tlog::VisitLog(file.path, visitor,
                   tlog::VisitOptions{checkpoints[*first_match].offset});
  }
  for (Series& series : out) {
    for (std::size_t b = 0; b < points; ++b) {
      // Empty, or every sample NaN (fmin and fmax skip one, so min stays
      // +Inf above max's -Inf): no extremes. The count stays.
      if (series.count[b] == 0 || series.min[b] > series.max[b]) {
        series.min[b] = series.max[b] = kNaN;
      }
    }
  }
  return out;
}

std::vector<Sample> Session::Snapshot(std::int64_t t_ns, ReadStats* stats,
                                      std::size_t max_channels) const {
  if (max_channels == 0 || topics_.empty()) return {};
  struct Latest {
    std::int64_t publish_ns{std::numeric_limits<std::int64_t>::min()};
    std::size_t file_index{0};
    std::vector<std::uint64_t> raw;
    std::vector<std::uint8_t> present;
  };
  std::vector<Latest> latest(topics_.size());
  std::vector<std::uint8_t> topic_has_value(topics_.size(), 0);
  std::size_t topic_cutoff = topics_.size();
  std::size_t filled_channels = 0;

  std::vector<std::uint32_t> all(topics_.size());
  for (std::uint32_t t = 0; t < all.size(); ++t) all[t] = t;
  const std::int64_t target_second = FloorSecond(t_ns);
  std::vector<std::uint64_t> raw;
  std::vector<std::uint8_t> present;

  // Order files by their smallest topic index so earlier topics in output
  // order fill max_channels first, letting later files be skipped completely.
  const FileSources by_file = SourcesByFile(topics_, all);
  std::vector<std::pair<std::size_t, std::uint32_t>> file_order;
  file_order.reserve(by_file.size());
  for (const auto& [file_index, sources] : by_file) {
    std::uint32_t min_topic = std::numeric_limits<std::uint32_t>::max();
    for (const auto& [id, topic] : sources) {
      min_topic = std::min(min_topic, topic);
    }
    file_order.emplace_back(file_index, min_topic);
  }
  std::sort(file_order.begin(), file_order.end(),
            [](const auto& a, const auto& b) {
              if (a.second != b.second) return a.second < b.second;
              return a.first < b.first;
            });

  for (const auto& [file_index, min_topic] : file_order) {
    if (min_topic >= topic_cutoff) continue;
    const auto& sources = by_file.at(file_index);
    const LogFile& file = files_[file_index];
    const auto& checkpoints = file.checkpoints;
    const std::int64_t last_second = target_second + file.max_lead_s;
    struct Item {
      std::uint16_t id;
      std::uint32_t topic;
      std::optional<std::size_t> overflow_bit;
      std::ptrdiff_t candidate;
      std::ptrdiff_t found_checkpoint{-1};
      Latest found;
    };
    // The latest second at or before `from` holding `item`'s source, or -1.
    const auto previous = [&](const Item& item, std::ptrdiff_t from) {
      for (std::ptrdiff_t c = from; c >= 0; --c) {
        const auto cp_idx = static_cast<std::size_t>(c);
        if (checkpoints[cp_idx].second <= last_second &&
            file.HasSource(cp_idx, item.id, item.overflow_bit)) {
          return c;
        }
      }
      return std::ptrdiff_t{-1};
    };
    std::vector<Item> items;
    items.reserve(sources.size());
    for (const auto& [id, topic] : sources) {
      if (topic >= topic_cutoff) continue;
      Item item{id, topic, file.OverflowBit(id), -1, -1, {}};
      item.candidate =
          previous(item, static_cast<std::ptrdiff_t>(checkpoints.size()) - 1);
      if (item.candidate >= 0) items.push_back(std::move(item));
    }
    const auto mark_topic_found = [&](std::uint32_t t) {
      if (t >= topic_cutoff || topic_has_value[t] != 0) return;
      topic_has_value[t] = 1;
      filled_channels += topics_[t].channels.size();
      if (filled_channels < max_channels) return;
      std::size_t sum = 0;
      std::size_t new_cutoff = topic_cutoff;
      for (std::size_t j = 0; j < topic_cutoff; ++j) {
        if (topic_has_value[j] == 0) continue;
        sum += topics_[j].channels.size();
        if (sum >= max_channels) {
          new_cutoff = j + 1;
          break;
        }
      }
      if (new_cutoff < topic_cutoff) {
        for (std::size_t j = new_cutoff; j < topic_cutoff; ++j) {
          if (topic_has_value[j] != 0) {
            filled_channels -= topics_[j].channels.size();
          }
          latest[j].raw.clear();
          latest[j].raw.shrink_to_fit();
          latest[j].present.clear();
          latest[j].present.shrink_to_fit();
        }
        topic_cutoff = new_cutoff;
        for (Item& item : items) {
          if (item.topic >= topic_cutoff) {
            item.candidate = -1;
            item.found.raw.clear();
            item.found.raw.shrink_to_fit();
            item.found.present.clear();
            item.found.present.shrink_to_fit();
          }
        }
      }
    };
    // Seconds latest first, each read once, for every item whose candidate
    // it is; an item keeps checking earlier checkpoints until none can hold a
    // newer publication (accounting for publish/receipt skew).
    while (true) {
      std::ptrdiff_t second = -1;
      for (const Item& item : items) second = std::max(second, item.candidate);
      if (second < 0) break;
      const auto at = static_cast<std::size_t>(second);
      const std::uint64_t stop =
          at + 1 < checkpoints.size()
              ? checkpoints[at + 1].offset
              : std::numeric_limits<std::uint64_t>::max();
      if (stats != nullptr) ++stats->seeks;
      tlog::LogVisitor visitor;
      visitor.manifest = [](std::span<const tlog::ManifestEntry>) {
        return true;
      };
      visitor.record = [&](const tlog::LogReader::Record& record) {
        if (record.offset >= stop) return false;
        Count(record, stats);
        const auto& header = record.header;
        for (Item& item : items) {
          if (item.candidate != second || item.id != header.source_id ||
              item.topic >= topic_cutoff) {
            continue;
          }
          const Topic& topic = topics_[item.topic];
          if (header.kind != static_cast<std::uint16_t>(topic.kind) ||
              !IsValueRecord(header) || header.publish_time_ns > t_ns ||
              header.publish_time_ns < item.found.publish_ns ||
              (header.publish_time_ns == item.found.publish_ns &&
               item.found_checkpoint > second)) {
            continue;
          }
          if (!Decode(topic, record.payload, raw, present)) continue;
          item.found.publish_ns = header.publish_time_ns;
          item.found.raw = raw;
          item.found.present = present;
          item.found_checkpoint = second;
          mark_topic_found(item.topic);
        }
        return true;
      };
      tlog::VisitLog(file.path, visitor,
                     tlog::VisitOptions{checkpoints[at].offset});
      for (Item& item : items) {
        if (item.candidate != second) continue;
        if (item.topic >= topic_cutoff) {
          item.candidate = -1;
          continue;
        }
        if (item.found.raw.empty()) {
          item.candidate = previous(item, second - 1);
        } else {
          const std::int64_t min_second =
              FloorSecond(item.found.publish_ns) - file.max_lag_s;
          const std::ptrdiff_t prev = previous(item, second - 1);
          item.candidate =
              (prev >= 0 &&
               checkpoints[static_cast<std::size_t>(prev)].second >= min_second)
                  ? prev
                  : -1;
        }
      }
    }
    for (Item& item : items) {
      if (item.topic >= topic_cutoff || item.found.raw.empty()) continue;
      Latest& cur = latest[item.topic];
      if (cur.raw.empty() || item.found.publish_ns > cur.publish_ns ||
          (item.found.publish_ns == cur.publish_ns &&
           file_index >= cur.file_index)) {
        item.found.file_index = file_index;
        cur = std::move(item.found);
      }
    }
  }
  std::vector<Sample> samples;
  samples.reserve(std::min(filled_channels, max_channels));
  for (std::uint32_t t = 0; t < topic_cutoff; ++t) {
    const Latest& value = latest[t];
    for (std::uint32_t c = 0; c < value.raw.size(); ++c) {
      if (samples.size() >= max_channels) return samples;
      samples.push_back(
          {{t, c}, value.raw[c], value.publish_ns, value.present[c] != 0});
    }
  }
  return samples;
}

}  // namespace studio::review
