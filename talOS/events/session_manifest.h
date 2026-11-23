#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

// The session manifest, `<session>/manifest.json`, as the tools below Studio
// read it: input replay (a member's log_gap, log_capped and recorder_failed
// marks, per-run gap_logs and recorder_failed_logs, and its log_paths,
// input_replay.h), the launcher's log tools
// (session_id, hardware_protocol_version and config_path,
// talOS/launcher/hardware_record.h) and session retention (simulation,
// start_wall_ns and end_wall_ns, talOS/launcher/session_retention.h). The
// launcher writes it (talOS/launcher/session.cc SessionManifest::ToJson, the
// format in talOS/launcher/README.md "Session manifest") with these key names,
// so a key the writer renames is renamed here too. It lives here, below both,
// because the node stack may not depend on the launcher.
namespace talos::event::session_manifest {

inline constexpr std::string_view kFileName = "manifest.json";

// Maximum manifest.json size ReadText will load (4 MiB, matching Studio's
// kMaxManifestBytes in studio/review/review.h).
inline constexpr std::size_t kMaxManifestBytes = std::size_t{4} << 20;

// The members this reader reads, as the launcher writes them.
inline constexpr std::string_view kSessionId = "session_id";
inline constexpr std::string_view kConfigPath = "config_path";
inline constexpr std::string_view kHardwareProtocolVersion =
    "hardware_protocol_version";
inline constexpr std::string_view kSimulation = "simulation";
inline constexpr std::string_view kStartWallNs = "start_wall_ns";
inline constexpr std::string_view kEndWallNs = "end_wall_ns";
inline constexpr std::string_view kNodes = "nodes";
inline constexpr std::string_view kLogPaths = "log_paths";
inline constexpr std::string_view kGapLogs = "gap_logs";
inline constexpr std::string_view kRecorderFailedLogs = "recorder_failed_logs";
inline constexpr std::string_view kLogGap = "log_gap";
inline constexpr std::string_view kLogCapped = "log_capped";
inline constexpr std::string_view kRecorderFailed = "recorder_failed";

// One entry of `nodes`: what the session says its logs lost. A mark it does
// not carry reads false; per-run arrays read absent in legacy manifests.
struct Member {
  // Absent when the entry has no log_paths.
  std::optional<std::vector<std::string>> log_paths;
  // Absent when the entry has no gap_logs (legacy manifest).
  std::optional<std::vector<std::string>> gap_logs;
  // Absent when the entry has no recorder_failed_logs (legacy manifest).
  std::optional<std::vector<std::string>> recorder_failed_logs;
  bool log_gap{false};
  bool log_capped{false};
  bool recorder_failed{false};
};

// The top-level members read, each absent when the manifest has none (or,
// for the integers, one that is not an unsigned decimal integer that fits in
// 64 bits; for simulation, one that is not true or false; for config_path,
// one that is not a string).
struct Fields {
  std::optional<std::uint64_t> session_id;
  std::optional<std::uint64_t> hardware_protocol_version;
  std::optional<std::string> config_path;
  std::optional<bool> simulation;
  std::optional<std::uint64_t> start_wall_ns;
  std::optional<std::uint64_t> end_wall_ns;
  std::vector<Member> nodes;
};

// A manifest that is not JSON, or whose nodes, marks or log_paths are not
// what the launcher writes.
class ManifestError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Reads `text`, a manifest.json, in one pass over it (members it does not
// read are skipped; nesting deeper than 32 is refused). A top-level value
// that is not an object reads as no members. Throws ManifestError.
Fields Parse(std::string_view text);

// The text of `<directory>/manifest.json`; nullopt when it cannot be opened,
// is not a regular file, or exceeds kMaxManifestBytes.
std::optional<std::string> ReadText(const std::filesystem::path& directory);

}  // namespace talos::event::session_manifest
