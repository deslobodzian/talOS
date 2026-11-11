#pragma once

// What retention (Launcher::ApplyRetention, launcher_retention.cc) needs to
// know about a recorded session beyond its size: when it started, and whether
// it ran a match; and how much room the log root's filesystem has left. Also
// whether it ran under --sim, which input replay repeats.

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string_view>

namespace talos::launcher {

// How many of the newest match and unclassifiable (kUnknown) sessions
// retention protects: never removed for log_keep_sessions or the byte budget
// older sessions get, which does not count them; only, last and oldest first,
// to keep room for the new session free on the log root's filesystem
// (FreeFloor, launcher_retention.cc).
inline constexpr int kProtectedMatches = 16;

// Where a launch records, in its own session directory, every session its
// retention removed or could not remove and why, so the record stays with
// the session whatever becomes of the launcher's stderr.
inline constexpr std::string_view kRetentionRecordFile = "retention.txt";

// Where RecordedMatch caches its verdict inside the session directory, so a
// session's logs are normally read once rather than at every launch.
inline constexpr std::string_view kSessionClassFile = "session_class";

enum class MatchVerdict : std::uint8_t {
  kOther,  // Its logs hold Driver Station packets, none FMS-attached, or none.
  kMatch,  // A logged Driver Station packet was FMS-attached.
  kUnknown,  // A log that may hold the answer cannot be read by this build.
};

struct SessionFacts {
  // The manifest's start_wall_ns: the session's age key. 0 when the
  // directory holds no manifest.json or it records none (replay output, a
  // manifest cut before its first write).
  std::int64_t start_wall_ns{0};
  // RecordedMatch, for a non-simulation launch's manifest; kOther otherwise.
  MatchVerdict verdict{MatchVerdict::kOther};
  // The manifest records a nonzero end_wall_ns: its launcher shut down, so
  // the session is no longer recording. False without one.
  bool finished{false};
};

// Reads `dir`'s manifest.json and, for a launch that was not a simulation,
// RecordedMatch. Never throws; a manifest that cannot be read reads as
// absent.
SessionFacts InspectSession(const std::filesystem::path& dir);

// Whether the session in `dir` was launched with --sim, as its manifest.json
// records; nullopt when it holds no readable manifest that says. Never
// throws.
std::optional<bool> SessionSimulation(const std::filesystem::path& dir);

// Whether the session recorded in `dir` saw a Driver Station attached to the
// FMS: a /hw/state/driver_station packet with kFmsAttached received by any
// node (a watcher or fetcher of that topic; its publisher logs no packets).
// Nodes are read smallest first, each log streamed, until one holds such a
// packet (event::log::VisitLog). kUnknown when none does but a log is of
// another format version (or not a log), or holds a packet this build cannot
// decode. The verdict is cached in kSessionClassFile and read before any log
// is opened: a match for good, kOther until the session's event log bytes
// change, kUnknown until they change or this build's reader (log format,
// Packet size, Driver Station codec) differs. Never throws.
MatchVerdict RecordedMatch(const std::filesystem::path& dir);

// The bytes an unprivileged writer may still use on the filesystem holding
// `path` (statvfs: f_bavail * f_frsize), or nullopt when it cannot be read.
std::optional<std::uint64_t> AvailableBytes(const std::filesystem::path& path);

}  // namespace talos::launcher
