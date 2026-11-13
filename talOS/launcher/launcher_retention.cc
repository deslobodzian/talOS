// The session directory and log retention: Launcher's members that list,
// classify and prune earlier sessions under the log root before the new
// session's directory is created.

#include <sys/stat.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <vector>

#include "talOS/events/log/format.h"
#include "talOS/introspection/lock_file.h"
#include "talOS/launcher/child_output.h"
#include "talOS/launcher/launcher.h"
#include "talOS/launcher/session_retention.h"

namespace talos::launcher {
namespace {

// What retention makes of a directory under the log root.
enum class Retained {
  kNo,         // Not a session: left alone.
  kSession,    // A recording: counted against log_keep_sessions and the
               // byte budget, removed in ApplyRetention's order.
  kAbandoned,  // A launch that started no node and recorded nothing:
               // never counted, removed once another session directory
               // is newer.
};

// The id a session directory is named by -- std::to_string of a nonzero
// session id, as the constructor names it -- or nullopt for any other name.
std::optional<uint64_t> SessionIdNamed(std::string_view name) {
  uint64_t id = 0;
  const char* const end = name.data() + name.size();
  const auto [stop, error] = std::from_chars(name.data(), end, id);
  if (error != std::errc{} || stop != end || id == 0 ||
      std::to_string(id) != name)
    return std::nullopt;
  return id;
}

// The session id in the header of the event log at `path`
// (talOS/events/log/format.h), or 0 when the file does not start with one.
// Reads the fixed-size header only.
uint64_t LogSessionId(const std::filesystem::path& path) {
  event::log::FileHeader header{};
  std::ifstream in{path, std::ios::binary};
  if (!in.read(reinterpret_cast<char*>(&header), sizeof(header)) ||
      std::memcmp(header.magic, event::log::FILE_MAGIC, sizeof(header.magic)) !=
          0)
    return 0;
  return header.session_id;
}

// Classifies `entry`, one entry of the log root; a symlink is never a
// session. A session is a directory holding the manifest.json of a launch,
// or, named by a session id, an event log whose header carries that id
// (what --replay-inputs leaves). A directory named by a session id that
// holds nothing, or only what a launch writes before it starts its nodes
// -- graph.json, the declared graph, an interrupted atomic write's
// temporary, an empty event log, the launcher's own lines
// (kLauncherOutputFile), its directory's lock file -- is an abandoned launch:
// --describe-only, a refused graph, or a launch that failed or died before its
// manifest. Any other content (another session's log, a retention record of
// what that launch removed, any other file or directory) makes it no session.
// Sets `error` when it cannot be read.
Retained Classify(const std::filesystem::directory_entry& entry,
                  std::error_code& error) {
  namespace fs = std::filesystem;
  if (entry.symlink_status(error).type() != fs::file_type::directory)
    return Retained::kNo;
  const fs::file_type manifest =
      fs::status(entry.path() / "manifest.json", error).type();
  if (manifest == fs::file_type::regular) return Retained::kSession;
  if (manifest != fs::file_type::not_found) return Retained::kNo;
  error.clear();  // Absent is an answer, not an error.
  const auto id = SessionIdNamed(entry.path().filename().string());
  if (!id) return Retained::kNo;
  constexpr std::string_view kLaunchFiles[] = {
      "graph.json",        "declared_graph.json",
      "graph.json.tmp",    "declared_graph.json.tmp",
      "manifest.json.tmp", "retention.txt.tmp",
      kLauncherOutputFile, kSessionDirectoryLockFile};
  bool abandoned = true;
  for (fs::directory_iterator file{entry.path(), error};
       !error && file != fs::directory_iterator{}; file.increment(error)) {
    const fs::file_type type = file->symlink_status(error).type();
    if (error) break;
    const std::string name = file->path().filename().string();
    const bool regular = type == fs::file_type::regular;
    if (regular && name.ends_with(".tlog")) {
      if (LogSessionId(file->path()) == *id) return Retained::kSession;
      const uintmax_t bytes = file->file_size(error);
      if (error) break;
      abandoned = abandoned && bytes == 0;
    } else if (!regular ||
               std::ranges::find(kLaunchFiles, std::string_view{name}) ==
                   std::ranges::end(kLaunchFiles)) {
      abandoned = false;
    }
  }
  if (error) return Retained::kNo;
  return abandoned ? Retained::kAbandoned : Retained::kNo;
}

// Bytes in the regular files under `dir`, symlinks not followed. Sets
// `error` when any of the tree cannot be read.
uintmax_t TreeBytes(const std::filesystem::path& dir, std::error_code& error) {
  namespace fs = std::filesystem;
  uintmax_t bytes = 0;
  fs::recursive_directory_iterator file{dir, error};
  while (!error && file != fs::recursive_directory_iterator{}) {
    const fs::file_type type = file->symlink_status(error).type();
    if (error) break;
    if (type == fs::file_type::regular) {
      const uintmax_t size = file->file_size(error);
      if (error) break;
      bytes += size;
    }
    file.increment(error);
  }
  return bytes;
}

// The free space retention keeps on the log root's filesystem for a new
// session of `roster` members, without reading any node's parameters: its
// event-log half of `log_max_bytes`, kChildOutputLogMaxBytes of output log
// per member and for the launcher's own (kLauncherOutputFile), and the
// other half for camera frames and metadata. A session whose frame
// recording exceeds that half is not covered.
uint64_t FreeFloor(const config::RobotConfig& config, std::size_t roster) {
  return static_cast<uint64_t>(config.log_max_bytes) +
         kChildOutputLogMaxBytes * (static_cast<uint64_t>(roster) + 1);
}

// Whether `error` says the filesystem has no room (ENOSPC, EDQUOT).
bool IsNoRoom(const std::error_code& error) {
  return error == std::errc::no_space_on_device ||
         error == std::error_condition{EDQUOT, std::generic_category()};
}

// Where retention moves a session directory to remove it:
// <log root>/<kRemovingPrefix><session directory's name>. Not a session
// name, so no launch ever creates or locks a directory there.
constexpr std::string_view kRemovingPrefix = ".talos-removing-";

// Whether `entry` of the log root is a directory retention moved there to
// remove (kRemovingPrefix), which a removal cut short left behind.
bool IsRemoving(const std::filesystem::directory_entry& entry,
                std::error_code& error) {
  const std::string name = entry.path().filename().string();
  return name.starts_with(kRemovingPrefix) &&
         SessionIdNamed(std::string_view{name}.substr(kRemovingPrefix.size()))
             .has_value() &&
         entry.symlink_status(error).type() ==
             std::filesystem::file_type::directory;
}

}  // namespace

uint64_t Launcher::ApplyRetention(const config::RobotConfig& config,
                                  std::size_t roster, Pruning pruning,
                                  std::string& record) {
  namespace fs = std::filesystem;
  // Config load refuses a wrong type or a nonpositive value.
  const int64_t cap = config.log_keep_sessions;
  const int64_t budget = config.log_max_bytes;
  const auto session_budget = static_cast<uint64_t>(budget) / 2;
  const uint64_t retained_budget =
      static_cast<uint64_t>(budget) - session_budget;
  const uint64_t free_floor = FreeFloor(config, roster);
  if (pruning == Pruning::kNone) return session_budget;
  const bool unlocked = pruning == Pruning::kUnlocked;
  const fs::path current{options_.output_dir};
  // `record` holds removals only: a launch that removed nothing leaves no
  // retention.txt, so the directory of one that stopped before its
  // manifest stays an abandoned launch (Classify), whatever else it saw.
  const auto note = [&record](const std::string& line, bool warn) {
    record += line + "\n";
    if (warn) std::cerr << "[launcher] " << line << "\n";
  };
  // Everything else retention has to say goes to stderr only.
  const auto say = [](const std::string& line) {
    std::cerr << "[launcher] " << line << "\n";
  };
  const auto report = [&say](std::string_view what, const fs::path& path,
                             const std::error_code& error) {
    say("retention " + std::string{what} + " " + path.string() + " (" +
        error.message() + ")");
  };
  struct Session {
    fs::path path;
    fs::file_time_type time;
    uintmax_t bytes;
    int64_t start_ns{0};  // The age key: wall clock nanoseconds.
    MatchVerdict verdict{MatchVerdict::kOther};
    bool protected_match{false};
  };
  // A directory's modification time on the wall clock, for a session whose
  // manifest records no start time.
  const auto wall_ns =
      [file_now = fs::file_time_type::clock::now(),
       wall_now = std::chrono::system_clock::now()](fs::file_time_type time) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   (time - file_now) + wall_now.time_since_epoch())
            .count();
      };
  // Removes the session directory `dir` unless a launch holds its lock, in
  // any shared-memory namespace: takes the lock itself
  // (SessionDirectoryLock::TakeIdle), and while holding it moves the
  // directory out of the session's path, then removes it there. A launch
  // creating that path meanwhile (SessionDirectoryLock::Create) finds the
  // lock held or the directory gone and creates it anew, so no removal
  // reaches a directory a launch has locked. True when it is gone; a launch
  // holding it is recorded, any other failure reported.
  const auto remove_idle = [&](const fs::path& dir) {
    bool running = false;
    std::error_code removal;
    const std::optional<SessionDirectoryLock> lock =
        SessionDirectoryLock::TakeIdle(dir, running, removal,
                                       options_.lock_session_directory);
    if (running) {
      say("retention passes over running session " + dir.string() +
          " (a launch holds its " + std::string{kSessionDirectoryLockFile} +
          ")");
      return false;
    }
    // A directory with no lock file and no room to create one: no launch
    // could create one to hold either.
    if (!lock && !IsNoRoom(removal)) {
      report("cannot remove", dir, removal);
      return false;
    }
    removal.clear();
    const fs::path removing =
        dir.parent_path() /
        (std::string{kRemovingPrefix} + dir.filename().string());
    fs::rename(dir, removing, removal);
    if (removal) {
      report("cannot remove", dir, removal);
      return false;
    }
    fs::remove_all(removing, removal);
    if (removal) report("cannot remove", removing, removal);
    return !removal;
  };
  std::vector<Session> sessions;
  std::vector<Session> abandoned;
  // Directories a removal cut short left behind (kRemovingPrefix).
  std::vector<fs::path> leftovers;
  // The newest session directory other than the new one. A launch still
  // describing has no manifest.json yet and reads as abandoned; one is
  // removed only when another session directory is newer, so the newest
  // launch before this one is kept, and a crash loop of launches that stop
  // before their nodes (refused graphs included) leaves one behind rather
  // than one per attempt, and evicts no recording.
  auto newest = fs::file_time_type::min();
  std::error_code error;
  // No log root yet: nothing to remove.
  if (fs::status(current.parent_path(), error).type() ==
      fs::file_type::not_found)
    return session_budget;
  error.clear();
  for (fs::directory_iterator entry{current.parent_path(), error};
       !error && entry != fs::directory_iterator{}; entry.increment(error)) {
    if (entry->path() == current) continue;  // Counted below.
    std::error_code unreadable;
    if (IsRemoving(*entry, unreadable)) {
      leftovers.push_back(entry->path());
      continue;
    }
    const Retained kind =
        unreadable ? Retained::kNo : Classify(*entry, unreadable);
    if (kind == Retained::kNo) {
      if (unreadable) report("skips", entry->path(), unreadable);
      continue;
    }
    // A launch in any shared-memory namespace is writing it: neither counted
    // nor read.
    if (SessionDirectoryLock::Held(entry->path(),
                                   options_.lock_session_directory)) {
      say("retention passes over running session " + entry->path().string() +
          " (a launch holds its " + std::string{kSessionDirectoryLockFile} +
          ")");
      continue;
    }
    const auto time = entry->last_write_time(unreadable);
    // Before TreeBytes: it may leave its verdict in the directory.
    const SessionFacts facts = unreadable || kind == Retained::kAbandoned
                                   ? SessionFacts{}
                                   : InspectSession(entry->path());
    const uintmax_t bytes = unreadable || kind == Retained::kAbandoned
                                ? 0
                                : TreeBytes(entry->path(), unreadable);
    if (unreadable) {
      report("skips", entry->path(), unreadable);
      continue;
    }
    newest = std::max(newest, time);
    if (kind == Retained::kAbandoned) {
      if (!unlocked) abandoned.push_back({entry->path(), time, bytes});
      continue;
    }
    if (unlocked && !facts.finished) continue;
    sessions.push_back(
        {entry->path(), time, bytes,
         facts.start_wall_ns > 0 ? facts.start_wall_ns : wall_ns(time),
         facts.verdict});
  }
  if (error) {
    // A partial listing could remove a session newer than one it missed.
    report("removes nothing; cannot list", current.parent_path(), error);
    return session_budget;
  }
  // Held only by a retention removing it now.
  for (const auto& leftover : leftovers) {
    if (SessionDirectoryLock::Held(leftover, options_.lock_session_directory))
      continue;
    std::error_code removal;
    fs::remove_all(leftover, removal);
    if (removal) report("cannot remove", leftover, removal);
  }
  for (const auto& launch : abandoned) {
    if (launch.time >= newest) continue;
    remove_idle(launch.path);
  }
  std::sort(
      sessions.begin(), sessions.end(), [](const Session& a, const Session& b) {
        return std::tie(a.start_ns, a.path) < std::tie(b.start_ns, b.path);
      });
  int protected_left = kProtectedMatches;
  for (auto session = sessions.rbegin(); session != sessions.rend();
       ++session) {
    if (session->verdict != MatchVerdict::kOther && protected_left > 0) {
      session->protected_match = true;
      --protected_left;
    }
  }
  // Removal order, each group oldest first: no match or unclassifiable, an
  // older match, one of the protected.
  std::stable_sort(sessions.begin(), sessions.end(),
                   [](const Session& a, const Session& b) {
                     const auto group = [](const Session& session) {
                       return session.protected_match                   ? 2
                              : session.verdict == MatchVerdict::kMatch ? 1
                                                                        : 0;
                     };
                     return group(a) < group(b);
                   });
  // Removes `session`; false when it stays in place (a launch started
  // writing it since it was listed, or it cannot be removed), which passes
  // it over like an unreadable one rather than costing a newer session its
  // place.
  const auto remove_session = [&](const Session& session,
                                  const std::string& why) {
    if (!remove_idle(session.path)) return false;
    const bool warn = session.verdict != MatchVerdict::kOther;
    note(
        std::string{warn ? "WARNING: " : ""} + "retention removes " +
            (session.verdict == MatchVerdict::kMatch ? "match session "
             : warn ? "session (match or not, unreadable by this build) "
                    : "session ") +
            session.path.string() + " (" + std::to_string(session.bytes) +
            " bytes): " + why +
            (session.protected_match ? ", although it is one of the newest " +
                                           std::to_string(kProtectedMatches) +
                                           " matches or unclassifiable sessions"
                                     : std::string{}),
        warn);
    return true;
  };
  // The new session counts too, before it holds anything.
  std::size_t count = sessions.size() + 1;
  uintmax_t total = 0;  // The unprotected sessions' bytes.
  for (const auto& session : sessions) {
    if (!session.protected_match) total += session.bytes;
  }
  auto next = sessions.begin();
  for (; next != sessions.end() && !next->protected_match; ++next) {
    const bool over_bytes = total > retained_budget;
    if (count <= static_cast<uint64_t>(cap) && !over_bytes) break;
    remove_session(*next, over_bytes
                              ? "unprotected older sessions exceed half of "
                                "[robot] log_max_bytes"
                              : "more than [robot] log_keep_sessions");
    total -= next->bytes;
    --count;
  }
  // The disk's floor: what is left goes, protected last, only while the
  // filesystem lacks room for another session.
  std::optional<uint64_t> available =
      options_.available_bytes(current.parent_path());
  if (!available) {
    say("WARNING: retention cannot read the free space under " +
        current.parent_path().string() +
        "; removes nothing to keep room for the new session");
    return session_budget;
  }
  for (; next != sessions.end() && *available < free_floor; ++next) {
    if (remove_session(*next, std::to_string(*available) +
                                  " bytes free under " +
                                  current.parent_path().string() +
                                  ", less than " + std::to_string(free_floor) +
                                  " the new session may need")) {
      *available += next->bytes;
    }
  }
  if (*available < free_floor) {
    say("WARNING: retention leaves " + std::to_string(*available) +
        " bytes free under " + current.parent_path().string() + ", less than " +
        std::to_string(free_floor) +
        " the new session may need, with no older session it can remove");
  }
  return session_budget;
}

bool Launcher::CreateSessionDirectory(const config::RobotConfig& config,
                                      std::size_t roster, bool pruned,
                                      const std::string& record) {
  const std::filesystem::path current{options_.output_dir};
  std::error_code error;
  session_directory_ =
      SessionDirectoryLock::Create(current, options_.create_session_directory,
                                   error, options_.lock_session_directory);
  if (!session_directory_) {
    std::cerr << "launcher: cannot create the session directory "
              << current.string() << " (" << error.message() << ")";
    if (error == std::errc::device_or_resource_busy) {
      std::cerr << "; another launch holds its " << kSessionDirectoryLockFile;
    }
    if (pruned && IsNoRoom(error)) {
      const std::optional<uint64_t> available =
          options_.available_bytes(current.parent_path());
      std::cerr << "; "
                << (available ? std::to_string(*available) + " bytes free"
                              : std::string{"free space unknown"})
                << " under " << current.parent_path().string()
                << " once retention removed what it could, and a new "
                   "session may need "
                << FreeFloor(config, roster) << " ([robot] log_max_bytes plus "
                << kChildOutputLogMaxBytes
                << " per roster member and for talos.log)";
    }
    std::cerr << "; nothing was started\n";
    ReportUnwrittenRetention(record);
    return false;
  }
  if (!session_directory_->held()) {
    std::cerr << "[launcher] WARNING: cannot lock " << current.string() << "/"
              << kSessionDirectoryLockFile << " (" << error.message()
              << "; its filesystem takes no advisory locks): retention in "
                 "another shared-memory namespace may remove this session "
                 "while it runs\n";
  }
  if (record.empty()) return true;
  const std::string path =
      (current / std::string{kRetentionRecordFile}).string();
  std::string write_error;
  if (!WriteFileAtomically(path, record, options_.sync_file, write_error)) {
    std::cerr << "[launcher] retention cannot write " << path << " ("
              << write_error << ")\n";
    ReportUnwrittenRetention(record);
  }
  return true;
}

bool Launcher::LockSharesLogFilesystem() const {
  struct stat lock_dir {};
  if (::stat(introspect::LockDirectory().c_str(), &lock_dir) != 0) return false;
  std::filesystem::path dir =
      std::filesystem::path{options_.output_dir}.parent_path();
  struct stat log_dir {};
  while (::stat(dir.c_str(), &log_dir) != 0) {
    if (errno != ENOENT || !dir.has_relative_path()) return false;
    dir = dir.parent_path();
  }
  return lock_dir.st_dev == log_dir.st_dev;
}

void Launcher::ReportUnwrittenRetention(const std::string& record) {
  if (record.empty()) return;
  std::cerr << "[launcher] retention record, not written:\n" << record;
}

}  // namespace talos::launcher
