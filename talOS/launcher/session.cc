#include "talOS/launcher/session.h"

#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <random>
#include <sstream>
#include <utility>

namespace talos::launcher {

uint64_t GenerateSessionId() {
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  std::random_device rd;
  const uint64_t r = (static_cast<uint64_t>(rd()) << 32) | rd();
  const uint64_t id = static_cast<uint64_t>(now) ^ r;
  return id == 0 ? 1 : id;
}

std::string Quoted(std::string_view value) {
  std::string out;
  introspect::AppendJsonString(out, value);
  return out;
}

bool WriteFileAtomically(const std::string& path, std::string_view contents,
                         const SyncFunction& sync, std::string& error) {
  const std::string temporary = path + ".tmp";
  const auto fail = [&](std::string_view step, int code) {
    std::remove(temporary.c_str());
    error = std::string{step} + ": " + std::strerror(code);
    return false;
  };
  const int temporary_fd =
      ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (temporary_fd < 0) return fail("create", errno);
  FILE* file = ::fdopen(temporary_fd, "wb");
  if (file == nullptr) {
    const int fdopen_error = errno;
    ::close(temporary_fd);
    return fail("create", fdopen_error);
  }
  int code = 0;
  if (std::fwrite(contents.data(), 1, contents.size(), file) !=
          contents.size() ||
      std::fflush(file) != 0 || (sync && sync(::fileno(file)) != 0))
    code = errno;
  if (std::fclose(file) != 0 && code == 0) code = errno;
  if (code != 0) return fail("write", code);
  if (std::rename(temporary.c_str(), path.c_str()) != 0)
    return fail("rename", errno);
  if (!sync) return true;
  const std::filesystem::path parent =
      std::filesystem::path{path}.parent_path();
  const int directory = ::open(parent.empty() ? "." : parent.c_str(),
                               O_RDONLY | O_CLOEXEC | O_DIRECTORY);
  if (directory < 0) return fail("open directory", errno);
  const int result = sync(directory);
  code = errno;
  ::close(directory);
  if (result != 0) return fail("sync directory", code);
  return true;
}

void ManifestWriter::Start() {
  if (running()) return;
  stopping_ = false;
  sigset_t all;
  sigset_t previous;
  sigfillset(&all);
  ::pthread_sigmask(SIG_SETMASK, &all, &previous);
  worker_ = std::thread{[this] { Run(); }};
  ::pthread_sigmask(SIG_SETMASK, &previous, nullptr);
}

void ManifestWriter::Post(std::string contents) {
  {
    const std::lock_guard lock{mutex_};
    pending_ = std::move(contents);
  }
  wake_.notify_one();
}

void ManifestWriter::Stop() {
  if (!running()) return;
  {
    const std::lock_guard lock{mutex_};
    stopping_ = true;
  }
  wake_.notify_one();
  worker_.join();
}

void ManifestWriter::Run() {
  std::unique_lock lock{mutex_};
  for (;;) {
    wake_.wait(lock, [this] { return pending_ || stopping_; });
    if (!pending_) return;
    const std::string contents = std::move(*pending_);
    pending_.reset();
    lock.unlock();
    Persist(contents, sync_);
    lock.lock();
  }
}

void ManifestWriter::Persist(const std::string& contents,
                             const SyncFunction& sync) {
  std::string error;
  if (!WriteFileAtomically(path_, contents, sync, error)) {
    if (!failing_) {
      std::cerr << "[launcher] cannot persist " << path_ << " (" << error
                << "); supervision continues and retries at the next "
                   "change\n";
    }
    failing_ = true;
    return;
  }
  if (failing_) {
    std::cerr << "[launcher] " << path_ << " persisted again\n";
    failing_ = false;
  }
}

int64_t WallClockNanos() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

bool NodeProcessInfo::RecordRealtime(const process::RealtimeOutcome& outcome) {
  if (realtime && realtime->grant() >= outcome.grant()) return false;
  realtime = outcome;
  return true;
}

void NodeProcessInfo::BeginRun(pid_t run_pid) {
  if (runs.size() >= kMaxRecordedRuns) {
    runs.erase(runs.begin() + 1);
    ++runs_dropped;
  }
  NodeRun& run = runs.emplace_back();
  run.pid = run_pid;
  run.start_wall_ns = WallClockNanos();
}

void NodeProcessInfo::EndRun(std::string_view reason) {
  if (runs.empty() || runs.back().end_wall_ns != 0) return;
  NodeRun& run = runs.back();
  run.end_wall_ns = WallClockNanos();
  run.exit_code = exit_code;
  run.exit_signal = exit_signal;
  run.reason = reason;
}

void NodeProcessInfo::RecordSpawnFailure() {
  BeginRun(-1);
  NodeRun& run = runs.back();
  run.end_wall_ns = run.start_wall_ns;
  run.reason = kRunSpawnFailed;
}

std::string RealtimeWarning(std::string_view name, pid_t pid,
                            const process::RealtimeOutcome& outcome,
                            bool simulation) {
  const process::RealtimeGrant grant = outcome.grant();
  if (!outcome.actuation_path() || grant == process::RealtimeGrant::kFull ||
      (simulation && grant == process::RealtimeGrant::kNotRequested)) {
    return "";
  }
  std::string line =
      "[launcher] warning: " + std::string{name} + " (pid " +
      std::to_string(pid) + ") " + process::TierName(outcome.tier) +
      " loop runs without its realtime grant: " + process::GrantName(grant);
  if (outcome.refused != 0) {
    line += " (" + process::RefusedStepNames(outcome.refused) + ")";
  }
  return line + "; the reasons are in its log\n";
}

std::string SessionManifest::ToJson() const {
  std::ostringstream ss;
  ss << "{\n";
  ss << "  \"session_id\": " << session_id << ",\n";
  ss << "  \"config_path\": " << Quoted(config_path) << ",\n";
  ss << "  \"config_sha256\": " << Quoted(config_sha256) << ",\n";
  ss << "  \"git_revision\": " << Quoted(git_revision) << ",\n";
  ss << "  \"git_dirty\": " << (git_dirty ? "true" : "false") << ",\n";
  ss << "  \"output_dir\": " << Quoted(output_dir) << ",\n";
  ss << "  \"simulation\": " << (simulation ? "true" : "false") << ",\n";
  ss << "  \"start_wall_ns\": " << start_wall_ns << ",\n";
  ss << "  \"end_wall_ns\": " << end_wall_ns << ",\n";
  ss << "  \"nodes\": [\n";
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const auto& n = nodes[i];
    ss << "    {\n";
    ss << "      \"name\": " << Quoted(n.name) << ",\n";
    ss << "      \"target\": " << Quoted(n.target) << ",\n";
    ss << "      \"binary_path\": " << Quoted(n.binary_path) << ",\n";
    ss << "      \"binary_sha256\": " << Quoted(n.binary_sha256) << ",\n";
    ss << "      \"log_path\": " << Quoted(n.log_path) << ",\n";
    ss << "      \"log_paths\": [";
    for (std::size_t j = 0; j < n.log_paths.size(); ++j) {
      ss << (j ? ", " : "") << Quoted(n.log_paths[j]);
    }
    ss << "],\n";
    ss << "      \"pid\": " << n.pid << ",\n";
    ss << "      \"exit_code\": " << n.exit_code << ",\n";
    ss << "      \"exit_signal\": " << n.exit_signal << ",\n";
    ss << "      \"restart_count\": " << n.restart_count << ",\n";
    ss << "      \"quarantined\": " << (n.quarantined ? "true" : "false")
       << ",\n";
    ss << "      \"retry_at_monotonic_ns\": " << n.retry_at_monotonic_ns
       << ",\n";
    ss << "      \"recorder_failed\": " << (n.failed ? "true" : "false")
       << ",\n";
    ss << "      \"log_capped\": " << (n.log_capped ? "true" : "false")
       << ",\n";
    ss << "      \"described\": " << (n.described ? "true" : "false") << ",\n";
    ss << "      \"endpoint_mismatch\": "
       << (n.endpoint_mismatch ? "true" : "false") << ",\n";
    ss << "      \"loop_tier\": "
       << Quoted(n.realtime ? process::TierName(n.realtime->tier) : "")
       << ",\n";
    ss << "      \"realtime\": "
       << Quoted(n.realtime ? process::GrantName(n.realtime->grant()) : "")
       << ",\n";
    ss << "      \"realtime_refused\": "
       << Quoted(n.realtime ? process::RefusedStepNames(n.realtime->refused)
                            : "")
       << ",\n";
    ss << "      \"record_count\": " << n.record_count << ",\n";
    ss << "      \"runs_dropped\": " << n.runs_dropped << ",\n";
    // Last: a run is the only nested object in a node's entry.
    ss << "      \"runs\": [";
    for (std::size_t j = 0; j < n.runs.size(); ++j) {
      const NodeRun& run = n.runs[j];
      ss << (j ? ",\n        {" : "\n        {") << "\"pid\": " << run.pid
         << ", \"start_wall_ns\": " << run.start_wall_ns
         << ", \"end_wall_ns\": " << run.end_wall_ns
         << ", \"exit_code\": " << run.exit_code
         << ", \"exit_signal\": " << run.exit_signal
         << ", \"reason\": " << Quoted(run.reason) << "}";
    }
    ss << (n.runs.empty() ? "]\n" : "\n      ]\n");
    ss << "    }" << (i + 1 < nodes.size() ? "," : "") << "\n";
  }
  ss << "  ]\n";
  ss << "}\n";
  return ss.str();
}

std::string NodeLogPath(const std::string& output_dir, std::string_view name,
                        int restart) {
  std::string path = output_dir + "/" + std::string{name};
  if (restart > 0) path += ".restart" + std::to_string(restart);
  return path + ".tlog";
}

}  // namespace talos::launcher
