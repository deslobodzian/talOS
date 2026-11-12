#include "talOS/launcher/child_output.h"

#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <optional>
#include <system_error>
#include <utility>

#include "talOS/launcher/spawn.h"

namespace talos::launcher {

std::string NodeOutputPath(const std::string& output_dir,
                           std::string_view name) {
  return output_dir + "/" + std::string{name} + ".log";
}

ChildOutput::ChildOutput() {
  {
    const std::lock_guard lock{SpawnMutex()};
    if (!CloseOnExecPipe(wake_))
      throw std::system_error(errno, std::generic_category(),
                              "child output wake pipe");
  }
  for (const int fd : wake_)
    ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
  sigset_t all;
  sigset_t previous;
  sigfillset(&all);
  ::pthread_sigmask(SIG_SETMASK, &all, &previous);
  try {
    reader_ = std::thread{[this] { ReadLoop(); }};
    writer_ = std::thread{[this] { WriteLoop(); }};
    echoer_ = std::thread{[this] { EchoLoop(); }};
  } catch (...) {
    ::pthread_sigmask(SIG_SETMASK, &previous, nullptr);
    Stop();
    throw;
  }
  ::pthread_sigmask(SIG_SETMASK, &previous, nullptr);
}

ChildOutput::~ChildOutput() { Stop(); }

void ChildOutput::Add(int reader, const std::string& path,
                      std::string_view name, pid_t pid, bool echo) {
  ::fcntl(reader, F_SETFL, ::fcntl(reader, F_GETFL) | O_NONBLOCK);
  {
    const std::lock_guard lock{mutex_};
    if (stopping_) {
      ::close(reader);
      return;
    }
    // The reader writes the header once it has drained the previous run's
    // pipe, so that run's last output is not credited to this one.
    Pending& pending = pending_[path];
    added_.push_back({reader, path, echo,
                      "[launcher] " + std::string{name} + " run " +
                          std::to_string(pending.runs++) + ", pid " +
                          std::to_string(pid) + "\n"});
    dirty_ = true;
  }
  arrived_.notify_all();
  Wake();
}

void ChildOutput::Stop(std::chrono::milliseconds linger) {
  {
    const std::lock_guard lock{mutex_};
    if (stopping_ && !reader_.joinable() && !writer_.joinable() &&
        !echoer_.joinable())
      return;
    stopping_ = true;
    linger_ = linger;
  }
  Wake();
  if (reader_.joinable()) reader_.join();
  {
    const std::lock_guard lock{mutex_};
    reader_done_ = true;
  }
  arrived_.notify_all();
  if (writer_.joinable()) writer_.join();
  if (echoer_.joinable()) echoer_.join();
  for (int& fd : wake_) {
    if (fd >= 0) ::close(fd);
    fd = -1;
  }
}

uint64_t ChildOutput::Dropped(const std::string& path) const {
  const std::lock_guard lock{mutex_};
  const auto found = pending_.find(path);
  return found == pending_.end() ? 0 : found->second.dropped_total;
}

void ChildOutput::Wake() {
  if (wake_[1] < 0) return;
  const char byte = 0;
  [[maybe_unused]] const ssize_t wrote = ::write(wake_[1], &byte, 1);
}

std::string ChildOutput::DropNote(const std::string& before, uint64_t count,
                                  std::string_view why) {
  std::string note = before.empty() || before.back() == '\n' ? "" : "\n";
  return note + "[launcher] " + std::to_string(count) +
         " bytes of output dropped here: " + std::string{why} + "\n";
}

void ChildOutput::Deliver(const Stream& stream, const char* data,
                          std::size_t size) {
  {
    const std::lock_guard lock{mutex_};
    Pending& pending = pending_[stream.path];
    const std::size_t room =
        kChildOutputBufferBytes -
        std::min(kChildOutputBufferBytes, pending.bytes.size());
    const std::size_t kept = std::min(size, room);
    pending.bytes.append(data, kept);
    pending.dropped += size - kept;
    pending.dropped_total += size - kept;
    if (stream.echo && !echo_closed_) {
      const std::size_t echo_room =
          kChildOutputBufferBytes -
          std::min(kChildOutputBufferBytes, echo_.size());
      const std::size_t echoed = std::min(size, echo_room);
      echo_.append(data, echoed);
      echo_dropped_ += size - echoed;
    }
    dirty_ = true;
  }
  arrived_.notify_all();
}

void ChildOutput::ReadLoop() {
  std::vector<Stream> streams;
  std::vector<Stream> picked;
  std::vector<::pollfd> fds;
  std::vector<char> buffer(std::size_t{64} * 1024);
  std::optional<Clock::time_point> give_up;
  for (;;) {
    {
      const std::lock_guard lock{mutex_};
      picked.swap(added_);
      if (stopping_ && !give_up) give_up = Clock::now() + linger_;
    }
    for (auto& stream : picked) {
      // An earlier run's pipe for the same log may still hold its last
      // output (a restart can follow the exit at once): read what it holds
      // before this run's header. Its descendants may keep it open.
      for (auto& earlier : streams) {
        if (earlier.fd < 0 || earlier.path != stream.path) continue;
        for (;;) {
          const ssize_t got = ::read(earlier.fd, buffer.data(), buffer.size());
          if (got > 0) {
            Deliver(earlier, buffer.data(), static_cast<std::size_t>(got));
          } else if (got < 0 && errno == EINTR) {
            continue;
          } else {
            if (got == 0 || errno != EAGAIN) {
              ::close(earlier.fd);
              earlier.fd = -1;
            }
            break;
          }
        }
      }
      {
        const std::lock_guard lock{mutex_};
        pending_[stream.path].bytes += stream.header;
        dirty_ = true;
      }
      arrived_.notify_all();
      stream.header.clear();
      streams.push_back(std::move(stream));
    }
    picked.clear();
    std::erase_if(streams, [](const Stream& stream) { return stream.fd < 0; });
    if (give_up && (streams.empty() || Clock::now() >= *give_up)) break;
    fds.assign(1, ::pollfd{wake_[0], POLLIN, 0});
    for (const auto& stream : streams)
      fds.push_back(::pollfd{stream.fd, POLLIN, 0});
    int timeout = -1;
    if (give_up) {
      timeout = static_cast<int>(std::max<int64_t>(
          0, std::chrono::duration_cast<std::chrono::milliseconds>(*give_up -
                                                                   Clock::now())
                     .count() +
                 1));
    }
    if (::poll(fds.data(), fds.size(), timeout) < 0) {
      // Never give the pipes up while their children may still write.
      if (errno != EINTR)
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
      continue;
    }
    if (fds[0].revents != 0) {
      char drained[64];
      while (::read(wake_[0], drained, sizeof drained) > 0) {
      }
    }
    std::size_t kept = 0;
    for (std::size_t i = 0; i < streams.size(); ++i) {
      bool open = true;
      if (fds[i + 1].revents != 0) {
        const ssize_t got = ::read(streams[i].fd, buffer.data(), buffer.size());
        if (got > 0) {
          Deliver(streams[i], buffer.data(), static_cast<std::size_t>(got));
        } else if (got == 0 || (errno != EINTR && errno != EAGAIN)) {
          open = false;
        }
      }
      if (open) {
        // Never a self-move, which may empty the path.
        if (kept != i) streams[kept] = std::move(streams[i]);
        ++kept;
      } else {
        ::close(streams[i].fd);
      }
    }
    streams.resize(kept);
  }
  for (const auto& stream : streams) ::close(stream.fd);
}

uint64_t ChildOutput::WriteLog(LogFile& file, const std::string& path,
                               std::string bytes) {
  if (file.capped) return bytes.size();
  if (file.fd < 0) {
    file.fd =
        ::open(path.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    if (file.fd < 0) {
      if (!file.reported)
        std::cerr << "[launcher] cannot open " << path << " ("
                  << std::strerror(errno) << "); its output is dropped\n";
      file.reported = true;
      return bytes.size();
    }
  }
  uint64_t lost = 0;
  const std::string cap_note = "\n[launcher] output log reached " +
                               std::to_string(kChildOutputLogMaxBytes) +
                               " bytes; later output is dropped\n";
  if (file.written + bytes.size() > kChildOutputLogMaxBytes) {
    const uint64_t room = kChildOutputLogMaxBytes -
                          std::min<uint64_t>(kChildOutputLogMaxBytes,
                                             file.written + cap_note.size());
    lost = bytes.size() - std::min<uint64_t>(room, bytes.size());
    bytes.resize(bytes.size() - lost);
    bytes += cap_note;
    file.capped = true;
  }
  const std::size_t unwritten = WriteAll(file.fd, bytes);
  file.written += bytes.size() - unwritten;
  if (unwritten != 0 && !file.reported) {
    std::cerr << "[launcher] cannot write " << path << " ("
              << std::strerror(errno) << "); output is dropped\n";
    file.reported = true;
  }
  return lost + unwritten;
}

std::size_t ChildOutput::WriteAll(int fd, std::string_view bytes) {
  while (!bytes.empty()) {
    const ssize_t wrote = ::write(fd, bytes.data(), bytes.size());
    if (wrote < 0) {
      if (errno == EINTR) continue;
      return bytes.size();
    }
    bytes.remove_prefix(static_cast<std::size_t>(wrote));
  }
  return 0;
}

void ChildOutput::WriteLoop() {
  std::map<std::string, LogFile> files;
  std::unique_lock lock{mutex_};
  for (;;) {
    arrived_.wait(lock, [this] { return dirty_ || reader_done_; });
    if (!dirty_ && reader_done_) break;
    dirty_ = false;
    std::vector<std::pair<std::string, std::string>> batches;
    for (auto& [path, pending] : pending_) {
      if (pending.bytes.empty() && pending.dropped == 0) continue;
      std::string bytes = std::move(pending.bytes);
      pending.bytes.clear();
      if (pending.dropped != 0) {
        bytes += DropNote(bytes, pending.dropped,
                          "the launcher's log writer fell behind");
        pending.dropped = 0;
      }
      batches.emplace_back(path, std::move(bytes));
    }
    lock.unlock();
    std::vector<std::pair<std::string, uint64_t>> lost;
    for (auto& [path, bytes] : batches) {
      const uint64_t count = WriteLog(files[path], path, std::move(bytes));
      if (count != 0) lost.emplace_back(path, count);
    }
    lock.lock();
    for (const auto& [path, count] : lost)
      pending_[path].dropped_total += count;
  }
  lock.unlock();
  for (auto& [path, file] : files)
    if (file.fd >= 0) ::close(file.fd);
}

void ChildOutput::EchoLoop() {
  std::unique_lock lock{mutex_};
  for (;;) {
    arrived_.wait(lock, [this] {
      return !echo_.empty() || echo_dropped_ != 0 || reader_done_;
    });
    if (echo_.empty() && echo_dropped_ == 0) break;  // reader_done_
    std::string echo = std::move(echo_);
    echo_.clear();
    if (echo_dropped_ != 0) {
      echo +=
          DropNote(echo, echo_dropped_, "the launcher's stdout fell behind");
      echo_dropped_ = 0;
    }
    lock.unlock();
    // A write that fails drops the echo. One that cannot succeed later (a
    // pipe whose reader closed it: EPIPE, as the launcher ignores SIGPIPE; a
    // closed or broken descriptor) ends it for the session; only a
    // nonblocking stdout's EAGAIN drops just this batch.
    if (WriteAll(STDOUT_FILENO, echo) != 0 && errno != EAGAIN &&
        errno != EWOULDBLOCK) {
      const int error = errno;
      lock.lock();
      echo_closed_ = true;
      echo_.clear();
      echo_dropped_ = 0;
      lock.unlock();
      std::cerr << "[launcher] stdout: " << std::strerror(error)
                << "; node output is no longer echoed, only logged\n";
    }
    lock.lock();
  }
}

pid_t SpawnDrained(std::vector<std::string> args, std::string& error,
                   ChildOutput& output, const std::string& path,
                   std::string_view name, bool echo) {
  int reader = -1;
  const pid_t pid =
      SpawnProcess(std::move(args), error, SpawnOutput::kPipe, &reader);
  if (pid > 0) output.Add(reader, path, name, pid, echo);
  return pid;
}

}  // namespace talos::launcher
