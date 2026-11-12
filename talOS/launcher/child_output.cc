#include "talOS/launcher/child_output.h"

#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <pthread.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <limits>
#include <optional>
#include <system_error>
#include <utility>

#include "talOS/launcher/spawn.h"

namespace talos::launcher {
namespace {

// A line marking `count` lost bytes, starting a line of its own.
std::string DropNote(const std::string& before, uint64_t count,
                     std::string_view why) {
  std::string note = before.empty() || before.back() == '\n' ? "" : "\n";
  return note + "[launcher] " + std::to_string(count) +
         " bytes of output dropped here: " + std::string{why} + "\n";
}

// Writes all of `bytes` to `fd`. Returns the bytes left unwritten, with
// errno set, after an error.
std::size_t WriteAll(int fd, std::string_view bytes) {
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

// One output log a writer thread keeps open.
struct LogFile {
  int fd{-1};
  uint64_t written{0};
  bool capped{false};
  bool reported{false};
};

// Appends `bytes` to `path`'s log up to kChildOutputLogMaxBytes. Returns the
// bytes it could not write.
uint64_t WriteLog(LogFile& file, const std::string& path, std::string bytes) {
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

// A pipe for waking a thread out of poll(): close-on-exec, both ends
// nonblocking, so a full one already holds a wakeup.
void WakePipe(int (&fds)[2], const char* what) {
  {
    const std::lock_guard lock{SpawnMutex()};
    if (!CloseOnExecPipe(fds))
      throw std::system_error(errno, std::generic_category(), what);
  }
  for (const int fd : fds)
    ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
}

void WakeUp(const int (&fds)[2]) {
  if (fds[1] < 0) return;
  const char byte = 0;
  [[maybe_unused]] const ssize_t wrote = ::write(fds[1], &byte, 1);
}

void DrainWakes(int reader) {
  char drained[64];
  while (::read(reader, drained, sizeof drained) > 0) {
  }
}

// While it lives, threads the calling thread starts block every signal.
class SignalsBlocked {
 public:
  SignalsBlocked() {
    sigset_t all;
    sigfillset(&all);
    ::pthread_sigmask(SIG_SETMASK, &all, &previous_);
  }
  ~SignalsBlocked() { ::pthread_sigmask(SIG_SETMASK, &previous_, nullptr); }
  SignalsBlocked(const SignalsBlocked&) = delete;
  SignalsBlocked& operator=(const SignalsBlocked&) = delete;

 private:
  sigset_t previous_{};
};

}  // namespace

std::string NodeOutputPath(const std::string& output_dir,
                           std::string_view name) {
  return output_dir + "/" + std::string{name} + ".log";
}

// ---------------------------------------------------------------------------
// LauncherOutput
// ---------------------------------------------------------------------------

LauncherOutput::Sink::int_type LauncherOutput::Sink::overflow(int_type c) {
  if (traits_type::eq_int_type(c, traits_type::eof()))
    return traits_type::not_eof(c);
  const char byte = traits_type::to_char_type(c);
  owner_.Say(fd_, &byte, 1);
  return c;
}

std::streamsize LauncherOutput::Sink::xsputn(const char* data,
                                             std::streamsize size) {
  if (size > 0) owner_.Say(fd_, data, static_cast<std::size_t>(size));
  return size;
}

LauncherOutput::LauncherOutput() {
  WakePipe(wake_, "launcher output wake pipe");
  struct stat seen[2]{};
  bool usable[2]{false, false};
  for (std::size_t i = 0; i < 2; ++i) {
    Descriptor& descriptor = descriptors_[i];
    descriptor.name = i == 0 ? "stdout" : "stderr";
    const int fd = static_cast<int>(i) + 1;
    usable[i] = ::fstat(fd, &seen[i]) == 0;
    // Its own copy: the descriptor number cannot be reused under it.
    descriptor.fd = usable[i] ? ::fcntl(fd, F_DUPFD_CLOEXEC, 3) : -1;
    descriptor.closed = descriptor.fd < 0;
    const mode_t type = seen[i].st_mode & S_IFMT;
    // A pipe that poll() calls writable takes PIPE_BUF bytes without
    // blocking (Linux: a free page; macOS: PIPE_BUF free); a socket's send
    // buffer, a terminal's too, in practice. A file takes what it is given.
    descriptor.chunk = type == S_IFIFO || type == S_IFSOCK || type == S_IFCHR
                           ? std::size_t{PIPE_BUF}
                           : std::numeric_limits<std::size_t>::max();
  }
  if (!descriptors_[0].closed && !descriptors_[1].closed &&
      seen[0].st_dev == seen[1].st_dev && seen[0].st_ino == seen[1].st_ino) {
    // `2>&1`, a terminal: one queue keeps stdout's and stderr's lines in
    // the order they were written.
    target_[1] = 0;
    ::close(descriptors_[1].fd);
    descriptors_[1].fd = -1;
    descriptors_[1].closed = true;
  }
  try {
    const SignalsBlocked blocked;
    descriptor_writer_ = std::thread{[this] { DescriptorLoop(); }};
    file_writer_ = std::thread{[this] { FileLoop(); }};
  } catch (...) {
    Stop(std::chrono::milliseconds{0});
    throw;
  }
  std::cout.clear();
  std::cerr.clear();
  std::clog.clear();
  previous_[0] = std::cout.rdbuf(&out_);
  previous_[1] = std::cerr.rdbuf(&err_);
  previous_[2] = std::clog.rdbuf(&err_);
}

LauncherOutput::~LauncherOutput() { Stop(); }

void LauncherOutput::Record(const std::string& path) {
  const std::lock_guard lock{mutex_};
  file_path_ = path;
}

void LauncherOutput::Echo(std::string_view bytes) {
  {
    const std::lock_guard lock{mutex_};
    Descriptor& out = descriptors_[0];
    if (!out.closed)
      out.dropped += Append(out.pending, bytes.data(), bytes.size());
  }
  Wake();
}

void LauncherOutput::Say(int fd, const char* data, std::size_t size) {
  {
    const std::lock_guard lock{mutex_};
    Descriptor& to = descriptors_[target_[fd == 1 ? 0 : 1]];
    if (!to.closed) to.dropped += Append(to.pending, data, size);
    if (!file_path_.empty()) file_dropped_ += Append(file_pending_, data, size);
  }
  file_wake_.notify_one();
  Wake();
}

std::size_t LauncherOutput::Append(std::string& to, const char* data,
                                   std::size_t size) {
  const std::size_t room =
      kChildOutputBufferBytes - std::min(kChildOutputBufferBytes, to.size());
  const std::size_t kept = std::min(size, room);
  to.append(data, kept);
  return size - kept;
}

void LauncherOutput::Wake() { WakeUp(wake_); }

void LauncherOutput::Stop(std::chrono::milliseconds linger) {
  if (stopped_) return;
  stopped_ = true;
  {
    const std::lock_guard lock{mutex_};
    stopping_ = true;
    linger_ = linger;
    give_up_ = Clock::now() + linger;
  }
  Wake();
  if (descriptor_writer_.joinable()) descriptor_writer_.join();
  {
    const std::lock_guard lock{mutex_};
    descriptors_done_ = true;
  }
  file_wake_.notify_all();
  if (file_writer_.joinable()) file_writer_.join();
  if (previous_[0] != nullptr) {
    std::cout.rdbuf(previous_[0]);
    std::cerr.rdbuf(previous_[1]);
    std::clog.rdbuf(previous_[2]);
  }
  for (Descriptor& descriptor : descriptors_) {
    if (descriptor.fd >= 0) ::close(descriptor.fd);
    descriptor.fd = -1;
  }
  for (int& fd : wake_) {
    if (fd >= 0) ::close(fd);
    fd = -1;
  }
}

void LauncherOutput::DescriptorLoop() {
  // What each descriptor's writer has taken, and how much of it is written.
  std::string batch[2];
  std::size_t written[2]{0, 0};
  for (;;) {
    bool any = false;
    bool stopping = false;
    Clock::time_point give_up;
    {
      const std::lock_guard lock{mutex_};
      for (std::size_t i = 0; i < 2; ++i) {
        Descriptor& descriptor = descriptors_[i];
        if (descriptor.closed) continue;
        if (written[i] == batch[i].size() &&
            (!descriptor.pending.empty() || descriptor.dropped != 0)) {
          batch[i] = std::move(descriptor.pending);
          descriptor.pending.clear();
          written[i] = 0;
          if (descriptor.dropped != 0) {
            batch[i] += DropNote(batch[i], descriptor.dropped,
                                 std::string{"the launcher's "} +
                                     descriptor.name + " fell behind");
            descriptor.dropped = 0;
          }
        }
        if (written[i] < batch[i].size()) any = true;
      }
      stopping = stopping_;
      give_up = give_up_;
    }
    if (stopping && !any) break;
    int timeout = -1;
    if (stopping) {
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                            give_up - Clock::now())
                            .count();
      if (left <= 0) break;
      timeout = static_cast<int>(left) + 1;
    }
    ::pollfd fds[3]{{wake_[0], POLLIN, 0}};
    std::size_t which[3]{};
    nfds_t count = 1;
    for (std::size_t i = 0; i < 2; ++i) {
      if (written[i] >= batch[i].size()) continue;
      which[count] = i;
      fds[count++] = ::pollfd{descriptors_[i].fd, POLLOUT, 0};
    }
    const int ready = ::poll(fds, count, timeout);
    if (ready < 0 && errno == EINTR) continue;
    if (ready < 0) {
      // Nothing says which is writable: try each, which may block here.
      for (nfds_t f = 1; f < count; ++f) fds[f].revents = POLLOUT;
    }
    if (fds[0].revents != 0) DrainWakes(wake_[0]);
    for (nfds_t f = 1; f < count; ++f) {
      if (fds[f].revents == 0) continue;
      const std::size_t i = which[f];
      Descriptor& descriptor = descriptors_[i];
      const std::size_t size =
          std::min(descriptor.chunk, batch[i].size() - written[i]);
      const ssize_t wrote =
          ::write(descriptor.fd, batch[i].data() + written[i], size);
      if (wrote > 0) {
        written[i] += static_cast<std::size_t>(wrote);
        continue;
      }
      if (wrote < 0 &&
          (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
        continue;
      // A write that cannot succeed later (EPIPE: a reader that closed the
      // pipe, the launcher ignoring SIGPIPE; a broken descriptor): nothing
      // more goes to it this session.
      const int error = wrote < 0 ? errno : EIO;
      {
        const std::lock_guard lock{mutex_};
        descriptor.closed = true;
        descriptor.pending.clear();
        descriptor.dropped = 0;
      }
      batch[i].clear();
      written[i] = 0;
      const std::string line = std::string{"[launcher] "} + descriptor.name +
                               ": " + std::strerror(error) +
                               "; the launcher no longer writes to it (its "
                               "lines are in talos.log, node output in each "
                               "<name>.log)\n";
      Say(i == 0 ? 2 : 1, line.data(), line.size());
    }
  }
  // Stopped: what the descriptors did not take within the linger.
  const std::lock_guard lock{mutex_};
  for (std::size_t i = 0; i < 2; ++i) {
    Descriptor& descriptor = descriptors_[i];
    if (descriptor.closed) continue;
    const uint64_t left = batch[i].size() - written[i] +
                          descriptor.pending.size() + descriptor.dropped;
    if (left == 0 || file_path_.empty()) continue;
    const std::string line =
        "[launcher] " + std::to_string(left) + " bytes for the launcher's " +
        descriptor.name + " were not written: it did not drain within " +
        std::to_string(linger_.count()) + " ms of the end\n";
    file_dropped_ += Append(file_pending_, line.data(), line.size());
  }
}

void LauncherOutput::FileLoop() {
  LogFile file;
  std::unique_lock lock{mutex_};
  for (;;) {
    file_wake_.wait(lock, [this] {
      return !file_pending_.empty() || file_dropped_ != 0 || descriptors_done_;
    });
    if (file_pending_.empty() && file_dropped_ == 0) break;  // Done.
    std::string bytes = std::move(file_pending_);
    file_pending_.clear();
    if (file_dropped_ != 0) {
      bytes += DropNote(bytes, file_dropped_,
                        "the launcher's log writer fell behind");
      file_dropped_ = 0;
    }
    const std::string path = file_path_;
    lock.unlock();
    WriteLog(file, path, std::move(bytes));
    lock.lock();
  }
  lock.unlock();
  if (file.fd >= 0) ::close(file.fd);
}

// ---------------------------------------------------------------------------
// ChildOutput
// ---------------------------------------------------------------------------

ChildOutput::ChildOutput(LauncherOutput* echo) : echo_{echo} {
  WakePipe(wake_, "child output wake pipe");
  try {
    const SignalsBlocked blocked;
    reader_ = std::thread{[this] { ReadLoop(); }};
    writer_ = std::thread{[this] { WriteLoop(); }};
  } catch (...) {
    Stop();
    throw;
  }
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
    if (stopping_ && !reader_.joinable() && !writer_.joinable()) return;
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

void ChildOutput::Wake() { WakeUp(wake_); }

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
    dirty_ = true;
  }
  arrived_.notify_all();
  if (stream.echo && echo_ != nullptr) echo_->Echo({data, size});
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
    if (fds[0].revents != 0) DrainWakes(wake_[0]);
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
