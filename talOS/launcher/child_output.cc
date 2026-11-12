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

// Everything the writer threads touch.
struct LauncherOutput::State {
  // One real descriptor: 1, or 2 unless it is the same file as 1, whose
  // lines then share 1's queue and keep their order.
  struct Descriptor {
    int fd{-1};           // A close-on-exec copy; -1 when unusable.
    std::size_t chunk{};  // Bytes per write (PIPE_BUF for a pipe or socket).
    const char* name{""};
    // Under mutex:
    std::string pending;
    uint64_t dropped{0};   // Since the writer last took `pending`.
    std::size_t taken{0};  // Taken by the writer and not yet written.
    bool closed{false};
  };

  State() = default;
  // Only once no thread holds it: a writer left inside a write() keeps its
  // descriptor open, so the number is never reused under it.
  ~State() {
    for (Descriptor& descriptor : descriptors)
      if (descriptor.fd >= 0) ::close(descriptor.fd);
    for (const int fd : wake)
      if (fd >= 0) ::close(fd);
  }
  State(const State&) = delete;
  State& operator=(const State&) = delete;

  // Appends `size` bytes written to std::cout (fd 1) or std::cerr and
  // std::clog (fd 2) to that descriptor's buffer and to talos.log's.
  void Say(int fd, const char* data, std::size_t size);

  // Node output for descriptor 1 only.
  void Echo(std::string_view bytes);

  // Appends to `to` up to the buffer bound; returns the bytes dropped.
  static std::size_t Append(std::string& to, const char* data,
                            std::size_t size);

  // A full wake pipe already holds a wakeup.
  void Wake() const { WakeUp(wake); }

  // Under mutex: puts in talos.log's buffer how many bytes each descriptor
  // never took.
  void NoteUnwritten();

  // Writes the descriptors' buffers, waiting on poll() for each to take
  // more; its poll() never waits past the stop's linger.
  void DescriptorLoop();

  // Writes talos.log's buffer until the stop has let it finish and nothing
  // is left.
  void FileLoop();

  // Descriptor 2's lines go to `descriptors[target[1]]`.
  Descriptor descriptors[2];
  std::size_t target[2]{0, 1};
  int wake[2]{-1, -1};
  std::mutex mutex;
  std::condition_variable file_wake;
  std::condition_variable writer_ended;
  // Guarded by mutex:
  std::string file_path;
  std::string file_pending;
  uint64_t file_dropped{0};
  bool stopping{false};
  std::chrono::milliseconds linger{0};
  Clock::time_point give_up{};
  bool descriptors_ended{false};  // DescriptorLoop has returned.
  bool abandoned{false};          // Stop stopped waiting for it.
  bool file_done{false};          // FileLoop may end once it is empty.
};

LauncherOutput::Sink::int_type LauncherOutput::Sink::overflow(int_type c) {
  if (traits_type::eq_int_type(c, traits_type::eof()))
    return traits_type::not_eof(c);
  const char byte = traits_type::to_char_type(c);
  state_.Say(fd_, &byte, 1);
  return c;
}

std::streamsize LauncherOutput::Sink::xsputn(const char* data,
                                             std::streamsize size) {
  if (size > 0) state_.Say(fd_, data, static_cast<std::size_t>(size));
  return size;
}

LauncherOutput::LauncherOutput()
    : state_{std::make_shared<State>()}, out_{*state_, 1}, err_{*state_, 2} {
  State& state = *state_;
  WakePipe(state.wake, "launcher output wake pipe");
  struct stat seen[2]{};
  for (std::size_t i = 0; i < 2; ++i) {
    State::Descriptor& descriptor = state.descriptors[i];
    descriptor.name = i == 0 ? "stdout" : "stderr";
    const int fd = static_cast<int>(i) + 1;
    // Its own copy: the descriptor number cannot be reused under it.
    descriptor.fd =
        ::fstat(fd, &seen[i]) == 0 ? ::fcntl(fd, F_DUPFD_CLOEXEC, 3) : -1;
    descriptor.closed = descriptor.fd < 0;
    const mode_t type = seen[i].st_mode & S_IFMT;
    // A pipe that poll() calls writable takes PIPE_BUF bytes without
    // blocking (Linux: a free page; macOS: PIPE_BUF free), unless another
    // process writes to it in between. A socket or a terminal may take less
    // and block on the rest, which Stop does not wait for. A file takes
    // what it is given.
    descriptor.chunk = type == S_IFIFO || type == S_IFSOCK || type == S_IFCHR
                           ? std::size_t{PIPE_BUF}
                           : std::numeric_limits<std::size_t>::max();
  }
  if (!state.descriptors[0].closed && !state.descriptors[1].closed &&
      seen[0].st_dev == seen[1].st_dev && seen[0].st_ino == seen[1].st_ino) {
    // `2>&1`, a terminal: one queue keeps stdout's and stderr's lines in
    // the order they were written.
    state.target[1] = 0;
    ::close(state.descriptors[1].fd);
    state.descriptors[1].fd = -1;
    state.descriptors[1].closed = true;
  }
  try {
    const SignalsBlocked blocked;
    descriptor_writer_ =
        std::thread{[held = state_] { held->DescriptorLoop(); }};
    file_writer_ = std::thread{[held = state_] { held->FileLoop(); }};
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
  const std::lock_guard lock{state_->mutex};
  state_->file_path = path;
}

void LauncherOutput::Echo(std::string_view bytes) { state_->Echo(bytes); }

void LauncherOutput::State::Echo(std::string_view bytes) {
  {
    const std::lock_guard lock{mutex};
    Descriptor& out = descriptors[0];
    if (!out.closed)
      out.dropped += Append(out.pending, bytes.data(), bytes.size());
  }
  Wake();
}

void LauncherOutput::State::Say(int fd, const char* data, std::size_t size) {
  {
    const std::lock_guard lock{mutex};
    Descriptor& to = descriptors[target[fd == 1 ? 0 : 1]];
    if (!to.closed) to.dropped += Append(to.pending, data, size);
    if (!file_path.empty()) file_dropped += Append(file_pending, data, size);
  }
  file_wake.notify_one();
  Wake();
}

std::size_t LauncherOutput::State::Append(std::string& to, const char* data,
                                          std::size_t size) {
  const std::size_t room =
      kChildOutputBufferBytes - std::min(kChildOutputBufferBytes, to.size());
  const std::size_t kept = std::min(size, room);
  to.append(data, kept);
  return size - kept;
}

void LauncherOutput::State::NoteUnwritten() {
  for (const Descriptor& descriptor : descriptors) {
    if (descriptor.closed) continue;
    const uint64_t left =
        descriptor.taken + descriptor.pending.size() + descriptor.dropped;
    if (left == 0 || file_path.empty()) continue;
    const std::string line =
        "[launcher] " + std::to_string(left) + " bytes for the launcher's " +
        descriptor.name + " were not written: it did not drain within " +
        std::to_string(linger.count()) + " ms of the end\n";
    file_dropped += Append(file_pending, line.data(), line.size());
  }
}

void LauncherOutput::Stop(std::chrono::milliseconds linger) {
  if (stopped_) return;
  stopped_ = true;
  State& state = *state_;
  bool ended = false;
  {
    std::unique_lock lock{state.mutex};
    state.stopping = true;
    state.linger = linger;
    state.give_up = Clock::now() + linger;
    state.Wake();
    // The writer's poll() returns by give_up; only a write() it is inside
    // can keep it longer.
    ended = state.writer_ended.wait_until(
        lock, state.give_up + kLauncherOutputWriterGrace,
        [&state] { return state.descriptors_ended; });
    if (!ended) {
      // From here the writer adds nothing to talos.log.
      state.abandoned = true;
      state.NoteUnwritten();
    }
    state.file_done = true;
  }
  state.file_wake.notify_all();
  if (descriptor_writer_.joinable()) {
    if (ended) {
      descriptor_writer_.join();
    } else {
      descriptor_writer_.detach();
    }
  }
  if (file_writer_.joinable()) file_writer_.join();
  if (previous_[0] != nullptr) {
    std::cout.rdbuf(previous_[0]);
    std::cerr.rdbuf(previous_[1]);
    std::clog.rdbuf(previous_[2]);
  }
}

void LauncherOutput::State::DescriptorLoop() {
  // What each descriptor's writer has taken, and how much of it is written.
  std::string batch[2];
  std::size_t written[2]{0, 0};
  for (;;) {
    bool any = false;
    bool stopped = false;
    Clock::time_point until;
    {
      const std::lock_guard lock{mutex};
      for (std::size_t i = 0; i < 2; ++i) {
        Descriptor& descriptor = descriptors[i];
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
        descriptor.taken = batch[i].size() - written[i];
        if (descriptor.taken != 0) any = true;
      }
      stopped = stopping;
      until = give_up;
    }
    if (stopped && !any) break;
    int timeout = -1;
    if (stopped) {
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                            until - Clock::now())
                            .count();
      if (left <= 0) break;
      timeout = static_cast<int>(left) + 1;
    }
    ::pollfd fds[3]{{wake[0], POLLIN, 0}};
    std::size_t which[3]{};
    nfds_t count = 1;
    for (std::size_t i = 0; i < 2; ++i) {
      if (written[i] >= batch[i].size()) continue;
      which[count] = i;
      fds[count++] = ::pollfd{descriptors[i].fd, POLLOUT, 0};
    }
    const int ready = ::poll(fds, count, timeout);
    if (ready < 0 && errno == EINTR) continue;
    if (ready < 0) {
      // Nothing says which is writable: try each, which may block here.
      for (nfds_t f = 1; f < count; ++f) fds[f].revents = POLLOUT;
    }
    if (fds[0].revents != 0) DrainWakes(wake[0]);
    for (nfds_t f = 1; f < count; ++f) {
      if (fds[f].revents == 0) continue;
      const std::size_t i = which[f];
      Descriptor& descriptor = descriptors[i];
      const std::size_t size =
          std::min(descriptor.chunk, batch[i].size() - written[i]);
      // May block on what the descriptor cannot take at once (a terminal);
      // Stop does not wait for that.
      const ssize_t wrote =
          ::write(descriptor.fd, batch[i].data() + written[i], size);
      if (wrote > 0) {
        written[i] += static_cast<std::size_t>(wrote);
        const std::lock_guard lock{mutex};
        descriptor.taken = batch[i].size() - written[i];
        continue;
      }
      if (wrote < 0 &&
          (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK))
        continue;
      // A write that cannot succeed later (EPIPE: a reader that closed the
      // pipe, the launcher ignoring SIGPIPE; a broken descriptor): nothing
      // more goes to it this session.
      const int error = wrote < 0 ? errno : EIO;
      bool left_behind = false;
      {
        const std::lock_guard lock{mutex};
        descriptor.closed = true;
        descriptor.pending.clear();
        descriptor.dropped = 0;
        descriptor.taken = 0;
        left_behind = abandoned;
      }
      batch[i].clear();
      written[i] = 0;
      if (left_behind) continue;
      const std::string line = std::string{"[launcher] "} + descriptor.name +
                               ": " + std::strerror(error) +
                               "; the launcher no longer writes to it (its "
                               "lines are in talos.log, node output in each "
                               "<name>.log)\n";
      Say(i == 0 ? 2 : 1, line.data(), line.size());
    }
  }
  // Stopped: what the descriptors did not take within the linger, unless
  // Stop, no longer waiting, has said so itself.
  {
    const std::lock_guard lock{mutex};
    for (std::size_t i = 0; i < 2; ++i)
      descriptors[i].taken = batch[i].size() - written[i];
    if (!abandoned) NoteUnwritten();
    descriptors_ended = true;
  }
  writer_ended.notify_all();
}

void LauncherOutput::State::FileLoop() {
  LogFile file;
  std::unique_lock lock{mutex};
  for (;;) {
    file_wake.wait(lock, [this] {
      return !file_pending.empty() || file_dropped != 0 || file_done;
    });
    if (file_pending.empty() && file_dropped == 0) break;  // file_done.
    std::string bytes = std::move(file_pending);
    file_pending.clear();
    if (file_dropped != 0) {
      bytes += DropNote(bytes, file_dropped,
                        "the launcher's log writer fell behind");
      file_dropped = 0;
    }
    const std::string path = file_path;
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
