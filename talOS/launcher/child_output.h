#pragma once

#include <sys/types.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <streambuf>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace talos::launcher {

// ---------------------------------------------------------------------------
// The launcher's output: its own lines and its members'
//
// No launcher thread waits on the launcher's stdout or stderr. They are a
// terminal, a pipe or a socket that may be closed, full or never read (a
// paused pager, an ssh session sshd has not yet found dead), and a write to
// one can then block for good; the supervision loop that restarts crashed
// nodes and stops the session must not be the thread that waits.
//
// LauncherOutput takes the launcher's stdout and stderr for a Launcher::Run.
// std::cout, std::cerr and std::clog write into it rather than into
// descriptors 1 and 2, and so does the echo of node output: a write appends
// to a bounded buffer in memory, under a mutex no thread holds across I/O,
// and returns. One thread writes those buffers to the real descriptors and
// is the only one that may wait on them; once the session directory exists,
// a second writes every launcher line (not the echo) to its
// `<output_dir>/<session_id>/talos.log` as well, so a terminal that is gone,
// slow or never read loses only what it would have shown.
//
// ChildOutput drains every roster member's stdout and stderr, one pipe per
// run, into the member's `<name>.log`, every run appended after a line naming
// the run and its pid, and, except for the Studio bridge, echoes it to the
// launcher's stdout through the LauncherOutput. Its reading thread never
// waits for a disk: it only moves what arrives into a bounded buffer in
// memory, and a second thread writes the logs. A one-line write to a file on
// a loaded disk can take hundreds of milliseconds (222 ms at p99 with eight
// disk writers on the Mac mini).
//
// Every buffer holds at most kChildOutputBufferBytes while its writer is
// behind, besides the batch the writer has taken; what arrives beyond that
// is dropped, counted, and marked where it was lost once the writer catches
// up. A descriptor that fails for good (EPIPE: the launcher ignores SIGPIPE;
// a closed or broken descriptor) is not written again in the session, and
// one line on the other descriptor and in talos.log says so.
//
// A member's end of its pipe stays blocking. A nonblocking one would turn a
// full pipe into EAGAIN inside the child: stdio loses the data and sets the
// stream's error flag, iostreams set badbit, which silences std::cout or
// std::cerr for the rest of the run, and Python raises BlockingIOError out of
// print(). Losing output here is the gentler failure, and the one that can be
// counted; a child still blocks if the drain thread is not scheduled while it
// fills its pipe (64 KiB on macOS, kChildOutputPipeBytes on Linux).
// ---------------------------------------------------------------------------

// What one buffer -- a member's output, the echo, each launcher descriptor,
// talos.log -- may hold in memory while its writer is behind.
inline constexpr std::size_t kChildOutputBufferBytes = std::size_t{1} << 20;
// What one member's output log, or talos.log, may hold in a session; later
// output is dropped, counted, and the log says so.
inline constexpr uint64_t kChildOutputLogMaxBytes = uint64_t{64} << 20;
// The launcher's own lines in a session directory. `talos` is a reserved
// owner, so no member's `<name>.log` takes this name; `launcher` is a legal
// node name.
inline constexpr std::string_view kLauncherOutputFile = "talos.log";
// How long LauncherOutput::Stop waits for descriptors 1 and 2 to take what
// is buffered for them before it drops the rest.
inline constexpr std::chrono::milliseconds kLauncherOutputLinger{1000};
// How much longer LauncherOutput::Stop waits for its descriptor writer to
// see the linger end (its poll() returns then) before leaving it behind.
inline constexpr std::chrono::milliseconds kLauncherOutputWriterGrace{100};

// The log a member's stdout and stderr are drained into, all runs together.
std::string NodeOutputPath(const std::string& output_dir,
                           std::string_view name);

class LauncherOutput {
 public:
  using Clock = std::chrono::steady_clock;

  // Takes std::cout, std::cerr and std::clog (and clears their error state)
  // and starts the writer threads, which block every signal so that the
  // launcher's handlers run on its own threads. Throws std::system_error
  // when a pipe or thread cannot be created; then nothing is taken. One at a
  // time per process.
  LauncherOutput();
  ~LauncherOutput();
  LauncherOutput(const LauncherOutput&) = delete;
  LauncherOutput& operator=(const LauncherOutput&) = delete;

  // From here on every launcher line also goes to `path` (talos.log).
  // Lines before it go to the descriptors only.
  void Record(const std::string& path);

  // Node output for descriptor 1 only (ChildOutput's echo). Never waits.
  void Echo(std::string_view bytes);

  // Gives descriptors 1 and 2 `linger` to take what is buffered for them,
  // then drops the rest (counted in talos.log), writes everything left to
  // talos.log, stops the file writer and gives std::cout, std::cerr and
  // std::clog back. Returns within `linger` plus kLauncherOutputWriterGrace
  // and the disk's time for talos.log, whatever the descriptors do: a
  // descriptor writer still inside a write() then (a terminal nobody reads
  // takes part of a chunk and blocks on the rest) is left to finish or die
  // with the process. It owns, with this object, everything it touches
  // (State), so it never reaches freed memory or a closed descriptor
  // number, and it writes nothing more to talos.log. Idempotent.
  void Stop(std::chrono::milliseconds linger = kLauncherOutputLinger);

 private:
  struct State;

  // A streambuf that hands every byte written to it to State::Say.
  // Unbuffered, so several threads may write through one (each call is
  // atomic, as on a descriptor; their lines can interleave).
  class Sink : public std::streambuf {
   public:
    Sink(State& state, int fd) : state_{state}, fd_{fd} {}

   protected:
    int_type overflow(int_type c) override;
    std::streamsize xsputn(const char* data, std::streamsize size) override;
    int sync() override { return 0; }

   private:
    State& state_;
    const int fd_;
  };

  // Shared with the threads, each holding a reference, so that it outlives
  // a descriptor writer Stop leaves behind.
  std::shared_ptr<State> state_;
  Sink out_;
  Sink err_;
  std::streambuf* previous_[3]{nullptr, nullptr, nullptr};
  bool stopped_{false};
  std::thread descriptor_writer_;
  std::thread file_writer_;
};

class ChildOutput {
 public:
  using Clock = std::chrono::steady_clock;

  // Starts the reader and writer threads, which block every signal so that
  // the launcher's handlers run on its own threads. Echoed output goes to
  // `echo` (none without one), which must outlive Stop. Throws
  // std::system_error when a pipe or thread cannot be created.
  explicit ChildOutput(LauncherOutput* echo = nullptr);
  ~ChildOutput();
  ChildOutput(const ChildOutput&) = delete;
  ChildOutput& operator=(const ChildOutput&) = delete;

  // Drains `reader`, an output pipe SpawnProcess returned for `name`'s run
  // `pid`, into `path`; with `echo`, also to the launcher's stdout. Takes the
  // descriptor.
  void Add(int reader, const std::string& path, std::string_view name,
           pid_t pid, bool echo);

  // Waits up to `linger` for every pipe to end -- its run has exited, but a
  // descendant that left the process group can hold it open -- writes out
  // everything that arrived, and stops the threads. Idempotent.
  void Stop(std::chrono::milliseconds linger = std::chrono::milliseconds{500});

  // Bytes of the output bound for `path` that were dropped so far: the
  // writer fell behind, the log reached kChildOutputLogMaxBytes, or a write
  // to it failed.
  uint64_t Dropped(const std::string& path) const;

 private:
  struct Stream {
    int fd;
    std::string path;
    bool echo;
    std::string header;  // Written to the log when the reader picks it up.
  };
  // Output bound for one path that the writer has not taken yet.
  struct Pending {
    std::string bytes;
    uint64_t dropped{0};  // Since the writer last took `bytes`.
    uint64_t dropped_total{0};
    int runs{0};
  };

  // A full wake pipe already holds a wakeup.
  void Wake();

  // Moves `size` bytes read from `stream` into its buffers, dropping what
  // they cannot hold.
  void Deliver(const Stream& stream, const char* data, std::size_t size);

  // Reads every registered pipe until it ends, or, once stopping, until
  // `linger_` has passed. Touches no file.
  void ReadLoop();

  // Takes what the reader buffered and writes it out, until the reader has
  // ended and nothing is left.
  void WriteLoop();

  LauncherOutput* const echo_;
  int wake_[2]{-1, -1};
  mutable std::mutex mutex_;
  std::condition_variable arrived_;
  // Guarded by mutex_.
  std::vector<Stream> added_;
  std::map<std::string, Pending> pending_;
  bool dirty_{false};
  bool stopping_{false};
  bool reader_done_{false};
  std::chrono::milliseconds linger_{0};
  std::thread reader_;
  std::thread writer_;
};

// SpawnProcess with the child's stdout and stderr drained into `path` by
// `output` (echoed to the launcher's stdout with `echo`).
pid_t SpawnDrained(std::vector<std::string> args, std::string& error,
                   ChildOutput& output, const std::string& path,
                   std::string_view name, bool echo);

}  // namespace talos::launcher
