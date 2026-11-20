#pragma once

#include <sys/types.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
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
// launcher's stdout through the LauncherOutput. The echo goes in whole lines
// per pipe: a trailing partial line waits for its newline, the pipe's end,
// or more than kChildOutputReadBytes held, so that a member's line that
// reaches the pipe in two writes (stdio to a pipe is fully buffered) is not
// split on the console by a launcher line or another member's output. A
// partial line forwarded without its newline gets one on the console. The
// log gets every byte as it is read, and nothing more. Its reading thread never
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
// What ChildOutput reads from a member's pipe at once, and the most of a
// line without its newline the echo holds per pipe before it forwards it
// anyway (splitting that line).
inline constexpr std::size_t kChildOutputReadBytes = std::size_t{64} << 10;
// Maximum line length buffered while watching a member's output stream for a
// prefix; longer lines are discarded until the next newline.
inline constexpr std::size_t kChildOutputWatchLineBytes = 256;
// The launcher's own lines in a session directory. The launcher refuses a
// roster member whose name breaks the node-name rule, `talos` (a reserved
// owner) and `Talos` included, so no member's `<name>.log` takes this name;
// `launcher` is a legal node name.
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
  // Lines before it go to the descriptors only. A note this object writes
  // there about the stdout or stderr (a drop, a stream given up, what the
  // end left unwritten) starts at a line boundary: it waits for a launcher
  // line in progress there to end, at the latest for Stop, which ends that
  // line first.
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
  // atomic, as on a descriptor; a line written in several calls can
  // interleave with another thread's, so launcher lines use ConsoleLine).
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
  // `pid`, into `path`; with `echo`, also to the launcher's stdout. With a
  // non-empty `watch_prefix`, the reader thread assembles lines (up to
  // kChildOutputWatchLineBytes) even when `echo` is false, captures the first
  // complete line starting with `watch_prefix` for this run (MatchedLine), and
  // wakes SupervisionWakeup when it arrives. Takes the descriptor.
  void Add(int reader, const std::string& path, std::string_view name,
           pid_t pid, bool echo, std::string watch_prefix = {});

  // The first complete line matching `watch_prefix` read from `path`'s run
  // `pid` (without its trailing newline), or nullopt until one arrives.
  std::optional<std::string> MatchedLine(const std::string& path,
                                         pid_t pid) const;

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
    pid_t pid{-1};
    std::string path;
    bool echo;
    std::string header;  // Written to the log when the reader picks it up.
    // The reader's only: a partial line the echo has not forwarded yet.
    std::string echo_held;
    // The reader's only: watched-prefix line assembly for this run.
    std::string watch_prefix;
    std::string watch_held;
    bool watch_discard{false};
    bool watch_matched{false};
  };
  // Output bound for one path that the writer has not taken yet.
  struct Pending {
    std::string bytes;
    uint64_t dropped{0};  // Since the writer last took `bytes`.
    uint64_t dropped_total{0};
    int runs{0};
    pid_t watched_pid{-1};
    std::optional<std::string> watched_line;
  };

  // A full wake pipe already holds a wakeup.
  void Wake();

  // Scans `bytes` for the first complete line starting with
  // `stream.watch_prefix` (reader thread only).
  static std::optional<std::string> ScanWatchedLine(Stream& stream,
                                                    std::string_view bytes);

  // Moves `size` bytes read from `stream` into its buffers, dropping what
  // they cannot hold, and echoes the whole lines among them.
  void Deliver(Stream& stream, const char* data, std::size_t size);

  // `stream` has ended, or holds more than kChildOutputReadBytes: echoes the
  // partial line it held, ended with a newline the member did not write.
  void EndEcho(Stream& stream);

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
// `output` (echoed to the launcher's stdout with `echo`), the stop signals
// held with `hold_stop`, and optional `watch_prefix` passed to
// ChildOutput::Add.
pid_t SpawnDrained(std::vector<std::string> args, std::string& error,
                   ChildOutput& output, const std::string& path,
                   std::string_view name, bool echo, bool hold_stop = false,
                   std::string watch_prefix = {});

}  // namespace talos::launcher
