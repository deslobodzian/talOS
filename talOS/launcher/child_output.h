#pragma once

#include <sys/types.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace talos::launcher {

// ---------------------------------------------------------------------------
// Child output
//
// Every roster member writes its stdout and stderr into one pipe, which a
// ChildOutput drains into the member's `<output_dir>/<name>.log`, every run
// appended to the one file after a line naming the run and its pid, and,
// except for the Studio bridge, echoes to the launcher's stdout.
//
// A child's write blocks only while its pipe is full, so the pipe is drained
// by a thread that never waits for a disk: it only moves what arrives into a
// bounded buffer in memory. A second thread writes those buffers to the log
// files, and a third the echo to stdout: a one-line write to a file on a
// loaded disk can take hundreds of milliseconds (222 ms at p99 with eight
// disk writers on the Mac mini), and so can stdout when it is a file or a
// pipe read slowly. While a writer is behind, a member's buffer (or the
// echo's) grows to kChildOutputBufferBytes; what arrives beyond it is
// dropped, counted, and marked where it was lost. A write to stdout that
// cannot succeed later (a pipe whose reader closed it: EPIPE, since the
// launcher ignores SIGPIPE) ends the echo for the session; the logs go on.
//
// The child's end of the pipe stays blocking. A nonblocking one would turn a
// full pipe into EAGAIN inside the child: stdio loses the data and sets the
// stream's error flag, iostreams set badbit, which silences std::cout or
// std::cerr for the rest of the run, and Python raises BlockingIOError out of
// print(). Losing output here is the gentler failure, and the one that can be
// counted; a child still blocks if the drain thread is not scheduled while it
// fills its pipe (64 KiB on macOS, kChildOutputPipeBytes on Linux).
// ---------------------------------------------------------------------------

// What one member's output, or the echo, may hold in memory while the writer
// is behind.
inline constexpr std::size_t kChildOutputBufferBytes = std::size_t{1} << 20;
// What one member's output log may hold in a session; later output is
// dropped, counted, and the log says so.
inline constexpr uint64_t kChildOutputLogMaxBytes = uint64_t{64} << 20;

// The log a member's stdout and stderr are drained into, all runs together.
std::string NodeOutputPath(const std::string& output_dir,
                           std::string_view name);

class ChildOutput {
 public:
  using Clock = std::chrono::steady_clock;

  // Starts the reader, writer and echo threads, which block every signal so
  // that the launcher's handlers run on its own threads. Throws
  // std::system_error when a pipe or thread cannot be created.
  ChildOutput();
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
  // everything that arrived, and stops the threads. The echo still waits for
  // a stdout that does not drain, as every launcher line does. Idempotent.
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
  struct LogFile {
    int fd{-1};
    uint64_t written{0};
    bool capped{false};
    bool reported{false};
  };

  // A full wake pipe already holds a wakeup.
  void Wake();

  // A line marking `count` lost bytes, starting a line of its own.
  static std::string DropNote(const std::string& before, uint64_t count,
                              std::string_view why);

  // Moves `size` bytes read from `stream` into its buffers, dropping what
  // they cannot hold.
  void Deliver(const Stream& stream, const char* data, std::size_t size);

  // Reads every registered pipe until it ends, or, once stopping, until
  // `linger_` has passed. Touches no file.
  void ReadLoop();

  // Appends `bytes` to `path`'s log up to kChildOutputLogMaxBytes. Returns
  // the bytes it could not write.
  static uint64_t WriteLog(LogFile& file, const std::string& path,
                           std::string bytes);

  // Writes all of `bytes` to `fd`. Returns the bytes left unwritten, with
  // errno set, after an error.
  static std::size_t WriteAll(int fd, std::string_view bytes);

  // Takes what the reader buffered and writes it out, until the reader has
  // ended and nothing is left.
  void WriteLoop();

  // Copies echoed output to the launcher's stdout, on its own thread: a
  // stdout that is slow, or a pipe nobody reads, never holds up the logs.
  void EchoLoop();

  int wake_[2]{-1, -1};
  mutable std::mutex mutex_;
  std::condition_variable arrived_;
  // Guarded by mutex_.
  std::vector<Stream> added_;
  std::map<std::string, Pending> pending_;
  std::string echo_;
  uint64_t echo_dropped_{0};
  bool echo_closed_{false};  // A write to stdout failed for good (EchoLoop).
  bool dirty_{false};
  bool stopping_{false};
  bool reader_done_{false};
  std::chrono::milliseconds linger_{0};
  std::thread reader_;
  std::thread writer_;
  std::thread echoer_;
};

// SpawnProcess with the child's stdout and stderr drained into `path` by
// `output` (echoed to the launcher's stdout with `echo`).
pid_t SpawnDrained(std::vector<std::string> args, std::string& error,
                   ChildOutput& output, const std::string& path,
                   std::string_view name, bool echo);

}  // namespace talos::launcher
