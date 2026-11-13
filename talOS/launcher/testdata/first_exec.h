#pragma once

// Stub scripts that launcher process tests write and then hand to the
// launcher under test, paid for before the launcher sees them.
//
// macOS charges the first exec of a newly created file about 100 ms, and
// serializes that charge across the whole host: measured on the mac-mini
// deployment host, 104 ms for one fresh script, 702 ms each for 8 at once,
// 2.9 s each for 32 at once; the second exec of the same file costs 9 ms.
// The charge belongs to the file, not its content: reading it (`sh -n`,
// `sh FILE`, `cat`) does not pay it, a new file with the same bytes pays it
// again, and a hard link to a paid file, or a paid file rewritten in place,
// costs 3 ms. A test that writes a stub and gives the launcher's --describe
// probe 1 to 4 s to answer therefore fails whenever several test processes
// run, measuring the host rather than the launcher (docs/TESTING.md §8); and
// as the host pays only about ten such charges a second, a test binary that
// creates 140 stubs costs 14 s of host time per run however they are timed.
//
// So every stub starts, after its `#!` line, with a line that exits 0 at once
// when its first argument is kFirstExecArgument, and each distinct script is
// written once per test process, to a file executed once with that argument;
// every stub path a test asks for is a hard link to that file. The launcher
// never passes that argument, so the stub behaves for the launcher exactly as
// written: a stub meant not to answer --describe still does not, and nothing
// a stub records or counts happens during the warm-up exec.

#include <spawn.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <mutex>
#include <string>
#include <string_view>

extern char** environ;

namespace talos::launcher::testing {

inline constexpr std::string_view kFirstExecArgument = "--talos-first-exec";

// `script` with the warm-up line after its `#!` line. A script without one
// is returned as is: execv refuses it, so it cannot be warmed either.
inline std::string WithFirstExecLine(std::string_view script) {
  if (!script.starts_with("#!")) return std::string{script};
  const auto end = script.find('\n');
  if (end == std::string_view::npos) return std::string{script};
  return std::string{script.substr(0, end + 1)} + "[ \"$1\" = \"" +
         std::string{kFirstExecArgument} + "\" ] && exit 0\n" +
         std::string{script.substr(end + 1)};
}

// Executes `path` once with kFirstExecArgument and waits for it. A file that
// cannot be executed (a test's deliberately broken binary) is left alone.
inline void PayFirstExec(const std::string& path) {
  std::string argument{kFirstExecArgument};
  char* argv[] = {const_cast<char*>(path.c_str()), argument.data(), nullptr};
  pid_t pid = 0;
  if (::posix_spawn(&pid, path.c_str(), nullptr, nullptr, argv, environ) != 0)
    return;
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }
}

namespace detail {

inline std::string ReadAll(const std::string& path) {
  std::ifstream in{path, std::ios::binary};
  return {std::istreambuf_iterator<char>{in}, std::istreambuf_iterator<char>{}};
}

inline bool WriteExecutable(const std::string& path, const std::string& text,
                            mode_t mode) {
  {
    std::ofstream out{path, std::ios::binary | std::ios::trunc};
    out << text;
    if (!out) return false;
  }
  return ::chmod(path.c_str(), mode) == 0;
}

}  // namespace detail

// Makes `path` an executable holding `script` with the warm-up line, whose
// first exec is already paid, and returns `path`. A script this process has
// written before is a hard link to the file written then; any file at `path`
// is replaced, not written through.
inline std::string WriteWarmStub(const std::string& path,
                                 std::string_view script, mode_t mode = 0755) {
  static std::mutex mutex;
  const std::lock_guard lock{mutex};
  const std::string text = WithFirstExecLine(script);
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string store = std::string{tmp != nullptr ? tmp : "/tmp"} +
                            "/talos_first_exec_" + std::to_string(::getpid());
  std::error_code ignored;
  std::filesystem::create_directories(store, ignored);
  const std::string paid =
      store + "/" + std::to_string(std::hash<std::string>{}(text));
  std::filesystem::remove(path, ignored);
  if (!std::filesystem::exists(paid)) {
    if (detail::WriteExecutable(paid, text, mode)) PayFirstExec(paid);
  }
  // A hash collision, or a store that cannot be written: a file of its own.
  if (detail::ReadAll(paid) != text ||
      ::link(paid.c_str(), path.c_str()) != 0) {
    detail::WriteExecutable(path, text, mode);
    PayFirstExec(path);
    return path;
  }
  // A stub may have changed its own mode (`chmod a-x "$0"`) on an earlier
  // run of this script; every link shares it.
  ::chmod(path.c_str(), mode);
  return path;
}

}  // namespace talos::launcher::testing
