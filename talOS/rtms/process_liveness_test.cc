#include "talOS/rtms/process_liveness.h"

#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>

namespace talos {
namespace {

// The pid of a child that has exited and been reaped: no process has it.
std::uint64_t DeadPid() {
  const pid_t pid = ::fork();
  if (pid == 0) ::_exit(0);
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }
  return static_cast<std::uint64_t>(pid);
}

// A pid from another pid namespace names another process, or none, here: a
// check from a namespace other than the one that recorded the word must count
// its claimant as running, or a live process in a container sharing /dev/shm
// loses its RTMS claims and registry row. The namespaces are faked, so this
// runs on every platform.
TEST(ProcessLiveness, AWordFromAnotherPidNamespaceIsNeverProbed) {
  const std::uint64_t pid = DeadPid();
  const auto word = [pid](std::uint64_t space) {
    return (space << kProcessNamespaceShift) | pid;
  };
  // Namespaces numbered past the first thousand /proc entries, as on a busy
  // container host, still have codes of their own.
  constexpr std::uint64_t kHere = PidNamespaceCode(0xF0000000ULL + 5000);
  constexpr std::uint64_t kThere = PidNamespaceCode(0xF0000000ULL + 5001);
  static_assert(kHere != kProcessNamespaceOpaque &&
                kThere != kProcessNamespaceOpaque);
  EXPECT_FALSE(ProcessWordAliveFrom(word(kHere), kHere))
      << "in its own namespace a dead pid is dead";
  EXPECT_TRUE(ProcessWordAliveFrom(word(kThere), kHere))
      << "a pid from another namespace was looked up in this one";
  EXPECT_TRUE(ProcessWordAliveFrom(word(kProcessNamespaceOpaque),
                                   kProcessNamespaceOpaque))
      << "two opaque namespaces were taken for one";
  EXPECT_FALSE(ProcessWordAliveFrom(word(kProcessNamespaceNone), kHere))
      << "a word without a namespace compares the pid alone";
}

}  // namespace
}  // namespace talos
