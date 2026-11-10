#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <format>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "talOS/memory/residency.h"

namespace {
bool fail_truncate = false;
bool fail_close = false;
int unmap_calls = 0;
// Set to have the next exclusive create find the name and its last holder
// unlink it before the attach that follows.
bool unlink_after_exists = false;

int TestTruncate(int fd, off_t size) {
  if (std::exchange(fail_truncate, false)) {
    errno = ENOSPC;
    return -1;
  }
  return ::ftruncate(fd, size);
}

int TestClose(int fd) {
  const int result = ::close(fd);
  if (std::exchange(fail_close, false)) {
    errno = EIO;
    return -1;
  }
  return result;
}

int TestUnmap(void* address, std::size_t size) {
  ++unmap_calls;
  return ::munmap(address, size);
}

int TestShmOpen(const char* name, int oflag, mode_t mode) {
  const int fd = ::shm_open(name, oflag, mode);
  if (fd == -1 && errno == EEXIST &&
      std::exchange(unlink_after_exists, false)) {
    ::shm_unlink(name);
    errno = EEXIST;
  }
  return fd;
}
}  // namespace

// Inject only the OS behaviour needed to reach otherwise unreachable paths:
// failures that need cleanup, and a name unlinked in the middle of an open.
#define ftruncate TestTruncate
#define close TestClose
#define munmap TestUnmap
#define shm_open TestShmOpen
#include "shared_memory_ptr.h"
#undef ftruncate
#undef close
#undef munmap
#undef shm_open

TEST(SharedMemory, FailedCreatorTruncateRemovesPublishedName) {
  const std::string path = "/truncate_" + std::to_string(::getpid());
  const auto name = ShmObjectName(path);
  shm_unlink(name.c_str());
  fail_truncate = true;
  EXPECT_THROW((SharedMemoryPtr{path, 4096}), std::runtime_error);
  const int fd = shm_open(name.c_str(), O_RDWR, 0666);
  EXPECT_EQ(fd, -1);
  if (fd != -1) ::close(fd);
  shm_unlink(name.c_str());
}

TEST(SharedMemory, FailedCloseUnmapsSuccessfulMapping) {
  const std::string path = "/close_" + std::to_string(::getpid());
  const auto name = ShmObjectName(path);
  shm_unlink(name.c_str());
  const int before = unmap_calls;
  fail_close = true;
  EXPECT_THROW((SharedMemoryPtr{path, 4096}), std::runtime_error);
  EXPECT_EQ(unmap_calls, before + 1);
  shm_unlink(name.c_str());
}

// A name the exclusive create finds can be unlinked by its last holder before
// the attach that follows. The name is then free, so the mapping creates it
// rather than failing.
TEST(SharedMemory, ANameUnlinkedBetweenCreateAndAttachIsCreated) {
  const std::string path = "/vanish_" + std::to_string(::getpid());
  const auto name = ShmObjectName(path);
  shm_unlink(name.c_str());
  const int fd = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
  ASSERT_NE(fd, -1);
  ::close(fd);
  unlink_after_exists = true;
  SharedMemoryPtr mapping{path, 4096};
  EXPECT_FALSE(unlink_after_exists) << "the unlink was never injected";
  EXPECT_EQ(mapping.mode(), SharedMemoryMode::CREATE);
  EXPECT_NE(mapping.ptr(), nullptr);
  shm_unlink(name.c_str());
}

TEST(SharedMemory, SecondMappingAttachesAndNeitherUnlinksTheName) {
  // One path on both platforms. This was an #ifdef -- a flat name on Linux
  // and a slashed one elsewhere -- because glibc's shm_open rejects an
  // interior slash and macOS accepts one. ShmObjectName handles that at the
  // one boundary that opens the segment, so the difference no longer leaks
  // into every caller.
  auto path = "/rtms/test";
  shm_unlink(ShmObjectName(path).c_str());  // a crashed earlier run's
  {
    SharedMemoryPtr test{path, 100};
    EXPECT_NE(test.ptr(), nullptr);
    EXPECT_EQ(test.mode(), SharedMemoryMode::CREATE);
    // A second mapping of the same name attaches to it.
    SharedMemoryPtr again{path, 100};
    EXPECT_EQ(again.mode(), SharedMemoryMode::ATTACH);
  }
  // A mapping never unlinks its name: the segment outlives both mappings
  // until its owner (RTMS's lifetime rules) removes it.
  const int fd = shm_open(ShmObjectName(path).c_str(), O_RDWR, 0666);
  EXPECT_NE(fd, -1);
  if (fd != -1) close(fd);
  shm_unlink(ShmObjectName(path).c_str());
}

// Every talOS topic has two or three segments, so this mapping is on the path
// of every topic in the robot rather than being an edge case.
TEST(ShmObjectNameTest, ProducesAPortablePosixName) {
  for (const auto* topic : {"/hw/state", "/hw/state/driver_station",
                            "/drivetrain/target/teleop", "/talos/telemetry"}) {
    const std::string name = ShmObjectName(topic, "");
    ASSERT_FALSE(name.empty());
    EXPECT_EQ(name.front(), '/') << name;
    // POSIX allows exactly one slash, at the front.
    EXPECT_EQ(name.find('/', 1), std::string::npos)
        << topic << " maps to " << name << ", which glibc rejects";
    // Length is preserved, so RTMS's 30-character cap means the same thing
    // on both sides of the mapping.
    EXPECT_EQ(name.size(), std::string_view{topic}.size());
  }

  EXPECT_EQ(ShmObjectName("/hw/state/driver_station", ""),
            "/hw.state.driver_station");
  // A name that already conforms comes back unchanged, and the mapping is
  // idempotent -- RTMS applies it and then SharedMemoryPtr applies it again.
  EXPECT_EQ(ShmObjectName("/talos_registry.1", ""), "/talos_registry.1");
  EXPECT_EQ(ShmObjectName(ShmObjectName("/hw/state", ""), ""),
            ShmObjectName("/hw/state", ""));
}

// Two different topics must never land on one segment. That would join two
// unrelated streams while both ends reported success, which is a worse failure
// than either topic refusing to open.
TEST(ShmObjectNameTest, IsInjectiveOverTheTopicGrammar) {
  // The reason the separator is '.' and not '_': topic segments may contain
  // underscores (`driver_station`, `operator_interface`), so '_' would make
  // these two the same segment name. The grammar allows no '.' in a segment.
  EXPECT_NE(ShmObjectName("/hw/state/driver_station", ""),
            ShmObjectName("/hw/state_driver/station", ""));
  EXPECT_NE(ShmObjectName("/a/state/b", ""), ShmObjectName("/a/state_b", ""));
  EXPECT_NE(ShmObjectName("/operator_interface/state", ""),
            ShmObjectName("/operator/interface/state", ""));
}

TEST(ShmObjectNameTest, NamespacesAreStableIsolatedAndIdempotent) {
  const auto first = ShmObjectName("/hw/state", "run-a");
  const auto second = ShmObjectName("/hw/state", "run-b");
  EXPECT_NE(first, second);
  EXPECT_EQ(first, ShmObjectName("/hw/state", "run-a"));
  EXPECT_EQ(first, ShmObjectName("/hw.state", "run-a"));
  EXPECT_EQ(first, ShmObjectName(first, "run-a"));
  EXPECT_EQ(first, ShmObjectName(first, "run-b"));
  EXPECT_EQ(first, ShmObjectName(first, ""));
  EXPECT_EQ(first.size(), 18u);
  EXPECT_TRUE(first.starts_with("/~"));
  EXPECT_EQ(first.find_first_not_of("0123456789abcdef", 2), std::string::npos);
  EXPECT_NE(first, ShmObjectName("/hw/command", "run-a"));
  EXPECT_NE(ShmObjectName("/c", "ab"), ShmObjectName("/bc", "a"));
}

namespace {

long MinorFaults() {
  rusage usage{};
  ::getrusage(RUSAGE_SELF, &usage);
  return usage.ru_minflt;
}

// Writes one byte per cache line, as a ring's publishes eventually do.
[[gnu::noinline]] void WriteEveryLine(SharedMemoryPtr& mapping,
                                      std::size_t size) {
  auto* bytes = static_cast<volatile unsigned char*>(mapping.ptr());
  for (std::size_t offset = 0; offset < size; offset += 64) {
    bytes[offset] = static_cast<unsigned char>(offset);
  }
}

}  // namespace

// A realtime loop's first publish into a ring page must not fault: after
// KeepMappingsResident, rings mapped before it (a node's topics, opened in
// its constructor) and after it are written with no minor fault. The control
// mapping shows the same writes fault without it, so the count measures
// something.
TEST(SharedMemory, MappingsKeptResidentTakeNoFaultOnFirstWrite) {
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || \
    __has_feature(memory_sanitizer)
  // The sanitizer's shadow memory is mapped lazily, so the instrumented
  // writes fault on shadow pages no matter what the ring does.
  GTEST_SKIP() << "shadow memory faults under a sanitizer";
#endif
#endif
  constexpr std::size_t kSize = 1 << 20;
  const std::string base = "/resident_" + std::to_string(::getpid());
  for (const char* suffix : {"_control", "_before", "_after"}) {
    shm_unlink(ShmObjectName(base + suffix).c_str());
  }
  SharedMemoryPtr control{base + "_control", kSize};
  SharedMemoryPtr before{base + "_before", kSize};

  MinorFaults();
  const long control_start = MinorFaults();
  WriteEveryLine(control, kSize);
  const long control_faults = MinorFaults() - control_start;

  const talos::memory::Residency resident =
      talos::memory::KeepMappingsResident();
  EXPECT_GE(resident.ranges, 2u);
  SharedMemoryPtr after{base + "_after", kSize};

  const long start = MinorFaults();
  WriteEveryLine(before, kSize);
  WriteEveryLine(after, kSize);
  const long faults = MinorFaults() - start;

  EXPECT_GT(control_faults, 0);
  EXPECT_EQ(faults, 0) << "mlock refused " << resident.failed << " of "
                       << resident.ranges << " (errno " << resident.first_error
                       << ")";
  for (const char* suffix : {"_control", "_before", "_after"}) {
    shm_unlink(ShmObjectName(base + suffix).c_str());
  }
}
