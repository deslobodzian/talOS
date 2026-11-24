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
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "talOS/memory/residency.h"

#if defined(__linux__)
#include <sys/wait.h>

#include <sstream>
#include <vector>
#endif

namespace {
bool fail_truncate = false;
bool fail_close = false;
bool fail_mmap = false;
int unmap_calls = 0;
// Set to have the next exclusive create find the name and its last holder
// unlink it before the attach that follows.
bool unlink_after_exists = false;
// Set to have the next ATTACH shm_open unlink the opened 0-byte segment right
// after returning its fd (and optionally recreate a sized segment with a
// sentinel byte at offset 0).
bool unlink_after_attach = false;
std::size_t recreate_after_attach_size = 0;
unsigned char recreate_after_attach_sentinel = 0;
// Number of EINTR failures to inject into fstat().
int fail_fstat_eintr = 0;

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

void* TestMmap(void* addr, std::size_t length, int prot, int flags, int fd,
               off_t offset) {
  if (std::exchange(fail_mmap, false)) {
    errno = ENOMEM;
    return MAP_FAILED;
  }
  return ::mmap(addr, length, prot, flags, fd, offset);
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
  if (fd != -1 && (oflag & O_CREAT) == 0 &&
      std::exchange(unlink_after_attach, false)) {
    ::shm_unlink(name);
    const std::size_t recreate_size =
        std::exchange(recreate_after_attach_size, 0);
    const unsigned char sentinel =
        std::exchange(recreate_after_attach_sentinel, 0);
    if (recreate_size > 0) {
      const int replacement = ::shm_open(name, O_CREAT | O_RDWR | O_EXCL, 0666);
      if (replacement != -1) {
        if (::ftruncate(replacement, static_cast<off_t>(recreate_size)) == 0) {
          void* mapped = ::mmap(nullptr, recreate_size, PROT_READ | PROT_WRITE,
                                MAP_SHARED, replacement, 0);
          if (mapped != MAP_FAILED) {
            static_cast<unsigned char*>(mapped)[0] = sentinel;
            ::munmap(mapped, recreate_size);
          }
        }
        ::close(replacement);
      }
    }
  }
  return fd;
}

int TestFstat(int fd, struct stat* st) {
  if (fail_fstat_eintr > 0) {
    --fail_fstat_eintr;
    errno = EINTR;
    return -1;
  }
  return ::fstat(fd, st);
}
}  // namespace

// Inject only the OS behaviour needed to reach otherwise unreachable paths:
// failures that need cleanup, and a name unlinked in the middle of an open.
#define ftruncate TestTruncate
#define close TestClose
#define mmap TestMmap
#define munmap TestUnmap
#define shm_open TestShmOpen
#define fstat TestFstat
#include "shared_memory_ptr.h"
#undef ftruncate
#undef close
#undef mmap
#undef munmap
#undef shm_open
#undef fstat

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
  // A failed CREATE must also unlink the published name so an uninitialized
  // segment is not leaked.
  const int fd = shm_open(name.c_str(), O_RDWR, 0666);
  EXPECT_EQ(fd, -1);
  if (fd != -1) ::close(fd);
  shm_unlink(name.c_str());
}

// REQ-CONTRACT-21: requesting a zero-byte mapping fails upfront without
// creating or leaking an empty shared-memory object.
TEST(SharedMemory, ZeroSizeThrowsWithoutCreatingSegment) {
  const std::string path = "/zero_" + std::to_string(::getpid());
  const auto name = ShmObjectName(path);
  shm_unlink(name.c_str());
  EXPECT_THROW((SharedMemoryPtr{path, 0}), std::invalid_argument);
  const int fd = shm_open(name.c_str(), O_RDWR, 0666);
  EXPECT_EQ(fd, -1);
  if (fd != -1) ::close(fd);
  shm_unlink(name.c_str());
}

// REQ-CONTRACT-22: a size past off_t::max() fails upfront with overflow_error
// without creating a shared-memory segment.
TEST(SharedMemory, SizeExceedingOffTThrowsWithoutCreatingSegment) {
  constexpr auto kMaxOffT =
      static_cast<std::size_t>(std::numeric_limits<off_t>::max());
  if constexpr (std::numeric_limits<std::size_t>::max() <= kMaxOffT) {
    GTEST_SKIP() << "size_t cannot exceed off_t on this platform";
  } else {
    const std::string path = "/overflow_" + std::to_string(::getpid());
    const auto name = ShmObjectName(path);
    shm_unlink(name.c_str());
    EXPECT_THROW((SharedMemoryPtr{path, kMaxOffT + 1}), std::overflow_error);
    const int fd = shm_open(name.c_str(), O_RDWR, 0666);
    EXPECT_EQ(fd, -1);
    if (fd != -1) ::close(fd);
    shm_unlink(name.c_str());
  }
}

// REQ-CONTRACT-22: fstat() returning EINTR during ATTACH is retried rather
// than failing the attach.
TEST(SharedMemory, AttachRetriesFstatOnEintr) {
  struct ResetHooks {
    ~ResetHooks() { fail_fstat_eintr = 0; }
  } reset;
  const std::string path = "/fstat_eintr_" + std::to_string(::getpid());
  const auto name = ShmObjectName(path);
  shm_unlink(name.c_str());
  SharedMemoryPtr creator{path, 4096};
  ASSERT_EQ(creator.mode(), SharedMemoryMode::CREATE);

  fail_fstat_eintr = 2;
  SharedMemoryPtr attacher{path, 4096};
  EXPECT_EQ(fail_fstat_eintr, 0);
  EXPECT_EQ(attacher.mode(), SharedMemoryMode::ATTACH);
  EXPECT_NE(attacher.ptr(), nullptr);
  shm_unlink(name.c_str());
}

// REQ-CONTRACT-22: if an attacher opens a 0-byte segment whose creator then
// fails ftruncate and unlinks the name, the attacher drops the dead unlinked
// inode and either creates a fresh segment or attaches to a replacement
// creator instead of pinning to the dead inode for 50 ms and failing.
TEST(SharedMemory, AttachRecoversWhenZeroByteSegmentIsUnlinked) {
  struct ResetHooks {
    ~ResetHooks() {
      unlink_after_attach = false;
      recreate_after_attach_size = 0;
      recreate_after_attach_sentinel = 0;
    }
  } reset;
  {
    const std::string path = "/dead_zero_create_" + std::to_string(::getpid());
    const auto name = ShmObjectName(path);
    shm_unlink(name.c_str());
    const int fd = shm_open(name.c_str(), O_CREAT | O_RDWR | O_EXCL, 0666);
    ASSERT_NE(fd, -1);
    ::close(fd);

    unlink_after_attach = true;
    SharedMemoryPtr recovered{path, 4096};
    EXPECT_FALSE(unlink_after_attach);
    EXPECT_EQ(recovered.mode(), SharedMemoryMode::CREATE);
    EXPECT_NE(recovered.ptr(), nullptr);
    shm_unlink(name.c_str());
  }
  {
    const std::string path = "/dead_zero_replace_" + std::to_string(::getpid());
    const auto name = ShmObjectName(path);
    shm_unlink(name.c_str());
    const int fd = shm_open(name.c_str(), O_CREAT | O_RDWR | O_EXCL, 0666);
    ASSERT_NE(fd, -1);
    ::close(fd);

    unlink_after_attach = true;
    recreate_after_attach_size = 4096;
    recreate_after_attach_sentinel = 0x5Au;
    SharedMemoryPtr recovered{path, 4096};
    EXPECT_FALSE(unlink_after_attach);
    EXPECT_EQ(recovered.mode(), SharedMemoryMode::ATTACH);
    ASSERT_NE(recovered.ptr(), nullptr);
    EXPECT_EQ(static_cast<const unsigned char*>(recovered.ptr())[0], 0x5Au);
    shm_unlink(name.c_str());
  }
}

// REQ-OPS-03: an mmap failure during CREATE unlinks the newly created name so
// no zero-initialized orphan remains; during ATTACH it leaves the existing
// segment untouched.
TEST(SharedMemory, FailedMmapUnlinksOnlyOnCreate) {
  const std::string path = "/mmap_fail_" + std::to_string(::getpid());
  const auto name = ShmObjectName(path);
  shm_unlink(name.c_str());

  fail_mmap = true;
  EXPECT_THROW((SharedMemoryPtr{path, 4096}), std::runtime_error);
  const int after_create_fail = shm_open(name.c_str(), O_RDWR, 0666);
  EXPECT_EQ(after_create_fail, -1);
  if (after_create_fail != -1) ::close(after_create_fail);

  SharedMemoryPtr existing{path, 4096};
  ASSERT_EQ(existing.mode(), SharedMemoryMode::CREATE);
  fail_mmap = true;
  EXPECT_THROW((SharedMemoryPtr{path, 4096}), std::runtime_error);
  const int after_attach_fail = shm_open(name.c_str(), O_RDWR, 0666);
  EXPECT_NE(after_attach_fail, -1);
  if (after_attach_fail != -1) ::close(after_attach_fail);
  shm_unlink(name.c_str());
}

// REQ-CONTRACT-21: attaching with a larger size than the existing segment
// rejects with std::runtime_error and leaves the existing segment intact.
TEST(SharedMemory, AttachRejectsUndersizedSegmentWithoutUnlinking) {
  const long raw_page = ::sysconf(_SC_PAGESIZE);
  ASSERT_GT(raw_page, 0);
  const std::size_t page = static_cast<std::size_t>(raw_page);
  const std::string path = "/undersized_" + std::to_string(::getpid());
  const auto name = ShmObjectName(path);
  shm_unlink(name.c_str());

  SharedMemoryPtr creator{path, page};
  ASSERT_EQ(creator.mode(), SharedMemoryMode::CREATE);
  ASSERT_NE(creator.ptr(), nullptr);
  static_cast<unsigned char*>(creator.ptr())[0] = 0xA5u;

  EXPECT_THROW((SharedMemoryPtr{path, page * 2}), std::runtime_error);

  const int fd = shm_open(name.c_str(), O_RDWR, 0666);
  ASSERT_NE(fd, -1) << "rejected attach unlinked the existing segment";
  struct stat st {};
  EXPECT_EQ(::fstat(fd, &st), 0);
  EXPECT_EQ(st.st_size, static_cast<off_t>(page));
  ::close(fd);

  SharedMemoryPtr intact{path, page};
  EXPECT_EQ(intact.mode(), SharedMemoryMode::ATTACH);
  ASSERT_NE(intact.ptr(), nullptr);
  EXPECT_EQ(static_cast<const unsigned char*>(intact.ptr())[0], 0xA5u);

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
// REQ-CONTRACT-22
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

// REQ-CONTRACT-22
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

[[noreturn]] void RunShmEnvCheck(const char* shm_ns, const char* test_tmpdir,
                                 std::string_view expected_ns) {
  if (shm_ns != nullptr) {
    ::setenv("TALOS_SHM_NAMESPACE", shm_ns, 1);
  } else {
    ::unsetenv("TALOS_SHM_NAMESPACE");
  }
  if (test_tmpdir != nullptr) {
    ::setenv("TEST_TMPDIR", test_tmpdir, 1);
  } else {
    ::unsetenv("TEST_TMPDIR");
  }
  const std::string actual = ShmObjectName("/hw/state");
  const std::string expected = ShmObjectName("/hw/state", expected_ns);
  if (actual != expected) {
    std::fprintf(stderr, "actual=%s expected=%s\n", actual.c_str(),
                 expected.c_str());
    std::fflush(stderr);
    std::_Exit(1);
  }
  std::_Exit(0);
}

}  // namespace

// 1-arg ShmObjectName caches process namespace precedence: TALOS_SHM_NAMESPACE
// (even when empty, disabling TEST_TMPDIR), else TEST_TMPDIR, else
// unnamespaced. Runs each combination in a fresh re-executed child so the
// function-local static namespace cache initializes cleanly from that child's
// environment. REQ-CONTRACT-22
TEST(ShmObjectNameTest, OneArgOverloadResolvesEnvironmentPrecedence) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(RunShmEnvCheck("custom-ns", "/tmp/ignored", "custom-ns"),
              ::testing::ExitedWithCode(0), "");
  EXPECT_EXIT(RunShmEnvCheck("", "/tmp/ignored", ""),
              ::testing::ExitedWithCode(0), "");
  EXPECT_EXIT(RunShmEnvCheck(nullptr, "/tmp/test-run", "/tmp/test-run"),
              ::testing::ExitedWithCode(0), "");
  EXPECT_EXIT(RunShmEnvCheck(nullptr, nullptr, ""),
              ::testing::ExitedWithCode(0), "");
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
// REQ-RT-09
TEST(SharedMemory, MappingsKeptResidentTakeNoFaultOnFirstWrite) {
  // The sanitizer's shadow memory is mapped lazily, so the instrumented
  // writes fault on shadow pages no matter what the ring does. GCC marks
  // sanitizer builds with __SANITIZE_*__; clang with __has_feature.
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
  GTEST_SKIP() << "shadow memory faults under a sanitizer";
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || \
    __has_feature(memory_sanitizer)
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

// MakeResident must stay strictly within [address, address + size) and use
// 1-byte atomic operations so unaligned or sub-8-byte buffers never fault past
// their end or trigger misaligned 8-byte atomic UB.
// REQ-RT-09
TEST(SharedMemory, MakeResidentHandlesUnalignedSmallBuffersWithinBounds) {
  const auto page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
  ASSERT_GT(page, 8u);

  // Place a 3-byte unaligned buffer at the very end of a writable page right
  // before a PROT_NONE guard page: an 8-byte RMW crosses into the guard page
  // and crashes immediately.
  void* raw = ::mmap(nullptr, 2 * page, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  ASSERT_NE(raw, MAP_FAILED);
  auto* base = static_cast<unsigned char*>(raw);
  ASSERT_EQ(::mprotect(base + page, page, PROT_NONE), 0);

  unsigned char* tail = base + page - 3;
  tail[0] = 0x11;
  tail[1] = 0x22;
  tail[2] = 0x33;

  const int guard_err = talos::memory::MakeResident(tail, 3);
  EXPECT_TRUE(guard_err == 0 || guard_err == ENOMEM || guard_err == EPERM)
      << "unexpected errno " << guard_err;
  EXPECT_EQ(tail[0], 0x11);
  EXPECT_EQ(tail[1], 0x22);
  EXPECT_EQ(tail[2], 0x33);

  ASSERT_EQ(::munmap(raw, 2 * page), 0);

  // Also cover an unaligned 3-byte stack buffer and 3-byte heap buffer.
  alignas(8) unsigned char stack_buf[4] = {0xaa, 0x44, 0x55, 0x66};
  const int stack_err = talos::memory::MakeResident(stack_buf + 1, 3);
  EXPECT_TRUE(stack_err == 0 || stack_err == ENOMEM || stack_err == EPERM)
      << "unexpected errno " << stack_err;
  EXPECT_EQ(stack_buf[0], 0xaa);
  EXPECT_EQ(stack_buf[1], 0x44);
  EXPECT_EQ(stack_buf[2], 0x55);
  EXPECT_EQ(stack_buf[3], 0x66);

  auto heap_buf = std::make_unique<unsigned char[]>(3);
  heap_buf[0] = 0x77;
  heap_buf[1] = 0x88;
  heap_buf[2] = 0x99;
  const int heap_err = talos::memory::MakeResident(heap_buf.get(), 3);
  EXPECT_TRUE(heap_err == 0 || heap_err == ENOMEM || heap_err == EPERM)
      << "unexpected errno " << heap_err;
  EXPECT_EQ(heap_buf[0], 0x77);
  EXPECT_EQ(heap_buf[1], 0x88);
  EXPECT_EQ(heap_buf[2], 0x99);
}

#if defined(__linux__)
namespace {

// Probes in a forked child whether RLIMIT_MEMLOCK=0 actually causes mlock to
// fail with ENOMEM or EPERM. Processes with CAP_IPC_LOCK (including root by
// default) bypass RLIMIT_MEMLOCK, in which case the caller skips rather than
// reporting a false pass.
enum class MemlockProbe { kRefused, kBypassed, kError };

MemlockProbe ProbeMemlockZero() {
  const auto page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
  const pid_t child = ::fork();
  if (child < 0) return MemlockProbe::kError;
  if (child == 0) {
    const rlimit zero{0, 0};
    if (::setrlimit(RLIMIT_MEMLOCK, &zero) != 0) ::_exit(2);
    void* p = ::mmap(nullptr, page, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (p == MAP_FAILED) ::_exit(3);
    const int err = ::mlock(p, page) == 0 ? 0 : errno;
    if (err == 0) {
      ::munlock(p, page);
      ::_exit(1);
    }
    ::_exit((err == ENOMEM || err == EPERM) ? 0 : 4);
  }
  int status = 0;
  if (::waitpid(child, &status, 0) != child || !WIFEXITED(status)) {
    return MemlockProbe::kError;
  }
  if (WEXITSTATUS(status) == 0) return MemlockProbe::kRefused;
  if (WEXITSTATUS(status) == 1) return MemlockProbe::kBypassed;
  return MemlockProbe::kError;
}

// Child body for LateSharedMemoryUnderMemlockZeroReportsRefusalOnce, factored
// out of EXPECT_EXIT to avoid preprocessor comma splitting.
[[noreturn]] void RunLateSharedMemoryMemlockZeroChild() {
  const rlimit zero{0, 0};
  if (::setrlimit(RLIMIT_MEMLOCK, &zero) != 0) {
    std::fprintf(stderr, "setrlimit failed: %s\n", std::strerror(errno));
    std::fflush(stderr);
    std::_Exit(10);
  }

  const auto page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
  void* probe_page = ::mmap(nullptr, page, PROT_READ | PROT_WRITE,
                            MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  if (probe_page == MAP_FAILED) {
    std::fprintf(stderr, "mmap probe failed: %s\n", std::strerror(errno));
    std::fflush(stderr);
    std::_Exit(11);
  }
  const int probe_err = ::mlock(probe_page, page) == 0 ? 0 : errno;
  if (probe_err == 0) ::munlock(probe_page, page);
  ::munmap(probe_page, page);
  if (probe_err != ENOMEM && probe_err != EPERM) {
    std::fprintf(stderr, "unexpected mlock probe errno: %d (%s)\n", probe_err,
                 std::strerror(probe_err));
    std::fflush(stderr);
    std::_Exit(12);
  }

  const std::string base = "/memlock_late_" + std::to_string(::getpid());
  const std::string early_path = base + "_early";
  const std::string late1_path = base + "_late1";
  const std::string late2_path = base + "_late2";
  const auto unlink_all = [&] {
    ::shm_unlink(ShmObjectName(early_path).c_str());
    ::shm_unlink(ShmObjectName(late1_path).c_str());
    ::shm_unlink(ShmObjectName(late2_path).c_str());
  };
  unlink_all();

  std::fflush(stderr);
  int pipefd[2]{-1, -1};
  if (::pipe(pipefd) != 0) std::_Exit(13);
  const int saved_stderr = ::dup(STDERR_FILENO);
  if (saved_stderr < 0 || ::dup2(pipefd[1], STDERR_FILENO) < 0) {
    std::_Exit(14);
  }
  ::close(pipefd[1]);

  bool before_keep_ok = false;
  talos::memory::Residency kept{};
  bool after_keep_ok = false;
  bool after_late1_ok = false;
  bool after_late2_ok = false;
  bool after_attach_ok = false;
  std::string caught_error;
  try {
    SharedMemoryPtr early{early_path, page};
    before_keep_ok = !talos::memory::MappingsKeptResident() &&
                     !talos::memory::LateLockRefused();

    kept = talos::memory::KeepMappingsResident();
    after_keep_ok = talos::memory::MappingsKeptResident() &&
                    !talos::memory::LateLockRefused() && kept.ranges == 1 &&
                    kept.bytes == page && kept.failed == 1 &&
                    kept.first_error == probe_err;

    // Late SharedMemoryPtr allocations (CREATE and ATTACH) after
    // KeepMappingsResident(): all succeed and remain writable,
    // LateLockRefused() flips true on the first, and only the first prints the
    // refusal warning.
    SharedMemoryPtr late1{late1_path, page};
    static_cast<unsigned char*>(late1.ptr())[0] = 0x5a;
    after_late1_ok = late1.ptr() != nullptr &&
                     late1.mode() == SharedMemoryMode::CREATE &&
                     talos::memory::LateLockRefused();

    SharedMemoryPtr late2{late2_path, page};
    static_cast<unsigned char*>(late2.ptr())[0] = 0xa5;
    after_late2_ok = late2.ptr() != nullptr &&
                     late2.mode() == SharedMemoryMode::CREATE &&
                     talos::memory::LateLockRefused();

    SharedMemoryPtr late_attach{early_path, page};
    static_cast<unsigned char*>(late_attach.ptr())[0] = 0x3c;
    after_attach_ok = late_attach.ptr() != nullptr &&
                      late_attach.mode() == SharedMemoryMode::ATTACH &&
                      talos::memory::LateLockRefused();
  } catch (const std::exception& e) {
    caught_error = e.what();
  }
  unlink_all();

  std::fflush(stderr);
  ::dup2(saved_stderr, STDERR_FILENO);
  ::close(saved_stderr);

  std::string captured;
  char buf[512];
  for (ssize_t n = 0; (n = ::read(pipefd[0], buf, sizeof buf)) > 0;) {
    captured.append(buf, static_cast<std::size_t>(n));
  }
  ::close(pipefd[0]);

  std::vector<std::string> lines;
  std::istringstream stream{captured};
  for (std::string line; std::getline(stream, line);) {
    lines.push_back(line);
  }

  const std::string expected_late =
      std::string{
          "talos: shared memory mapped after realtime setup is prefaulted "
          "but not locked (mlock: "} +
      std::strerror(probe_err) + "); further refusals are not reported";
  const bool lines_ok = lines.size() == 1 && lines[0] == expected_late;

  if (!caught_error.empty() || !before_keep_ok || !after_keep_ok ||
      !after_late1_ok || !after_late2_ok || !after_attach_ok || !lines_ok) {
    std::fprintf(stderr,
                 "caught='%s' before_keep=%d after_keep=%d (ranges=%zu "
                 "bytes=%zu failed=%zu err=%d) late1=%d late2=%d attach=%d "
                 "lines=%zu\ncaptured:\n%s\nexpected: %s\n",
                 caught_error.c_str(), before_keep_ok ? 1 : 0,
                 after_keep_ok ? 1 : 0, kept.ranges, kept.bytes, kept.failed,
                 kept.first_error, after_late1_ok ? 1 : 0,
                 after_late2_ok ? 1 : 0, after_attach_ok ? 1 : 0, lines.size(),
                 captured.c_str(), expected_late.c_str());
    std::fflush(stderr);
    std::_Exit(1);
  }
  std::_Exit(0);
}

}  // namespace

// Under RLIMIT_MEMLOCK=0 on Linux (macOS ignores RLIMIT_MEMLOCK in favor of
// the system-wide wired limit), opening a new SharedMemoryPtr while
// MappingsKeptResident() is active keeps the mapping usable, sets
// LateLockRefused(), and prints the late-refusal warning to stderr once even
// across multiple late SharedMemoryPtr allocations. Runs in a re-executed
// child so MappingsKeptResident() starts false and RLIMIT_MEMLOCK=0 stays
// isolated to the child.
// REQ-RT-09
TEST(SharedMemory, LateSharedMemoryUnderMemlockZeroReportsRefusalOnce) {
  const MemlockProbe probe = ProbeMemlockZero();
  if (probe == MemlockProbe::kBypassed) {
    GTEST_SKIP() << "process bypasses RLIMIT_MEMLOCK=0 (e.g. CAP_IPC_LOCK)";
  }
  ASSERT_EQ(probe, MemlockProbe::kRefused)
      << "failed to probe RLIMIT_MEMLOCK=0 behaviour";
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(RunLateSharedMemoryMemlockZeroChild(),
              ::testing::ExitedWithCode(0), "");
}
#endif
