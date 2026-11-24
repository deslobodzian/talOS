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
