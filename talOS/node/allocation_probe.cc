#include "talOS/node/allocation_probe.h"

#include <execinfo.h>
#include <pthread.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>

// A sanitizer serves every allocation from its own allocator: macOS then
// never calls malloc_logger, and glibc's malloc cannot be replaced beside it.
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || \
    __has_feature(memory_sanitizer) || __has_feature(hwaddress_sanitizer)
#define TALOS_PROBE_SANITIZED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define TALOS_PROBE_SANITIZED 1
#endif

#if !defined(TALOS_PROBE_SANITIZED) && defined(__APPLE__)
#define TALOS_PROBE_MALLOC_LOGGER 1
#elif !defined(TALOS_PROBE_SANITIZED) && defined(__GLIBC__)
#define TALOS_PROBE_GLIBC 1
#endif

namespace {

// Hook state, read inside malloc: plain atomics and a fixed buffer, since the
// hook must not allocate (backtrace() does not, once primed).
struct Site {
  void* frames[32];
  int depth;
};
constexpr std::size_t kMaxSites = 8;
Site g_sites[kMaxSites];
std::atomic<bool> g_counting{false};
std::atomic<pthread_t> g_thread{};
std::atomic<std::uint64_t> g_count{0};
// Set while the hook records a site, so an allocation the recording itself
// makes (glibc's first backtrace() loads libgcc_s) is neither counted nor
// recursed into. Only the counted thread writes it.
bool g_in_hook = false;

[[maybe_unused]] void Count() {
  if (!g_counting.load(std::memory_order_relaxed)) return;
  if (pthread_equal(pthread_self(), g_thread.load(std::memory_order_relaxed)) ==
      0) {
    return;
  }
  if (g_in_hook) return;
  g_in_hook = true;
  const std::uint64_t n = g_count.fetch_add(1, std::memory_order_relaxed);
  if (n < kMaxSites) {
    Site& site = g_sites[n];
    site.depth =
        ::backtrace(site.frames, static_cast<int>(std::size(site.frames)));
  }
  g_in_hook = false;
}

}  // namespace

#if defined(TALOS_PROBE_MALLOC_LOGGER)
// libsystem_malloc calls malloc_logger, when set, on every allocation and
// free in the process, from the allocating thread.
using MallocLogger = void(std::uint32_t, std::uintptr_t, std::uintptr_t,
                          std::uintptr_t, std::uintptr_t, std::uint32_t);
extern "C" MallocLogger* malloc_logger;

namespace {
void LogAllocation(std::uint32_t type, std::uintptr_t, std::uintptr_t,
                   std::uintptr_t, std::uintptr_t, std::uint32_t) {
  constexpr std::uint32_t kAllocate = 2;  // MALLOC_LOG_TYPE_ALLOCATE
  if ((type & kAllocate) != 0) Count();
}
}  // namespace
#endif

#if defined(TALOS_PROBE_GLIBC)
// glibc supports replacing its allocator by defining these in the program
// (glibc's headers declare them noexcept); each forwards to glibc's own entry
// point, so memory from either side frees through the other.
extern "C" {
// NOLINTBEGIN(bugprone-reserved-identifier)
void* __libc_malloc(std::size_t);
void* __libc_calloc(std::size_t, std::size_t);
void* __libc_realloc(void*, std::size_t);
void* __libc_memalign(std::size_t, std::size_t);
void* __libc_valloc(std::size_t);
void* __libc_pvalloc(std::size_t);
void __libc_free(void*);
// NOLINTEND(bugprone-reserved-identifier)

void* malloc(std::size_t size) noexcept {
  Count();
  return __libc_malloc(size);
}
void* calloc(std::size_t count, std::size_t size) noexcept {
  Count();
  return __libc_calloc(count, size);
}
void* realloc(void* pointer, std::size_t size) noexcept {
  Count();
  return __libc_realloc(pointer, size);
}
void free(void* pointer) noexcept { __libc_free(pointer); }
void* memalign(std::size_t alignment, std::size_t size) noexcept {
  Count();
  return __libc_memalign(alignment, size);
}
// glibc 2.38's own: a zero or non-power-of-two alignment fails with EINVAL,
// where memalign would round it up.
void* aligned_alloc(std::size_t alignment, std::size_t size) noexcept {
  if (alignment == 0 || (alignment & (alignment - 1)) != 0) {
    errno = EINVAL;
    return nullptr;
  }
  Count();
  return __libc_memalign(alignment, size);
}
int posix_memalign(void** out, std::size_t alignment,
                   std::size_t size) noexcept {
  if (alignment < sizeof(void*) || (alignment & (alignment - 1)) != 0) {
    return EINVAL;
  }
  Count();
  void* pointer = __libc_memalign(alignment, size);
  if (pointer == nullptr) return ENOMEM;
  *out = pointer;
  return 0;
}
void* valloc(std::size_t size) noexcept {
  Count();
  return __libc_valloc(size);
}
void* pvalloc(std::size_t size) noexcept {
  Count();
  return __libc_pvalloc(size);
}
}  // extern "C"
#endif

namespace talos::testing {

std::string_view AllocationProbe::Unsupported() {
#if defined(TALOS_PROBE_MALLOC_LOGGER) || defined(TALOS_PROBE_GLIBC)
  return {};
#elif defined(TALOS_PROBE_SANITIZED)
  return "a sanitizer's allocator bypasses the allocation hooks";
#else
  return "allocation counting needs macOS malloc_logger or glibc";
#endif
}

AllocationProbe::~AllocationProbe() {
  if (counting_) stop();
}

void AllocationProbe::start() {
  if (const std::string_view why = Unsupported(); !why.empty()) {
    throw std::logic_error(std::string{why});
  }
  if (counting_ || g_counting.load()) {
    throw std::logic_error("another AllocationProbe is counting");
  }
  // glibc's first backtrace() loads libgcc_s, which allocates.
  void* prime[1];
  ::backtrace(prime, 1);
  g_count = 0;
  g_thread = pthread_self();
  counting_ = true;
  g_counting = true;
#if defined(TALOS_PROBE_MALLOC_LOGGER)
  malloc_logger = LogAllocation;
#endif
}

std::uint64_t AllocationProbe::stop() {
  if (!counting_) throw std::logic_error("AllocationProbe is not counting");
#if defined(TALOS_PROBE_MALLOC_LOGGER)
  malloc_logger = nullptr;
#endif
  g_counting = false;
  counting_ = false;
  return g_count.load();
}

std::string AllocationProbe::sites() const {
  std::string out;
  const std::uint64_t recorded = std::min<std::uint64_t>(g_count, kMaxSites);
  for (std::uint64_t n = 0; n < recorded; ++n) {
    const Site& site = g_sites[n];
    out += "allocation " + std::to_string(n + 1) + ":\n";
    char** symbols = ::backtrace_symbols(site.frames, site.depth);
    for (int i = 0; i < site.depth; ++i) {
      out += "  ";
      out += symbols != nullptr ? symbols[i] : "?";
      out += '\n';
    }
    // reinterpret_cast: clang-tidy 18's multi-level-pointer check flags the
    // operand of a static_cast to void*.
    std::free(reinterpret_cast<void*>(symbols));
  }
  return out;
}

}  // namespace talos::testing
