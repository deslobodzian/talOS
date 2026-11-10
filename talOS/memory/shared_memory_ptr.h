#pragma once

#include <fcntl.h>
#include <sys/mman.h>
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

// The name a topic gets in the operating system's shared-memory namespace.
//
// POSIX defines the portable form of an shm name as an initial slash followed
// by characters "none of which are slashes", and glibc enforces exactly that:
// shm_open returns EINVAL on an interior slash. macOS keeps shm names in a flat
// kernel namespace and accepts them, so a slashed topic works there and fails
// in CI. Every talOS topic has two or three segments (talOS/NAMING.md), so the
// mapping is not an edge case -- it is on the path of every topic in the robot.
//
// The separator is '.' rather than '_' because the mapping has to be
// injective. Topic segments may contain underscores -- `driver_station`,
// `operator_interface` -- so mapping the slash to '_' makes
// `/hw/state/driver_station` and `/hw/state_driver/station` the same segment
// name, which silently joins two unrelated streams. That is a worse failure
// than either topic refusing to open. The grammar allows no '.' inside a
// segment, so '.' cannot collide, and it costs nothing: '.' is an ordinary
// character in /dev/shm, and the mapping preserves length so the 30-character
// cap keeps meaning the same thing on both sides.
//
// Public because anything that opens a segment directly -- RTMS's unlink of a
// closing segment, a test, a tool listing /dev/shm -- needs this name and not
// the topic.
inline std::string ShmObjectName(std::string_view topic, std::string_view ns) {
  if (topic.starts_with("/~")) return std::string{topic};
  std::string_view stripped = topic;
  while (stripped.starts_with('/')) {
    stripped.remove_prefix(1);
  }
  std::string result;
  result.reserve(stripped.size() + 1);
  result.push_back('/');
  for (const char c : stripped) {
    result.push_back(c == '/' ? '.' : c);
  }
  if (!ns.empty()) {
    std::uint64_t hash = 14695981039346656037ULL;
    const auto append = [&hash](std::string_view bytes) {
      for (const unsigned char byte : bytes) {
        hash ^= byte;
        hash *= 1099511628211ULL;
      }
    };
    append(ns);
    append(std::string_view{"\0", 1});
    append(result);
    constexpr char hex[] = "0123456789abcdef";
    result.assign("/~");
    for (int shift = 60; shift >= 0; shift -= 4) {
      result.push_back(hex[(hash >> shift) & 0xf]);
    }
  }
  return result;
}

// Cache once: children inherit the same test/session namespace, while normal
// production processes retain the readable, unnamespaced POSIX names.
inline std::string ShmObjectName(std::string_view topic) {
  static const std::string ns = [] {
    if (const char* value = std::getenv("TALOS_SHM_NAMESPACE")) {
      return std::string{value};
    }
    if (const char* value = std::getenv("TEST_TMPDIR")) {
      return std::string{value};
    }
    return std::string{};
  }();
  return ShmObjectName(topic, ns);
}

enum class SharedMemoryMode { CREATE, ATTACH };

// A mapping of one named segment: created when the name is absent, attached
// otherwise (mode() says which). It never unlinks the name: who removes a
// segment is a lifetime policy of the segment's contents (RTMSQueue's
// session and attach-count rules in talOS/rtms/rtms.h), not of one mapping.
class SharedMemoryPtr {
 public:
  SharedMemoryPtr() = default;  // maps nothing; ptr() is null

  SharedMemoryPtr(std::string_view name, std::size_t size)
      : shm_name_(ShmObjectName(name)), size_(size) {
    if (map_ptr() < 0) {
      throw std::runtime_error("Failed to attach ptr!");
    }
  }

  ~SharedMemoryPtr() { reset(); }

  SharedMemoryPtr(const SharedMemoryPtr&) = delete;
  SharedMemoryPtr& operator=(const SharedMemoryPtr&) = delete;

  SharedMemoryPtr(SharedMemoryPtr&& other) noexcept
      : shm_name_(std::move(other.shm_name_)),
        size_(std::exchange(other.size_, 0)),
        mode_(std::exchange(other.mode_, SharedMemoryMode::ATTACH)),
        ptr_(std::exchange(other.ptr_, nullptr)) {}

  SharedMemoryPtr& operator=(SharedMemoryPtr&& other) noexcept {
    if (this != &other) {
      reset();
      shm_name_ = std::move(other.shm_name_);
      size_ = std::exchange(other.size_, 0);
      mode_ = std::exchange(other.mode_, SharedMemoryMode::ATTACH);
      ptr_ = std::exchange(other.ptr_, nullptr);
    }
    return *this;
  }

  // The segment's name in the shared-memory namespace (ShmObjectName).
  const std::string& name() const { return shm_name_; }

  void* ptr() { return ptr_; }

  const void* ptr() const { return ptr_; }

  SharedMemoryMode mode() const { return mode_; }

 private:
  // Bounded so a real mismatch is still reported promptly: 200 x 250us.
  static constexpr int kAttachAttempts = 200;
  static constexpr useconds_t kAttachRetryUs = 250;

  static off_t to_off_t(std::size_t size) {
    constexpr auto max_off_t =
        static_cast<std::size_t>(std::numeric_limits<off_t>::max());
    if (size > max_off_t) {
      throw std::overflow_error("shared-memory size exceeds off_t range");
    }
    return static_cast<off_t>(size);
  }

  // Opens the name, creating it when absent; mode_ says which. A name found
  // by the exclusive create (EEXIST) can be gone by the attach (ENOENT),
  // unlinked by its last holder in between, and is then free to create, so
  // the create is retried. -1 with errno set when neither succeeds.
  int create_or_attach() {
    for (int attempt = 0; attempt < kAttachAttempts; ++attempt) {
      mode_ = SharedMemoryMode::CREATE;
      int shm_fd = shm_open(shm_name_.c_str(), O_CREAT | O_RDWR | O_EXCL, 0666);
      if (shm_fd != -1 || errno != EEXIST) return shm_fd;
      mode_ = SharedMemoryMode::ATTACH;
      shm_fd = shm_open(shm_name_.c_str(), O_RDWR, 0666);
      if (shm_fd != -1 || errno != ENOENT) return shm_fd;
    }
    return -1;
  }

  int map_ptr() {
    const off_t file_size = to_off_t(size_);
    const int shm_fd = create_or_attach();

    switch (mode_) {
      case SharedMemoryMode::CREATE:
        if (shm_fd == -1) {
          std::perror(std::format("shm_open failed on create for {}", shm_name_)
                          .c_str());
          return -1;
        }

        if (ftruncate(shm_fd, file_size)) {
          const int err = errno;
          std::fprintf(stderr, "ftruncate: fd=%d size=%zu errno=%d (%s)\n",
                       shm_fd, size_, err, std::strerror(err));
          close(shm_fd);
          shm_unlink(shm_name_.c_str());
          return -1;
        }

        break;
      case SharedMemoryMode::ATTACH: {
        if (shm_fd == -1) {
          std::perror(std::format("shm_open failed on attach for {}", shm_name_)
                          .c_str());
          return -1;
        }

        // Mapping more than the creator allocated faults on access,
        // so refuse instead of handing back a landmine.
        //
        // shm_open publishes the name before the creator can call
        // ftruncate, so a peer starting at the same moment can arrive
        // between the two and measure a segment that is still empty.
        // That window is microseconds wide; a genuinely wrong segment
        // stays wrong, so waiting briefly separates the two without
        // masking either.
        struct stat st;
        bool sized = false;
        for (int attempt = 0; attempt < kAttachAttempts; ++attempt) {
          if (fstat(shm_fd, &st) < 0) {
            std::perror("fstat failed");
            close(shm_fd);
            return -1;
          }
          if (static_cast<std::size_t>(st.st_size) >= size_) {
            sized = true;
            break;
          }
          usleep(kAttachRetryUs);
        }

        if (!sized) {
          std::fprintf(stderr,
                       "shared memory %s is %lld bytes, need %zu; it was "
                       "created by a process built against a different "
                       "message layout\n",
                       shm_name_.c_str(), static_cast<long long>(st.st_size),
                       size_);
          close(shm_fd);
          return -1;
        }
        break;
      }
      default:
        return -1;
    }

    void* shm_addr =
        mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_addr == MAP_FAILED) {
      std::perror("mmap failed");
      close(shm_fd);
      return -1;
    }

    // The Linux Programming Interface — Michael Kerrisk
    // Says we can close mmap after mapping.
    if (close(shm_fd) == -1) {
      std::perror("failed to close shm_fd");
      munmap(shm_addr, size_);
      return -1;
    }

    // Registered so a realtime loop keeps it resident (residency.h): made
    // resident here when the process already asked, or when it asks later.
    try {
      talos::memory::TrackMapping(shm_addr, size_);
    } catch (...) {
      munmap(shm_addr, size_);
      throw;
    }
    ptr_ = shm_addr;
    return 0;
  }

  void reset() noexcept {
    if (ptr_ != nullptr) {
      talos::memory::UntrackMapping(ptr_);
      if (munmap(ptr_, size_) == -1) {
        std::perror("munmap failed");
      }
      ptr_ = nullptr;
    }
  }
  std::string shm_name_{"error"};
  std::size_t size_{0};
  SharedMemoryMode mode_{SharedMemoryMode::CREATE};
  void* ptr_ = nullptr;
};
