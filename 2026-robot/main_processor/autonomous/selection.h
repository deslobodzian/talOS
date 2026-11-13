#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>

namespace autonomous {

// Session persistence, not a durable robot setting. Mapping is opened only at
// construction; selecting a routine performs no allocation or filesystem I/O.
// MAP_SHARED retains the byte across process death without waiting for disk.
class Selection {
 public:
  Selection(const std::optional<std::filesystem::path>& session,
            std::uint64_t signature, unsigned count, std::uint8_t fallback)
      : value_{fallback} {
    if (!session) return;
    const auto path = *session / "autonomous-selection";
    const int fd = ::open(path.c_str(), O_RDWR | O_CREAT, 0600);
    if (fd < 0) throw std::runtime_error("autonomous: open selection failed");
    if (::ftruncate(fd, sizeof(Record)) != 0) {
      ::close(fd);
      throw std::runtime_error("autonomous: size selection failed");
    }
    void* memory = ::mmap(nullptr, sizeof(Record), PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd, 0);
    ::close(fd);
    if (memory == MAP_FAILED)
      throw std::runtime_error("autonomous: map selection failed");
    record_ = static_cast<Record*>(memory);
    if (record_->signature == signature && record_->index < count &&
        record_->inverse == static_cast<std::uint8_t>(~record_->index)) {
      value_ = record_->index;
    }
    record_->signature = signature;
    set(value_);
  }
  ~Selection() {
    if (record_) ::munmap(record_, sizeof(Record));
  }
  Selection(const Selection&) = delete;
  Selection& operator=(const Selection&) = delete;
  std::uint8_t get() const { return value_; }
  void set(std::uint8_t index) {
    value_ = index;
    if (record_) {
      record_->index = index;
      record_->inverse = static_cast<std::uint8_t>(~index);
    }
  }

 private:
  struct Record {
    std::uint64_t signature;
    std::uint8_t index;
    std::uint8_t inverse;
  };
  Record* record_ = nullptr;
  std::uint8_t value_;
};
}  // namespace autonomous
