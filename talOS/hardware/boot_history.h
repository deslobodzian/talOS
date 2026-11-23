#pragma once

// The gateway boots a stream has been in: the current one and the last
// kRetiredBoots it left. Every gateway program start picks a fresh random
// boot id and restarts its sample clock, so a stream's ordering (sample and
// mode times) holds only within one boot. A boot id that is neither current
// nor retired is a genuine reboot: the stream enters it and starts its
// ordering over. A retired boot id is a datagram delayed from a boot the
// gateway has left; it must never be taken for current again.
//
//   if (boots.retired(id)) drop;
//   else if (!boots.current(id)) { boots.Enter(id); restart ordering; }
//   else if (not newer than the last accepted) drop;

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace talos::hardware {

class BootHistory {
 public:
  static constexpr std::size_t kRetiredBoots = 4;

  // Whether the stream is in `boot_id`.
  bool current(std::uint64_t boot_id) const {
    return have_ && boot_id == current_;
  }
  // Whether `boot_id` is one of the last kRetiredBoots the stream left.
  bool retired(std::uint64_t boot_id) const {
    const auto end = retired_.begin() + static_cast<std::ptrdiff_t>(count_);
    return std::find(retired_.begin(), end, boot_id) != end;
  }
  // The current boot; 0 before the first.
  std::uint64_t id() const { return current_; }
  bool empty() const { return !have_; }

  // Moves the stream into `boot_id`, retiring the boot it was in (the oldest
  // retired one is forgotten). For a boot that is neither current nor
  // retired.
  void Enter(std::uint64_t boot_id) {
    if (have_) {
      retired_[next_] = current_;
      next_ = (next_ + 1) % kRetiredBoots;
      count_ = std::min(count_ + 1, kRetiredBoots);
    }
    current_ = boot_id;
    have_ = true;
  }

 private:
  std::array<std::uint64_t, kRetiredBoots> retired_{};  // ring buffer
  std::size_t next_{0};
  std::size_t count_{0};
  std::uint64_t current_{0};
  bool have_{false};
};

}  // namespace talos::hardware
