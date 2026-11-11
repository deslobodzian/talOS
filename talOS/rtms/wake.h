#pragma once

// A cross-process wake channel: a publisher tells a reader that is asleep on
// a topic that there is something to read. A notification is only ever a
// hint to go read; what arrived, and whether anything did, is still decided
// by the RTMS sequence numbers. See README.md "Wake channel".
//
// One AF_UNIX datagram socket per receiver, bound to an address derived only
// from its random 64-bit id: Linux's abstract namespace
// ("\0talos-wake-<16 hex>", no filesystem entry), or on macOS
// /tmp/talos-wake-<16 hex> (sun_path holds 104 bytes), unlinked when the
// receiver is destroyed.

#include <cstdint>

namespace talos {

// The reading side of a wake channel: an fd that turns readable when any
// process notifies this receiver's id. Move-only.
class WakeReceiver {
 public:
  // Throws std::system_error when no socket can be created or bound.
  WakeReceiver();
  ~WakeReceiver();

  WakeReceiver(WakeReceiver&& other) noexcept;
  WakeReceiver& operator=(WakeReceiver&& other) noexcept;
  WakeReceiver(const WakeReceiver&) = delete;
  WakeReceiver& operator=(const WakeReceiver&) = delete;

  // Never 0 while the receiver holds its socket; unique across the
  // machine's live receivers (a collision fails the bind and draws again).
  std::uint64_t id() const { return id_; }
  // For a poller's add_fd(): readable while notifications are pending.
  int fd() const { return fd_; }
  // Consumes every pending notification without blocking.
  void drain() noexcept;

 private:
  void close() noexcept;

  int fd_{-1};
  std::uint64_t id_{0};
};

// The notifying side: one cached endpoint. Default-constructible, move-only.
class WakeSender {
 public:
  WakeSender() = default;
  ~WakeSender();

  WakeSender(WakeSender&& other) noexcept;
  WakeSender& operator=(WakeSender&& other) noexcept;
  WakeSender(const WakeSender&) = delete;
  WakeSender& operator=(const WakeSender&) = delete;

  // Notifies receiver `id` (nonzero). Connects on first use and reconnects
  // when `id` differs from the cached one. Never blocks, never raises a
  // signal. Returns false when the receiver no longer exists; true
  // otherwise, including when the receiver already has notifications pending
  // (its queue is full) and when this process cannot open a socket (the hint
  // is then lost; the receiver is not known to be gone).
  bool notify(std::uint64_t id) noexcept;
  // Closes the cached endpoint.
  void reset() noexcept;

 private:
  int fd_{-1};
  std::uint64_t id_{0};
};

}  // namespace talos
