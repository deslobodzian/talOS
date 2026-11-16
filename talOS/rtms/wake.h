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
// receiver is destroyed; the file of a receiver killed before that is removed
// by a later receiver's open once it is kWakeFileReclaimAgeS old
// (ReclaimDeadWakeFiles), and a receiver that finds its own file gone right
// after its bind binds afresh.

#include <cstddef>
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

// How old (inode change time, wall-clock seconds) a socket file must be
// before ReclaimDeadWakeFiles judges it. macOS's bind() creates the file
// before it attaches the socket, and a connect in between is refused, so a
// refused connect proves a file dead only once no bind can still be in
// progress on it: bind is one system call, so a binder stays in that window
// only while it is preempted, far below this on a working host.
inline constexpr std::int64_t kWakeFileReclaimAgeS = 10;

// Removes each `talos-wake-<16 hex digits>` socket file in `directory` that
// changed at least kWakeFileReclaimAgeS before `now_wall_s` and that no
// socket is bound to any more, as a connect to it proves (refused): the file
// a receiver killed before its destructor ran leaves on macOS. A live
// receiver's file accepts the connect and is kept. Returns how many it
// removed. Every WakeReceiver runs it on /tmp, with the wall clock, before it
// binds there (not on Linux, whose receivers make no file).
std::size_t ReclaimDeadWakeFiles(const char* directory,
                                 std::int64_t now_wall_s) noexcept;

namespace detail {
// Test seam: called with a receiver's path between its bind and its check
// that the file is still there (macOS only; a Linux receiver has no file).
// nullptr outside tests.
inline void (*after_wake_bind_for_testing)(const char* path) = nullptr;
}  // namespace detail

// What one notification came to.
enum class WakeOutcome : std::uint8_t {
  // The receiver has a notification pending: this one, or an earlier one
  // its full queue already held.
  DELIVERED,
  // No receiver is bound at the id any more.
  RECEIVER_GONE,
  // The hint was lost for another reason (no descriptor left, a permission
  // refused, an unexpected errno); the receiver is not known to be gone.
  FAILED,
};

struct WakeStatus {
  WakeOutcome outcome{WakeOutcome::DELIVERED};
  int error{0};  // the errno behind RECEIVER_GONE or FAILED, else 0
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

  // Notifies receiver `id`. Connects on first use and reconnects when `id`
  // differs from the cached one. Never blocks, never raises a signal. An id
  // of 0 names no receiver: RECEIVER_GONE with error 0.
  WakeStatus send(std::uint64_t id) noexcept;
  // send(), as false only when the receiver no longer exists: a FAILED
  // notification is true, since the receiver may still be there.
  bool notify(std::uint64_t id) noexcept {
    return send(id).outcome != WakeOutcome::RECEIVER_GONE;
  }
  // Closes the cached endpoint.
  void reset() noexcept;

 private:
  int fd_{-1};
  std::uint64_t id_{0};
};

}  // namespace talos
