#pragma once

#include <cstdint>

#include "protocol/udp.h"

// When the RoboRIO loop (Robot.cpp) tries to open the gateway's socket, and
// to replace one that can no longer send.
//
// The socket is connected to the companion, so at program start after a
// RoboRIO boot, before the network has an address and a route to it, the
// open's connect() fails (ENETUNREACH); until it opens the loop ticks the
// gateway disabled, every output neutral, and no State leaves. An attempt is a
// few non-blocking calls (socket, setsockopt, bind, connect, close; no name
// lookup, nothing waits), so retrying every kPeriodUs costs the 5 ms loop
// nothing it would notice, and State goes out within one period of the route
// appearing.
//
// A send or receive that fails with an error the socket may never recover from
// (protocol::SendErrorNeedsNewSocket / protocol::ReceiveErrorNeedsNewSocket: a
// socket shut down, invalid fd, or its source address gone from the host,
// which Linux reports as ENETUNREACH) loses the socket: nothing more is sent
// on that tick, and the next attempt opens a new socket on the same ports
// (Endpoint::Reopen, keeping the frame sequence), on the same cadence: at once
// if no attempt was made in the last kPeriodUs, otherwise kPeriodUs after the
// last, so a condition that also breaks every new socket costs at most ten
// sockets a second, and a network with no route at all costs one failed open
// per kPeriodUs, as at boot. Usually that is the next tick, which then runs as
// any other: a replaced socket is a link gap of one tick, and the command in
// force keeps its usual expiry. Closing the old socket drops the datagrams
// queued on it, and a command that arrives later passes the gateway's usual
// checks (sequence, boot, epoch, observation age). Only a tick with no socket
// open (an attempt failed, or none is due yet) ticks the gateway disabled and
// sends nothing. Any other send error (ENOBUFS, ECONNREFUSED, ...) clears on
// the same socket and keeps it; EAGAIN is reported as kWouldBlock via
// SendWouldBlock while keeping the socket open without triggering a reopen
// (the cumulative count lives on RuntimeUdpPeer::stats().send_would_block).
// Transient receive errors (such as ECONNREFUSED) and non-socket outcomes
// (kOk, kWouldBlock, kDecodeError) keep the socket open.
// Every other failed send is counted in its spell (SendEvent), which the loop
// reports when it starts and when sends succeed again, so a failure that keeps
// the socket shows on the console too.
namespace talos::rio {

class SocketRetry {
 public:
  static constexpr uint64_t kPeriodUs = 100'000;

  enum class Event : uint8_t {
    kNone,      // open already, or no attempt due
    kFailed,    // an attempt failed
    kOpened,    // an attempt opened the socket
    kReplaced,  // an attempt opened a socket to replace one that was lost
  };

  // The loop tick at `now_us` (steady clock): calls `open` (returning true
  // when the socket opened) if the socket is not open and an attempt is due,
  // at most once per kPeriodUs. Never allocates.
  template <typename Open>
  Event Poll(uint64_t now_us, const Open& open) {
    if (open_ || (attempted_ && (now_us < last_attempt_us_ ||
                                 now_us - last_attempt_us_ < kPeriodUs))) {
      return Event::kNone;
    }
    attempted_ = true;
    last_attempt_us_ = now_us;
    if (open()) {
      open_ = true;
      const bool replaced = replacing_;
      replacing_ = false;
      return replaced ? Event::kReplaced : Event::kOpened;
    }
    if (failures_++ == 0) first_failure_us_ = now_us;
    return Event::kFailed;
  }

  // What a send's outcome is for the console (Robot.cpp prints one line for
  // each but kNone). A spell is a run of failed sends, on the socket or its
  // replacements, ended by a send that succeeds kSpellQuietUs after the last
  // of them: good sends between failures do not end it (while the
  // companion's port is closed, every other send fails with ECONNREFUSED).
  enum class SendEvent : uint8_t {
    kNone,
    // The spell's first failure with an error that keeps the socket.
    kFailing,
    // An error the socket can never send past: it is lost (see SendFailed).
    kLost,
    // A send succeeded, ending a spell that reported kFailing.
    kRecovered,
    // A non-blocking send returned EAGAIN/EWOULDBLOCK (send buffer full):
    // counted without losing the socket, triggering a reopen, or ending an
    // open failing spell.
    kWouldBlock,
  };

  // A send on the open socket failed at `now_us` with OS error `error`
  // (UdpStats::last_send_error; 0 for a short send). When the socket can
  // never send again it is lost: open() is false from here, replacing() true
  // until a Poll replaces it, and failures() counts that replacement's
  // failed attempts. Any other error keeps it. Counts the failure in the
  // spell, starting one if none is open.
  SendEvent SendFailed(uint64_t now_us, int error) {
    if (!open_) return SendEvent::kNone;
    if (!in_spell_) {
      in_spell_ = true;
      spell_reported_ = false;
      spell_failures_ = 0;
      spell_start_us_ = now_us;
    }
    ++spell_failures_;
    spell_last_failure_us_ = now_us;
    last_send_error_ = error;
    if (protocol::SendErrorNeedsNewSocket(error)) {
      open_ = false;
      replacing_ = true;
      lost_us_ = now_us;
      failures_ = 0;
      first_failure_us_ = 0;
      return SendEvent::kLost;
    }
    if (spell_reported_) return SendEvent::kNone;
    spell_reported_ = true;
    spell_error_ = error;
    return SendEvent::kFailing;
  }

  // A send on the open socket succeeded at `now_us`: ends the spell once
  // this long has passed since its last failure.
  static constexpr uint64_t kSpellQuietUs = 1'000'000;
  SendEvent SendSucceeded(uint64_t now_us) {
    if (!in_spell_ || now_us < spell_last_failure_us_ ||
        now_us - spell_last_failure_us_ < kSpellQuietUs) {
      return SendEvent::kNone;
    }
    in_spell_ = false;
    return spell_reported_ ? SendEvent::kRecovered : SendEvent::kNone;
  }

  // A non-blocking send on the open socket returned UdpStatus::kWouldBlock
  // (EAGAIN/EWOULDBLOCK, send buffer full). Keeps the socket open without
  // triggering a reopen and leaves any open failing spell untouched.
  SendEvent SendWouldBlock() const {
    if (!open_) return SendEvent::kNone;
    return SendEvent::kWouldBlock;
  }

  // Dispatches one send outcome on the open socket.
  SendEvent ObserveSend(uint64_t now_us, protocol::UdpStatus status,
                        int last_send_error) {
    switch (status) {
      case protocol::UdpStatus::kOk:
        return SendSucceeded(now_us);
      case protocol::UdpStatus::kSocketError:
        return SendFailed(now_us, last_send_error);
      case protocol::UdpStatus::kWouldBlock:
        return SendWouldBlock();
      case protocol::UdpStatus::kInvalidArgument:
      case protocol::UdpStatus::kDecodeError:
        return SendEvent::kNone;
    }
    return SendEvent::kNone;
  }

  // A receive on the open socket failed at `now_us` with OS error `error`
  // (UdpStats::last_recv_error). Permanent socket errors
  // (protocol::ReceiveErrorNeedsNewSocket) lose the socket so the next due
  // Poll replaces it on the same kPeriodUs cadence as SendFailed; transient
  // errors (such as ECONNREFUSED) keep it open. Returns true when this call
  // lost the socket.
  bool ReceiveFailed(uint64_t now_us, int error) {
    if (!open_) return false;
    if (!protocol::ReceiveErrorNeedsNewSocket(error)) return false;
    last_recv_error_ = error;
    open_ = false;
    replacing_ = true;
    lost_us_ = now_us;
    failures_ = 0;
    first_failure_us_ = 0;
    return true;
  }

  // Dispatches one receive outcome on the open socket: permanent receive
  // socket errors transition into replacing state on the same kPeriodUs
  // cadence as ObserveSend, while kOk, kWouldBlock, kDecodeError,
  // kInvalidArgument, and transient errors such as ECONNREFUSED keep the
  // socket open.
  void ObserveReceive(uint64_t now_us, protocol::UdpStatus status,
                      int last_recv_error) {
    if (status == protocol::UdpStatus::kSocketError) {
      ReceiveFailed(now_us, last_recv_error);
    }
  }

  // The current or last spell: its failed sends, the times of the first and
  // the last, the error its kFailing named, and the last send error.
  uint32_t spell_failures() const { return spell_failures_; }
  uint64_t spell_start_us() const { return spell_start_us_; }
  uint64_t spell_last_failure_us() const { return spell_last_failure_us_; }
  int spell_error() const { return spell_error_; }
  int last_send_error() const { return last_send_error_; }
  int last_recv_error() const { return last_recv_error_; }

  bool open() const { return open_; }
  // Lost and not yet replaced: `open` must open the same ports again
  // (Endpoint::Reopen) rather than a first socket.
  bool replacing() const { return replacing_; }
  // Failed attempts since the program started or, after a loss, since the
  // socket was lost, and the time of the first of them (0 before one); the
  // time of the last loss (0 before one).
  uint32_t failures() const { return failures_; }
  uint64_t first_failure_us() const { return first_failure_us_; }
  uint64_t lost_us() const { return lost_us_; }

 private:
  bool open_{false};
  bool replacing_{false};
  bool attempted_{false};
  uint64_t last_attempt_us_{0};
  uint32_t failures_{0};
  uint64_t first_failure_us_{0};
  uint64_t lost_us_{0};
  bool in_spell_{false};
  bool spell_reported_{false};
  uint32_t spell_failures_{0};
  uint64_t spell_start_us_{0};
  uint64_t spell_last_failure_us_{0};
  int spell_error_{0};
  int last_send_error_{0};
  int last_recv_error_{0};
};

}  // namespace talos::rio
