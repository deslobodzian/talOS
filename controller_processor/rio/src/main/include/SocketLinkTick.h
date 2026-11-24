#pragma once

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include "DriverStationSampler.h"
#include "LoopConsole.h"
#include "RioLog.h"
#include "RobotSafety.h"
#include "SocketRetry.h"
#include "driver_station/driver_station.h"
#include "protocol/udp.h"

namespace talos::rio {

// RioLog kMessage indices (rate-limit keys) for socket-link reports in
// Robot.cpp; 2 is Robot.cpp's kPersistMessage, 0, 7, 8, 13 and 16 to 31 are
// PhoenixBackend's (configure failure, CAN bus status, CAN bus reader, restore
// failure, licence refusals), and 9 to 12 are LoopConsole's.
inline constexpr uint16_t kSocketMessage = 1;
inline constexpr uint16_t kSocketOpenMessage = 3;
inline constexpr uint16_t kSocketLostMessage = 4;
inline constexpr uint16_t kSocketReplacedMessage = 5;
inline constexpr uint16_t kSocketSendMessage = 6;
inline constexpr uint16_t kSocketSendRecoveredMessage = 14;

inline void ReportRioMessage(RioLog& log, uint16_t index,
                             std::string_view text) {
  LogEvent event;
  event.index = index;
  event.SetText(text);
  log.Post(event);
}

// Per-tick inputs and mutable loop state for TickSocketLink(), the
// Driver Station mode-observe, socket-open/reopen, receive/send error
// handling, endpoint/gateway tick, and Driver Station send/revert step of
// Robot::StartCompetition() (Robot.cpp). Extracted so SocketRetry_test.cpp
// drives Robot.cpp's production socket-loss tick path deterministically
// without NetComm or wall-clock sleeps (REQ-RT-22).
struct SocketLinkTickContext {
  uint64_t now_us{0};
  uint64_t boot_id{0};
  uint64_t period_us{5'000};
  uint64_t ds_refreshed_us{0};
  const driver_station::DriverStationData& driver_station_view;
  SocketRetry& socket_retry;
  DriverStationSampler& ds_sampler;
  RioLog& log;
  LoopConsole& loop_console;
  std::array<uint8_t, protocol::kMaxPayloadSize>& ds_buffer;
  driver_station::DriverStationData& ds_packet;
};

template <typename EndpointLike, typename GatewayLike, typename OpenInitial,
          typename PreArmWatchdog, typename NowNsFn>
void TickSocketLink(SocketLinkTickContext& ctx, EndpointLike& endpoint,
                    GatewayLike& gateway, const OpenInitial& open_initial,
                    const PreArmWatchdog& pre_arm_watchdog,
                    const NowNsFn& now_ns) {
  const uint64_t now_us = ctx.now_us;
  const uint32_t mode_flags = ctx.driver_station_view.fms.flags;
  ctx.ds_sampler.Observe(mode_flags, now_us);
  ctx.log.ClosePhase(TickPhase::kDriverStation, now_ns());

  auto open_status = protocol::UdpStatus::kOk;
  int open_errno = 0;
  const auto socket_event = ctx.socket_retry.Poll(now_us, [&] {
    // The failing call's errno (Open's close of the half-made socket leaves
    // it): ENETUNREACH while the network has no route yet. A lost socket is
    // replaced on the same ports, its frame sequence carried on.
    errno = 0;
    open_status =
        ctx.socket_retry.replacing() ? endpoint.Reopen() : open_initial();
    open_errno = errno;
    return open_status == protocol::UdpStatus::kOk;
  });
  if (socket_event == SocketRetry::Event::kFailed) {
    // RioLog coalesces these to one line a second with their count. The errno
    // is printed as a number (101 is ENETUNREACH): strerror may take a locale
    // lock on this thread.
    char text[128];
    std::snprintf(text, sizeof(text),
                  "hardware gateway socket failed: %s (errno %d)",
                  protocol::UdpStatusName(open_status), open_errno);
    ReportRioMessage(ctx.log, kSocketMessage, text);
  } else if (socket_event == SocketRetry::Event::kOpened &&
             ctx.socket_retry.failures() > 0) {
    char text[128];
    std::snprintf(
        text, sizeof(text),
        "hardware gateway socket open after %u failed attempts, %.3f s "
        "after the first",
        static_cast<unsigned>(ctx.socket_retry.failures()),
        static_cast<double>(now_us > ctx.socket_retry.first_failure_us()
                                ? now_us - ctx.socket_retry.first_failure_us()
                                : 0) /
            1e6);
    ReportRioMessage(ctx.log, kSocketOpenMessage, text);
  } else if (socket_event == SocketRetry::Event::kReplaced) {
    char text[128];
    std::snprintf(
        text, sizeof(text),
        "hardware gateway socket replaced after %u failed attempts, %.3f s "
        "after it was lost",
        static_cast<unsigned>(ctx.socket_retry.failures()),
        static_cast<double>(now_us > ctx.socket_retry.lost_us()
                                ? now_us - ctx.socket_retry.lost_us()
                                : 0) /
            1e6);
    ReportRioMessage(ctx.log, kSocketReplacedMessage, text);
  }

  // A send that finds the socket unable to send again loses it
  // (SocketRetry::SendFailed): nothing more is sent this tick, and the next
  // due Poll replaces it, usually on the next tick, which then runs as usual;
  // a tick with no socket open ticks the gateway disabled and sends nothing,
  // as before the first open. Any other send error keeps it, and a spell of
  // them prints a line when it starts and one once sends have gone a second
  // without failing (separate keys so recovery is not rate-limited behind the
  // spell's opening line).
  const auto sent = [&](protocol::UdpStatus status) {
    using SendEvent = SocketRetry::SendEvent;
    const SendEvent event = ctx.socket_retry.ObserveSend(
        now_us, status, endpoint.udp_stats().last_send_error);
    char text[160];
    switch (event) {
      case SendEvent::kNone:
        return;
      case SendEvent::kWouldBlock:
        ctx.loop_console.ObserveSendWouldBlock(
            endpoint.udp_stats().send_would_block);
        return;
      case SendEvent::kLost:
        endpoint.Close();
        std::snprintf(text, sizeof(text),
                      "hardware gateway socket cannot send (errno %d); "
                      "replacing it, nothing is sent until then",
                      ctx.socket_retry.last_send_error());
        ReportRioMessage(ctx.log, kSocketLostMessage, text);
        return;
      case SendEvent::kFailing:
        std::snprintf(text, sizeof(text),
                      "hardware gateway socket send failed (errno %d); "
                      "keeping the socket",
                      ctx.socket_retry.spell_error());
        ReportRioMessage(ctx.log, kSocketSendMessage, text);
        return;
      case SendEvent::kRecovered: {
        // The last errno only when the spell's changed.
        char last[32] = "";
        if (ctx.socket_retry.last_send_error() !=
            ctx.socket_retry.spell_error()) {
          std::snprintf(last, sizeof(last), ", last errno %d",
                        ctx.socket_retry.last_send_error());
        }
        // From the spell's first failure to its last: the line comes
        // SocketRetry::kSpellQuietUs after that.
        std::snprintf(
            text, sizeof(text),
            "hardware gateway socket sending again: %u sends failed over "
            "%.3f s (errno %d%s)",
            static_cast<unsigned>(ctx.socket_retry.spell_failures()),
            static_cast<double>(ctx.socket_retry.spell_last_failure_us() >
                                        ctx.socket_retry.spell_start_us()
                                    ? ctx.socket_retry.spell_last_failure_us() -
                                          ctx.socket_retry.spell_start_us()
                                    : 0) /
                1e6,
            ctx.socket_retry.spell_error(), last);
        ReportRioMessage(ctx.log, kSocketSendRecoveredMessage, text);
        return;
      }
    }
  };

  if (ctx.socket_retry.open()) {
    const bool enabled = (mode_flags & driver_station::kDsAttached) != 0 &&
                         (mode_flags & driver_station::kEnabled) != 0 &&
                         (mode_flags & driver_station::kEStop) == 0;
    // Pre-arm the watchdog before Endpoint::Tick() when enabled with no active
    // command so a stall inside the first Apply()/ApplyOutputs() after enable,
    // disable or recovery is still neutralized (REQ-SAFE-25).
    if (const uint64_t pre_arm_us = WatchdogPreArmDeadlineUs(
            now_us, enabled, gateway.command_expiry_us(),
            gateway.config().command_timeout_us, ctx.period_us);
        pre_arm_us != 0) {
      pre_arm_watchdog(pre_arm_us);
    }
    // Endpoint handles clock probes even disabled/uncommissioned, echoing only
    // on a subsequent tick so its sample stamp follows probe receipt.
    const auto tick_status = endpoint.Tick(now_us, enabled);
    // Permanent receive errors close the socket and schedule a 100 ms
    // SocketRetry reopen; transient errors (ECONNREFUSED) keep it open.
    ctx.socket_retry.ObserveReceive(now_us, endpoint.last_receive_status(),
                                    endpoint.last_receive_error());
    if (!ctx.socket_retry.open()) {
      endpoint.Close();
      char text[160];
      std::snprintf(text, sizeof(text),
                    "hardware gateway socket cannot receive (errno %d); "
                    "replacing it, nothing is sent until then",
                    ctx.socket_retry.last_recv_error());
      ReportRioMessage(ctx.log, kSocketLostMessage, text);
    } else {
      sent(tick_status);
    }
    // The socket retry and the endpoint's tick.
    const uint64_t ticked_ns = now_ns();
    ctx.log.ClosePhase(TickPhase::kLink, ticked_ns);
    // The age of the drained command's observation at the end of the tick that
    // drained it, on this clock: observed_time_us is the sample_time_us of a
    // State this gateway sent (REQ-RT-25). RioLog drops a difference that
    // wrapped (an observation in this clock's future).
    if (const auto& drained = endpoint.drained(); drained.present) {
      ctx.log.RecordCommandAge(ticked_ns / 1000 - drained.observed_time_us,
                               drained.accepted);
    }

    // The Driver Station only produces new data at 50 Hz, so sampling it on
    // every hardware tick would put three duplicate packets on the link, and
    // in the dispatch log downstream, for every real one. A new mode goes out
    // on the tick that observes it: an enabled switch (autonomous to teleop)
    // leaves the State unchanged, so until its generation arrives the old
    // mode's target lanes still drive. Not on a socket the State send just
    // lost: the replacement sends it.
    if (ctx.ds_sampler.Sample(ctx.driver_station_view, now_us, ctx.boot_id,
                              ctx.socket_retry.open(), ctx.ds_packet)) {
      // Encode() returns 0 if the payload will not fit; send nothing rather
      // than an empty frame the receiver would forward as a decode failure.
      const auto ds_size = driver_station::Encode(ctx.ds_packet, ctx.ds_buffer);
      ctx.log.ClosePhase(TickPhase::kDriverStation, now_ns());
      if (ds_size) {
        const auto ds_status =
            endpoint.SendFrame(protocol::FrameType::kDriverStation, now_us,
                               ctx.ds_buffer.data(), ds_size);
        sent(ds_status);
        const uint64_t ds_sent_ns = now_ns();
        ctx.log.ClosePhase(TickPhase::kLink, ds_sent_ns);
        if (ds_status != protocol::UdpStatus::kOk) {
          ctx.ds_sampler.RevertSample();
        } else if (ctx.ds_refreshed_us != 0) {
          // The sent snapshot's refresh age at its send (REQ-RT-25); none for
          // the neutral snapshot, which no refresh made.
          ctx.log.RecordDriverStationAge(ds_sent_ns / 1000 -
                                         ctx.ds_refreshed_us);
        }
      } else {
        ctx.ds_sampler.RevertSample();
      }
    }
  } else {
    gateway.Tick(now_us, false);
    // The socket retry and the gateway's tick.
    ctx.log.ClosePhase(TickPhase::kLink, now_ns());
  }
}

}  // namespace talos::rio
