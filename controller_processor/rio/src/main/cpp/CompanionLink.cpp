#include "CompanionLink.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/DriverStation.h>

#include <chrono>
#include <cinttypes>
#include <cstdio>

#include "RioLog.h"
#include "protocol/types.h"

#ifdef __linux__
#include <pthread.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace talos::rio {
namespace {

uint64_t SteadyNowUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// Seconds from `then_us` to `now_us`; -1 for never. A contact stored after
// the poll sampled its clock reads as 0, not as a wrapped age.
double AgeSeconds(uint64_t then_us, uint64_t now_us) {
  if (then_us == 0) return -1.0;
  return then_us >= now_us ? 0.0 : static_cast<double>(now_us - then_us) / 1e6;
}

bool Within(uint64_t then_us, uint64_t now_us, uint64_t window_us) {
  return then_us != 0 && (then_us >= now_us || now_us - then_us < window_us);
}

class DashboardSink final : public CompanionSink {
 public:
  void Publish(const CompanionStatus& status) override {
    frc::SmartDashboard::PutBoolean("companion/connected", status.connected);
    frc::SmartDashboard::PutBoolean("companion/protocol_ok",
                                    status.protocol_ok);
    frc::SmartDashboard::PutNumber("companion/protocol",
                                   protocol::kProtocolVersion);
    frc::SmartDashboard::PutNumber("companion/peer_protocol",
                                   status.peer_protocol);
    frc::SmartDashboard::PutNumber("companion/frame_age_s", status.frame_age_s);
    frc::SmartDashboard::PutNumber("companion/command_age_s",
                                   status.command_age_s);
    frc::SmartDashboard::PutBoolean("companion/configured", status.configured);
    frc::SmartDashboard::PutBoolean("companion/commissioned",
                                    status.commissioned);
    // A 64-bit id does not survive a double; hex matches the bridge's logs.
    char id[24];
    std::snprintf(id, sizeof(id), "%016" PRIx64, status.config_id);
    frc::SmartDashboard::PutString("companion/config_id", id);
  }
  void Report(bool error, std::string_view text) override {
    char details[160];
    std::snprintf(details, sizeof(details), "%.*s",
                  static_cast<int>(text.size()), text.data());
    // printMsg also copies it to the console (the RoboRIO's netconsole log).
    HAL_SendError(error ? 1 : 0, 1, 0, details, "talos companion link", "", 1);
  }
};

}  // namespace

std::unique_ptr<CompanionSink> MakeDashboardSink() {
  return std::make_unique<DashboardSink>();
}

CompanionReporter::~CompanionReporter() { Stop(); }

void CompanionReporter::Observe(const CompanionContact& contact) {
  frame_us_.store(contact.frame_us, std::memory_order_relaxed);
  command_us_.store(contact.command_us, std::memory_order_relaxed);
  // The version before its time (release), so a poll that sees the time
  // (acquire) never reports a mismatch with version 0.
  mismatched_version_.store(contact.mismatched_version,
                            std::memory_order_relaxed);
  version_mismatch_us_.store(contact.version_mismatch_us,
                             std::memory_order_release);
  configured_.store(contact.configured, std::memory_order_relaxed);
  commissioned_.store(contact.commissioned, std::memory_order_relaxed);
  config_id_.store(contact.config_id, std::memory_order_relaxed);
}

void CompanionReporter::Start() {
  if (thread_.joinable()) return;
  stop_.store(false, std::memory_order_relaxed);
  thread_ = std::thread([this] { Run(); });
}

void CompanionReporter::Stop() {
  if (!thread_.joinable()) return;
  stop_.store(true, std::memory_order_relaxed);
  thread_.join();
}

void CompanionReporter::Run() {
#ifdef __linux__
  // Below the loop thread: NetworkTables and the DS error channel may block.
  pthread_setname_np(pthread_self(), "talos-companion");
  setpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)), 19);
#endif
  while (!stop_.load(std::memory_order_relaxed)) {
    Poll(SteadyNowUs());
    std::this_thread::sleep_for(std::chrono::microseconds{kPollPeriodUs});
  }
}

void CompanionReporter::Poll(uint64_t now_us) {
  if (!polled_) {
    polled_ = true;
    first_poll_us_ = now_us;
  }
  const uint64_t frame_us = frame_us_.load(std::memory_order_relaxed);
  const uint64_t mismatch_us =
      version_mismatch_us_.load(std::memory_order_acquire);
  const uint64_t command_us = command_us_.load(std::memory_order_relaxed);

  CompanionStatus status;
  status.connected = Within(frame_us, now_us, kLinkTimeoutUs);
  status.protocol_ok = !Within(mismatch_us, now_us, kLinkTimeoutUs);
  status.peer_protocol = mismatched_version_.load(std::memory_order_relaxed);
  status.frame_age_s = AgeSeconds(frame_us, now_us);
  status.command_age_s = AgeSeconds(command_us, now_us);
  status.configured = configured_.load(std::memory_order_relaxed);
  status.commissioned = commissioned_.load(std::memory_order_relaxed);
  status.config_id = config_id_.load(std::memory_order_relaxed);
  sink_.Publish(status);

  // Every message starts with the console's time prefix (RioLog's
  // ConsoleTime), so it can be placed against the program's console lines.
  char time[kConsoleTimeSize];
  FormatConsoleTime(now_us, time);
  char text[160];
  if (status.protocol_ok != reported_protocol_ok_) {
    reported_protocol_ok_ = status.protocol_ok;
    if (status.protocol_ok) {
      std::snprintf(text, sizeof(text), "%scompanion protocol mismatch cleared",
                    time);
      sink_.Report(false, text);
    } else {
      std::snprintf(text, sizeof(text),
                    "%scompanion speaks protocol %u, this RoboRIO program %u: "
                    "deploy both from one build",
                    time, static_cast<unsigned>(status.peer_protocol),
                    static_cast<unsigned>(protocol::kProtocolVersion));
      sink_.Report(true, text);
    }
  }
  if (status.connected && !reported_connected_) {
    reported_connected_ = true;
    reported_unheard_ = false;
    connected_since_us_ = now_us;
    std::snprintf(text, sizeof(text), "%scompanion connected", time);
    sink_.Report(false, text);
  } else if (!status.connected && reported_connected_) {
    reported_connected_ = false;
    reported_unheard_ = true;
    std::snprintf(text, sizeof(text),
                  "%scompanion lost: no frame for %.1f s; every output is "
                  "neutral",
                  time, status.frame_age_s);
    sink_.Report(true, text);
  } else if (!status.connected && !reported_unheard_ && status.protocol_ok &&
             now_us - first_poll_us_ >= kStartupGraceUs) {
    // A mismatch already says why nothing is heard.
    reported_unheard_ = true;
    std::snprintf(text, sizeof(text),
                  "%scompanion not heard in %.0f s since robot code start",
                  time, static_cast<double>(now_us - first_poll_us_) / 1e6);
    sink_.Report(true, text);
  }

  // Probes alone keep the link up; a companion whose actuator nodes are down
  // sends no commands. Once per stretch without them, a startup grace after
  // the link (and robot code) came up, since nodes start after the bridge.
  if (Within(command_us, now_us, kLinkTimeoutUs)) {
    reported_no_commands_ = false;
  } else if (status.connected && !reported_no_commands_ &&
             now_us - first_poll_us_ >= kStartupGraceUs &&
             now_us - connected_since_us_ >= kStartupGraceUs) {
    reported_no_commands_ = true;
    if (command_us == 0) {
      std::snprintf(text, sizeof(text),
                    "%scompanion up, but no actuator commands since it "
                    "connected: are its nodes running?",
                    time);
    } else {
      std::snprintf(text, sizeof(text),
                    "%scompanion up, but no actuator commands for %.1f s: are "
                    "its nodes running?",
                    time, status.command_age_s);
    }
    sink_.Report(false, text);
  }
}

}  // namespace talos::rio
