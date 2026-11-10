#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string_view>
#include <thread>

// The drive team's view of the companion (main processor) link.
//
// The Driver Station shows green robot code and communications whenever this
// program runs, whether or not the companion is talking to it. So the loop
// thread hands what its Endpoint last heard (hardware::PeerContact) to this
// reporter, which a low-priority thread turns into dashboard values and one
// Driver Station message per change: link up, link lost (or never heard after
// a startup grace), a companion speaking another protocol version, that
// mismatch clearing, and a link that stays up a startup grace with no command
// frames (its actuator nodes down or crash-looping).
//
// Observe() is the loop thread's only call: relaxed atomic stores, never a
// lock, an allocation or a write. NetworkTables and the Driver Station error
// channel are touched only from the reporter's own thread (Start), or from
// whoever calls Poll() in a test.
namespace talos::rio {

// What the loop thread knows each tick: the Endpoint's PeerContact times
// (steady-clock microseconds, 0 for never) and the gateway's configuration.
struct CompanionContact {
  uint64_t frame_us{0};
  uint64_t command_us{0};
  uint64_t version_mismatch_us{0};
  uint16_t mismatched_version{0};
  bool configured{false};
  bool commissioned{false};
  uint64_t config_id{0};
};

struct CompanionStatus {
  // A valid frame (the bridge's 20 ms clock probe at least) within
  // CompanionReporter::kLinkTimeoutUs.
  bool connected{false};
  // No frame of another protocol version within kLinkTimeoutUs.
  bool protocol_ok{true};
  // The version of the last such frame; 0 before one.
  uint16_t peer_protocol{0};
  // Seconds since the last valid frame and the last command frame; negative
  // when none has arrived. Once a node has hardware state its devices flush a
  // request every tick, neutral while disabled, and the bridge forwards the
  // merge at least every 20 ms, so a healthy link keeps both near 0.02 s
  // enabled or not.
  double frame_age_s{-1.0};
  double command_age_s{-1.0};
  bool configured{false};
  bool commissioned{false};
  uint64_t config_id{0};
};

// Where reports go: the dashboard on the robot, a recorder in tests.
class CompanionSink {
 public:
  virtual ~CompanionSink() = default;
  // Every poll, with the current status.
  virtual void Publish(const CompanionStatus& status) = 0;
  // Once per change; `error` marks a lost link or a protocol mismatch.
  virtual void Report(bool error, std::string_view text) = 0;
};

// The production sink: SmartDashboard values under "companion/" and the
// Driver Station's error/warning log (HAL_SendError).
std::unique_ptr<CompanionSink> MakeDashboardSink();

class CompanionReporter {
 public:
  // The bridge probes every 20 ms; config load caps command_timeout_us at
  // 500 ms, so a link silent this long has stopped every output too.
  static constexpr uint64_t kLinkTimeoutUs = 500'000;
  // From the first poll, how long the companion may stay unheard (it boots
  // and launches its nodes alongside the RoboRIO) before that is reported;
  // also how long a connected companion may send no commands.
  static constexpr uint64_t kStartupGraceUs = 30'000'000;
  static constexpr uint64_t kPollPeriodUs = 100'000;

  explicit CompanionReporter(CompanionSink& sink) : sink_{sink} {}
  ~CompanionReporter();
  CompanionReporter(const CompanionReporter&) = delete;
  CompanionReporter& operator=(const CompanionReporter&) = delete;

  // Loop thread, every tick. Never blocks or allocates.
  void Observe(const CompanionContact& contact);

  // Starts the reporter thread (nice 19 on Linux), which polls every
  // kPollPeriodUs; Stop() (also the destructor) joins it.
  void Start();
  void Stop();

  // Publishes the status as of `now_us` (steady clock) and reports each
  // change since the previous poll. The reporter thread calls it; tests call
  // it directly on a reporter that was not started.
  void Poll(uint64_t now_us);

 private:
  void Run();

  CompanionSink& sink_;
  std::atomic<uint64_t> frame_us_{0};
  std::atomic<uint64_t> command_us_{0};
  std::atomic<uint64_t> version_mismatch_us_{0};
  std::atomic<uint16_t> mismatched_version_{0};
  std::atomic<bool> configured_{false};
  std::atomic<bool> commissioned_{false};
  std::atomic<uint64_t> config_id_{0};

  // Poll-side state.
  uint64_t first_poll_us_{0};
  bool polled_{false};
  bool reported_connected_{false};
  bool reported_unheard_{false};
  bool reported_protocol_ok_{true};
  uint64_t connected_since_us_{0};
  bool reported_no_commands_{false};

  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace talos::rio
