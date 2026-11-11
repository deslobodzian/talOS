#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "talOS/bridge/clock_offset.h"
#include "talOS/bridge/merge.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/hardware/config.h"
#include "talOS/hardware/config_wire.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"
#include "talOS/ipc/publisher.h"
#include "talOS/ipc/subscriber.h"
#include "talOS/protocol/udp.h"
#include "talOS/rtms/wake.h"

namespace talos::hardware {

// This node's identity, in one place: the registry row, the `--describe`
// output and the launcher's config all have to agree on both strings, and a
// second copy of either is a second thing to forget.
inline constexpr const char* kNodeName = "hardware_node";
inline constexpr const char* kNodeTarget = "//talOS/bridge:hardware_node";

struct SubsystemTracker {
  std::string name;
  Devices devices;
  uint32_t period_us{5000};
  // Its standing in the merge (merge.h): the owner timeout
  // (OwnerTimeoutUs), last accepted request on the bridge clock and its
  // observation on the gateway clock, reset on every controller transition.
  MergeOwner merge;
  std::unique_ptr<ipc::Subscriber<talos::hardware::Packet>> subscriber;

  // Requests actually decoded off this subsystem's topic, and where that topic
  // sits in the introspection manifest. Kept here because the registry reports
  // per-topic traffic, and a subsystem that has gone quiet is the single most
  // useful thing to see when actuators stop responding.
  uint64_t requests{0};
  std::size_t source_index{0};

  // Positions in the merged command of the actuators this subsystem owns,
  // resolved from `devices` by HardwareNode::IndexActuators so a timeout
  // neutralizes without searching. Ids absent from the merge are left out.
  std::vector<uint16_t> motor_slots;
  std::vector<uint16_t> digital_output_slots;
  std::vector<uint16_t> pwm_output_slots;
};

class HardwareNode {
 public:
  explicit HardwareNode(config::RobotConfig robot_config,
                        std::string remote_ip = "127.0.0.1",
                        uint16_t remote_port = 5802, uint16_t local_port = 5803,
                        bool simulation = false, uint64_t session_id = 0,
                        std::string log_path = {}, uint64_t log_max_bytes = 0);

  ~HardwareNode() = default;

  bool Open();
  // One merge step at `now_us` (the bridge clock): receives every waiting
  // gateway datagram up to kReceiveBudget, reads each request topic, merges,
  // sends the command when it changed or the heartbeat is due, and runs the
  // time-driven duties (clock probe, owner timeouts, /hw/status). Run() calls
  // it on every wakeup; tests call it with synthetic time.
  void Tick(uint64_t now_us);
  // The earliest bridge-clock instant after a Tick(now_us) at which Tick has
  // time-driven work: the next clock probe or its retry, the completion of an
  // echoed probe (now_us itself), /hw/status, each merged owner's timeout,
  // and the command heartbeat while one would be sent. Each is solved from
  // the condition Tick tests. A duty already due at now_us, which that Tick
  // could not do (a failed send), is retried kRetryUs later.
  uint64_t NextDutyUs(uint64_t now_us) const;
  // Run()'s loop body, without its wait: Tick(now_us), the registry refresh
  // Run() owes, then arming every request subscriber's wake. Returns the
  // bridge-clock instant Run() may sleep until: now_us when input is known
  // to be waiting (an arm found an unread request, or Tick stopped at its
  // receive budget), otherwise the next duty, refresh or kMaxWaitUs.
  uint64_t Pass(uint64_t now_us);
  int Run(int duration_s = 0);

  // Run()'s wakeups, by what the loop then found: a gateway datagram, a
  // request, or neither and the wait reached its deadline. A wakeup that
  // found both counts in both. Read only while Run() is not running.
  struct Wakeups {
    uint64_t passes{0};    // every Pass(), immediate ones included
    uint64_t udp{0};       // woken readable; Tick received a datagram
    uint64_t request{0};   // woken readable; Tick read a request
    uint64_t deadline{0};  // the wait reached its deadline
  };
  const Wakeups& wakeups() const { return wakeups_; }

  bool is_configured() const { return is_configured_; }
  uint64_t states_received() const { return states_received_; }
  uint64_t commands_sent() const { return commands_sent_; }
  uint64_t driver_station_packets_received() const {
    return driver_station_packets_received_;
  }
  bool is_subsystem_timed_out(const std::string& name) const {
    auto it = subsystems_.find(name);
    return it != subsystems_.end() && it->second.merge.timed_out;
  }
  const Command& current_command() const { return full_cmd_; }
  uint64_t identity_rejected() const { return identity_rejected_; }
  uint64_t ownership_rejected() const { return ownership_rejected_; }
  // Bounded-heartbeat interval for unchanged merges. Derived from the
  // configured command timeout (timeout/5, clamped to [5, 20] ms) so a live
  // merge always refreshes the gateway well before it expires.
  uint64_t heartbeat_us() const { return heartbeat_us_; }
  // The gateway-to-node clock mapping, fed by every decoded /hw/state frame.
  // It is also the State ordering gate: a frame it ignores is dropped.
  const ClockOffsetEstimator& clock() const { return clock_; }

  // The /hw/status message as of `now_us` (the bridge's monotonic clock, the
  // same one Tick() receives). Tick() publishes it every kStatusPeriodUs.
  // Run() reads that clock from talos::event::Poller::now(), the clock every
  // node's Node::now() runs on, so the clock_* mapping lands in node time.
  bridge::BridgeStatus Status(uint64_t now_us) const;

  void PushConfig(uint64_t now_us);

  // The shape this node would take, answered without opening a socket, a
  // shared-memory segment or any hardware. `--describe` prints this; the
  // launcher assembles the declared graph out of it before it spawns anything.
  // It comes from the same BuildIntrospectionManifest() that Open() registers,
  // so the declared graph and the running one differ only when the node does.
  introspect::Description Describe();

  // Copies the counters above into this node's registry slot. Called from
  // Run(); exposed so a test can drive it without a real event loop.
  void PublishIntrospection();

 private:
  // The sum of the per-topic counters: the registry's dispatch count.
  std::uint64_t DispatchCount() const;
  // Heartbeat and dispatch count alone, one registry sample; Run() calls it
  // every kLivenessPeriodUs between PublishIntrospection() calls.
  void RefreshLiveness();

  // Describes this node to the live registry. The hardware node predates the
  // event loop and drives its own tick, so it has no manifest to hand over --
  // it declares one, in the order Open() creates the transports.
  void BuildIntrospectionManifest();

  // How this node's ends behave, by topic. The manifest cannot say it: to the
  // loop, /hw/command is a sender like any other. Without this the graph
  // reports the RoboRIO link as a broken topic, which is how a report stops
  // being read.
  static const std::vector<introspect::EndpointAttribute>& EndpointAttributes();

  // Whether a subsystem request may merge: same configuration, boot, and
  // epoch as last observed (exact match), and an observation its MergeOwner
  // admits against last_sample_time_us_ (MergeOwner::Admits: bounded age,
  // never future, monotonic per subsystem). The gateway re-enforces the age
  // bound, so it remains the safety backstop.
  bool HasRequestIdentity(const SubsystemTracker& tracker,
                          const Command& partial) const;
  // Whether every actuator a request addresses belongs to the subsystem that
  // published it. One stray id rejects the whole request: a partial merge
  // would let a buggy subsystem drive hardware it does not own.
  bool OwnsRequestDevices(const SubsystemTracker& tracker,
                          const Command& partial) const;
  // Drops every cached merge and forces all subsystems back to timed-out, so
  // nothing is sent until each publishes a fresh request. Called on every
  // controller transition: reboot, config change, disable/re-enable, and fault
  // recovery (all of which the gateway reports as a new boot, config, epoch,
  // or enabled/fault edge).
  void InvalidateRequests();
  void NeutralizeAll();
  // Rebuilds the id -> position tables below and every tracker's slots from
  // full_cmd_'s actuator layout. The first occurrence of an id wins, matching
  // a front-to-back search.
  void IndexActuators();
  // Change-driven publication: send when the merge differs
  // from the last send, otherwise only on the bounded heartbeat. Timeout
  // neutralization changes the merge, so it always sends immediately.
  bool ShouldSendCommand(uint64_t now_us);
  void LogPacket(uint16_t source_id, const Packet& packet, uint64_t now_us,
                 uint64_t sequence);
  // One stderr line per gateway boot naming its program and build (the
  // launcher keeps it in the node's session log; /hw/status carries it too).
  static void ReportGatewayBuild(const State& state);
  // Prints the packet log's error once, whether capture stopped or the log
  // failed (e.g. its file could not be created).
  void ReportLogFailure();

  config::RobotConfig robot_config_;
  Config config_;
  uint64_t config_id_{0};
  std::string remote_ip_;
  uint16_t remote_port_;
  uint16_t local_port_;
  bool simulation_;

  protocol::RuntimeUdpPeer peer_;
  // What every request subscriber's reader slot names while Run() waits.
  // Declared before subsystems_, so their slots are released first and no
  // publisher notifies a closed receiver.
  std::optional<WakeReceiver> request_wake_;
  std::unique_ptr<ipc::Publisher<talos::hardware::Packet>> state_pub_;
  std::unique_ptr<ipc::Publisher<talos::hardware::Packet>> cmd_pub_;
  std::unique_ptr<ipc::Publisher<talos::hardware::Packet>> ds_pub_;
  std::unique_ptr<ipc::Publisher<bridge::BridgeStatus>> status_pub_;

  std::map<std::string, SubsystemTracker> subsystems_;
  Command full_cmd_{};
  // Where each actuator id sits in full_cmd_, so merging a request is one
  // lookup per addressed actuator. Ids not in the table are ignored.
  std::unordered_map<uint16_t, uint16_t> motor_index_;
  std::unordered_map<uint16_t, uint16_t> digital_output_index_;
  std::unordered_map<uint16_t, uint16_t> pwm_output_index_;

  event::Manifest introspection_manifest_;
  std::optional<introspect::NodeRegistration> registration_;
  uint64_t session_id_{0};
  std::string log_path_;
  uint64_t log_max_bytes_{0};
  std::unique_ptr<event::log::LogWriter> log_;
  bool log_failure_reported_{false};
  State latest_state_{};
  // Run()'s registry refreshes, on the bridge clock.
  uint64_t next_introspection_us_{0};
  uint64_t next_liveness_us_{0};
  // What the last Tick did: datagrams it received, requests it read, and
  // whether it stopped at kReceiveBudget rather than an empty socket.
  uint32_t tick_datagrams_{0};
  uint32_t tick_requests_{0};
  bool tick_receive_capped_{false};
  // Consecutive passes that found input waiting but consumed none.
  uint32_t idle_passes_{0};
  // The longest single wait: kMaxWaitUs, or kRetryUs when Run() cannot wait
  // on its inputs and polls them instead.
  uint64_t max_wait_us_{kMaxWaitUs};
  Wakeups wakeups_{};

  bool is_configured_{false};
  uint64_t last_boot_id_{0};
  uint64_t last_ds_sample_time_us_{0};
  uint64_t last_epoch_{0};
  uint64_t last_sample_time_us_{0};
  uint32_t last_state_flags_{0};
  uint64_t states_received_{0};
  uint64_t identity_rejected_{0};
  uint64_t ownership_rejected_{0};
  uint64_t commands_sent_{0};
  uint64_t driver_station_packets_received_{0};
  // Bridge-clock receipt time of the last /hw/state frame; meaningful once
  // states_received_ is non-zero.
  uint64_t last_state_receipt_us_{0};
  ClockOffsetEstimator clock_;
  uint64_t last_probe_send_us_{};
  bool probe_waiting_{false};
  // Set once a frame of another protocol version is logged, until a frame
  // of this version arrives.
  bool version_mismatch_logged_{false};
  // Set once an undecodable State is logged, until a State decodes.
  bool state_decode_failure_logged_{false};
  uint64_t pending_probe_send_us_{};
  uint64_t pending_probe_sample_us_{};
  uint64_t pending_probe_boot_id_{};
  uint64_t statuses_published_{0};
  uint64_t last_status_us_{0};
  uint64_t last_push_time_us_{0};
  uint64_t config_push_backoff_us_{50000};

  // Run() sleeps until a gateway datagram, a request publication or the next
  // time-driven duty. kMaxWaitUs caps one wait: the stop handler only sets
  // process::stop_requested and cannot wake the poller, so the loop looks at
  // the flag at least this often (and --duration-s ends on time).
  static constexpr uint64_t kMaxWaitUs = 10'000;
  // A duty Tick could not do (a failed send) is retried this much later,
  // so a refused send costs at most a 1 kHz retry, never a busy loop.
  static constexpr uint64_t kRetryUs = 1'000;
  // The macOS time-constraint period ConfigureRealtime is given: after each
  // wakeup the bridge's work (computation half of it) is due within it. Not
  // a polling period; the kernel orders time-constraint threads by deadline,
  // so a longer one would put the bridge behind the control nodes.
  static constexpr uint64_t kWakeBudgetUs = 1'000;
  // Datagrams one Tick receives at most, so a burst under overload delays
  // rather than stalls the merge; Run() then ticks again without waiting.
  static constexpr int kReceiveBudget = 32;
  // Passes that may skip the wait on a failed arm without consuming input
  // (a read the writer keeps tearing), before the loop waits anyway: the
  // arm still requested a notification, and kMaxWaitUs bounds the wait.
  static constexpr uint32_t kMaxIdlePasses = 3;
  // Registry: heartbeat and dispatch count, and the full sample.
  static constexpr uint64_t kLivenessPeriodUs = 10'000;
  static constexpr uint64_t kIntrospectionPeriodUs = 250'000;
  static constexpr uint64_t kStatusPeriodUs = 20000;
  // Clock probes: one outstanding, at most every kProbePeriodUs. An
  // unanswered probe (or a lost echo) is replaced after kProbeRetryUs, well
  // inside ClockOffsetEstimator::kExchangeLifetimeUs, so one lost datagram
  // never leaves the estimator without an unexpired exchange. An echo whose
  // round trip exceeds the retry interval is therefore never used.
  static constexpr uint64_t kProbePeriodUs = 20'000;
  static constexpr uint64_t kProbeRetryUs = 100'000;
  static_assert(kProbeRetryUs * 5 <= ClockOffsetEstimator::kExchangeLifetimeUs);
  uint64_t heartbeat_us_{20000};
  Command last_sent_cmd_{};
  uint64_t last_send_us_{0};
  bool have_last_sent_{false};
  bool last_sent_had_active_subsystem_{false};
};

}  // namespace talos::hardware
