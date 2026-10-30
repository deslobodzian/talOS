#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/hardware/config.h"
#include "talOS/hardware/config_wire.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"
#include "talOS/ipc/publisher.h"
#include "talOS/ipc/subscriber.h"
#include "talOS/protocol/udp.h"

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
  uint64_t last_seen_us{0};
  bool timed_out{true};
  // Last accepted Command::observed_time_us for this subsystem (gateway
  // clock). Monotonic per subsystem: a request older than this is a replay
  // and is rejected even when it is still within the age bound. Reset to 0
  // on every controller transition alongside last_seen_us.
  uint64_t last_observed_time_us{0};
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
                        bool simulation = false, uint64_t session_id = 0);

  ~HardwareNode() = default;

  bool Open();
  void Tick(uint64_t now_us);
  int Run(int duration_s = 0);

  bool is_configured() const { return is_configured_; }
  uint64_t states_received() const { return states_received_; }
  uint64_t commands_sent() const { return commands_sent_; }
  uint64_t driver_station_packets_received() const {
    return driver_station_packets_received_;
  }
  bool is_subsystem_timed_out(const std::string& name) const {
    auto it = subsystems_.find(name);
    return it != subsystems_.end() && it->second.timed_out;
  }
  const Command& current_command() const { return full_cmd_; }
  uint64_t identity_rejected() const { return identity_rejected_; }
  uint64_t ownership_rejected() const { return ownership_rejected_; }
  // Bounded-heartbeat interval for unchanged merges. Derived from the
  // configured command timeout (timeout/5, clamped to [5, 20] ms) so a live
  // merge always refreshes the gateway well before it expires.
  uint64_t heartbeat_us() const { return heartbeat_us_; }

  // The /hw/status message as of `now_us` (the bridge's monotonic clock, the
  // same one Tick() receives). Tick() publishes it every kStatusPeriodUs.
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
  // epoch as last observed (exact match), plus a bounded-age freshness check
  // on the gateway clock: last_sample_time_us_ - observed_time_us must be
  // below command_timeout_us (future observations rejected). Monotonic per
  // subsystem: observed_time_us below the tracker's last accepted value is a
  // replay and is rejected. The gateway re-enforces the age bound, so it
  // remains the safety backstop.
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

  config::RobotConfig robot_config_;
  Config config_;
  uint64_t config_id_{0};
  std::string remote_ip_;
  uint16_t remote_port_;
  uint16_t local_port_;
  bool simulation_;

  protocol::RuntimeUdpPeer peer_;
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
  std::chrono::steady_clock::time_point next_introspection_{};

  bool is_configured_{false};
  uint64_t last_boot_id_{0};
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
  uint64_t statuses_published_{0};
  uint64_t last_status_us_{0};
  uint64_t cmd_sequence_{0};
  uint64_t last_push_time_us_{0};

  // Explicit periodic scheduling: 1 kHz tick period as a
  // named deadline, not an implicit sleep constant.
  static constexpr uint64_t kTickPeriodUs = 1000;
  static constexpr uint64_t kStatusPeriodUs = 20000;
  uint64_t heartbeat_us_{20000};
  Command last_sent_cmd_{};
  uint64_t last_send_us_{0};
  bool have_last_sent_{false};
};

}  // namespace talos::hardware
