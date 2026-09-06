#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

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
  std::unique_ptr<ipc::Subscriber<talos::hardware::Packet>> subscriber;

  // Requests actually decoded off this subsystem's topic, and where that topic
  // sits in the introspection manifest. Kept here because the registry reports
  // per-topic traffic, and a subsystem that has gone quiet is the single most
  // useful thing to see when actuators stop responding.
  uint64_t requests{0};
  std::size_t source_index{0};
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
  // loop, /hw/command is a sender like any other and /hw/command/override is a
  // fetcher like any other. Without these the graph reports the RoboRIO link
  // and an unused debug hook as two broken topics, which is how a report stops
  // being read.
  static const std::vector<introspect::EndpointAttribute>& EndpointAttributes();

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
  std::unique_ptr<ipc::Subscriber<talos::hardware::Packet>> legacy_cmd_sub_;

  std::map<std::string, SubsystemTracker> subsystems_;
  Command full_cmd_{};

  event::Manifest introspection_manifest_;
  std::optional<introspect::NodeRegistration> registration_;
  uint64_t session_id_{0};
  uint64_t legacy_commands_{0};
  std::chrono::steady_clock::time_point next_introspection_{};

  bool is_configured_{false};
  uint64_t last_boot_id_{0};
  uint64_t last_epoch_{0};
  uint64_t last_sample_time_us_{0};
  uint32_t last_state_flags_{0};
  uint64_t states_received_{0};
  uint64_t commands_sent_{0};
  uint64_t driver_station_packets_received_{0};
  uint64_t cmd_sequence_{0};
  uint64_t last_push_time_us_{0};
};

}  // namespace talos::hardware
