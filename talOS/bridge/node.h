#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "talOS/hardware/config.h"
#include "talOS/hardware/config_wire.h"
#include "talOS/hardware/messages.h"
#include "talOS/protocol/udp.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/hardware/packet.h"
#include "talOS/ipc/publisher.h"
#include "talOS/ipc/subscriber.h"

namespace talos::hardware {

struct SubsystemTracker {
  std::string name;
  Devices devices;
  uint32_t period_us{5000};
  uint64_t last_seen_us{0};
  bool timed_out{true};
  std::unique_ptr<ipc::Subscriber<talos::hardware::Packet>> subscriber;
  std::unique_ptr<ipc::Subscriber<talos::hardware::Packet>> alt_subscriber;
};

class HardwareNode {
 public:
  explicit HardwareNode(config::RobotConfig robot_config,
                        std::string remote_ip = "127.0.0.1",
                        uint16_t remote_port = 5802,
                        uint16_t local_port = 5803,
                        bool simulation = false);

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

 private:
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
