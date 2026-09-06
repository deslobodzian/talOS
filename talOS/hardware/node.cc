#include "node.h"

#include <cstdio>
#include <thread>
#include <utility>

#include "talOS/drivetrain/process.h"

namespace talos::hardware {

HardwareNode::HardwareNode(config::RobotConfig robot_config,
                           std::string remote_ip, uint16_t remote_port,
                           uint16_t local_port, bool simulation)
    : robot_config_{std::move(robot_config)},
      config_{robot_config_.hardware},
      remote_ip_{std::move(remote_ip)},
      remote_port_{remote_port},
      local_port_{local_port},
      simulation_{simulation} {
  if (simulation_ || remote_ip_ == "127.0.0.1") {
    config_.commissioned = true;
  }
  config_id_ = ConfigurationId(config_);

  // Initialize full_cmd_ layout based on config_
  full_cmd_.config_id = config_id_;
  full_cmd_.count = static_cast<uint16_t>(config_.motors.size());
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    full_cmd_.motors[i].id = config_.motors[i].id;
    full_cmd_.motors[i].mode = Mode::kNeutral;
    full_cmd_.motors[i].slot = 0;
    full_cmd_.motors[i].demand = 0.0;
    full_cmd_.motors[i].feedforward_v = 0.0;
  }
  full_cmd_.digital_output_count =
      static_cast<uint16_t>(config_.digital_outputs.size());
  for (std::size_t i = 0; i < config_.digital_outputs.size(); ++i) {
    full_cmd_.digital_outputs[i].id = config_.digital_outputs[i].id;
    full_cmd_.digital_outputs[i].value = false;
  }
  full_cmd_.pwm_output_count = static_cast<uint16_t>(config_.pwm_outputs.size());
  for (std::size_t i = 0; i < config_.pwm_outputs.size(); ++i) {
    full_cmd_.pwm_outputs[i].id = config_.pwm_outputs[i].id;
    full_cmd_.pwm_outputs[i].output = 0.0;
  }

  // Setup subsystem trackers
  for (const auto& [name, devs] : robot_config_.subsystems) {
    SubsystemTracker tracker;
    tracker.name = name;
    tracker.devices = devs;
    if (const auto* tbl = robot_config_.GetSubsystemTable(name)) {
      tracker.period_us =
          static_cast<uint32_t>((*tbl)["period_us"].value_or(5000));
    }
    subsystems_[name] = std::move(tracker);
  }
}

bool HardwareNode::Open() {
  if (peer_.Open("0.0.0.0", local_port_, remote_ip_.c_str(), remote_port_) !=
      protocol::UdpStatus::kOk) {
    return false;
  }

  state_pub_ = std::make_unique<ipc::Publisher<talos::drive::Packet>>("/hw/state");
  cmd_pub_ = std::make_unique<ipc::Publisher<talos::drive::Packet>>("/hw/cmd");

  for (auto& [name, tracker] : subsystems_) {
    std::string topic = "/hw/req/" + name;
    tracker.subscriber = std::make_unique<ipc::Subscriber<talos::drive::Packet>>(
        topic, RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST});
    if (name == "drivetrain") {
      tracker.alt_subscriber =
          std::make_unique<ipc::Subscriber<talos::drive::Packet>>(
              "/hw/req/drive",
              RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST});
    }
  }

  legacy_cmd_sub_ = std::make_unique<ipc::Subscriber<talos::drive::Packet>>(
      "/hw/cmd_in",
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST});

  return true;
}

void HardwareNode::PushConfig(uint64_t now_us) {
  auto chunks = CreateConfigChunks(config_);
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    peer_.SendFrame(protocol::FrameType::kHardwareConfig, static_cast<uint16_t>(i + 1),
                    now_us, chunks[i].data(), chunks[i].size());
  }
  last_push_time_us_ = now_us;
}

void HardwareNode::Tick(uint64_t now_us) {
  // 1. Process incoming UDP frames from Gateway
  for (int i = 0; i < 32; ++i) {
    protocol::DecodedFrame frame;
    const auto status = peer_.TryReceive(&frame);
    if (status == protocol::UdpStatus::kWouldBlock) break;
    if (status != protocol::UdpStatus::kOk) continue;

    if (frame.header.type == protocol::FrameType::kHardwareConfigAck) {
      ConfigAckPayload ack{};
      if (DecodeConfigAck({frame.payload, frame.payload_size}, ack)) {
        if (ack.status == ConfigAckStatus::kOk && ack.config_id == config_id_) {
          is_configured_ = true;
        }
      }
      continue;
    }

    if (frame.header.type == protocol::FrameType::kHardwareState) {
      State state;
      if (!Decode({frame.payload, frame.payload_size}, state)) continue;

      ++states_received_;
      if (state.boot_id != last_boot_id_ || state.config_id != config_id_ ||
          !is_configured_) {
        if (now_us - last_push_time_us_ >= 50000 || state.boot_id != last_boot_id_) {
          PushConfig(now_us);
        }
      }

      last_boot_id_ = state.boot_id;
      last_epoch_ = state.epoch;
      last_sample_time_us_ = state.sample_time_us;
      last_state_flags_ = state.flags;

      talos::drive::Packet pkt{};
      pkt.size = static_cast<uint32_t>(Encode(state, pkt.data));
      if (state_pub_) state_pub_->write(pkt);
    }
  }

  // 2. Read subsystem requests and perform timeout neutralization
  bool any_subsystem_active = false;
  for (auto& [name, tracker] : subsystems_) {
    std::optional<talos::drive::Packet> pkt;
    if (tracker.subscriber) pkt = tracker.subscriber->read();
    if (!pkt && tracker.alt_subscriber) pkt = tracker.alt_subscriber->read();

    if (pkt) {
      Command partial{};
      if (Decode(pkt->bytes(), partial)) {
        // Apply motors belonging to this subsystem
        for (std::size_t i = 0; i < partial.count; ++i) {
          const auto& pm = partial.motors[i];
          for (std::size_t j = 0; j < full_cmd_.count; ++j) {
            if (full_cmd_.motors[j].id == pm.id) {
              full_cmd_.motors[j] = pm;
              break;
            }
          }
        }
        // Apply digital outputs
        for (std::size_t i = 0; i < partial.digital_output_count; ++i) {
          const auto& pd = partial.digital_outputs[i];
          for (std::size_t j = 0; j < full_cmd_.digital_output_count; ++j) {
            if (full_cmd_.digital_outputs[j].id == pd.id) {
              full_cmd_.digital_outputs[j] = pd;
              break;
            }
          }
        }
        // Apply PWM outputs
        for (std::size_t i = 0; i < partial.pwm_output_count; ++i) {
          const auto& pp = partial.pwm_outputs[i];
          for (std::size_t j = 0; j < full_cmd_.pwm_output_count; ++j) {
            if (full_cmd_.pwm_outputs[j].id == pp.id) {
              full_cmd_.pwm_outputs[j] = pp;
              break;
            }
          }
        }
        tracker.last_seen_us = now_us;
        tracker.timed_out = false;
      }
    }

    // Check timeout: > 2x period
    const uint64_t timeout_us = 2 * tracker.period_us;
    if (tracker.last_seen_us == 0 || (now_us - tracker.last_seen_us > timeout_us)) {
      tracker.timed_out = true;
      // Neutralize actuators owned by this subsystem
      for (auto id : tracker.devices.motors) {
        for (std::size_t j = 0; j < full_cmd_.count; ++j) {
          if (full_cmd_.motors[j].id == id) {
            full_cmd_.motors[j].mode = Mode::kNeutral;
            full_cmd_.motors[j].demand = 0.0;
            full_cmd_.motors[j].feedforward_v = 0.0;
            break;
          }
        }
      }
      for (auto id : tracker.devices.digital_outputs) {
        for (std::size_t j = 0; j < full_cmd_.digital_output_count; ++j) {
          if (full_cmd_.digital_outputs[j].id == id) {
            full_cmd_.digital_outputs[j].value = false;
            break;
          }
        }
      }
      for (auto id : tracker.devices.pwm_outputs) {
        for (std::size_t j = 0; j < full_cmd_.pwm_output_count; ++j) {
          if (full_cmd_.pwm_outputs[j].id == id) {
            full_cmd_.pwm_outputs[j].output = 0.0;
            break;
          }
        }
      }
    } else {
      any_subsystem_active = true;
    }
  }

  // Also check legacy direct /hw/cmd subscriber
  if (legacy_cmd_sub_) {
    if (auto pkt = legacy_cmd_sub_->read()) {
      Command direct{};
      if (Decode(pkt->bytes(), direct)) {
        full_cmd_ = direct;
        any_subsystem_active = true;
      }
    }
  }

  // 3. Send unified command if we have observed state
  if (last_boot_id_ != 0 && any_subsystem_active) {
    full_cmd_.config_id = config_id_;
    full_cmd_.boot_id = last_boot_id_;
    full_cmd_.epoch = last_epoch_;
    full_cmd_.observed_time_us = last_sample_time_us_;

    talos::drive::Packet cmd_pkt{};
    cmd_pkt.size = static_cast<uint32_t>(Encode(full_cmd_, cmd_pkt.data));
    if (cmd_pkt.size > 0) {
      if (cmd_pub_) cmd_pub_->write(cmd_pkt);
      peer_.SendFrame(protocol::FrameType::kHardwareCommand,
                      static_cast<uint16_t>(++cmd_sequence_), now_us,
                      cmd_pkt.data.data(), cmd_pkt.size);
      ++commands_sent_;
    }
  }
}

int HardwareNode::Run(int duration_s) {
  talos::drive::InstallStopHandlers();
  const auto end =
      std::chrono::steady_clock::now() + std::chrono::seconds{duration_s};

  while (!talos::drive::stop_requested.load() &&
         (!duration_s || std::chrono::steady_clock::now() < end)) {
    const auto now_us =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
    Tick(now_us);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::printf("hardware_node: states=%llu commands=%llu\n",
              static_cast<unsigned long long>(states_received_),
              static_cast<unsigned long long>(commands_sent_));
  return states_received_ > 0 ? 0 : 1;
}

}  // namespace talos::hardware
