#include "node.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <thread>
#include <utility>

#include "talOS/process/process.h"

namespace talos::hardware {

HardwareNode::HardwareNode(config::RobotConfig robot_config,
                           std::string remote_ip, uint16_t remote_port,
                           uint16_t local_port, bool simulation,
                           uint64_t session_id)
    : robot_config_{std::move(robot_config)},
      config_{robot_config_.hardware},
      remote_ip_{std::move(remote_ip)},
      remote_port_{remote_port},
      local_port_{local_port},
      simulation_{simulation},
      session_id_{session_id} {
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
  full_cmd_.pwm_output_count =
      static_cast<uint16_t>(config_.pwm_outputs.size());
  for (std::size_t i = 0; i < config_.pwm_outputs.size(); ++i) {
    full_cmd_.pwm_outputs[i].id = config_.pwm_outputs[i].id;
    full_cmd_.pwm_outputs[i].output = 0.0;
  }

  // Setup subsystem trackers. Input-only subsystems own no actuators, so they
  // publish nothing under kRequestTopicPrefix and have nothing to neutralize.
  for (const auto& [name, devs] : robot_config_.subsystems) {
    if (devs.motors.empty() && devs.digital_outputs.empty() &&
        devs.pwm_outputs.empty()) {
      continue;
    }
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

  state_pub_ =
      std::make_unique<ipc::Publisher<talos::hardware::Packet>>(kStateTopic);
  cmd_pub_ =
      std::make_unique<ipc::Publisher<talos::hardware::Packet>>(kCommandTopic);
  ds_pub_ = std::make_unique<ipc::Publisher<talos::hardware::Packet>>(
      kDriverStationTopic);

  for (auto& [name, tracker] : subsystems_) {
    tracker.subscriber =
        std::make_unique<ipc::Subscriber<talos::hardware::Packet>>(
            RequestTopic(name),
            RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST});
  }

  legacy_cmd_sub_ = std::make_unique<ipc::Subscriber<talos::hardware::Packet>>(
      kCommandOverrideTopic,
      RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST});

  BuildIntrospectionManifest();

  // A registry that cannot be opened must not stop the robot: not being
  // visible in a viewer is a far smaller failure than not driving.
  try {
    registration_.emplace(introspect::NodeRegistration::Identity{
        .name = kNodeName,
        .target = kNodeTarget,
        .session_id = session_id_,
        .flags = simulation_ ? introspect::kFlagSimulation : 0u,
    });
    registration_->publish(introspection_manifest_, EndpointAttributes());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "hardware_node: node registry unavailable: %s\n",
                 e.what());
  }

  return true;
}

const std::vector<introspect::EndpointAttribute>&
HardwareNode::EndpointAttributes() {
  static const std::vector<introspect::EndpointAttribute> kAttributes = {
      // The command's consumer is the RoboRIO, across UDP. The shared-memory
      // copy exists so a viewer or a log can see what was sent; it is not how
      // the command is delivered, and nothing in shared memory ever subscribes
      // to it.
      {kCommandTopic, introspect::naming::kSourceFlagExternal},
      // Nothing in the tree publishes the override. Reporting it as a topic
      // with no writer would be reporting a hook as a fault.
      {kCommandOverrideTopic, introspect::naming::kSourceFlagOptional},
  };
  return kAttributes;
}

introspect::Description HardwareNode::Describe() {
  BuildIntrospectionManifest();
  return introspect::DescribeManifest(
      kNodeName, kNodeTarget, introspection_manifest_, EndpointAttributes());
}

void HardwareNode::BuildIntrospectionManifest() {
  introspection_manifest_.clear();

  const auto add = [this](event::SourceKind kind, std::string name) {
    const auto id = static_cast<std::uint16_t>(introspection_manifest_.size());
    introspection_manifest_.push_back({
        .id = id,
        .kind = kind,
        .name = std::move(name),
        .message_bytes = static_cast<std::uint32_t>(sizeof(Packet)),
        .alignment = static_cast<std::uint32_t>(alignof(Packet)),
    });
    return static_cast<std::size_t>(id);
  };

  // Same order as Open() creates them, so the ids stay stable across runs of
  // the same configuration.
  add(event::SourceKind::SENDER, kStateTopic);
  add(event::SourceKind::SENDER, kCommandTopic);
  add(event::SourceKind::SENDER, kDriverStationTopic);
  add(event::SourceKind::FETCHER, kCommandOverrideTopic);

  for (auto& [name, tracker] : subsystems_) {
    tracker.source_index = add(event::SourceKind::FETCHER, RequestTopic(name));
  }
}

void HardwareNode::PublishIntrospection() {
  if (!registration_) {
    return;
  }

  // One sample; see NodeRegistration::begin_sample.
  registration_->begin_sample();
  registration_->heartbeat();

  // Every message this node moved, which is the sum of its per-topic counters.
  // Subsystem requests belong in it: dropping them was what let the aliased
  // /hw/req/drive traffic go uncounted for as long as it did.
  std::uint64_t dispatches = states_received_ + commands_sent_ +
                             driver_station_packets_received_ +
                             legacy_commands_;
  for (const auto& [name, tracker] : subsystems_) {
    dispatches += tracker.requests;
  }
  registration_->set_dispatch_count(dispatches);

  const auto set = [this](std::size_t index, std::uint64_t events) {
    registration_->set_source(static_cast<std::uint32_t>(index), events,
                              /*dropped=*/0, /*sequence=*/events,
                              /*last_monotonic_ns=*/0, /*last_latency_ns=*/0,
                              /*max_latency_ns=*/0);
  };

  set(0, states_received_);
  set(1, commands_sent_);
  set(2, driver_station_packets_received_);
  set(3, legacy_commands_);
  for (const auto& [name, tracker] : subsystems_) {
    set(tracker.source_index, tracker.requests);
  }
  registration_->end_sample();
}

void HardwareNode::PushConfig(uint64_t now_us) {
  auto chunks = CreateConfigChunks(config_);
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    peer_.SendFrame(protocol::FrameType::kHardwareConfig,
                    static_cast<uint16_t>(i + 1), now_us, chunks[i].data(),
                    chunks[i].size());
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
        if (now_us - last_push_time_us_ >= 50000 ||
            state.boot_id != last_boot_id_) {
          PushConfig(now_us);
        }
      }

      last_boot_id_ = state.boot_id;
      last_epoch_ = state.epoch;
      last_sample_time_us_ = state.sample_time_us;
      last_state_flags_ = state.flags;

      talos::hardware::Packet pkt{};
      pkt.size = static_cast<uint32_t>(Encode(state, pkt.data));
      if (state_pub_) state_pub_->write(pkt);
      continue;
    }

    if (frame.header.type == protocol::FrameType::kDriverStation) {
      talos::hardware::Packet pkt{};
      const std::size_t n =
          std::min<std::size_t>(frame.payload_size, pkt.data.size());
      pkt.size = static_cast<uint32_t>(n);
      std::memcpy(pkt.data.data(), frame.payload, n);
      if (ds_pub_) ds_pub_->write(pkt);
      ++driver_station_packets_received_;
      continue;
    }
  }

  // 2. Read subsystem requests and perform timeout neutralization
  bool any_subsystem_active = false;
  for (auto& [name, tracker] : subsystems_) {
    std::optional<talos::hardware::Packet> pkt;
    if (tracker.subscriber) pkt = tracker.subscriber->read();

    if (pkt) {
      ++tracker.requests;
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
    if (tracker.last_seen_us == 0 ||
        (now_us - tracker.last_seen_us > timeout_us)) {
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

  // Also check the whole-command override, which replaces the merge outright
  if (legacy_cmd_sub_) {
    if (auto pkt = legacy_cmd_sub_->read()) {
      ++legacy_commands_;
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

    talos::hardware::Packet cmd_pkt{};
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
  talos::process::InstallStopHandlers();
  const auto end =
      std::chrono::steady_clock::now() + std::chrono::seconds{duration_s};

  while (!talos::process::stop_requested.load() &&
         (!duration_s || std::chrono::steady_clock::now() < end)) {
    const auto now_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
    Tick(now_us);

    // Four times a second: often enough that the registry never looks stale,
    // rare enough that it costs nothing against a 1 kHz tick.
    const auto steady_now = std::chrono::steady_clock::now();
    if (steady_now >= next_introspection_) {
      PublishIntrospection();
      next_introspection_ = steady_now + std::chrono::milliseconds{250};
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  PublishIntrospection();

  std::printf("hardware_node: states=%llu commands=%llu\n",
              static_cast<unsigned long long>(states_received_),
              static_cast<unsigned long long>(commands_sent_));
  return states_received_ > 0 ? 0 : 1;
}

}  // namespace talos::hardware
