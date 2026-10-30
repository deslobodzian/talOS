#include "node.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <utility>

#include "bridge/topics.h"
#include "talOS/process/process.h"

namespace talos::hardware {

// /hw/status is read by talos::Node subscribers, which open every slot at
// 8-byte alignment (talos::kMessageAlignment). ipc::Publisher lays the slot out
// at alignof(Message), so the two agree only while this holds.
static_assert(alignof(bridge::BridgeStatus) == 8);

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
  // Heartbeat keeps a live-but-unchanged merge refreshing the gateway at 5x
  // margin inside the command timeout, whatever the config chose. The clamp
  // keeps steady-state traffic near the slowest actuator period (20 ms)
  // without ever approaching the timeout floor (2x period).
  heartbeat_us_ = std::clamp(config_.command_timeout_us / 5, uint32_t{5000},
                             uint32_t{20000});

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
  IndexActuators();
}

void HardwareNode::IndexActuators() {
  const auto index = [](auto& table, const auto& entries, uint16_t count) {
    table.clear();
    for (uint16_t j = 0; j < count; ++j) table.try_emplace(entries[j].id, j);
  };
  index(motor_index_, full_cmd_.motors, full_cmd_.count);
  index(digital_output_index_, full_cmd_.digital_outputs,
        full_cmd_.digital_output_count);
  index(pwm_output_index_, full_cmd_.pwm_outputs, full_cmd_.pwm_output_count);

  const auto slots = [](std::vector<uint16_t>& out, const auto& table,
                        const std::vector<uint16_t>& ids) {
    out.clear();
    for (auto id : ids) {
      if (auto it = table.find(id); it != table.end())
        out.push_back(it->second);
    }
  };
  for (auto& [name, tracker] : subsystems_) {
    slots(tracker.motor_slots, motor_index_, tracker.devices.motors);
    slots(tracker.digital_output_slots, digital_output_index_,
          tracker.devices.digital_outputs);
    slots(tracker.pwm_output_slots, pwm_output_index_,
          tracker.devices.pwm_outputs);
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
  status_pub_ = std::make_unique<ipc::Publisher<bridge::BridgeStatus>>(
      bridge::topics::kStatus);

  for (auto& [name, tracker] : subsystems_) {
    tracker.subscriber =
        std::make_unique<ipc::Subscriber<talos::hardware::Packet>>(
            RequestTopic(name),
            RTMSOptions{OverflowPolicy::OVERWRITE_OLDEST, ReadMode::LATEST});
  }

  BuildIntrospectionManifest();

  // A registry that cannot be opened must not stop the robot: not being
  // visible in a viewer is a far smaller failure than not driving.
  try {
    auto& registration =
        registration_.emplace(introspect::NodeRegistration::Identity{
            .name = kNodeName,
            .target = kNodeTarget,
            .session_id = session_id_,
            .flags = simulation_ ? introspect::kFlagSimulation : 0u,
        });
    registration.publish(introspection_manifest_, EndpointAttributes());
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

  const auto add = [this](event::SourceKind kind, std::string name,
                          std::size_t bytes, std::size_t alignment) {
    const auto id = static_cast<std::uint16_t>(introspection_manifest_.size());
    introspection_manifest_.push_back({
        .id = id,
        .kind = kind,
        .name = std::move(name),
        .message_bytes = static_cast<std::uint32_t>(bytes),
        .alignment = static_cast<std::uint32_t>(alignment),
    });
    return static_cast<std::size_t>(id);
  };
  const auto add_packet = [&add](event::SourceKind kind, std::string name) {
    return add(kind, std::move(name), sizeof(Packet), alignof(Packet));
  };

  // The fixed ends first, then one request per subsystem, so the fixed ids
  // (PublishIntrospection's 0..3) do not depend on the configuration.
  add_packet(event::SourceKind::SENDER, kStateTopic);
  add_packet(event::SourceKind::SENDER, kCommandTopic);
  add_packet(event::SourceKind::SENDER, kDriverStationTopic);
  add(event::SourceKind::SENDER, bridge::topics::kStatus,
      sizeof(bridge::BridgeStatus), alignof(bridge::BridgeStatus));

  for (auto& [name, tracker] : subsystems_) {
    tracker.source_index =
        add_packet(event::SourceKind::FETCHER, RequestTopic(name));
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
                             statuses_published_;
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
  set(3, statuses_published_);
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
  // 1. Process incoming UDP frames from Gateway. Bounded receive budget:
  // at most 32 frames per tick, so a burst under overload delays rather than
  // stalls the merge below.
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
      last_state_receipt_us_ = now_us;
      if (state.boot_id != last_boot_id_ || state.config_id != config_id_ ||
          !is_configured_) {
        if (now_us - last_push_time_us_ >= 50000 ||
            state.boot_id != last_boot_id_) {
          PushConfig(now_us);
        }
      }

      // Controller transition: reboot, reconfiguration, epoch bump, or an
      // enabled/fault edge (the gateway invalidates its epoch on disable,
      // re-enable, and fault recovery, so the edge check is belt and braces).
      // Every cached merge dies here: motion resumes only on fresh subsystem
      // requests stamped with the new identity.
      const bool transition =
          (state.boot_id != last_boot_id_) || (state.config_id != config_id_) ||
          (state.epoch != last_epoch_) ||
          ((state.flags ^ last_state_flags_) & (kEnabled | kHardwareFault));
      if (transition) {
        InvalidateRequests();
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
        // Trust gates, in this order: an obsolete request must never merge,
        // and a current one must still own every device it addresses. Either
        // rejection drops the whole request without proving liveness, so a
        // stream of bad requests still times the subsystem out.
        if (!HasRequestIdentity(tracker, partial)) {
          ++identity_rejected_;
        } else if (!OwnsRequestDevices(tracker, partial)) {
          ++ownership_rejected_;
        } else {
          // Apply the addressed actuators; ids absent from the merge are
          // ignored.
          for (std::size_t i = 0; i < partial.count; ++i) {
            const auto& pm = partial.motors[i];
            if (auto it = motor_index_.find(pm.id); it != motor_index_.end()) {
              full_cmd_.motors[it->second] = pm;
            }
          }
          for (std::size_t i = 0; i < partial.digital_output_count; ++i) {
            const auto& pd = partial.digital_outputs[i];
            if (auto it = digital_output_index_.find(pd.id);
                it != digital_output_index_.end()) {
              full_cmd_.digital_outputs[it->second] = pd;
            }
          }
          for (std::size_t i = 0; i < partial.pwm_output_count; ++i) {
            const auto& pp = partial.pwm_outputs[i];
            if (auto it = pwm_output_index_.find(pp.id);
                it != pwm_output_index_.end()) {
              full_cmd_.pwm_outputs[it->second] = pp;
            }
          }
          // Adopt the request's identity words. The outgoing command carries
          // the observation its content was built from, never a fresher one.
          // The merged observed header is the minimum over active subsystems
          // (recomputed below), so one subsystem cannot refresh another's
          // stale demand.
          full_cmd_.config_id = partial.config_id;
          full_cmd_.boot_id = partial.boot_id;
          full_cmd_.epoch = partial.epoch;
          tracker.last_seen_us = now_us;
          tracker.last_observed_time_us = partial.observed_time_us;
          tracker.timed_out = false;
        }
      }
    }

    // Check timeout: > 2x period
    const uint64_t timeout_us = 2 * static_cast<uint64_t>(tracker.period_us);
    if (tracker.last_seen_us == 0 ||
        (now_us - tracker.last_seen_us > timeout_us)) {
      tracker.timed_out = true;
      // Neutralize actuators owned by this subsystem
      for (auto j : tracker.motor_slots) {
        full_cmd_.motors[j].mode = Mode::kNeutral;
        full_cmd_.motors[j].demand = 0.0;
        full_cmd_.motors[j].feedforward_v = 0.0;
      }
      for (auto j : tracker.digital_output_slots) {
        full_cmd_.digital_outputs[j].value = false;
      }
      for (auto j : tracker.pwm_output_slots) {
        full_cmd_.pwm_outputs[j].output = 0.0;
      }
    } else {
      any_subsystem_active = true;
    }
  }

  // The merged observation is the oldest demand still driving the command:
  // the minimum accepted observed_time_us over live subsystems. A fresh
  // subsystem must not make a quiet one's cached demand look current, and a
  // timed-out subsystem stops pinning the merge once neutralized.
  {
    bool have_active_observed = false;
    uint64_t min_observed_us = 0;
    for (const auto& [name, tracker] : subsystems_) {
      if (!tracker.timed_out) {
        if (!have_active_observed ||
            tracker.last_observed_time_us < min_observed_us) {
          min_observed_us = tracker.last_observed_time_us;
          have_active_observed = true;
        }
      }
    }
    if (have_active_observed) {
      full_cmd_.observed_time_us = min_observed_us;
    }
  }

  // 3. Send unified command if we have observed state. The headers travel
  // untouched from the subsystem requests that built the merge: stamping the
  // current state here would make an obsolete request look current, and the
  // gateway would accept it as fresh. Publication is change-driven with a
  // bounded heartbeat: each new sample changes the merge's
  // observation headers, so a live 200 Hz link sends at the 200 Hz sample
  // rate instead of the 1 kHz tick rate, and an unchanged merge resends only
  // when the heartbeat elapses. The gateway's 100 ms timeout keeps 5x margin
  // over the heartbeat. Subsystem timeouts still neutralize through the
  // merge, and a changed merge always sends immediately.
  if (last_boot_id_ != 0 && any_subsystem_active) {
    if (ShouldSendCommand(now_us)) {
      talos::hardware::Packet cmd_pkt{};
      cmd_pkt.size = static_cast<uint32_t>(Encode(full_cmd_, cmd_pkt.data));
      if (cmd_pkt.size > 0) {
        if (cmd_pub_) cmd_pub_->write(cmd_pkt);
        peer_.SendFrame(protocol::FrameType::kHardwareCommand,
                        static_cast<uint16_t>(++cmd_sequence_), now_us,
                        cmd_pkt.data.data(), cmd_pkt.size);
        ++commands_sent_;
        last_sent_cmd_ = full_cmd_;
        last_send_us_ = now_us;
        have_last_sent_ = true;
      }
    }
  }

  // 4. Bridge health, on the bridge clock.
  if (status_pub_ && (statuses_published_ == 0 ||
                      now_us - last_status_us_ >= kStatusPeriodUs)) {
    status_pub_->write(Status(now_us));
    ++statuses_published_;
    last_status_us_ = now_us;
  }
}

bridge::BridgeStatus HardwareNode::Status(uint64_t now_us) const {
  uint32_t timed_out_mask = 0;
  uint32_t bit = 0;
  // subsystems_ is a std::map, so this walks the names in sorted order.
  for (const auto& [name, tracker] : subsystems_) {
    if (bit < 32 && tracker.timed_out) timed_out_mask |= uint32_t{1} << bit;
    ++bit;
  }
  const uint64_t state_age_us =
      states_received_ > 0 && now_us > last_state_receipt_us_
          ? now_us - last_state_receipt_us_
          : 0;
  return bridge::BridgeStatus{
      states_received_,
      commands_sent_,
      identity_rejected_,
      ownership_rejected_,
      driver_station_packets_received_,
      state_age_us,
      timed_out_mask,
      static_cast<uint32_t>(subsystems_.size()),
      is_configured_,
  };
}

bool HardwareNode::ShouldSendCommand(uint64_t now_us) {
  if (!have_last_sent_ || full_cmd_ != last_sent_cmd_) return true;
  return now_us - last_send_us_ >= heartbeat_us_;
}

int HardwareNode::Run(int duration_s) {
  talos::process::InstallStopHandlers();
  // The 1 kHz merge runs above the nodes (priority 50) so a busy node cannot
  // delay the command the RoboRIO is waiting for. Best effort: a refusal is
  // reported once and the bridge runs unprivileged. Simulation stays an
  // ordinary process.
  if (!simulation_) {
    talos::process::ConfigureRealtime(
        {.period = std::chrono::microseconds{kTickPeriodUs}, .priority = 60});
  }
  const auto end =
      std::chrono::steady_clock::now() + std::chrono::seconds{duration_s};

  // Explicit periodic scheduling: each tick has a deadline
  // one period after the previous, so work time does not accumulate drift
  // the way work-then-sleep(1 ms) does. An overrun skips the sleep once and
  // re-anchors the deadline instead of queuing catch-up ticks.
  auto next_tick = std::chrono::steady_clock::now();
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

    next_tick += std::chrono::microseconds{kTickPeriodUs};
    if (next_tick > std::chrono::steady_clock::now()) {
      std::this_thread::sleep_until(next_tick);
    } else {
      next_tick = std::chrono::steady_clock::now();
    }
  }

  PublishIntrospection();

  std::printf("hardware_node: states=%llu commands=%llu\n",
              static_cast<unsigned long long>(states_received_),
              static_cast<unsigned long long>(commands_sent_));
  std::printf("hardware_node: identity_rejected=%llu ownership_rejected=%llu\n",
              static_cast<unsigned long long>(identity_rejected_),
              static_cast<unsigned long long>(ownership_rejected_));
  for (const auto& [name, tracker] : subsystems_) {
    std::printf("hardware_node: sub=%s requests=%llu timed_out=%d\n",
                name.c_str(), static_cast<unsigned long long>(tracker.requests),
                tracker.timed_out ? 1 : 0);
  }
  return states_received_ > 0 ? 0 : 1;
}

bool HardwareNode::HasRequestIdentity(const SubsystemTracker& tracker,
                                      const Command& partial) const {
  if (partial.config_id != config_id_ || partial.boot_id != last_boot_id_ ||
      partial.epoch != last_epoch_) {
    return false;
  }
  // Gateway-clock age bound: both stamps are gateway clock, never
  // differenced against bridge now_us. A
  // request computed from sample N stays acceptable while newer samples
  // arrive, until its age reaches the command timeout. The gateway
  // re-enforces the same bound against its own now, so it stays the safety
  // backstop.
  if (partial.observed_time_us > last_sample_time_us_) return false;
  if (last_sample_time_us_ - partial.observed_time_us >=
      config_.command_timeout_us) {
    return false;
  }
  // Per-subsystem monotonic acceptance: old demands cannot replay, even
  // within the age bound. Equal re-publishes from the same sample are fine.
  if (partial.observed_time_us < tracker.last_observed_time_us) return false;
  return true;
}

bool HardwareNode::OwnsRequestDevices(const SubsystemTracker& tracker,
                                      const Command& partial) const {
  const auto owns = [](const auto& owned, uint16_t id) {
    return std::find(owned.begin(), owned.end(), id) != owned.end();
  };
  for (std::size_t i = 0; i < partial.count; ++i) {
    if (!owns(tracker.devices.motors, partial.motors[i].id)) return false;
  }
  for (std::size_t i = 0; i < partial.digital_output_count; ++i) {
    if (!owns(tracker.devices.digital_outputs, partial.digital_outputs[i].id)) {
      return false;
    }
  }
  for (std::size_t i = 0; i < partial.pwm_output_count; ++i) {
    if (!owns(tracker.devices.pwm_outputs, partial.pwm_outputs[i].id)) {
      return false;
    }
  }
  return true;
}

void HardwareNode::NeutralizeAll() {
  for (std::size_t j = 0; j < full_cmd_.count; ++j) {
    full_cmd_.motors[j].mode = Mode::kNeutral;
    full_cmd_.motors[j].demand = 0.0;
    full_cmd_.motors[j].feedforward_v = 0.0;
  }
  for (std::size_t j = 0; j < full_cmd_.digital_output_count; ++j) {
    full_cmd_.digital_outputs[j].value = false;
  }
  for (std::size_t j = 0; j < full_cmd_.pwm_output_count; ++j) {
    full_cmd_.pwm_outputs[j].output = 0.0;
  }
}

void HardwareNode::InvalidateRequests() {
  NeutralizeAll();
  for (auto& [name, tracker] : subsystems_) {
    tracker.last_seen_us = 0;
    tracker.last_observed_time_us = 0;
    tracker.timed_out = true;
  }
  // Force the next merged command onto the wire even if its demands happen
  // to equal the pre-transition ones: the identity headers differ, but the
  // heartbeat timer must not get a vote on the first post-transition send.
  have_last_sent_ = false;
}

}  // namespace talos::hardware
