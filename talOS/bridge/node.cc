#include "node.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

#include "bridge/topics.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/os/poller.h"
#include "talOS/hardware/gate.h"
#include "talOS/introspection/names.h"
#include "talOS/process/process.h"

namespace talos::hardware {

// The name the stock bridge describes itself by is the roster name the
// launcher lists it under.
static_assert(std::string_view{kNodeName} ==
              introspect::naming::kHardwareBridgeNode);

// /hw/status is read by talos::Node subscribers, which open every slot at
// 8-byte alignment (talos::kMessageAlignment). ipc::Publisher lays the slot out
// at alignof(Message), so the two agree only while this holds.
static_assert(alignof(bridge::BridgeStatus) == 8);
static_assert(sizeof(bridge::BridgeStatus) == 192,
              "BridgeStatus size changed; update docs/DATA_FLOW.md");
// Each actuator-owning subsystem owns at least one such device.
static_assert(kMaxMotors + kMaxDigitalOutputs + kMaxPwmOutputs <= 32,
              "BridgeStatus masks are 32 bits wide");

HardwareNode::HardwareNode(config::RobotConfig robot_config,
                           std::string remote_ip, uint16_t remote_port,
                           uint16_t local_port, bool simulation,
                           uint64_t session_id, std::string log_path,
                           uint64_t log_max_bytes,
                           std::chrono::nanoseconds log_window)
    : robot_config_{std::move(robot_config)},
      config_{robot_config_.hardware},
      remote_ip_{std::move(remote_ip)},
      remote_port_{remote_port},
      local_port_{local_port},
      simulation_{simulation},
      session_id_{session_id},
      log_path_{std::move(log_path)},
      log_max_bytes_{log_max_bytes},
      log_window_{log_window} {
  if (simulation_ && remote_ip_ != "127.0.0.1") {
    throw std::invalid_argument(
        "--sim requires --remote 127.0.0.1, got --remote " + remote_ip_);
  }
  // Simulation stands in for commissioned hardware, in memory only. Nodes do
  // the same under the same --sim flag (HardwareLink), and the commissioned
  // bit is part of the config id both ends compare: the loopback address
  // alone must not decide it, since nodes never see the remote.
  if (simulation_) config_.commissioned = true;
  config_id_ = ConfigurationId(config_);
  // Heartbeat keeps a live-but-unchanged merge refreshing the gateway at 5x
  // margin inside the command timeout, whatever the config chose. The clamp
  // keeps steady-state traffic near the slowest actuator period (20 ms)
  // without ever approaching the timeout floor (hardware::Validate).
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
    if (!devs.owns_actuators()) continue;
    SubsystemTracker tracker;
    tracker.name = name;
    tracker.devices = devs;
    tracker.period_us = config_.period_us;
    if (const auto* tbl = robot_config_.GetSubsystemTable(name)) {
      tracker.period_us = static_cast<uint32_t>(
          (*tbl)["period_us"].value_or(config_.period_us));
    }
    tracker.merge.timeout_us =
        OwnerTimeoutUs(tracker.period_us, config_.command_timeout_us);
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
      if (auto it = table.find(id); it != table.end()) {
        out.push_back(it->second);
      }
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
  OpenTopics();
  return true;
}

void HardwareNode::OpenTopics() {
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
  if (!log_path_.empty()) {
    event::log::LogWriter::Options options;
    options.max_bytes = log_max_bytes_;
    options.window = log_window_;
    log_ = std::make_unique<event::log::LogWriter>(log_path_, kNodeName,
                                                   options, session_id_);
    log_->start(introspection_manifest_, event::Poller::now());
  }

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
  return introspect::DescribeManifest(kNodeName, kNodeTarget,
                                      introspection_manifest_, MAX_SLOTS,
                                      EndpointAttributes());
}

void HardwareNode::BuildIntrospectionManifest() {
  introspection_manifest_.clear();

  const auto add = [this]<class M>(event::SourceKind kind,
                                   const std::string& name) {
    const auto id = static_cast<std::uint16_t>(introspection_manifest_.size());
    introspection_manifest_.push_back({
        .id = id,
        .kind = kind,
        .name = name,
        .message_bytes = static_cast<std::uint32_t>(sizeof(M)),
        .alignment = static_cast<std::uint32_t>(alignof(M)),
        .schema = schema::Of<M>(),
    });
    return static_cast<std::size_t>(id);
  };
  const auto add_packet = [&add](event::SourceKind kind,
                                 const std::string& name) {
    return add.template operator()<Packet>(kind, name);
  };

  // The fixed ends first, then one request per subsystem, so the fixed ids
  // (PublishIntrospection's 0..3) do not depend on the configuration.
  add_packet(event::SourceKind::SENDER, kStateTopic);
  add_packet(event::SourceKind::SENDER, kCommandTopic);
  add_packet(event::SourceKind::SENDER, kDriverStationTopic);
  add.template operator()<bridge::BridgeStatus>(event::SourceKind::SENDER,
                                                bridge::topics::kStatus);

  for (auto& [name, tracker] : subsystems_) {
    tracker.source_index =
        add_packet(event::SourceKind::FETCHER, RequestTopic(name));
  }
}

std::uint64_t HardwareNode::DispatchCount() const {
  // Every message this node moved, which is the sum of its per-topic counters.
  // Subsystem requests belong in it: dropping them was what let the aliased
  // /hw/req/drive traffic go uncounted for as long as it did.
  std::uint64_t dispatches = states_received_ + commands_sent_ +
                             driver_station_packets_received_ +
                             statuses_published_;
  for (const auto& [name, tracker] : subsystems_) {
    dispatches += tracker.requests;
  }
  return dispatches;
}

void HardwareNode::RefreshLiveness() {
  if (!registration_) return;
  registration_->begin_sample();
  registration_->heartbeat();
  registration_->set_dispatch_count(DispatchCount());
  registration_->end_sample();
}

void HardwareNode::PublishIntrospection() {
  if (!registration_) {
    return;
  }

  // One sample; see NodeRegistration::begin_sample.
  registration_->begin_sample();
  registration_->heartbeat();
  registration_->set_dispatch_count(DispatchCount());
  if (log_) registration_->set_recording(log_->failed(), log_->capped());

  const auto set = [this](std::size_t index, std::uint64_t events,
                          std::uint64_t dropped = 0) {
    registration_->set_source(static_cast<std::uint32_t>(index), events,
                              dropped, /*sequence=*/events,
                              /*last_monotonic_ns=*/0, /*last_latency_ns=*/0,
                              /*max_latency_ns=*/0);
  };

  set(0, states_received_);
  set(1, commands_sent_);
  set(2, driver_station_packets_received_);
  set(3, statuses_published_);
  for (const auto& [name, tracker] : subsystems_) {
    set(tracker.source_index, tracker.requests, tracker.requests_skipped);
  }

  // The bridge's loop is its own, so its wake loss is too: the wake
  // notifications its publishers failed to send, the request topics whose
  // failed arms its idle-pass guard (kMaxIdlePasses) stopped holding it
  // awake, and the runs that could not wait on their inputs (OpenWaits).
  std::uint64_t wake_failures = 0;
  for (auto* publisher : {state_pub_.get(), cmd_pub_.get(), ds_pub_.get()}) {
    if (publisher != nullptr)
      wake_failures += publisher->queue().wake_failures();
  }
  if (status_pub_) wake_failures += status_pub_->queue().wake_failures();
  registration_->set_wake_loss(wake_failures, wakeups_.stalled_arms,
                               wakeups_.wait_fallbacks);
  registration_->end_sample();
}

void HardwareNode::PushConfig(uint64_t now_us) {
  auto chunks = CreateConfigChunks(config_);
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    SendToGateway(protocol::FrameType::kHardwareConfig, now_us,
                  chunks[i].data(), chunks[i].size());
  }
  last_push_time_us_ = now_us;
}

protocol::UdpStatus HardwareNode::SendToGateway(protocol::FrameType type,
                                                uint64_t now_us,
                                                const void* payload,
                                                std::size_t payload_size) {
  if (frame_sink_.send != nullptr) {
    frame_sink_.send(frame_sink_.context, type, now_us,
                     {static_cast<const uint8_t*>(payload),
                      payload == nullptr ? 0 : payload_size});
    return protocol::UdpStatus::kOk;
  }
  if (!peer_.IsOpen()) return protocol::UdpStatus::kSocketError;
  int error = seams_.send_error != nullptr ? seams_.send_error() : 0;
  const protocol::UdpStatus status =
      error != 0 ? protocol::UdpStatus::kSocketError
                 : peer_.SendFrame(type, 0, now_us, payload, payload_size);
  if (status == protocol::UdpStatus::kOk) {
    if (socket_lost_) {
      socket_lost_ = false;
      std::fprintf(stderr,
                   "hardware_node: the UDP socket to the gateway sends again "
                   "(%llu reopened so far)\n",
                   static_cast<unsigned long long>(socket_counters_.reopens));
    }
    return status;
  }
  if (status != protocol::UdpStatus::kSocketError) return status;
  if (error == 0) error = peer_.stats().last_send_error;
  ++socket_counters_.send_failures;
  socket_counters_.last_send_error = error;
  if (!protocol::SendErrorNeedsNewSocket(error)) return status;
  if (poller_) poller_->remove_fd(peer_.NativeFd());
  peer_.Close();
  if (!socket_lost_) {
    socket_lost_ = true;
    std::fprintf(stderr,
                 "hardware_node: the UDP socket to the gateway cannot send "
                 "(%s); replacing it, nothing is sent until then\n",
                 std::strerror(error));
  }
  return status;
}

void HardwareNode::ReopenSocket(uint64_t now_us) {
  if (peer_.IsOpen() || !socket_lost_) return;
  if (reopen_attempted_ && now_us - last_reopen_us_ < kReopenRetryUs) return;
  reopen_attempted_ = true;
  last_reopen_us_ = now_us;
  // A failure (the port taken meanwhile, out of descriptors) is retried
  // kReopenRetryUs later.
  if (peer_.Reopen() != protocol::UdpStatus::kOk) return;
  ++socket_counters_.reopens;
  if (!poller_) return;
  try {
    poller_->add_fd(peer_.NativeFd());
  } catch (const std::exception& e) {
    FallBackToPolling(e.what());
  }
}

void HardwareNode::Tick(uint64_t now_us) {
  tick_datagrams_ = 0;
  tick_requests_ = 0;
  ReopenSocket(now_us);
  // Tick's timestamp precedes socket reads. Complete the previous tick's
  // exchange now, so the upper endpoint is definitely after receipt (Run()
  // ticks again at once for it: NextDutyUs).
  if (pending_probe_send_us_ != 0) {
    clock_.ObserveExchange(pending_probe_boot_id_, pending_probe_send_us_,
                           pending_probe_sample_us_, now_us);
    pending_probe_send_us_ = 0;
    probe_waiting_ = false;
  }
  if ((!probe_waiting_ && (last_probe_send_us_ == 0 ||
                           now_us - last_probe_send_us_ >= kProbePeriodUs)) ||
      (probe_waiting_ && now_us - last_probe_send_us_ > kProbeRetryUs)) {
    if (SendToGateway(protocol::FrameType::kClockProbe, now_us, nullptr, 0) ==
        protocol::UdpStatus::kOk) {
      last_probe_send_us_ = now_us;
      probe_waiting_ = true;
    }
  }
  // 1. Process incoming UDP frames from Gateway, at most kReceiveBudget, so
  // a burst under overload delays rather than stalls the merge below; Run()
  // ticks again without waiting when the budget runs out.
  tick_receive_capped_ = true;
  for (int i = 0; i < kReceiveBudget; ++i) {
    // Closed (SocketCounters), maybe by a State's config push: nothing to
    // read.
    if (!peer_.IsOpen()) {
      tick_receive_capped_ = false;
      break;
    }
    protocol::DecodedFrame frame;
    const auto status = peer_.TryReceive(&frame);
    if (status == protocol::UdpStatus::kWouldBlock) {
      tick_receive_capped_ = false;
      break;
    }
    // A datagram consumed, decodable or not. A socket error need not have
    // consumed one, so a loop of them is not input.
    if (status == protocol::UdpStatus::kOk ||
        status == protocol::UdpStatus::kDecodeError) {
      ++tick_datagrams_;
    }
    if (status == protocol::UdpStatus::kDecodeError &&
        peer_.stats().last_decode_status ==
            protocol::DecodeStatus::kUnsupportedVersion) {
      // Once per mismatch: a stale RoboRIO program floods every tick.
      if (!version_mismatch_logged_) {
        std::fprintf(
            stderr,
            "hardware_node: the gateway speaks protocol %u, this "
            "build %u; deploy the RoboRIO program from this build\n",
            static_cast<unsigned>(peer_.stats().last_unsupported_version),
            static_cast<unsigned>(protocol::kProtocolVersion));
        version_mismatch_logged_ = true;
      }
      continue;
    }
    if (status != protocol::UdpStatus::kOk) continue;
    version_mismatch_logged_ = false;

    HandleGatewayFrame(frame, now_us);
  }

  // 2. Read subsystem requests and perform timeout neutralization
  bool any_subsystem_active = false;
  for (auto& [name, tracker] : subsystems_) {
    std::optional<ipc::Received<talos::hardware::Packet>> received;
    if (tracker.subscriber) received = tracker.subscriber->read_next();

    if (received) {
      ++tracker.requests;
      ++tick_requests_;
      tracker.arm_stalled = false;
      tracker.requests_skipped += received->dropped;
      Command partial{};
      if (Decode(received->message.bytes(), partial)) {
        // Trust gates, in this order: an obsolete request must never merge,
        // and a current one must still own every device it addresses. Either
        // rejection drops the whole request without proving liveness, so a
        // stream of bad requests still times the subsystem out.
        if (!HasRequestIdentity(tracker, partial)) {
          ++identity_rejected_;
        } else if (!OwnsRequestDevices(tracker, partial)) {
          ++ownership_rejected_;
        } else {
          // A request replaces this subsystem's whole desired output, not
          // just the addressed devices. Omitted outputs are neutral.
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
          tracker.merge.Accept(now_us, partial.observed_time_us);
          tracker.last_accepted_us = now_us;
        }
      }
    }

    // Owner timeout, on the bridge clock since the last accepted request.
    if (tracker.merge.Expire(now_us)) {
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
    if (const auto merged = MergedObservationUs(
            subsystems_, [](const auto& entry) -> const MergeOwner& {
              return entry.second.merge;
            })) {
      full_cmd_.observed_time_us = *merged;
    }
    if (!any_subsystem_active && last_sent_had_active_subsystem_) {
      full_cmd_.observed_time_us = latest_state_.sample_time_us;
    }
  }

  // 3. Send unified command if we have observed state. Active demands retain
  // their request observation; only the final timeout's fully neutral command
  // adopts the latest state so the gateway accepts it. Publication is
  // change-driven with a bounded heartbeat (CommandDueUs): a request that
  // changes the merge sends at once, except that while the gateway cannot
  // take a command a new observation alone waits for the heartbeat, and an
  // unchanged merge resends only when the heartbeat elapses. The gateway's
  // 100 ms timeout keeps 5x margin over the heartbeat. Subsystem timeouts
  // neutralize through the merge, and a changed demand always sends
  // immediately.
  if (last_boot_id_ != 0 &&
      (any_subsystem_active || last_sent_had_active_subsystem_)) {
    if (now_us >= CommandDueUs(now_us)) {
      talos::hardware::Packet cmd_pkt{};
      cmd_pkt.size = static_cast<uint32_t>(Encode(full_cmd_, cmd_pkt.data));
      if (cmd_pkt.size > 0) {
        if (cmd_pub_) Publish(*cmd_pub_, 1, cmd_pkt, now_us);
        if (SendToGateway(protocol::FrameType::kHardwareCommand, now_us,
                          cmd_pkt.data.data(),
                          cmd_pkt.size) == protocol::UdpStatus::kOk) {
          ++commands_sent_;
          last_sent_cmd_ = full_cmd_;
          last_send_us_ = now_us;
          have_last_sent_ = true;
          last_sent_had_active_subsystem_ = any_subsystem_active;
        }
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

void HardwareNode::HandleGatewayFrame(const protocol::DecodedFrame& frame,
                                      uint64_t now_us) {
  if (frame.header.type == protocol::FrameType::kHardwareConfigAck) {
    ConfigAckPayload ack{};
    if (DecodeConfigAck({frame.payload, frame.payload_size}, ack)) {
      if (ack.config_id == config_id_) {
        if (ack.status == ConfigAckStatus::kOk) {
          is_configured_ = true;
          config_push_backoff_us_ = 50000;
        } else {
          is_configured_ = false;
          config_push_backoff_us_ =
              std::min<uint64_t>(config_push_backoff_us_ * 2, 2000000);
          last_push_time_us_ = now_us;
        }
      }
    }
    return;
  }

  if (frame.header.type == protocol::FrameType::kHardwareState) {
    State state;
    if (!Decode({frame.payload, frame.payload_size}, state)) {
      // Once per stretch: a gateway sending malformed States sends them
      // every tick, and dropping them silently looks like a dead link.
      if (!state_decode_failure_logged_) {
        std::fprintf(stderr,
                     "hardware_node: dropping a State that does not decode "
                     "(%zu bytes)\n",
                     frame.payload_size);
        state_decode_failure_logged_ = true;
      }
      return;
    }
    state_decode_failure_logged_ = false;

    ++states_received_;
    // Ordering first, before anything is touched: the clock mapping takes
    // a State only when it is newer than the last one of its gateway boot
    // or opens a boot never seen. A duplicate, a reordered frame (with
    // whatever older epoch or enable edge it carries) or a datagram delayed
    // from a retired boot is dropped whole, counted in
    // clock_ignored_count: it must not roll back the identity, refresh
    // freshness, invalidate healthy requests or trigger a config push.
    if (!clock_.Observe(state.boot_id, state.sample_time_us, now_us)) {
      return;
    }
    if (probe_waiting_ && state.clock_probe_time_us != 0 &&
        state.clock_probe_time_us == last_probe_send_us_ &&
        pending_probe_send_us_ == 0) {
      pending_probe_send_us_ = state.clock_probe_time_us;
      pending_probe_sample_us_ = state.sample_time_us;
      pending_probe_boot_id_ = state.boot_id;
    }
    last_state_receipt_us_ = now_us;
    state_accepted_ = true;
    if (state.boot_id != last_boot_id_) {
      config_push_backoff_us_ = 50000;
      // The new boot's Driver Station clock restarted with it.
      last_ds_sample_time_us_ = 0;
      ReportGatewayBuild(state);
    }
    // Configured means the current gateway boot acknowledged our config.
    // A new boot (not the first State this bridge sees, which an earlier
    // ack may precede) or a State naming another config voids it until the
    // next kOk ack; a State naming ours does not restore it, since the
    // gateway reports that id even when applying the config failed.
    if ((last_boot_id_ != 0 && state.boot_id != last_boot_id_) ||
        state.config_id != config_id_) {
      is_configured_ = false;
    }
    if (state.boot_id != last_boot_id_ || state.config_id != config_id_ ||
        !is_configured_) {
      if (now_us - last_push_time_us_ >= config_push_backoff_us_ ||
          state.boot_id != last_boot_id_) {
        PushConfig(now_us);
      }
    }

    // Controller transition: reboot, reconfiguration, epoch bump, or an
    // enabled/fault edge (the gateway advances its epoch on disable,
    // re-enable and total feedback loss, not when that fault clears, so the
    // edge check is belt and braces).
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
    latest_state_ = state;

    talos::hardware::Packet pkt{};
    pkt.size = static_cast<uint32_t>(Encode(state, pkt.data));
    if (state_pub_) Publish(*state_pub_, 0, pkt, now_us);
    return;
  }

  // Driver Station packets are forwarded only from the boot the accepted
  // State is in (a retired boot's delayed packet, or one from a boot whose
  // State has not arrived yet, is dropped), in sample order within it.
  if (frame.header.type == protocol::FrameType::kDriverStation) {
    driver_station::DriverStationData data{};
    if (!driver_station::Decode({frame.payload, frame.payload_size}, data) ||
        data.boot_id != last_boot_id_ ||
        data.sample_time_us < last_ds_sample_time_us_) {
      return;
    }
    last_ds_sample_time_us_ = data.sample_time_us;
    talos::hardware::Packet pkt{};
    const std::size_t n =
        std::min<std::size_t>(frame.payload_size, pkt.data.size());
    pkt.size = static_cast<uint32_t>(n);
    std::memcpy(pkt.data.data(), frame.payload, n);
    if (ds_pub_) ds_pub_->write(pkt);
    ++driver_station_packets_received_;
  }
}

// /hw/status carries the State's program value as is.
static_assert(static_cast<uint8_t>(bridge::GatewayProgram::UNKNOWN) ==
              static_cast<uint8_t>(GatewayProgram::kUnknown));
static_assert(static_cast<uint8_t>(bridge::GatewayProgram::ROBORIO) ==
              static_cast<uint8_t>(GatewayProgram::kRoboRio));
static_assert(
    static_cast<uint8_t>(bridge::GatewayProgram::ROBORIO_SIMULATION) ==
    static_cast<uint8_t>(GatewayProgram::kRoboRioSimulation));
static_assert(static_cast<uint8_t>(bridge::GatewayProgram::SIM_GATEWAY) ==
              static_cast<uint8_t>(GatewayProgram::kSimGateway));
static_assert(bridge::GatewayProgram::MAX ==
                  bridge::GatewayProgram::SIM_GATEWAY &&
              kLastGatewayProgram == GatewayProgram::kSimGateway);

bridge::BridgeStatus HardwareNode::Status(uint64_t now_us) const {
  uint32_t timed_out_mask = 0;
  uint32_t bit = 0;
  // An owner silent past its owner timeout, measured from its last accepted
  // request rather than from the merge (MergeOwner::Expire's test, but
  // surviving the Reset() a controller transition makes).
  bool owner_silent = false;
  // subsystems_ is a std::map, so this walks the names in sorted order.
  for (const auto& [name, tracker] : subsystems_) {
    if (bit < 32 && tracker.merge.timed_out) {
      timed_out_mask |= uint32_t{1} << bit;
    }
    ++bit;
    if (!tracker.last_accepted_us ||
        (now_us > *tracker.last_accepted_us &&
         now_us - *tracker.last_accepted_us > tracker.merge.timeout_us)) {
      owner_silent = true;
    }
  }
  uint32_t motor_fault_mask = 0;
  uint32_t forward_limit_mask = 0;
  uint32_t reverse_limit_mask = 0;
  for (std::size_t i = 0; i < config_.motors.size() && i < 32; ++i) {
    for (std::size_t j = 0; j < latest_state_.motor_count; ++j) {
      const MotorSample& sample = latest_state_.motors[j];
      if (sample.id != config_.motors[i].id) continue;
      const uint32_t motor_bit = uint32_t{1} << i;
      if (!sample.valid || MotorFaulted(sample.fault_flags)) {
        motor_fault_mask |= motor_bit;
      }
      if (!sample.valid) continue;
      if ((sample.fault_flags & kMotorForwardHardLimitFlag) != 0) {
        forward_limit_mask |= motor_bit;
      }
      if ((sample.fault_flags & kMotorReverseHardLimitFlag) != 0) {
        reverse_limit_mask |= motor_bit;
      }
    }
  }
  const uint64_t state_age_us =
      states_received_ > 0 && now_us > last_state_receipt_us_
          ? now_us - last_state_receipt_us_
          : 0;
  const HealthSample& health = latest_state_.health;
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
      config_.commissioned,
      /*alert_error_link_lost=*/!state_accepted_ ||
          state_age_us > config_.command_timeout_us,
      /*alert_error_not_configured=*/!is_configured_,
      /*alert_error_uncommissioned=*/!config_.commissioned,
      /*alert_error_motor_fault=*/motor_fault_mask != 0,
      /*alert_warning_owner_timed_out=*/owner_silent,
      /*alert_warning_brownout=*/health.valid && health.brownout,
      clock_.boot_id(),
      clock_.accuracy_valid() ? clock_.bounded_offset_us() : clock_.offset_us(),
      clock_.spread_us(),
      clock_.resets_count(),
      clock_.ignored_count(),
      static_cast<uint32_t>(clock_.window_count()),
      clock_.converged() && clock_.accuracy_valid(),
      health.valid,
      health.battery_v,
      health.brownout,
      health.can_utilization_frac,
      motor_fault_mask,
      forward_limit_mask,
      reverse_limit_mask,
      latest_state_.counters.rejected_command_count,
      latest_state_.counters.neutralized_motor_count,
      latest_state_.counters.apply_failure_count,
      latest_state_.counters.device_reset_count,
      static_cast<bridge::GatewayProgram>(latest_state_.build.program),
      latest_state_.build.dirty,
      latest_state_.build.revision,
      clock_.uncertainty_us(),
      clock_.round_trip_us(),
  };
}

void HardwareNode::ReportGatewayBuild(const State& state) {
  char revision[17] = "unknown";
  if (state.build.revision != 0) {
    std::snprintf(revision, sizeof(revision), "%016llx",
                  static_cast<unsigned long long>(state.build.revision));
  }
  std::fprintf(stderr, "hardware_node: gateway boot %llu: %s build %s%s\n",
               static_cast<unsigned long long>(state.boot_id),
               GatewayProgramName(state.build.program), revision,
               state.build.dirty ? " (dirty)" : "");
}

void HardwareNode::Publish(ipc::Publisher<Packet>& publisher,
                           uint16_t source_id, const Packet& packet,
                           uint64_t now_us) {
  // RTMS stamps a publication with the Poller clock when not given one; the
  // stamp is taken here so the log can carry it.
  const event::MonotonicTime published = event::Poller::now();
  const WriteStatus status = publisher.write(packet, published.nanos());
  if (!log_) return;
  // The SEND record's aux is the event loop's SendStatus
  // (talOS/events/event_loop_base.h): 0 OK, 1 refused, 2 failed.
  std::uint32_t aux = 0;
  if (status.result == WriteResult::BUFFER_FULL) {
    aux = 1;
  } else if (status.result != WriteResult::SUCCESS) {
    aux = 2;
  }
  const event::Context context{
      .event_time = event::MonotonicTime::from_nanos(
          static_cast<std::int64_t>(now_us * 1000)),
      .now = published,
  };
  log_->send(context, source_id,
             std::as_bytes(std::span{&packet, std::size_t{1}}), aux,
             status.sequence);
  ReportLogFailure();
}

void HardwareNode::ReportLogFailure() {
  // The writer thread creates the file, so an open failure surfaces as
  // failed(), not capture_stopped(); report either once, with its reason.
  if (!log_ || log_failure_reported_) return;
  if (!log_->capture_stopped() && !log_->failed()) return;
  std::fprintf(stderr, "hardware_node: log capture stopped: %s\n",
               log_->error().c_str());
  log_failure_reported_ = true;
}

uint64_t HardwareNode::CommandDueUs(uint64_t now_us) const {
  if (!have_last_sent_) return now_us;
  bool changed = full_cmd_ != last_sent_cmd_;
  if (changed && !GatewayReady(latest_state_)) {
    Command unobserved = full_cmd_;
    unobserved.observed_time_us = last_sent_cmd_.observed_time_us;
    changed = unobserved != last_sent_cmd_;
  }
  return changed ? now_us : last_send_us_ + heartbeat_us_;
}

uint64_t HardwareNode::NextDutyUs(uint64_t now_us) const {
  // Tick completes an echoed probe with its own clock reading, taken before
  // it receives anything; the next one is the first known to follow receipt.
  if (pending_probe_send_us_ != 0) return now_us;
  uint64_t next_us = std::numeric_limits<uint64_t>::max();
  const auto due = [&next_us, now_us](uint64_t at_us) {
    next_us = std::min(next_us, at_us > now_us ? at_us : now_us + kRetryUs);
  };
  // Clock probe: every kProbePeriodUs, replaced once more than kProbeRetryUs
  // goes unanswered. A closed socket's reopen stands in for it, and for the
  // command below: neither can go out before it.
  const bool socket_closed = socket_lost_ && !peer_.IsOpen();
  if (socket_closed) {
    due(reopen_attempted_ ? last_reopen_us_ + kReopenRetryUs : now_us);
  } else if (!probe_waiting_) {
    due(last_probe_send_us_ == 0 ? now_us
                                 : last_probe_send_us_ + kProbePeriodUs);
  } else {
    due(last_probe_send_us_ + kProbeRetryUs + 1);
  }
  if (status_pub_) {
    due(statuses_published_ == 0 ? now_us : last_status_us_ + kStatusPeriodUs);
  }
  // Each merged owner's timeout. That some owner is merged is Tick's
  // any_subsystem_active, which with the boot gates every command.
  bool any_subsystem_active = false;
  for (const auto& [name, tracker] : subsystems_) {
    if (const auto expires_us = tracker.merge.ExpiresAtUs()) {
      due(*expires_us);
      any_subsystem_active = true;
    }
  }
  if (!socket_closed && last_boot_id_ != 0 &&
      (any_subsystem_active || last_sent_had_active_subsystem_)) {
    // Tick's own send rule: a changed merge at once, else the heartbeat.
    due(CommandDueUs(now_us));
  }
  // No config push: Tick pushes only on receiving a State, which wakes the
  // loop itself.
  return next_us;
}

HardwareNode::Wait HardwareNode::Pass(uint64_t now_us) {
  Tick(now_us);
  ++wakeups_.passes;

  // The per-source counters four times a second, which is enough for a
  // viewer. Heartbeat and dispatch count on every pass, from this loop, as an
  // event-loop node's loop thread stores its own dispatch count at each
  // dispatch: the launcher's stall check samples it, and a sample up to
  // 250 ms old plus a short pause would read as a stall. A pass costs them a
  // few atomic stores; a wake of their own (kLivenessBackstopUs) would cost a
  // context switch.
  if (now_us >= next_introspection_us_) {
    PublishIntrospection();
    next_introspection_us_ = now_us + kIntrospectionPeriodUs;
  } else {
    RefreshLiveness();
  }
  next_liveness_us_ = now_us + kLivenessBackstopUs;

  // Every request subscriber is armed before the loop may sleep: a request
  // published after its arm notifies request_wake_, and one published
  // before it fails the arm (talOS/rtms README.md "Wake channel").
  bool input_waiting = tick_receive_capped_;
  for (auto& [name, tracker] : subsystems_) {
    tracker.arm_failed = false;
    // One without a slot cannot be armed; each Tick's read retries the
    // claim, and the longest wait bounds the time to the next Tick.
    if (!tracker.subscriber || !tracker.subscriber->registered()) continue;
    const bool caught_up = seams_.arm_request != nullptr
                               ? seams_.arm_request(*tracker.subscriber)
                               : tracker.subscriber->arm_wake();
    if (caught_up) {
      tracker.arm_stalled = false;
    } else {
      tracker.arm_failed = true;
      input_waiting = true;
    }
  }

  const uint64_t timed_us =
      std::min({NextDutyUs(now_us), next_liveness_us_, next_introspection_us_,
                now_us + max_wait_us_});

  // CPU budget: the window restarts at the first pass after it ended.
  if (now_us - cpu_window_start_us_ >= kCpuWindowUs) {
    cpu_window_start_us_ = now_us;
    cpu_window_passes_ = 0;
    cpu_window_datagrams_ = 0;
  }
  ++cpu_window_passes_;
  cpu_window_datagrams_ += tick_datagrams_;
  if (cpu_window_passes_ >= kPassBudget ||
      cpu_window_datagrams_ >= kDatagramBudget) {
    // Spent: input waits for the window's end, a duty does not.
    ++wakeups_.throttled;
    idle_passes_ = 0;
    return {std::min(cpu_window_start_us_ + kCpuWindowUs, timed_us), true};
  }

  if (input_waiting) {
    if (tick_datagrams_ > 0 || tick_requests_ > 0) {
      idle_passes_ = 0;
      return {now_us, false};
    }
    if (idle_passes_ < kMaxIdlePasses) {
      ++idle_passes_;
      return {now_us, false};
    }
    // The guard: each topic whose failed arm it now stops holding the loop
    // awake is a stalled arm, counted once until it arms or delivers again,
    // as an event loop counts its watchers' (Activity::stalled_arms).
    for (auto& [name, tracker] : subsystems_) {
      if (tracker.arm_failed && !tracker.arm_stalled) {
        tracker.arm_stalled = true;
        ++wakeups_.stalled_arms;
      }
    }
  }
  idle_passes_ = 0;
  return {timed_us, false};
}

bool HardwareNode::OpenWaits() {
  // The loop sleeps until there is something to do: a datagram from the
  // gateway, a request on any /hw/request topic (each subscriber's slot
  // names request_wake_), a stop, or the next time-driven duty.
  try {
    poller_.emplace();
    if (peer_.NativeFd() < 0) {
      throw std::runtime_error("the UDP socket has no fd");
    }
    poller_->add_fd(peer_.NativeFd());
    if (seams_.make_wake_receiver != nullptr) {
      request_wake_.emplace(seams_.make_wake_receiver());
    } else {
      request_wake_.emplace();
    }
    poller_->add_fd(request_wake_->fd());
    // A stop (SIGINT, SIGTERM, process::RequestStop) makes the stop pipe
    // readable, which ends the wait at once.
    if (const int stop_fd = talos::process::StopFd(); stop_fd >= 0) {
      poller_->add_fd(stop_fd);
    }
    for (auto& [name, tracker] : subsystems_) {
      if (tracker.subscriber) {
        tracker.subscriber->set_wake_receiver(request_wake_->id());
      }
    }
    max_wait_us_ = kMaxWaitUs;
    return true;
  } catch (const std::exception& e) {
    FallBackToPolling(e.what());
    return false;
  }
}

void HardwareNode::FallBackToPolling(const char* reason) {
  // Out of descriptors, say: the bridge still runs, at the cost of a
  // pass every kRetryUs, as a node's loop falls back to polling.
  CloseWaits();
  max_wait_us_ = kRetryUs;
  ++wakeups_.wait_fallbacks;
  std::fprintf(stderr,
               "hardware_node: cannot wait for input (%s); polling it "
               "every %llu us\n",
               reason, static_cast<unsigned long long>(kRetryUs));
}

void HardwareNode::CloseWaits() {
  for (auto& [name, tracker] : subsystems_) {
    if (tracker.subscriber) tracker.subscriber->set_wake_receiver(0);
  }
  request_wake_.reset();
  poller_.reset();
}

int HardwareNode::Run(int duration_s) {
  talos::process::InstallStopHandlers();
  // The merge runs in the highest loop tier: on Linux above every node's
  // SCHED_FIFO priority, so a busy node cannot delay the command the RoboRIO
  // is waiting for (its loop waits in epoll_pwait2, a hard hrtimer for a
  // realtime thread on PREEMPT_RT, and wakes on the RoboRIO's datagram); the
  // CPU budget (kCpuWindowUs) keeps it from starving those nodes in turn.
  // macOS has no priority among time-constraint threads, so there the
  // bridge shares that band with the control nodes and is ahead only of the
  // observers (talOS/process LoopPolicy); its kWakeBudgetUs constraint is
  // the shortest in that band. Best effort: a refusal is reported once, on
  // stderr and in the registry row, and the bridge runs unprivileged.
  // Simulation stays an ordinary process unless TALOS_REALTIME=1 asks
  // otherwise.
  talos::process::RealtimeOutcome realtime{
      .tier = talos::process::LoopTier::kBridge};
  if (talos::process::RealtimeRequested(simulation_)) {
    realtime = talos::process::ConfigureRealtime(
        {.period = std::chrono::microseconds{kWakeBudgetUs},
         .tier = realtime.tier});
  }
  if (registration_) registration_->set_realtime(realtime);
  OpenWaits();

  // The node clock (Node::now() reads the same Poller clock), not
  // std::chrono::steady_clock: on macOS the two differ by time asleep, and
  // the clock mapping on /hw/status must land in the nodes' time.
  const auto clock_us = [] {
    return static_cast<uint64_t>(event::Poller::now().nanos() / 1000);
  };
  const uint64_t end_us =
      duration_s > 0
          ? clock_us() + (static_cast<uint64_t>(duration_s) * 1'000'000)
          : std::numeric_limits<uint64_t>::max();
  bool woke_readable = false;
  while (!talos::process::stop_requested.load()) {
    if (request_wake_) request_wake_->drain();
    const uint64_t now_us = clock_us();
    if (now_us >= end_us) break;
    const Wait wait = Pass(now_us);
    if (woke_readable) {
      if (tick_datagrams_ > 0) ++wakeups_.udp;
      if (tick_requests_ > 0) ++wakeups_.request;
      woke_readable = false;
    }
    const uint64_t until_us = std::min(wait.until_us, end_us);
    if (until_us <= now_us) continue;
    if (wait.ignore_input || !poller_) {
      // A sleep no datagram or request cuts short: the CPU budget is spent,
      // or there is no poller to wait on.
      std::this_thread::sleep_for(std::chrono::microseconds{until_us - now_us});
      if (!wait.ignore_input) ++wakeups_.deadline;
      continue;
    }
    const auto reason = poller_->wait_until(event::MonotonicTime::from_nanos(
        static_cast<std::int64_t>(until_us * 1000)));
    if (reason == event::Poller::WakeReason::DEADLINE) {
      ++wakeups_.deadline;
    } else {
      woke_readable = true;
      // A stop ends the loop at its check above. Bytes a cleared stop left
      // are read out, or the level-triggered wait would never block again.
      if (poller_->reported_readable(talos::process::StopFd())) {
        static_cast<void>(talos::process::ConsumeStopWake());
      }
    }
  }
  CloseWaits();

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
                tracker.merge.timed_out ? 1 : 0);
  }
  std::printf(
      "hardware_node: passes=%llu wakeups udp=%llu request=%llu "
      "deadline=%llu throttled=%llu wait_fallbacks=%llu\n",
      static_cast<unsigned long long>(wakeups_.passes),
      static_cast<unsigned long long>(wakeups_.udp),
      static_cast<unsigned long long>(wakeups_.request),
      static_cast<unsigned long long>(wakeups_.deadline),
      static_cast<unsigned long long>(wakeups_.throttled),
      static_cast<unsigned long long>(wakeups_.wait_fallbacks));
  std::printf("hardware_node: send_failures=%llu socket_reopens=%llu\n",
              static_cast<unsigned long long>(socket_counters_.send_failures),
              static_cast<unsigned long long>(socket_counters_.reopens));
  // A log that failed exits 1, as RunMain does for event-loop nodes.
  bool log_failed = false;
  if (log_) {
    log_->flush();
    ReportLogFailure();
    log_failed = log_->failed();
  }
  return states_received_ > 0 && !log_failed ? 0 : 1;
}

bool HardwareNode::HasRequestIdentity(const SubsystemTracker& tracker,
                                      const Command& partial) const {
  if (partial.config_id != config_id_ || partial.boot_id != last_boot_id_ ||
      partial.epoch != last_epoch_) {
    return false;
  }
  return tracker.merge.Admits(partial.observed_time_us, last_sample_time_us_,
                              config_.command_timeout_us);
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
    tracker.merge.Reset();
  }
  // Force the next merged command onto the wire even if its demands happen
  // to equal the pre-transition ones: the identity headers differ, but the
  // heartbeat timer must not get a vote on the first post-transition send.
  have_last_sent_ = false;
  last_sent_had_active_subsystem_ = false;
}

}  // namespace talos::hardware
