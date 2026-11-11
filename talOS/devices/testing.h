#pragma once

// talos::testing::SimHardware: plays the bridge for a testing::Sim.
//
//   auto sim = talos::testing::Sim::FromToml(kRobotToml);
//   auto& shooter = sim.add<Shooter>("shooter");
//   talos::testing::SimHardware hw{sim};
//   hw.enable();
//   hw.ideal_motors();
//   sim.step(50ms);
//   EXPECT_EQ(hw.motor_request("shooter", "flywheel").mode,
//             talos::hardware::Mode::kVelocity);
//
// Every [robot] period it reads each subsystem's /hw/request/<name>, then
// publishes one /hw/state holding every device in the config: the feedback
// set here, this config's id (commissioned, as the bridge treats it under
// --sim), flags configured|enabled (configured only while disabled, the
// default), and the gateway tick sequence, one more each period. Once
// set_driver_station() has been called it also forwards that Driver Station
// packet on /hw/state/driver_station every period. Construct
// it before the first step to have a state at time 0; later, the first state
// goes out at that moment. Devices are keyed by (node, device) exactly as the
// node names them.
// Requests merge by the bridge's rules (talOS/bridge/merge.h): a node's
// outputs go neutral once its last admitted request is older than the
// bridge's owner timeout (ARCHITECTURE.md, "The tick"), and, as on the
// gateway, every output goes neutral (a new epoch) at a gateway tick that
// finds the oldest observation of any node still merged command_timeout_us
// old. Each node's latest request replaces its whole
// applied output: omitted devices are neutral, and disabled hardware applies
// only neutral outputs. As on the bridge and gateway, a request built from
// another boot's or epoch's State, or observed command_timeout_us before the
// latest State, is ignored, and a motor masked by reset_device() applies
// neutral, as does any later request observed before the State that last
// masked it. Unlike the gateway, a motor disconnected with set_connected()
// or whose feedback is too old (set_motor) still applies its request, losing
// every motor's and CAN sensor's feedback raises no hardware fault and starts
// no new epoch, enabling or disabling does not start a new epoch, and
// requests are not checked against the config id, motor limits or soft
// limits.
//
// Each period it also publishes the bridge's /hw/status: the states and
// Driver Station packets it sent, configured, and the exact simulated gateway
// clock mapping. The normal eight-State convergence gate is retained;
// uncertainty and RTT are zero because this in-process helper knows truth.
// Transport/clock-bound tests use the real Endpoint and ClockOffsetEstimator.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "talOS/devices/target.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/messages.h"
#include "talOS/node/testing.h"

namespace talos::testing {

class SimHardware {
 public:
  explicit SimHardware(Sim& sim);
  ~SimHardware();
  SimHardware(const SimHardware&) = delete;
  SimHardware& operator=(const SimHardware&) = delete;

  void enable(bool on = true);
  // Each period, velocity/position/motion-magic requests are reached
  // instantly (velocity integrates into position). Neutral removes drive:
  // velocity decays by min(1, period/tau) each tick, with tau = 50 ms for
  // brake motors and 1 s for coast motors (the Phoenix desktop sim's
  // constants), snapping below 0.01 rps. Position integrates the decayed
  // velocity. Other drive modes are not modeled; use the physics simulator
  // to model voltage/duty-cycle drive.
  void ideal_motors(bool on = true);

  // Feedback. Ages are microseconds before the gateway State sample time;
  // UINT32_MAX means unknown. Zero preserves simultaneous ideal feedback.
  // As on the gateway (hardware::FeedbackFresh), a motor, CANcoder or gyro
  // whose position or velocity age is command_timeout_us or more, or
  // unknown, goes out with valid == false, so its handle reads
  // connected() == false; a younger age is valid, whatever it is.
  void set_motor(std::string_view node, std::string_view device,
                 double position, double velocity,
                 std::uint32_t position_age_us = 0,
                 std::uint32_t velocity_age_us = 0, double temperature_c = 0,
                 double supply_current_a = 0, double stator_current_a = 0);
  // The motor's TalonFX limit switches, asserted or not, reported from the
  // next state on as its ForwardHardLimit/ReverseHardLimit fault bits
  // (Motor::forward_limit, reverse_limit) while it is connected. Reported
  // only: like soft limits, an asserted switch does not stop the request or
  // the ideal motor.
  void set_limit_switches(std::string_view node, std::string_view device,
                          bool forward, bool reverse);
  void set_digital_input(std::string_view node, std::string_view device,
                         bool value);
  void set_analog_input(std::string_view node, std::string_view device,
                        double volts);
  void set_encoder(std::string_view node, std::string_view device,
                   double position, double velocity);
  void set_cancoder(std::string_view node, std::string_view device,
                    double position, double velocity,
                    std::uint32_t position_age_us = 0,
                    std::uint32_t velocity_age_us = 0);
  void set_gyro(std::string_view node, std::string_view device, double yaw,
                double yaw_rate, std::uint32_t yaw_age_us = 0,
                std::uint32_t yaw_rate_age_us = 0);
  // The RoboRIO's clock and link: from now on the gateway clock reads sim
  // time + `gateway_offset` (states and Driver Station packets are stamped
  // with it), and each state is sampled `latency` before it goes out.
  // Defaults 0 and 0. sample_time() subtracts latency from publication time
  // using the exact known simulated clock offset. A latency increase of a
  // period or more makes the next states no newer than the last, dropped as
  // on a robot. Throws for negative latency or a gateway clock before 0.
  void set_gateway_clock(Duration gateway_offset, Duration latency);
  // A disconnected device keeps its slot in /hw/state with valid == false
  // and zero values, so its handle reads connected() == false. Every device
  // of that node named `device` is affected, whatever its kind; an unknown
  // name throws std::invalid_argument.
  void set_connected(std::string_view node, std::string_view device,
                     bool connected);
  // A device reset, as the gateway reports one: the next state counts it in
  // device_reset_count, and that state and the one after mask it (valid ==
  // false) with what the gateway masks along with it: its followers and the
  // motors it feeds as a remote CANcoder. Applies to the motor, CANcoder or
  // gyro of the node so named; throws std::invalid_argument when there is
  // none (an unknown name, or only another kind of device).
  void reset_device(std::string_view node, std::string_view device);
  // The next `count` gateway ticks happen (their sequence numbers are used)
  // but their States, Driver Station packets and /hw/status go nowhere, as
  // over a link outage. Each lost tick still observes the Driver Station mode,
  // as the RoboRIO does, so a mode change inside the outage starts a new
  // generation on the next delivered packet.
  void lose_states(int count);
  // A RoboRIO reboot: every output goes neutral until a node sends a request
  // built from a State of the new boot, later states and Driver Station
  // packets carry a new boot id, the tick sequence and device reset count
  // start over, and the Driver Station mode generation restarts. Feedback and
  // the gateway clock carry on.
  void reboot();

  // The Driver Station packet to forward from now on, stamped with the sim
  // time as the Rio stamps it, and with its mode generation by one
  // driver_station::ModeTracker, as the Rio's sampler does.
  void set_driver_station(const driver_station::DriverStationData& data);
  // Stops forwarding: the Driver Station goes silent, as when its link
  // drops, while /hw/state keeps coming.
  void stop_driver_station();

  // Raw outputs: the last value explicitly requested for each device
  // (neutral, false and 0 before any). Timeout, omission from a later
  // request, and disable do not change these inspection values.
  hardware::MotorRequest motor_request(std::string_view node,
                                       std::string_view device) const;
  bool digital_output(std::string_view node, std::string_view device) const;
  double pwm(std::string_view node, std::string_view device) const;

  // Applied outputs after per-node expiry, the gateway's merged-observation
  // check, omitted-output neutralization and the enabled gate. Inspecting
  // outputs never refreshes a request.
  hardware::MotorRequest applied_motor(std::string_view node,
                                       std::string_view device) const;
  bool applied_digital_output(std::string_view node,
                              std::string_view device) const;
  double applied_pwm(std::string_view node, std::string_view device) const;

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;  // the Sim's periodic callback holds a weak_ptr
};

// A Driver Station packet with `flags` (driver_station::kEnabled,
// kTeleop, ...) plus kDsAttached, and no joysticks:
//   hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
driver_station::DriverStationData DsPacket(std::uint32_t flags);

// Target lanes, as a producer node writes them (talos::TargetLane).
//   PublishTarget(sim, "shooter", "teleop", shooter::ShooterTarget{60.0});
//   ReleaseTarget<shooter::ShooterTarget>(sim, "shooter", "teleop");
//   LatestTarget<demo::ElevatorGoal>(sim, "elevator", "superstructure");
// Envelopes go through the same encoder TargetLane uses, stamped with
// DriverStationGeneration(sim): what a producer that has drained the latest
// Driver Station packet stamps. To stand in for a producer that has not (a
// target queued from an earlier mode, or a stale Driver Station view), pass
// the generation explicitly:
//   const std::uint64_t before = DriverStationGeneration(sim);
//   ... change mode ...
//   PublishTarget(sim, "shooter", "teleop", shooter::ShooterTarget{60.0},
//                 before);
//
// The mode generation of the newest packet on /hw/state/driver_station: its
// mode_since_us when its flags name a mode, 0 when they do not or there has
// been none.
std::uint64_t DriverStationGeneration(const Sim& sim);
void PublishTargetRaw(Sim& sim, std::string_view subsystem,
                      std::string_view source, const void* message,
                      std::size_t bytes, std::size_t alignment,
                      std::uint64_t mode_since_us);
// The newest envelope on the lane: its message copied into `out` and its
// header into `header`, or false when the lane has had none.
bool LatestTargetRaw(const Sim& sim, std::string_view subsystem,
                     std::string_view source, void* out, std::size_t bytes,
                     std::size_t alignment,
                     detail::TargetEnvelopeHeader& header);

template <Message M>
void PublishTarget(Sim& sim, std::string_view subsystem,
                   std::string_view source, const M& message,
                   std::uint64_t mode_since_us) {
  PublishTargetRaw(sim, subsystem, source, &message, sizeof(M), alignof(M),
                   mode_since_us);
}
template <Message M>
void PublishTarget(Sim& sim, std::string_view subsystem,
                   std::string_view source, const M& message) {
  PublishTarget(sim, subsystem, source, message, DriverStationGeneration(sim));
}
template <Message M>
void ReleaseTarget(Sim& sim, std::string_view subsystem,
                   std::string_view source) {
  PublishTargetRaw(sim, subsystem, source, nullptr, sizeof(M), alignof(M),
                   DriverStationGeneration(sim));
}
// The newest envelope on /<subsystem>/target/<source>, or nullopt.
template <Message M>
std::optional<TargetEnvelope<M>> LatestTarget(const Sim& sim,
                                              std::string_view subsystem,
                                              std::string_view source) {
  TargetEnvelope<M> envelope{};
  detail::TargetEnvelopeHeader header;
  if (!LatestTargetRaw(sim, subsystem, source, &envelope.message, sizeof(M),
                       alignof(M), header)) {
    return std::nullopt;
  }
  envelope.set_mode_since_us(header.mode_since_us);
  envelope.active = header.active ? 1 : 0;
  return envelope;
}

}  // namespace talos::testing
