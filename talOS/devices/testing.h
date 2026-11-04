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
// --sim), and flags configured|enabled (configured only while disabled, the
// default). Once set_driver_station() has been called it also forwards that
// Driver Station packet on /hw/state/driver_station every period. Construct
// it before the first step to have a state at time 0; later, the first state
// goes out at that moment. Devices are keyed by (node, device) exactly as the
// node names them.
//
// Each period it also publishes the bridge's /hw/status: the states and
// Driver Station packets it sent, configured, and the gateway clock mapping
// of the bridge's own filter (talOS/bridge/clock_offset.h) fed with each
// state at the sim time it goes out; the other counters stay 0. So, as on a
// robot, HardwareLink::sample_time() is nullopt until the
// ClockOffsetEstimator::kConvergedSamples-th state (40 ms at the default
// [robot] period_us = 5000) and then maps each state to the sim time of the
// gateway sample, plus the smallest latency the filter has seen.

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
  // instantly (velocity integrates into position).
  void ideal_motors(bool on = true);

  // Feedback.
  void set_motor(std::string_view node, std::string_view device,
                 double position, double velocity);
  void set_digital_input(std::string_view node, std::string_view device,
                         bool value);
  void set_analog_input(std::string_view node, std::string_view device,
                        double volts);
  void set_encoder(std::string_view node, std::string_view device,
                   double position, double velocity);
  void set_cancoder(std::string_view node, std::string_view device,
                    double position, double velocity);
  void set_gyro(std::string_view node, std::string_view device, double yaw,
                double yaw_rate);
  // The RoboRIO's clock and link: from now on the gateway clock reads sim
  // time + `gateway_offset` (states and Driver Station packets are stamped
  // with it), and each state is sampled `latency` before it goes out.
  // Defaults 0 and 0. A latency that grows leaves the filter's smaller
  // minimum in place for its window, so sample_time() reads that much
  // earlier than the publication; one that grows by a period or more makes
  // the next states no newer than the last, and they are dropped as on a
  // robot. Throws std::invalid_argument for a negative latency or a gateway
  // clock that would read before 0.
  void set_gateway_clock(Duration gateway_offset, Duration latency);
  // A disconnected device keeps its slot in /hw/state with valid == false
  // and zero values, so its handle reads connected() == false. Every device
  // of that node named `device` is affected, whatever its kind; an unknown
  // name throws std::invalid_argument.
  void set_connected(std::string_view node, std::string_view device,
                     bool connected);

  // The Driver Station packet to forward from now on, stamped with the sim
  // time as the Rio stamps it, and with its mode generation by one
  // driver_station::ModeTracker, as the Rio's sampler does.
  void set_driver_station(const driver_station::DriverStationData& data);
  // Stops forwarding: the Driver Station goes silent, as when its link
  // drops, while /hw/state keeps coming.
  void stop_driver_station();

  // Outputs: the last request flushed by that node for that device (neutral,
  // false and 0 before any).
  hardware::MotorRequest motor_request(std::string_view node,
                                       std::string_view device) const;
  bool digital_output(std::string_view node, std::string_view device) const;
  double pwm(std::string_view node, std::string_view device) const;

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
