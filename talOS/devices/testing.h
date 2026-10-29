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

#include <memory>
#include <string_view>

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
  // A disconnected device keeps its slot in /hw/state with valid == false
  // and zero values, so its handle reads connected() == false. Every device
  // of that node named `device` is affected, whatever its kind; an unknown
  // name throws std::invalid_argument.
  void set_connected(std::string_view node, std::string_view device,
                     bool connected);

  // The Driver Station packet to forward from now on, stamped with the sim
  // time as the Rio stamps it.
  void set_driver_station(const driver_station::DriverStationData& data);

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

}  // namespace talos::testing
