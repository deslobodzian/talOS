#pragma once

// Robot hardware for talOS nodes: motors, sensors and I/O reached through the
// RoboRIO bridge (//talOS/bridge:hardware_node).
//
//   #include "talOS/devices/devices.h"
//
//   struct Intake {
//     explicit Intake(talos::Node& n) : roller{n, "roller"}, beam{n, "beam"} {}
//     void update() {
//       if (!beam.get()) roller.set_voltage(6.0);
//     }
//     talos::Motor roller;
//     talos::DigitalInput beam;
//   };
//
// Handles are named by their device key in this subsystem's config
// ([motors.roller] is "roller") and are taken in the constructor.
//
// A layer over talos::Node's public API, nothing more: the first handle
// attaches this node's HardwareLink (node.extension<HardwareLink>()), which
// subscribes to /hw/state and, on the first output, publishes
// /hw/request/<name> and flushes after every update():
//   - one command per tick once a hardware state has been accepted, echoing
//     that state's identity words like hardware::NeutralCommand;
//   - every owned motor neutral unless set since the last flush, and all of
//     them neutral whenever robot_enabled() is false;
//   - digital/PWM outputs set since the last flush included as set.
// Incoming states pass hardware::AcceptState for this config's id and are
// latched with their receive time; getters read the latched state, and read
// 0 with connected() == false until the first one.

#include <cstddef>
#include <memory>
#include <string_view>

#include "talOS/devices/access.h"
#include "talOS/node/node.h"

namespace talos {

// One node's end of the hardware link, attached as node.extension<
// HardwareLink>() by the first handle.
class HardwareLink {
 public:
  explicit HardwareLink(Node& node);
  ~HardwareLink();
  HardwareLink(const HardwareLink&) = delete;
  HardwareLink& operator=(const HardwareLink&) = delete;

  // Accepted hardware state, gateway configured and enabled with no fault,
  // and that state younger than [robot] command_timeout_us.
  bool enabled() const;
  const config::RobotConfig& config() const;  // parsed node.config_path()

 private:
  friend struct detail::HardwareAccess;  // the handles' side: access.h
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Whether this node's outputs reach the hardware: see HardwareLink::enabled.
// The first call attaches the link, so make it (or take a handle) in the
// constructor.
bool robot_enabled(Node& node);

// Hardware handles: cheap copyable values. Commands apply to the next flush;
// anything not set by then is neutral. Feedback is in mechanism units
// (sensor_to_mechanism_ratio is applied on the controller).
class Motor {
 public:
  Motor() = default;
  Motor(Node& node, std::string_view name);

  void set_duty_cycle(double fraction);  // [-1, 1]
  void set_voltage(double volts);
  void set_velocity(double rps, double ff_volts = 0, int slot = 0);  // PID
  void set_position(double rot, double ff_volts = 0, int slot = 0);  // PID
  void set_motion_magic(double rot, double ff_volts = 0, int slot = 0);
  void stop();  // neutral

  double position() const;  // rotations
  double velocity() const;  // rotations/s
  double voltage() const;   // applied volts
  double current() const;   // stator amps
  bool connected() const;   // sample present and valid

 private:
  HardwareLink* link_{nullptr};
  std::size_t index_{0};
};

class CANcoder {
 public:
  CANcoder() = default;
  CANcoder(Node& node, std::string_view name);
  double position() const;  // rotations
  double velocity() const;  // rotations/s
  bool connected() const;

 private:
  HardwareLink* link_{nullptr};
  std::size_t index_{0};
};

class Gyro {  // Pigeon2
 public:
  Gyro() = default;
  Gyro(Node& node, std::string_view name);
  double yaw() const;       // rotations, CCW+
  double yaw_rate() const;  // rotations/s, CCW+
  bool connected() const;

 private:
  HardwareLink* link_{nullptr};
  std::size_t index_{0};
};

class DigitalInput {
 public:
  DigitalInput() = default;
  DigitalInput(Node& node, std::string_view name);
  bool get() const;
  bool connected() const;

 private:
  HardwareLink* link_{nullptr};
  std::size_t index_{0};
};

class DigitalOutput {
 public:
  DigitalOutput() = default;
  DigitalOutput(Node& node, std::string_view name);
  void set(bool value);

 private:
  HardwareLink* link_{nullptr};
  std::size_t index_{0};
};

class AnalogInput {
 public:
  AnalogInput() = default;
  AnalogInput(Node& node, std::string_view name);
  double voltage() const;
  bool connected() const;

 private:
  HardwareLink* link_{nullptr};
  std::size_t index_{0};
};

class Encoder {
 public:
  Encoder() = default;
  Encoder(Node& node, std::string_view name);
  double position() const;  // rotations
  double velocity() const;  // rotations/s
  bool connected() const;

 private:
  HardwareLink* link_{nullptr};
  std::size_t index_{0};
};

class PwmOutput {
 public:
  PwmOutput() = default;
  PwmOutput(Node& node, std::string_view name);
  void set(double output);

 private:
  HardwareLink* link_{nullptr};
  std::size_t index_{0};
};

}  // namespace talos
