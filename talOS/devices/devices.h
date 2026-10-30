#pragma once

// Robot hardware for talOS nodes: motors, sensors and I/O reached through the
// RoboRIO bridge (//talOS/bridge:hardware_node), and the Driver Station it
// forwards.
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
// latched with their receive time; getters read the latched state (0 before
// the first one), while connected() requires both a valid sample and a
// hardware state younger than [robot] command_timeout_us.

#include <cstddef>
#include <functional>
#include <memory>
#include <string_view>

#include "talOS/devices/access.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/node/node.h"

namespace talos {

namespace detail {
class DriverStationLink;  // devices.cc
}  // namespace detail

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
  // Accepted hardware state younger than [robot] command_timeout_us.
  bool fresh() const;
  // Age of the last accepted hardware state (Duration::max() if none).
  Duration age() const;
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
  bool connected() const;   // sample valid and hardware state fresh

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
  bool connected() const;   // sample valid and hardware state fresh

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
  bool connected() const;   // sample valid and hardware state fresh

 private:
  HardwareLink* link_{nullptr};
  std::size_t index_{0};
};

class DigitalInput {
 public:
  DigitalInput() = default;
  DigitalInput(Node& node, std::string_view name);
  bool get() const;
  bool connected() const;  // sample valid and hardware state fresh

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
  bool connected() const;  // sample valid and hardware state fresh

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
  bool connected() const;   // sample valid and hardware state fresh

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

// One Driver Station joystick as a DriverStation handle saw it. A value: it
// does not change after it is read.
class Joystick {
 public:
  Joystick() = default;  // disconnected

  bool connected() const { return data_.connected; }
  int axis_count() const { return data_.axis_count; }
  int button_count() const { return data_.button_count; }
  int pov_count() const { return data_.pov_count; }
  double axis(int index) const;   // [-1, 1]; 0 past axis_count
  bool button(int number) const;  // 1-based, as the DS numbers them
  int pov(int index = 0) const;   // degrees clockwise from up; -1 centred
  std::string_view name() const;

 private:
  friend class DriverStation;
  explicit Joystick(const driver_station::JoystickData& data) : data_{data} {}
  driver_station::JoystickData data_{};
};

// The Driver Station packet the bridge forwards on /hw/state/driver_station
// (talos::hardware::kDriverStationTopic), decoded and latched. Take it in the
// constructor; every handle on a node shares one subscription.
//
//   explicit Shooter(talos::Node& n) : ds{n} {}
//   void update() { if (ds.teleop() && ds.joystick(0).button(1)) ... }
//
// Mode getters are false unless connected(), and joystick() is disconnected,
// so a silent Driver Station reads as disabled rather than as its last word.
class DriverStation {
 public:
  DriverStation() = default;
  explicit DriverStation(Node& node);
  // on_packet runs as each packet arrives, once it is latched (corrupt
  // packets are dropped and run nothing).
  DriverStation(Node& node, std::function<void()> on_packet);

  // A packet younger than [robot] command_timeout_us says the DS is
  // attached.
  bool connected() const;
  bool enabled() const;
  bool autonomous() const;
  bool teleop() const;
  bool test() const;
  bool estop() const;
  bool fms_attached() const;

  driver_station::Alliance alliance() const;  // kUnknown until the first
  int station() const;                        // 1-3; 0 unknown
  double match_time() const;                  // seconds, as the DS counts

  Joystick joystick(std::size_t port) const;  // port < kMaxJoysticks

  // The latest decoded packet, as sent; zeros until the first.
  const driver_station::DriverStationData& data() const;

 private:
  bool flag(driver_station::DsFlags flag) const;
  detail::DriverStationLink* link_{nullptr};
};

}  // namespace talos
