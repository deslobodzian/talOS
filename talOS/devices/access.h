#pragma once

// Plumbing behind the hardware handles: what the handles, SimHardware and the
// C ABI (talOS/node_api) drive. Node code never uses it. Every device
// operation is non-template and addresses a device by the index add_device
// returned.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "talOS/devices/target_detail.h"
#include "talOS/events/time.h"
#include "talOS/hardware/messages.h"
#include "talOS/schema/identity.h"

namespace talos {
class DriverStation;
class HardwareLink;
class Node;
namespace config {
struct RobotConfig;
}  // namespace config
}  // namespace talos

namespace talos::detail {

enum class DeviceKind : std::uint8_t {
  kMotor,
  kCANcoder,
  kGyro,
  kDigitalInput,
  kDigitalOutput,
  kAnalogInput,
  kEncoder,
  kPwm,
};

struct HardwareAccess {
  // The logical id of `subsystem`'s device `name` of `kind`. Throws
  // std::invalid_argument naming the device, and listing what the subsystem
  // has of that kind, when there is no such device.
  static std::uint16_t device_id(const config::RobotConfig& config,
                                 std::string_view subsystem, DeviceKind kind,
                                 std::string_view name);

  static std::size_t add_device(HardwareLink& link, DeviceKind kind,
                                std::string_view name);
  static void motor_set(HardwareLink& link, std::size_t index,
                        hardware::Mode mode, double demand, double ff_volts,
                        int slot);
  static std::uint64_t motor_clamped_count(const HardwareLink& link,
                                           std::size_t index);
  static void motor_rezero(HardwareLink& link, std::size_t index,
                           double position_rot);
  static void gyro_rezero(HardwareLink& link, std::size_t index,
                          double yaw_rot);
  // Whether a rezero holds: set by one that took, cleared as Motor::zeroed
  // describes (boot change, a reset masking the device in the next State,
  // any reset across a sequence gap).
  static bool motor_zeroed(const HardwareLink& link, std::size_t index);
  static bool gyro_zeroed(const HardwareLink& link, std::size_t index);
  static std::optional<event::MonotonicTime> motor_time(
      const HardwareLink& link, std::size_t index, bool velocity);
  static std::optional<event::MonotonicTime> sensor_time(
      const HardwareLink& link, std::size_t index, bool velocity);
  // Motor::position_age / velocity_age, CANcoder::position_age / velocity_age.
  static std::optional<event::Duration> motor_age(const HardwareLink& link,
                                                  std::size_t index,
                                                  bool velocity);
  static std::optional<event::Duration> sensor_age(const HardwareLink& link,
                                                   std::size_t index,
                                                   bool velocity);
  // The latched sample for a device; all zeros (valid == false) until an
  // accepted hardware state carries it. A motor's, CAN sensor's or encoder's
  // invalid sample carries the last valid position instead of its own
  // (devices.h); a motor's and CAN sensor's position is in the rezero frame,
  // and on a stale state its sample is invalid with MarkInvalid's zeros.
  static hardware::MotorSample motor_sample(const HardwareLink& link,
                                            std::size_t index);
  // Motor::forward_limit / reverse_limit: the sample's hardware-limit fault
  // bit, false unless the sample is valid and the hardware state fresh.
  static bool motor_hard_limit(const hardware::MotorSample& sample,
                               bool forward);
  static bool motor_hard_limit(const HardwareLink& link, std::size_t index,
                               bool forward);
  static hardware::SensorSample sensor_sample(const HardwareLink& link,
                                              std::size_t index);
  static hardware::DigitalInputSample digital_input_sample(
      const HardwareLink& link, std::size_t index);
  static hardware::AnalogInputSample analog_input_sample(
      const HardwareLink& link, std::size_t index);
  // Encoder::position / velocity / connected: an invalid sample, or any
  // sample on a stale link, reads valid false, the last valid position and
  // velocity 0.
  static hardware::EncoderSample encoder_sample(const HardwareLink& link,
                                                std::size_t index);
  static void digital_output_set(HardwareLink& link, std::size_t index,
                                 bool value);
  static void pwm_set(HardwareLink& link, std::size_t index, double output);
  static void pwm_stop(HardwareLink& link, std::size_t index);
  // A talos::Target of the link's node returned a target inside the
  // update() running now: the flush right after it may send that update()'s
  // outputs (devices.h). Target::get() calls it on every selecting call made
  // inside update().
  static void note_live_target(HardwareLink& link);
};

// The Driver Station mode a target policy is keyed by: none while the DS is
// stale, detached, disabled or e-stopped.
enum class DsMode : std::uint8_t { kNone, kTeleop, kAutonomous, kTest };

// The mode a packet's flags name (driver_station::ModeKey), as if it were
// fresh: none when detached, disabled or e-stopped.
DsMode DsModeOf(std::uint32_t flags);

struct DriverStationAccess {
  // An internal handle for Target, TargetLane and OperatorArming plumbing:
  // shares the node's DriverStationLink without marking the node as an
  // explicit Driver Station reader (REQ-SAFE-14).
  static DriverStation internal(Node& node);
  static DsMode mode(const DriverStation& ds);
  // The current mode generation: the latest packet's mode_since_us while
  // mode() is not none, else 0.
  static std::uint64_t generation(const DriverStation& ds);
  // How many accepted packets arrived [robot] command_timeout_us or more
  // after the one before (by publication time): each is a recovery from a
  // stale link, whether or not anything sampled it while stale.
  static std::uint64_t recoveries(const DriverStation& ds);
};

}  // namespace talos::detail
