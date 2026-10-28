#pragma once

// Plumbing behind the hardware handles: what the handles, SimHardware and the
// C ABI (talOS/node_api) drive. Node code never uses it. Every device
// operation is non-template and addresses a device by the index add_device
// returned.

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "talOS/hardware/messages.h"

namespace talos {
class HardwareLink;
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
  // The latched sample for a device; all zeros (valid == false) until an
  // accepted hardware state carries it.
  static hardware::MotorSample motor_sample(const HardwareLink& link,
                                            std::size_t index);
  static hardware::SensorSample sensor_sample(const HardwareLink& link,
                                              std::size_t index);
  static hardware::DigitalInputSample digital_input_sample(
      const HardwareLink& link, std::size_t index);
  static hardware::AnalogInputSample analog_input_sample(
      const HardwareLink& link, std::size_t index);
  static hardware::EncoderSample encoder_sample(const HardwareLink& link,
                                                std::size_t index);
  static void digital_output_set(HardwareLink& link, std::size_t index,
                                 bool value);
  static void pwm_set(HardwareLink& link, std::size_t index, double output);
};

}  // namespace talos::detail
