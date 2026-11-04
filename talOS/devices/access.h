#pragma once

// Plumbing behind the hardware handles: what the handles, SimHardware and the
// C ABI (talOS/node_api) drive. Node code never uses it. Every device
// operation is non-template and addresses a device by the index add_device
// returned.

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

#include "talOS/events/time.h"
#include "talOS/hardware/messages.h"

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

// The Driver Station mode a target policy is keyed by: none while the DS is
// stale, detached, disabled or e-stopped.
enum class DsMode : std::uint8_t { kNone, kTeleop, kAutonomous, kTest };

struct DriverStationAccess {
  static DsMode mode(const DriverStation& ds);
  // When the current mode began: the receipt time of the first packet
  // observed in it.
  static event::MonotonicTime mode_since(const DriverStation& ds);
};

// Target and TargetLane (target.h) over message bytes, so the C ABI drives
// the same code. Indexes address this node's targets and lanes.
struct TargetAccess {
  // Subscribes to every lane of the node's policy and the Driver Station.
  // `bytes` and `alignment` are the message's (sizeof, alignof).
  static std::size_t open_target(Node& node, std::size_t bytes,
                                 std::size_t alignment);
  // The selected message (`bytes` long), or null; `source` is the selected
  // source's index in target_sources, or -1.
  static const void* target_get(Node& node, std::size_t target,
                                std::int32_t& source);
  // The policy's sources in subscription order.
  static std::span<const std::string> target_sources(Node& node,
                                                     std::size_t target);
  // The message size the target was opened with.
  static std::size_t target_bytes(Node& node, std::size_t target);

  static std::size_t open_lane(Node& node, std::string_view subsystem,
                               std::string_view source, std::size_t bytes,
                               std::size_t alignment);
  static bool lane_send(Node& node, std::size_t lane, const void* message,
                        std::size_t bytes);
  static bool lane_release(Node& node, std::size_t lane);
};

// sizeof(TargetEnvelope<M>) for a message of `bytes` and `alignment`. Throws
// std::invalid_argument unless alignment is a power of two at most 8 that
// divides bytes.
std::size_t TargetEnvelopeBytes(std::size_t bytes, std::size_t alignment);
// Writes the envelope: active, zero padding, then the message (zeros when
// `message` is null). `out` holds TargetEnvelopeBytes(bytes, alignment).
void EncodeTargetEnvelope(bool active, const void* message, std::size_t bytes,
                          std::size_t alignment, std::span<std::byte> out);

}  // namespace talos::detail
