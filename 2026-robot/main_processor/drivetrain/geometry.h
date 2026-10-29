#pragma once

// Builds SwerveGeometry from [subsystems.<name>.geometry] in the robot
// configuration. Modules name their devices, and every name is checked
// against this subsystem's devices here, so a typo or a crossed encoder fails
// at startup rather than as one module chasing another's azimuth.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "drivetrain/swerve.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace drivetrain {
namespace detail {

inline std::string RequiredString(const toml::table& table, const char* key,
                                  const std::string& where) {
  if (auto value = table[key].value<std::string_view>())
    return std::string{*value};
  throw std::invalid_argument(where + ": missing string field '" +
                              std::string(key) + "'");
}

inline double RequiredPositive(const toml::table& table, const char* key,
                               const std::string& where) {
  auto value = table[key].value<double>();
  if (!value || !std::isfinite(*value) || *value <= 0)
    throw std::invalid_argument(where + ": '" + std::string(key) +
                                "' must be a positive, finite number");
  return *value;
}

inline double RequiredFinite(const toml::table& table, const char* key,
                             const std::string& where) {
  auto value = table[key].value<double>();
  if (!value || !std::isfinite(*value))
    throw std::invalid_argument(where + ": '" + std::string(key) +
                                "' must be a finite number");
  return *value;
}

// `subsystem`'s device `name` from `all` (motors or sensors), restricted to
// the ids the subsystem owns. nullptr when it has none by that name.
template <class Device>
const Device* Owned(const std::vector<Device>& all,
                    const std::vector<std::uint16_t>& owned,
                    std::string_view name) {
  for (const Device& device : all)
    if (device.name == name &&
        std::find(owned.begin(), owned.end(), device.id) != owned.end())
      return &device;
  return nullptr;
}

}  // namespace detail

// `subsystem`'s motor `name`; throws when it owns none by that name.
inline const talos::hardware::MotorConfig& SubsystemMotor(
    const talos::config::RobotConfig& config, std::string_view subsystem,
    std::string_view name) {
  const talos::hardware::Devices* owned =
      config.GetDevices(std::string{subsystem});
  const auto* motor =
      owned ? detail::Owned(config.hardware.motors, owned->motors, name)
            : nullptr;
  if (motor == nullptr)
    throw std::invalid_argument(std::string{subsystem} + ": no motor named '" +
                                std::string{name} + "'");
  return *motor;
}

// `subsystem`'s CANcoder `name`; throws when it owns none by that name.
inline const talos::hardware::SensorConfig& SubsystemCANcoder(
    const talos::config::RobotConfig& config, std::string_view subsystem,
    std::string_view name) {
  const talos::hardware::Devices* owned =
      config.GetDevices(std::string{subsystem});
  const auto* sensor =
      owned ? detail::Owned(config.hardware.sensors, owned->sensors, name)
            : nullptr;
  if (sensor == nullptr ||
      sensor->kind != talos::hardware::SensorKind::kCANcoder)
    throw std::invalid_argument(std::string{subsystem} +
                                ": no CANcoder named '" + std::string{name} +
                                "'");
  return *sensor;
}

inline SwerveGeometry BuildSwerveGeometry(
    const talos::config::RobotConfig& config, std::string_view subsystem,
    const toml::table& geometry) {
  const double default_radius_m =
      detail::RequiredPositive(geometry, "wheel_radius_m", "swerve geometry");
  const auto* modules = geometry["modules"].as_array();
  if (!modules)
    throw std::invalid_argument("swerve geometry: missing 'modules' array");
  if (modules->size() != kModuleCount)
    throw std::invalid_argument(
        "swerve geometry: expected " + std::to_string(kModuleCount) +
        " modules, found " + std::to_string(modules->size()));

  SwerveGeometry out{};
  std::set<std::uint16_t> claimed;  // logical ids are unique robot-wide
  auto claim = [&](std::uint16_t id, const std::string& what) {
    if (!claimed.insert(id).second)
      throw std::invalid_argument(what + " is already used by another module");
  };
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    const auto* entry = modules->get(i)->as_table();
    if (!entry)
      throw std::invalid_argument("swerve geometry: module " +
                                  std::to_string(i) + " is not a table");
    SwerveModule& module = out.modules[i];
    module.name = detail::RequiredString(*entry, "name",
                                         "swerve module " + std::to_string(i));
    const std::string where = "swerve module '" + module.name + "'";
    module.x_m = detail::RequiredFinite(*entry, "x_m", where);
    module.y_m = detail::RequiredFinite(*entry, "y_m", where);
    // Per-module override exists so one odd wheel does not force the whole
    // drivetrain onto a fictional average radius.
    module.wheel_radius_m =
        (*entry)["wheel_radius_m"]
            ? detail::RequiredPositive(*entry, "wheel_radius_m", where)
            : default_radius_m;
    module.drive = detail::RequiredString(*entry, "drive", where);
    module.steer = detail::RequiredString(*entry, "steer", where);
    const std::string encoder =
        detail::RequiredString(*entry, "encoder", where);

    const auto& drive = SubsystemMotor(config, subsystem, module.drive);
    const auto& steer = SubsystemMotor(config, subsystem, module.steer);
    const auto& cancoder = SubsystemCANcoder(config, subsystem, encoder);
    claim(drive.id, where + ": motor '" + module.drive + "'");
    claim(steer.id, where + ": motor '" + module.steer + "'");
    claim(cancoder.id, where + ": encoder '" + encoder + "'");
    module.max_drive_rps = drive.max_velocity_rps;
    if (!steer.continuous_wrap)
      throw std::invalid_argument(where +
                                  ": steer motor needs continuous_wrap = true");
    // The steer motor closes its position loop on the module's own encoder.
    if (steer.feedback == talos::hardware::Feedback::kRemoteCANcoder &&
        steer.feedback_sensor_id != cancoder.id)
      throw std::invalid_argument(
          where +
          ": steer motor's feedback sensor is not this module's encoder");
  }
  return out;
}

inline SwerveGeometry BuildSwerveGeometry(
    const talos::config::RobotConfig& config, std::string_view subsystem) {
  const std::string name{subsystem};
  const toml::table* geometry = config.GetSubsystemTable(name, "geometry");
  if (!geometry)
    throw std::invalid_argument("configuration is missing [subsystems." + name +
                                ".geometry]");
  return BuildSwerveGeometry(config, subsystem, *geometry);
}

// The geometry of the subsystem `node` runs as, from its config.
inline SwerveGeometry LoadSwerveGeometry(talos::Node& node) {
  return BuildSwerveGeometry(node.extension<talos::HardwareLink>().config(),
                             node.name());
}

}  // namespace drivetrain
