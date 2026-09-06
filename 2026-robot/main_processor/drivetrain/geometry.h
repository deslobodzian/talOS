#pragma once

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>

#include "2026-robot/main_processor/drivetrain/swerve.h"
#include "talOS/configuration/config_parser.h"

// Builds SwerveGeometry from [subsystems.<name>.geometry] in the robot
// configuration. Modules name their devices, and the logical ids the swerve
// math needs are resolved here -- so renaming or adding a device renumbers the
// ids and this still resolves, where a hand-written id table would silently
// point at the wrong motor.
namespace talos::drive {
namespace detail {

inline std::string_view RequiredString(const toml::table& table,
                                       const char* key,
                                       const std::string& where) {
  if (auto value = table[key].value<std::string_view>()) return *value;
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

inline uint16_t MotorIdByName(const hardware::Config& config,
                              std::string_view name, const std::string& where,
                              const char* role) {
  for (const auto& motor : config.motors)
    if (motor.name == name) return motor.id;
  throw std::invalid_argument(where + ": " + role + " '" + std::string(name) +
                              "' is not a motor in this subsystem");
}

inline uint16_t SensorIdByName(const hardware::Config& config,
                               std::string_view name,
                               const std::string& where) {
  for (const auto& sensor : config.sensors)
    if (sensor.name == name) return sensor.id;
  throw std::invalid_argument(where + ": encoder '" + std::string(name) +
                              "' is not a sensor in this subsystem");
}

}  // namespace detail

inline SwerveGeometry BuildSwerveGeometry(const hardware::Config& config,
                                          const toml::table& geometry) {
  const double default_radius_m =
      detail::RequiredPositive(geometry, "wheel_radius_m", "swerve geometry");
  const auto* modules = geometry["modules"].as_array();
  if (!modules)
    throw std::invalid_argument("swerve geometry: missing 'modules' array");
  if (modules->size() != kSwerveModuleCount)
    throw std::invalid_argument(
        "swerve geometry: expected " + std::to_string(kSwerveModuleCount) +
        " modules, found " + std::to_string(modules->size()));

  SwerveGeometry out{};
  std::set<uint16_t> claimed;
  auto claim = [&](uint16_t id, const std::string& where) {
    if (!claimed.insert(id).second)
      throw std::invalid_argument(where +
                                  ": device is already used by another module");
  };

  for (std::size_t i = 0; i < kSwerveModuleCount; ++i) {
    const auto* entry = modules->get(i)->as_table();
    if (!entry)
      throw std::invalid_argument("swerve geometry: module " +
                                  std::to_string(i) + " is not a table");
    const auto name = detail::RequiredString(
        *entry, "name", "swerve module " + std::to_string(i));
    const std::string where = "swerve module '" + std::string(name) + "'";

    auto& module = out.modules[i];
    if (name.size() >= kMaxSwerveModuleName)
      throw std::invalid_argument(where + ": name must be under " +
                                  std::to_string(kMaxSwerveModuleName) +
                                  " characters");
    std::copy(name.begin(), name.end(), module.name.begin());
    module.x_m = detail::RequiredFinite(*entry, "x_m", where);
    module.y_m = detail::RequiredFinite(*entry, "y_m", where);
    // Per-module override exists so one odd wheel does not force the whole
    // drivetrain onto a fictional average radius.
    module.wheel_radius_m =
        (*entry)["wheel_radius_m"]
            ? detail::RequiredPositive(*entry, "wheel_radius_m", where)
            : default_radius_m;
    module.drive_id = detail::MotorIdByName(
        config, detail::RequiredString(*entry, "drive", where), where, "drive");
    module.steer_id = detail::MotorIdByName(
        config, detail::RequiredString(*entry, "steer", where), where, "steer");
    module.encoder_id = detail::SensorIdByName(
        config, detail::RequiredString(*entry, "encoder", where), where);
    claim(module.drive_id, where);
    claim(module.steer_id, where);
    claim(module.encoder_id, where);

    // The steer motor closes its position loop on the module's own encoder.
    // Catching a crossed reference here beats watching one module chase
    // another module's azimuth on the field.
    for (const auto& motor : config.motors) {
      if (motor.id != module.steer_id) continue;
      if (motor.feedback == hardware::Feedback::kRemoteCANcoder &&
          motor.feedback_sensor_id != module.encoder_id)
        throw std::invalid_argument(
            where +
            ": steer motor's feedback sensor is not this module's encoder");
    }
  }
  ValidateSwerve(config, out);
  return out;
}

inline SwerveGeometry BuildSwerveGeometry(
    const config::RobotConfig& robot_config,
    const std::string& subsystem = "drivetrain") {
  const auto* geometry = robot_config.GetSubsystemTable(subsystem, "geometry");
  if (!geometry)
    throw std::invalid_argument("configuration is missing [subsystems." +
                                subsystem + ".geometry]");
  return BuildSwerveGeometry(robot_config.hardware, *geometry);
}

}  // namespace talos::drive
