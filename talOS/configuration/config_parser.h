#pragma once

#include <string>
#include <iostream>
#include "toml++/toml.hpp"

namespace talos::config {

enum class MotorType {
    kUnknown = 0,
    TalonFX
};

struct MotorConfig {
    MotorType type;
    std::string name;
    bool inverted;
    double supply_current;
    double stator_current;
};

inline MotorConfig ParseSubsystemToml(const std::string& tomlPath) {
    toml::table config = toml::parse_file(tomlPath);

    MotorType type = MotorType::TalonFX;
    std::string name = "test";
    std::optional<bool> inverted = config["motors"]["left_leader"]["is_inverted"].value<bool>();
    std::optional<double> supply_current = config["motors"]["left_leader"]["supply_current"].value<double>();
    std::optional<double> stator_current = config["motors"]["left_leader"]["stator_current"].value<double>();
    return {
        .type = type,
        .name = name,
        .inverted = inverted.has_value() ? inverted.value() : false,
        .supply_current = supply_current.has_value() ? supply_current.value() : 0,
        .stator_current= stator_current.has_value() ? stator_current.value() : 0,
    };
};
} /* namespace talos::config */
