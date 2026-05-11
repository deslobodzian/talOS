#pragma once

#include <exception>
#include <string>
#include <iostream>
#include "toml.hpp"

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

inline std::vector<MotorConfig> ParseMotorConfigs(const toml::node_view<toml::node>& motors){
    std::vector<MotorConfig> motor_configs;
    if (auto* motors_table = motors.as_table()) {
        for (auto&& [key, value] : *motors_table) {
            try {
                if (auto* motor_data = value.as_table()) {
                    MotorConfig mc;
                    mc.name = key.str();
                    mc.inverted = (*motor_data)["is_inverted"].value_or(false);
                    mc.supply_current = (*motor_data)["supply_current"].value_or(0.0);
                    mc.stator_current = (*motor_data)["stator_current"].value_or(0.0);

                    motor_configs.push_back(mc);
                }
            } catch (const std::exception& e) {
                std::cerr << "Error parsing motor " << key << ": " << e.what() << "\n";
            }
        }
    }

    for (const auto& config : motor_configs) {
        std::cout << config.name << "\n";
    }
    //for (const auto& motor : motors) {
    //    MotorTyp type = otorType::TalonFX;
    //    std::string name = "test";
    //    std::optional<bool> inverted = motor["is_inverted"].value<bool>();
    //    std::optional<double> supply_current = motor["supply_current"].value<double>();
    //    std::optional<double> stator_current = motor["stator_current"].value<double>();
    //    motor_configs.push_back({
    //        .type = type,
    //        .name = name,
    //        .inverted = inverted.has_value() ? inverted.value() : false,
    //        .supply_current = supply_current.has_value() ? supply_current.value() : 0,
    //        .stator_current= stator_current.has_value() ? stator_current.value() : 0,
    //    });
    //}
    return motor_configs;
};

inline MotorConfig ParseSubsystemToml(const std::string& tomlPath) {
    toml::table config = toml::parse_file(tomlPath);
    auto test = ParseMotorConfigs(config["motors"]);

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
