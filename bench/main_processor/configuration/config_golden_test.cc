#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

#include "talOS/configuration/config_parser.h"

namespace {
using talos::hardware::Feedback;
using talos::hardware::MotorConfig;
using talos::hardware::MotorType;
using talos::hardware::SensorConfig;
using talos::hardware::SensorKind;

struct ExpectedMotor {
  const char* name;
  int can_id;
  MotorType type;
  double max_voltage;
  double supply_limit_a;
  double stator_limit_a;
};

struct ExpectedSensor {
  const char* name;
  int can_id;
  SensorKind kind;
};

// The devices on the bench, as read from them: every one on the CANivore
// named Shooter.
constexpr const char* kBus = "Shooter";
constexpr ExpectedMotor kMotors[] = {
    {"kraken", 1, MotorType::kKrakenX44, 2.0, 30.0, 40.0},
    {"drive", 5, MotorType::kKrakenX60, 4.0, 20.0, 40.0},
    {"steer", 6, MotorType::kKrakenX44, 4.0, 20.0, 40.0},
};
constexpr ExpectedSensor kSensors[] = {
    {"encoder", 3, SensorKind::kCANcoder},
    {"pigeon", 0, SensorKind::kPigeon2},
};

template <typename Devices>
const typename Devices::value_type* Find(const Devices& devices,
                                         std::string_view name) {
  const auto it = std::find_if(devices.begin(), devices.end(),
                               [&](const auto& d) { return d.name == name; });
  return it == devices.end() ? nullptr : &*it;
}

// Every bench config stays uncommissioned (RULES.md 4) and names the
// bench's devices at their own bus and CAN ids, with the conservative limits
// of its first supervised spins; an accidental id, bus, kind, limit or
// `commissioned` change fails here before it reaches the hardware.
TEST(BenchConfig, UncommissionedDevicesAtTheirBusAndIds) {
  for (const auto* path :
       {"bench/main_processor/configuration/robot.toml",
        "bench/main_processor/configuration/robot_bench.toml",
        "bench/main_processor/configuration/robot_py.toml"}) {
    SCOPED_TRACE(path);
    const auto config = talos::config::ParseRobotConfig(path);
    EXPECT_FALSE(config.hardware.commissioned);
    EXPECT_TRUE(config.require_target_coverage);
    EXPECT_EQ(config.hardware.command_timeout_us, 100000u);
    ASSERT_EQ(config.hardware.motors.size(), std::size(kMotors));
    ASSERT_EQ(config.hardware.sensors.size(), std::size(kSensors));
    for (const ExpectedMotor& want : kMotors) {
      SCOPED_TRACE(want.name);
      const MotorConfig* motor = Find(config.hardware.motors, want.name);
      ASSERT_NE(motor, nullptr);
      EXPECT_EQ(motor->bus, kBus);
      EXPECT_EQ(motor->can_id, want.can_id);
      EXPECT_EQ(motor->motor_type.value, want.type);
      EXPECT_DOUBLE_EQ(motor->max_voltage, want.max_voltage);
      EXPECT_DOUBLE_EQ(motor->supply_limit_a, want.supply_limit_a);
      EXPECT_DOUBLE_EQ(motor->stator_limit_a, want.stator_limit_a);
    }
    for (const ExpectedSensor& want : kSensors) {
      SCOPED_TRACE(want.name);
      const SensorConfig* sensor = Find(config.hardware.sensors, want.name);
      ASSERT_NE(sensor, nullptr);
      EXPECT_EQ(sensor->bus, kBus);
      EXPECT_EQ(sensor->can_id, want.can_id);
      EXPECT_EQ(sensor->kind, want.kind);
    }
    // The steer motor closes on the module's CANcoder, in its absolute frame.
    const MotorConfig* steer = Find(config.hardware.motors, "steer");
    const SensorConfig* encoder = Find(config.hardware.sensors, "encoder");
    ASSERT_NE(steer, nullptr);
    ASSERT_NE(encoder, nullptr);
    EXPECT_EQ(steer->feedback, Feedback::kRemoteCANcoder);
    EXPECT_EQ(steer->feedback_sensor_id, encoder->id);
    EXPECT_TRUE(steer->absolute_position_frame.value);
    EXPECT_EQ(Find(config.hardware.motors, "drive")->feedback,
              Feedback::kRotor);
  }
}
}  // namespace
