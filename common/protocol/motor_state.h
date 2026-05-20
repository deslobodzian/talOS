#pragma once

#include <cstdint>
#include "types.h"

namespace talos::protocol {

constexpr size_t kMaxMotors = 21; // 24 - 3 (rio, radio, mac power ports)

struct MotorState {
    float position;
    float velocity;
    float voltage;
    float torque_current;
    float supply_current;
    float stator_current;
};


enum class ControlMode : uint8_t {
    DISABLED = 0,
    VOLTAGE = 1,
    POSITION = 2,
    VELOCITY = 3,
    TORQUE = 4
};

struct MotorCommand {
    float demand;
    float feedforward;
    ControlMode mode;
    uint8_t enabled;
    uint16_t reserved;
};

struct MotorPayload {
    MotorState states[kMaxMotors];
};

static_assert(sizeof(MotorState) == 24);
static_assert(sizeof(MotorCommand) == 12);
} /* namespace talos::protocol */
