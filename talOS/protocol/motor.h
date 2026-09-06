#pragma once

#include "motor_state.h"
class Motor {
    virtual ~Motor() = default;
    virtual void AddToPayload(talos::protocol::MotorPayload& payload) = 0;
    virtual void UpdateState() = 0;
};
