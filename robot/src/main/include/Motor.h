#pragma once

#include "motor_state.h"

class Motor {
public:
    virtual ~Motor() = default;
    virtual void AddToPayload(talos::protocol::MotorPayload& payload) = 0;
};
