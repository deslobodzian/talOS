#pragma once

#include "Constants.h"
#include "motor.h"
#include "motor_state.h"
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/system/plant/DCMotor.h>


class SimMotor : public Motor {
public:
    explicit SimMotor(int payload_slot) :
        motor_sim_(
            frc::LinearSystemId::DCMotorSystem(
                frc::DCMotor::KrakenX60FOC(),
                0.025_kg_sq_m,
                1.0),
            frc::DCMotor::KrakenX60FOC()
    ),
    payload_slot_(payload_slot) {};

    void UpdateState() {
        motor_sim_.Update(Constants::kUpdateRate);

        motor_state_.position = static_cast<float>(motor_sim_.GetAngularPosition().value());
        motor_state_.velocity = static_cast<float>(motor_sim_.GetAngularVelocity().value());
        motor_state_.stator_current = static_cast<float>(motor_sim_.GetCurrentDraw().value());
        motor_state_.supply_current = static_cast<float>(motor_sim_.GetCurrentDraw().value());
        motor_state_.torque_current = static_cast<float>(motor_sim_.GetTorque().value());
        motor_state_.voltage = static_cast<float>(motor_sim_.GetInputVoltage().value());
    }

    void AddToPayload(talos::protocol::MotorPayload& payload) {
        payload.states[payload_slot_] = motor_state_;
    }
private:
    talos::protocol::MotorState motor_state_{};
    frc::sim::DCMotorSim motor_sim_;
    int payload_slot_ = -1;
};
