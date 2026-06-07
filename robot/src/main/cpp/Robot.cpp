// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <chrono>
#include <cstdio>
#include <cstring>

#include "Constants.h"
#include "motor_state.h"
#include "udp.h"
#include "types.h"

static_assert(talos::protocol::kProtocolVersion == 1);

namespace {

constexpr uint16_t kSimRioPort = 5802;
constexpr uint16_t kSimTalosPort = 5803;

uint64_t NowUs() {
  using Clock = std::chrono::steady_clock;
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          Clock::now().time_since_epoch())
          .count());
}

talos::protocol::RuntimeUdpPeer& SimPeer() {
  static talos::protocol::RuntimeUdpPeer peer;
  return peer;
}

talos::protocol::MotorPayload* MotorPayload() {
  auto& peer = SimPeer();
  auto* payload = reinterpret_cast<talos::protocol::MotorPayload*>(
      peer.MutablePayloadBuffer(sizeof(talos::protocol::MotorPayload)));
  return payload;
}

void SendSimState() {
  auto& peer = SimPeer();
  if (MotorPayload() == nullptr) {
    return;
  }

  const auto status = peer.SendBufferedFrame(
      talos::protocol::FrameType::kRioState, talos::protocol::kFlagNone,
      NowUs(), sizeof(talos::protocol::MotorPayload));
  if (status != talos::protocol::UdpStatus::kOk &&
      status != talos::protocol::UdpStatus::kWouldBlock) {
    std::printf("TalOS sim UDP send failed: %s\n",
                talos::protocol::UdpStatusName(status));
  }
}
}  // namespace

Robot::Robot() : frc::TimedRobot(Constants::kUpdateRate) {
    motors_.push_back(SimMotor(0));
    motors_.push_back(SimMotor(1));
    motors_.push_back(SimMotor(2));
    motors_.push_back(SimMotor(3));
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {
  talos::protocol::UdpSocketOptions options;
  options.non_blocking = true;
  const auto status = SimPeer().Open("127.0.0.1", kSimRioPort, "127.0.0.1",
                                     kSimTalosPort, options);
  if (status == talos::protocol::UdpStatus::kOk) {
    std::printf("TalOS sim UDP endpoint listening on 127.0.0.1:%u\n",
                kSimRioPort);
  } else {
    std::printf("TalOS sim UDP endpoint failed to open: %s\n",
                talos::protocol::UdpStatusName(status));
  }
}

void Robot::SimulationPeriodic() { SendSimState(); }

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
