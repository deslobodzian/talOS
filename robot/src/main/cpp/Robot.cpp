// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <chrono>
#include <cstdio>
#include <cstring>

#include "udp.h"
#include "types.h"

static_assert(talos::protocol::kProtocolVersion == 1);

namespace {

constexpr uint16_t kSimRioPort = 5802;
constexpr uint16_t kSimTalosPort = 5803;
constexpr uint64_t kSimManifestId = 1;

struct SimRuntimePayload {
  talos::protocol::RuntimeValuesHeader header;
  talos::protocol::SignalValue values[2];
};

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

uint64_t& SimTick() {
  static uint64_t tick = 0;
  return tick;
}

double& LastCommandValue() {
  static double value = 0.0;
  return value;
}

void SendSimState() {
  auto& peer = SimPeer();
  auto* payload = reinterpret_cast<SimRuntimePayload*>(
      peer.MutablePayloadBuffer(sizeof(SimRuntimePayload)));
  if (payload == nullptr) {
    return;
  }

  payload->header.manifest_id = kSimManifestId;
  payload->header.value_count = 2;
  payload->header.reserved = 0;
  payload->values[0].id = 1;
  payload->values[0].value_type = talos::protocol::ValueType::kFloat64;
  payload->values[0].reserved = 0;
  payload->values[0].float64_value = LastCommandValue();
  payload->values[1].id = 2;
  payload->values[1].value_type = talos::protocol::ValueType::kUint64;
  payload->values[1].reserved = 0;
  payload->values[1].uint64_value = ++SimTick();

  const auto status = peer.SendBufferedFrame(
      talos::protocol::FrameType::kRioState, talos::protocol::kFlagNone,
      NowUs(), sizeof(SimRuntimePayload));
  if (status != talos::protocol::UdpStatus::kOk &&
      status != talos::protocol::UdpStatus::kWouldBlock) {
    std::printf("TalOS sim UDP send failed: %s\n",
                talos::protocol::UdpStatusName(status));
  }
}

void DrainSimCommands() {
  auto& peer = SimPeer();
  for (;;) {
    talos::protocol::DecodedFrame frame;
    const auto status = peer.TryReceive(&frame);
    if (status == talos::protocol::UdpStatus::kWouldBlock) {
      return;
    }
    if (status != talos::protocol::UdpStatus::kOk) {
      std::printf("TalOS sim UDP receive failed: %s\n",
                  talos::protocol::UdpStatusName(status));
      return;
    }
    if (frame.header.type != talos::protocol::FrameType::kRioCommand ||
        frame.payload_size < sizeof(talos::protocol::RuntimeValuesHeader)) {
      continue;
    }

    SimRuntimePayload payload{};
    const std::size_t copy_size = frame.payload_size < sizeof(payload)
                                      ? frame.payload_size
                                      : sizeof(payload);
    std::memcpy(&payload, frame.payload, copy_size);
    if (payload.header.manifest_id != kSimManifestId ||
        payload.header.value_count == 0 ||
        payload.values[0].value_type != talos::protocol::ValueType::kFloat64) {
      continue;
    }
    LastCommandValue() = payload.values[0].float64_value;
    SendSimState();
  }
}

}  // namespace

Robot::Robot() : frc::TimedRobot(20_ms) {
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

void Robot::SimulationPeriodic() { DrainSimCommands(); }

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
