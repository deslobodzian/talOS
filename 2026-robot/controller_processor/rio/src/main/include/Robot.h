#pragma once

#include <frc/RobotBase.h>

#include <atomic>
#include <memory>

#include "hardware/endpoint.h"

// The entire RoboRIO program: a fixed-rate loop that reads the Driver Station,
// runs the hardware gateway, and does nothing else. All control logic lives in
// talOS processes on the companion computer.
//
// This derives from frc::RobotBase rather than frc::TimedRobot on purpose.
// TimedRobot brings a mode-callback framework, a Notifier, the command
// scheduler hooks and LiveWindow with it, none of which this program uses.
// What the Driver Station actually requires is narrow: say the program has
// started, and report the current mode on every iteration so the DS watchdog
// stays fed. That is what StartCompetition does, and nothing more.
class Robot : public frc::RobotBase {
 public:
  void StartCompetition() override;
  void EndCompetition() override;

 private:
  std::unique_ptr<talos::hardware::Backend> backend_;
  std::unique_ptr<talos::hardware::Gateway> gateway_;
  std::unique_ptr<talos::hardware::Endpoint> endpoint_;

  // Written by EndCompetition, which WPILib may call from another thread.
  std::atomic<bool> running_{true};
};
