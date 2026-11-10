#pragma once

#include <frc/RobotBase.h>

#include <atomic>
#include <memory>

#include "CompanionLink.h"
#include "RioLog.h"
#include "hardware/endpoint.h"

// The entire RoboRIO program: a fixed-rate loop that reads the Driver Station,
// runs the hardware gateway, reports the companion link to the drive team
// (CompanionLink.h), and does nothing else. All control logic lives in
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
  // Every report the loop thread makes goes through this queue to its logger
  // thread; it outlives the backend, which posts to it.
  std::unique_ptr<talos::rio::RioLog> log_;
  std::unique_ptr<talos::hardware::Backend> backend_;
  std::unique_ptr<talos::hardware::Gateway> gateway_;
  std::unique_ptr<talos::hardware::Endpoint> endpoint_;
  // The loop thread observes; the reporter's own thread publishes. The sink
  // outlives the reporter that writes to it.
  std::unique_ptr<talos::rio::CompanionSink> companion_sink_;
  std::unique_ptr<talos::rio::CompanionReporter> companion_;

  // Written by EndCompetition, which WPILib may call from another thread.
  std::atomic<bool> running_{true};
};
