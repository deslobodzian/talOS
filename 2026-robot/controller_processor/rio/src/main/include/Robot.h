#pragma once

#include <frc/RobotBase.h>

#include <atomic>
#include <memory>

#include "CompanionLink.h"
#include "LoopWatchdog.h"
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
// What the Driver Station needs from robot code is narrow: say the program
// has started, and report the mode the program is running in on every
// iteration, which the Driver Station displays. That is what StartCompetition
// does, and nothing more. Whether the Driver Station or the RoboRIO disables
// the robot when those reports stop is not established (docs/BENCH.md, loop
// stall check), and nothing relies on it: if this loop stops ticking with a
// command in force, LoopWatchdog neutralizes the outputs.
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
  // The loop beats; the watchdog's own thread acts on the backend's devices,
  // so both are released before the backend.
  std::unique_ptr<talos::rio::WatchdogOutputs> watchdog_outputs_;
  std::unique_ptr<talos::rio::LoopWatchdog> watchdog_;

  // Written by EndCompetition, which WPILib may call from another thread.
  std::atomic<bool> running_{true};
};
