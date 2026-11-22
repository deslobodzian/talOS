// Test-only autonomous target producer for the bench soak graph
// (testdata/robot_soak.toml). Publishes nonzero autonomous targets on
// /swerve_module/target/auto and /motor/target/auto while enabled in autonomous
// mode, and releases both lanes otherwise.
#include "motor/motor_message_generated.h"
#include "swerve_module/module_message_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace {

// Distinct from teleop's scripted commands (~5.56 rps drive, ~1.11 V motor
// after deadband rescaling) so the soak test can distinguish autonomous-driven
// actuation from teleop-driven actuation across mode-generation handoffs.
constexpr double kAutoDriveVelocityRps = 4.0;
constexpr double kAutoMotorVelocityRps = 8.0;

class AutoProducer {
 public:
  explicit AutoProducer(talos::Node& node)
      : ds_{node},
        swerve_lane_{node, "swerve_module", "auto"},
        motor_lane_{node, "motor", "auto"} {}

  void update() {
    if (ds_.enabled() && ds_.autonomous() && !ds_.estop()) {
      swerve_lane_.send(swerve_module::ModuleTarget{
          swerve_module::ModuleMode::DRIVE, kAutoDriveVelocityRps, 0.0});
      motor_lane_.send(motor::MotorTarget{motor::ControlMode::VELOCITY, 0.0,
                                          kAutoMotorVelocityRps, 0.0, 0.0});
      commanding_ = true;
    } else if (commanding_) {
      swerve_lane_.release();
      motor_lane_.release();
      commanding_ = false;
    }
  }

 private:
  talos::DriverStation ds_;
  talos::TargetLane<swerve_module::ModuleTarget> swerve_lane_;
  talos::TargetLane<motor::MotorTarget> motor_lane_;
  bool commanding_{false};
};

}  // namespace

TALOS_NODE(AutoProducer)
