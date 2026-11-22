// C++ twin nodes exported via TALOS_EXPORT_NODE for differential_parity_test.py
// (talos.parity C++ vs Python differential parity across devices,
// DriverStation, Target/TargetLane/OperatorArming, StateMachine, session_dir,
// and mechanisms).

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "control/control_generated.h"
#include "geometry/geometry_generated.h"
#include "python/talos_test_message_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/devices/target.h"
#include "talOS/mechanisms/position_mechanism.h"
#include "talOS/mechanisms/roller_with_sensor.h"
#include "talOS/mechanisms/velocity_mechanism.h"
#include "talOS/node/node.h"

namespace {

std::uint64_t Fnv1a64(std::string_view text) {
  std::uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char ch : text) {
    hash ^= static_cast<std::uint64_t>(ch);
    hash *= 1099511628211ULL;
  }
  return hash;
}

// Covers all 8 device kinds (Motor, CANcoder, Gyro, Encoder, DigitalInput,
// AnalogInput, DigitalOutput, PwmOutput), DriverStation, Target, HardwareLink,
// StateMachine, session_dir, rezero/zeroed, reset_device, reboot, pause/resume.
class DevicesRig {
 public:
  explicit DevicesRig(talos::Node& node)
      : node_{node},
        link_{node.extension<talos::HardwareLink>()},
        drive_{node, "drive"},
        cc_{node, "cc"},
        imu_{node, "imu"},
        enc_{node, "enc"},
        beam_{node, "beam"},
        pot_{node, "pot"},
        solenoid_{node, "solenoid"},
        servo_{node, "servo"},
        ds_{node},
        target_{node},
        sm_{node, demo::Phase::IDLE},
        state_{node.publish<geometry::PoseEstimate>("/rig/state")},
        status_{node.publish<geometry::PoseObservation>("/rig/status")},
        telemetry_{node.publish<demo::Status>("/rig/telemetry")} {}

  void update() {
    const demo::Goal* goal = target_.get();
    const std::string_view src = target_.source();

    if (goal == nullptr) {
      sm_.go(demo::Phase::IDLE);
    } else if (goal->enabled()) {
      sm_.go(demo::Phase::MOVING);
    } else {
      sm_.go(demo::Phase::HOLDING);
    }

    if (goal != nullptr) {
      switch (goal->tick_count()) {
        case 1:
          drive_.rezero(goal->height_rot());
          imu_.rezero(goal->height_rot());
          break;
        case 2:
          drive_.set_position(goal->height_rot());
          break;
        case 3:
          drive_.set_voltage(goal->height_rot());
          solenoid_.set(goal->enabled());
          servo_.set(goal->height_rot() / 10.0);
          break;
        case 4:
          drive_.stop();
          solenoid_.set(false);
          servo_.stop();
          break;
        default:
          break;
      }
    }

    const std::optional<talos::Time> sample_time = link_.sample_time();
    const std::uint64_t sample_time_ns =
        sample_time ? static_cast<std::uint64_t>(sample_time->nanos()) : 0ULL;
    const std::uint32_t device_flags =
        (static_cast<std::uint32_t>(drive_.connected()) << 0U) |
        (static_cast<std::uint32_t>(cc_.connected()) << 1U) |
        (static_cast<std::uint32_t>(imu_.connected()) << 2U) |
        (static_cast<std::uint32_t>(enc_.connected()) << 3U) |
        (static_cast<std::uint32_t>(beam_.connected()) << 4U) |
        (static_cast<std::uint32_t>(pot_.connected()) << 5U) |
        (static_cast<std::uint32_t>(drive_.zeroed()) << 6U) |
        (static_cast<std::uint32_t>(imu_.zeroed()) << 7U) |
        (static_cast<std::uint32_t>(beam_.get()) << 8U) |
        (static_cast<std::uint32_t>(drive_.forward_limit()) << 9U) |
        (static_cast<std::uint32_t>(drive_.reverse_limit()) << 10U) |
        (static_cast<std::uint32_t>(link_.enabled()) << 11U) |
        (static_cast<std::uint32_t>(link_.fresh()) << 12U) |
        (static_cast<std::uint32_t>(sm_.just_entered()) << 13U);

    state_.send(geometry::PoseEstimate{
        sample_time_ns,
        geometry::Pose2d{drive_.position(), cc_.position(), imu_.yaw()},
        geometry::Twist2d{drive_.velocity(), cc_.velocity(), imu_.yaw_rate()},
        drive_.connected() && cc_.connected() && imu_.connected(),
        drive_.zeroed() && imu_.zeroed(),
        device_flags,
        static_cast<std::uint32_t>(drive_.clamped_count()),
    });

    const std::uint32_t src_bits =
        src == "teleop" ? 1U : (src == "auto" ? 2U : 0U);
    const std::uint32_t ds_sm_flags =
        (static_cast<std::uint32_t>(ds_.connected()) << 0U) |
        (static_cast<std::uint32_t>(ds_.enabled()) << 1U) |
        (static_cast<std::uint32_t>(ds_.teleop()) << 2U) |
        (static_cast<std::uint32_t>(ds_.autonomous()) << 3U) |
        (static_cast<std::uint32_t>(ds_.test()) << 4U) |
        (static_cast<std::uint32_t>(ds_.estop()) << 5U) |
        (static_cast<std::uint32_t>(ds_.fms_attached()) << 6U) |
        (static_cast<std::uint32_t>(ds_.alliance()) << 8U) |
        (static_cast<std::uint32_t>(ds_.station()) << 10U) | (src_bits << 14U) |
        (static_cast<std::uint32_t>(sm_.state()) << 16U) |
        (static_cast<std::uint32_t>(sm_.previous()) << 24U);

    status_.send(geometry::PoseObservation{
        static_cast<std::uint64_t>(sm_.time_in_state().count()),
        geometry::Pose2d{enc_.position(), enc_.velocity(), pot_.voltage()},
        drive_.temperature_c(),
        drive_.supply_current_a(),
        ds_.match_time(),
        ds_sm_flags,
        sm_.is(demo::Phase::MOVING),
    });

    const std::optional<std::filesystem::path> session_dir =
        node_.session_dir();
    const std::uint64_t session_tag =
        session_dir ? Fnv1a64(session_dir->string()) : node_.tick();
    telemetry_.send(demo::Status{
        drive_.current() + ds_.joystick(0).axis(0),
        session_dir.has_value(),
        session_tag,
    });
  }

 private:
  talos::Node& node_;
  talos::HardwareLink& link_;
  talos::Motor drive_;
  talos::CANcoder cc_;
  talos::Gyro imu_;
  talos::Encoder enc_;
  talos::DigitalInput beam_;
  talos::AnalogInput pot_;
  talos::DigitalOutput solenoid_;
  talos::PwmOutput servo_;
  talos::DriverStation ds_;
  talos::Target<demo::Goal> target_;
  talos::StateMachine<demo::Phase> sm_;
  talos::Publisher<geometry::PoseEstimate> state_;
  talos::Publisher<geometry::PoseObservation> status_;
  talos::Publisher<demo::Status> telemetry_;
};

// Covers DriverStation joysticks + OperatorArming (evaluate with ArmingBlocker,
// armed, blocker, disarm) + TargetLane (send armed vs disarmed vs release).
class OperatorNode {
 public:
  explicit OperatorNode(talos::Node& node)
      : ds_{node},
        arming_{node},
        lane_{node, "rig", "teleop", arming_},
        status_{node.publish<geometry::PoseObservation>("/operator/status")} {}

  void update() {
    const talos::Joystick stick = ds_.joystick(0);
    const double axis0 = stick.axis(0);
    const bool btn1 = stick.button(1);
    const bool btn2 = stick.button(2);
    const bool btn3 = stick.button(3);
    const int pov0 = stick.pov(0);

    if (btn2) arming_.disarm();

    bool neutral = true;
    talos::ArmingBlocker input_blocker{};
    if (std::abs(axis0) > 0.1) {
      neutral = false;
      input_blocker = {talos::ArmingInputKind::kAxis, 0, 0};
    } else if (btn1) {
      neutral = false;
      input_blocker = {talos::ArmingInputKind::kButton, 0, 1};
    } else if (pov0 != -1) {
      neutral = false;
      input_blocker = {talos::ArmingInputKind::kPov, 0, 0};
    }

    const bool can_send = arming_.evaluate(neutral, input_blocker);
    bool sent = false;
    if ((can_send && !neutral) || btn3) {
      // When btn3 is held while disarmed, lane_.send() must return false and
      // emit a release instead of an active target.
      sent = lane_.send(demo::Goal{axis0, btn1, pov0});
    } else {
      lane_.release();
    }

    const talos::ArmingBlocker reported = arming_.blocker();
    const std::uint32_t flags =
        (static_cast<std::uint32_t>(can_send) << 0U) |
        (static_cast<std::uint32_t>(arming_.armed()) << 1U) |
        (static_cast<std::uint32_t>(sent) << 2U) |
        (static_cast<std::uint32_t>(reported.kind) << 4U) |
        (static_cast<std::uint32_t>(stick.connected()) << 8U) |
        (static_cast<std::uint32_t>(ds_.enabled()) << 9U);

    status_.send(geometry::PoseObservation{
        0ULL,
        geometry::Pose2d{axis0, static_cast<double>(reported.port),
                         static_cast<double>(reported.index)},
        static_cast<double>(pov0),
        static_cast<double>(stick.axis_count()),
        static_cast<double>(stick.button_count()),
        flags,
        arming_.armed(),
    });
  }

 private:
  talos::DriverStation ds_;
  talos::OperatorArming arming_;
  talos::TargetLane<demo::Goal> lane_;
  talos::Publisher<geometry::PoseObservation> status_;
};

// Covers PositionMechanism: set_goal, set_goal_units, hold, stop, clear_fault,
// and all state/property getters.
class PositionNode {
 public:
  explicit PositionNode(talos::Node& node)
      : arm_{node, "arm"},
        target_{node},
        mode_{node.subscribe<demo::Goal>("/arm/mode", {.optional = true})},
        state_{node.publish<control::PositionState>("/arm/state")},
        status_{node.publish<geometry::PoseObservation>("/arm/status")} {}

  void update() {
    const int action = mode_.has_value() ? mode_->tick_count() : 0;
    if (action == 3) {
      arm_.clear_fault();
    }
    const control::PositionTarget* t = target_.get();
    if (t == nullptr || action == 3) {
      arm_.stop();
    } else if (action == 1) {
      arm_.set_goal_units(t->position_rot() * 360.0, t->feedforward_v());
    } else if (action == 2) {
      arm_.hold(t->feedforward_v());
    } else {
      arm_.set_goal(t->position_rot(), t->feedforward_v());
    }
    arm_.update();
    state_.send(arm_.state());

    const std::uint32_t flags =
        (static_cast<std::uint32_t>(arm_.phase()) << 8U) |
        (static_cast<std::uint32_t>(arm_.fault()) << 4U) |
        (static_cast<std::uint32_t>(arm_.homed()) << 3U) |
        (static_cast<std::uint32_t>(arm_.at_goal()) << 2U) |
        (static_cast<std::uint32_t>(arm_.driving()) << 1U) |
        static_cast<std::uint32_t>(arm_.connected());
    status_.send(geometry::PoseObservation{
        0ULL,
        geometry::Pose2d{arm_.position_rot(), arm_.position_units(),
                         arm_.velocity_rps()},
        arm_.goal_rot(),
        arm_.gravity_v(),
        arm_.tolerance_rot(),
        flags,
        arm_.driving(),
    });
  }

 private:
  talos::mechanisms::PositionMechanism arm_;
  talos::Target<control::PositionTarget> target_;
  talos::Subscription<demo::Goal> mode_;
  talos::Publisher<control::PositionState> state_;
  talos::Publisher<geometry::PoseObservation> status_;
};

// Covers VelocityMechanism: set_goal, stop, and all state/property getters.
class VelocityNode {
 public:
  explicit VelocityNode(talos::Node& node)
      : flywheel_{node, "flywheel"},
        target_{node},
        state_{node.publish<control::VelocityState>("/shooter/state")},
        status_{node.publish<geometry::PoseObservation>("/shooter/status")} {}

  void update() {
    if (const control::VelocityTarget* t = target_.get()) {
      flywheel_.set_goal(t->velocity_rps(), t->feedforward_v());
    } else {
      flywheel_.stop();
    }
    flywheel_.update();
    state_.send(flywheel_.state());

    const std::uint32_t flags =
        (static_cast<std::uint32_t>(flywheel_.phase()) << 4U) |
        (static_cast<std::uint32_t>(flywheel_.at_speed()) << 2U) |
        (static_cast<std::uint32_t>(flywheel_.driving()) << 1U) |
        static_cast<std::uint32_t>(flywheel_.connected());
    status_.send(geometry::PoseObservation{
        0ULL,
        geometry::Pose2d{flywheel_.velocity_rps(), flywheel_.goal_rps(),
                         flywheel_.commanded_rps()},
        0.0,
        0.0,
        0.0,
        flags,
        flywheel_.at_speed(),
    });
  }

 private:
  talos::mechanisms::VelocityMechanism flywheel_;
  talos::Target<control::VelocityTarget> target_;
  talos::Publisher<control::VelocityState> state_;
  talos::Publisher<geometry::PoseObservation> status_;
};

// Covers RollerWithSensor: set_goal (IDLE/INTAKE/EJECT/FEED), stop, and all
// property getters.
class RollerNode {
 public:
  explicit RollerNode(talos::Node& node)
      : roller_{node, "roller"},
        target_{node},
        status_{node.publish<demo::Status>("/intake/status")} {}

  void update() {
    if (const demo::Goal* t = target_.get()) {
      roller_.set_goal(
          static_cast<talos::mechanisms::RollerCommand>(t->tick_count()));
    } else {
      roller_.stop();
    }
    roller_.update();
    const std::uint64_t flags =
        (static_cast<std::uint64_t>(roller_.phase()) << 4U) |
        (static_cast<std::uint64_t>(roller_.sensor_connected()) << 1U) |
        static_cast<std::uint64_t>(roller_.driving());
    status_.send(demo::Status{roller_.output_v(), roller_.has_piece(), flags});
  }

 private:
  talos::mechanisms::RollerWithSensor roller_;
  talos::Target<demo::Goal> target_;
  talos::Publisher<demo::Status> status_;
};

}  // namespace

TALOS_EXPORT_NODE(devices_rig, DevicesRig)
TALOS_EXPORT_NODE(operator_node, OperatorNode)
TALOS_EXPORT_NODE(position_node, PositionNode)
TALOS_EXPORT_NODE(velocity_node, VelocityNode)
TALOS_EXPORT_NODE(roller_node, RollerNode)
