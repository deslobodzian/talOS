// The autonomous drive action against the real drivetrain node, on the
// robot's own config: the taxi routine's straight path, followed through
// the drivetrain's slew limits and swerve kinematics on SimHardware's ideal
// motors. The test plays the pose estimator: it integrates the chassis
// motion /drivetrain/state measures into the true pose, and publishes that
// pose (plus any correction a case injects) on /pose_estimator/state at the
// estimator's 20 ms period, with the measured velocity. The autonomous node's
// test matrix is in autonomous/node_test.cc.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>

#include "autonomous/autonomous.h"
#include "drivetrain/drivetrain.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/testing.h"
#include "talOS/geometry/se2.h"
#include "talOS/node/testing.h"
#include "talOS/trajectory/straight_line.h"
#include "talOS/trajectory/trajectory.h"

namespace autonomous {
namespace {
using namespace std::chrono_literals;
namespace ds = talos::driver_station;

std::string RobotConfig() {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> files(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv("TALOS_TEST_CONFIG");
  if (!files || location == nullptr) throw std::runtime_error(error);
  return files->Rlocation(location);
}

double Seconds(talos::Duration d) {
  return std::chrono::duration<double>(d).count();
}

struct Pose {
  double x_m = 0, y_m = 0, yaw_rad = 0;
};

// The taxi routine's waypoint (subsystems/autonomous.toml) and a start that
// turns a quarter turn on the way.
constexpr Pose kStart{2, 2, -std::numbers::pi / 2};
constexpr Pose kGoal{6, 4, 0};
constexpr auto kTick = 5ms;               // the drivetrain's period
constexpr auto kAutonomousPeriod = 20ms;  // autonomous's and the estimator's

// What a case does, `at` after the drive starts.
struct Disturbance {
  enum Kind {
    kNone,
    // The robot is pushed `size_m` left of its path at a steady speed over
    // `duration`. The estimate follows the true pose, as odometry and vision
    // together would, with the velocity the wheels measure: each 20 ms
    // estimate moves 3 cm more than that explains, under replan_step_m.
    kShove,
    // The estimate reads invalid for `duration`.
    kOutage,
    // The estimate jumps `size_m` left of the true pose and stays there (a
    // vision correction).
    kStep,
  };
  Kind kind = kNone;
  talos::Duration at{};
  talos::Duration duration{};
  double size_m = 0;
};

double Distance(const Pose& a, const Pose& b) {
  return std::hypot(a.x_m - b.x_m, a.y_m - b.y_m);
}
double YawError(const Pose& a, const Pose& b) {
  return std::abs(std::remainder(a.yaw_rad - b.yaw_rad, 2 * std::numbers::pi));
}

struct Outcome {
  double planned_s = 0;  // the path's duration from kStart
  // Largest distance of the true pose from the straight path while
  // driving, before `at`, and from `at` + 1 s until HOLD.
  double path_error_before_m = 0;
  double path_error_after_m = 0;
  std::optional<double> hold_s;  // drive start to HOLD
  // 0.5 s after HOLD, the true pose and the estimate against the goal.
  Pose settled_truth;
  Pose settled_estimate;
  int released_ticks = 0;  // autonomous ticks in DRIVE with no drive target
  // The first published invalid estimate to the first released lane, and
  // the first active lane after the outage.
  std::optional<double> release_delay_s;
  std::optional<drivetrain::ChassisTarget> first_after_outage;
  // The true pose's furthest travel past the goal along the path, and its
  // heading's past the goal's in the direction of the turn (0 if never).
  double overshoot_m = 0;
  double overshoot_rad = 0;
};

std::ostream& operator<<(std::ostream& out, const Outcome& o) {
  return out << "planned " << o.planned_s << " s, HOLD at "
             << o.hold_s.value_or(-1) << " s, path error before "
             << o.path_error_before_m << " m, after " << o.path_error_after_m
             << " m, released ticks " << o.released_ticks << ", overshoot "
             << o.overshoot_m << " m, " << o.overshoot_rad
             << " rad, settled truth " << Distance(o.settled_truth, kGoal)
             << " m, " << YawError(o.settled_truth, kGoal) << " rad";
}

class ClosedLoop : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto config = talos::config::ParseRobotConfig(RobotConfig());
    const toml::table& table = *config.GetSubsystemTable("autonomous");
    position_tolerance_m_ = table["position_tolerance_m"].value_or(0.0);
    yaw_tolerance_rad_ = table["yaw_tolerance_rad"].value_or(0.0);
    talos::trajectory::Trajectory path;
    path.reserve(talos::trajectory::kStraightLineSamples);
    using talos::geometry::SE2;
    ASSERT_TRUE(talos::trajectory::MakeStraightLine(
        SE2<double>{kStart.yaw_rad,
                    SE2<double>::Vector2{kStart.x_m, kStart.y_m}},
        SE2<double>{kGoal.yaw_rad, SE2<double>::Vector2{kGoal.x_m, kGoal.y_m}},
        {table["max_linear_mps"].value_or(0.0),
         table["max_linear_accel_mps2"].value_or(0.0),
         table["max_angular_radps"].value_or(0.0),
         table["max_angular_accel_radps2"].value_or(0.0)},
        path));
    planned_s_ = path.total_time_s();

    hw.enable();
    hw.ideal_motors();
    // Disabled: POV left once selects taxi (routine 3, from preload's 0).
    for (const int pov : {-1, 270, -1}) {
      auto packet = talos::testing::DsPacket(0);
      packet.joysticks[0].connected = true;
      packet.joysticks[0].pov_count = 1;
      packet.joysticks[0].povs[0] = static_cast<std::int16_t>(pov);
      hw.set_driver_station(packet);
      Publish(kStart, true, {});
      sim.step(kAutonomousPeriod);
    }
    ASSERT_EQ(sim.latest<AutonomousState>(topics::kState)->routine_index(), 3);
  }

  void Publish(const Pose& p, bool valid,
               const drivetrain::DrivetrainState& measured) {
    sim.publish("/pose_estimator/state",
                geometry::PoseEstimate{
                    0, geometry::Pose2d{p.x_m, p.y_m, p.yaw_rad},
                    geometry::Twist2d{measured.vx_mps(), measured.vy_mps(),
                                      measured.omega_radps()},
                    valid, true, 0, 0});
  }

  // Distance from `p` to the segment from kStart to kGoal.
  static double PathError(const Pose& p) {
    const double dx = kGoal.x_m - kStart.x_m;
    const double dy = kGoal.y_m - kStart.y_m;
    const double along =
        std::clamp(((p.x_m - kStart.x_m) * dx + (p.y_m - kStart.y_m) * dy) /
                       (dx * dx + dy * dy),
                   0.0, 1.0);
    return std::hypot(p.x_m - (kStart.x_m + along * dx),
                      p.y_m - (kStart.y_m + along * dy));
  }

  // Enables autonomous with the robot at rest at kStart and runs until 0.5 s
  // after HOLD, or 6 s, applying `what`.
  Outcome Drive(const Disturbance& what) {
    Outcome run;
    run.planned_s = planned_s_;
    const double length = Distance(kStart, kGoal);
    const double left_x = -(kGoal.y_m - kStart.y_m) / length;
    const double left_y = (kGoal.x_m - kStart.x_m) / length;

    hw.set_driver_station(
        talos::testing::DsPacket(ds::kEnabled | ds::kAutonomous));
    Pose truth = kStart;
    Pose estimate = kStart;
    drivetrain::DrivetrainState measured{};
    std::optional<talos::Time> started, held, invalid_since;
    bool outage_over = false;
    for (auto elapsed = 0ms; elapsed < 6s; elapsed += kTick) {
      const talos::Time now = sim.now();
      const talos::Duration since =
          started ? now - *started : talos::Duration::zero();
      const bool during =
          started && since >= what.at && since < what.at + what.duration;
      estimate = truth;
      if (what.kind == Disturbance::kStep && started && since >= what.at) {
        estimate.x_m += what.size_m * left_x;
        estimate.y_m += what.size_m * left_y;
      }
      if (now.nanos() % std::chrono::nanoseconds{kAutonomousPeriod}.count() ==
          0) {
        const bool invalid = what.kind == Disturbance::kOutage && during;
        if (invalid && !invalid_since) invalid_since = now;
        if (!invalid && invalid_since) outage_over = true;
        Publish(estimate, !invalid, measured);
      }
      sim.step(kTick);
      measured =
          sim.latest<drivetrain::DrivetrainState>(drivetrain::topics::kState)
              .value_or(drivetrain::DrivetrainState{});
      // The chassis moves as its modules measure, about the mid-tick
      // heading.
      const double dt = Seconds(kTick);
      const double yaw = truth.yaw_rad + 0.5 * measured.omega_radps() * dt;
      truth.x_m += (std::cos(yaw) * measured.vx_mps() -
                    std::sin(yaw) * measured.vy_mps()) *
                   dt;
      truth.y_m += (std::sin(yaw) * measured.vx_mps() +
                    std::cos(yaw) * measured.vy_mps()) *
                   dt;
      truth.yaw_rad += measured.omega_radps() * dt;
      if (what.kind == Disturbance::kShove && during) {
        const double push = what.size_m * dt / Seconds(what.duration);
        truth.x_m += push * left_x;
        truth.y_m += push * left_y;
      }
      // How far the robot has gone past the goal, along the path and in the
      // direction of the turn (kStart turns CCW to kGoal).
      run.overshoot_m =
          std::max(run.overshoot_m, ((truth.x_m - kGoal.x_m) * left_y -
                                     (truth.y_m - kGoal.y_m) * left_x));
      run.overshoot_rad =
          std::max(run.overshoot_rad, truth.yaw_rad - kGoal.yaw_rad);

      const auto phase = sim.latest<AutonomousState>(topics::kState)->phase();
      if (!started && phase == autonomous::Phase::DRIVE) started = now;
      if (!started) continue;
      if (phase == autonomous::Phase::HOLD) {
        if (!held) {
          held = sim.now();
          run.hold_s = Seconds(*held - *started);
        }
        if (sim.now() - *held >= 500ms) {
          run.settled_truth = truth;
          run.settled_estimate = estimate;
          break;
        }
        continue;
      }
      if (phase != autonomous::Phase::DRIVE) continue;
      // Lanes are written once per autonomous tick.
      if (sim.now().nanos() %
              std::chrono::nanoseconds{kAutonomousPeriod}.count() ==
          0) {
        const auto lane =
            talos::testing::LatestTarget<drivetrain::ChassisTarget>(
                sim, "drivetrain", "auto");
        const bool active = lane && lane->active;
        if (!active) ++run.released_ticks;
        if (!active && invalid_since && !run.release_delay_s)
          run.release_delay_s = Seconds(sim.now() - *invalid_since);
        if (active && outage_over && !run.first_after_outage)
          run.first_after_outage = lane->message;
      }
      const double error = PathError(truth);
      const talos::Duration t = sim.now() - *started;
      if (what.kind == Disturbance::kNone || t < what.at) {
        run.path_error_before_m = std::max(run.path_error_before_m, error);
      } else if (t >= what.at + 1s) {
        run.path_error_after_m = std::max(run.path_error_after_m, error);
      }
    }
    return run;
  }

  // HOLD, and 0.5 s later `pose` within autonomous's tolerances of the goal.
  void ExpectArrived(const Outcome& run, const Pose& pose) const {
    ASSERT_TRUE(run.hold_s.has_value()) << run;
    EXPECT_LE(Distance(pose, kGoal), position_tolerance_m_) << run;
    EXPECT_LE(YawError(pose, kGoal), yaw_tolerance_rad_) << run;
  }

  double planned_s_ = 0;
  double position_tolerance_m_ = 0;
  double yaw_tolerance_rad_ = 0;
  talos::testing::Sim sim{RobotConfig()};
  Autonomous& autonomous_node = sim.add<Autonomous>("autonomous");
  drivetrain::Drivetrain& drivetrain_node =
      sim.add<drivetrain::Drivetrain>("drivetrain");
  talos::testing::SimHardware hw{sim};
};

// The robot keeps within 0.10 m of its straight path, holds within 1 s of
// the path's planned duration (2.74 s), and stands within the routine's
// tolerances of its goal 0.5 s later. On the way it never passes the goal,
// along the path or in heading, by more than those tolerances: a robot that
// did would have to turn back to arrive. REQ-DEV-27 states no tighter
// bound, so none is asserted.
// REQ-DEV-27
TEST_F(ClosedLoop, DrivesItsPathAndArrives) {
  const Outcome run = Drive({});
  EXPECT_LE(run.path_error_before_m, 0.10) << run;
  ASSERT_TRUE(run.hold_s.has_value()) << run;
  EXPECT_LE(*run.hold_s, run.planned_s + 1.0) << run;
  EXPECT_EQ(run.released_ticks, 0) << run;
  EXPECT_LE(run.overshoot_m, position_tolerance_m_) << run;
  EXPECT_LE(run.overshoot_rad, yaw_tolerance_rad_) << run;
  ExpectArrived(run, run.settled_truth);
}

// Pushed 0.3 m off its path at 1.5 s, gradually (over 0.2 s, an estimate
// that follows it without a step: a 0.3 m jump in one estimate would replan,
// REQ-DEV-28), the robot keeps within 0.10 m of the path from 1 s after the
// shove began until HOLD, and holds within 1.5 s of the planned duration.
// REQ-DEV-27
TEST_F(ClosedLoop, ReturnsToItsPathAfterAShove) {
  const Outcome run = Drive({Disturbance::kShove, 1500ms, 200ms, 0.3});
  EXPECT_LE(run.path_error_after_m, 0.10) << run;
  ASSERT_TRUE(run.hold_s.has_value()) << run;
  EXPECT_LE(*run.hold_s, run.planned_s + 1.5) << run;
  EXPECT_EQ(run.released_ticks, 0) << run;
}

// An estimate invalid from 1.0 s to 1.5 s releases the drive lane on the
// first autonomous tick that reads it; the first target after the outage is
// a new plan's first (rest, planned from where the robot stopped), and the
// robot still arrives.
// REQ-DEV-27, REQ-SAFE-34
TEST_F(ClosedLoop, ReplansAfterAPoseOutageAndArrives) {
  const Outcome run = Drive({Disturbance::kOutage, 1000ms, 500ms, 0});
  ASSERT_TRUE(run.release_delay_s.has_value()) << run;
  EXPECT_LE(*run.release_delay_s, Seconds(kAutonomousPeriod)) << run;
  EXPECT_LE(run.released_ticks, 26) << run;  // the outage's 25 ticks, +1
  ASSERT_TRUE(run.first_after_outage.has_value()) << run;
  EXPECT_EQ(run.first_after_outage->vx_mps(), 0);
  EXPECT_EQ(run.first_after_outage->vy_mps(), 0);
  EXPECT_EQ(run.first_after_outage->omega_radps(), 0);
  ExpectArrived(run, run.settled_truth);
}

// An estimate that jumps 0.2 m at 2 s never releases the drive lane, and
// the robot arrives where the estimate puts the goal.
// REQ-DEV-27, REQ-DEV-28
TEST_F(ClosedLoop, EstimateStepKeepsTheLaneAndArrives) {
  const Outcome run = Drive({Disturbance::kStep, 2000ms, 0ms, 0.2});
  EXPECT_EQ(run.released_ticks, 0) << run;
  ExpectArrived(run, run.settled_estimate);
}

}  // namespace
}  // namespace autonomous
