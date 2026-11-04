#include <gtest/gtest.h>

#include <chrono>

#include "control/control_generated.h"
#include "geometry/geometry_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/devices/testing.h"
#include "talOS/node/node.h"
#include "talOS/node/testing.h"

namespace talos::msgs {
namespace {

using namespace std::chrono_literals;
using testing::Sim;
using testing::SimHardware;
namespace ds = driver_station;

static_assert(Message<geometry::Translation2d>);
static_assert(Message<geometry::Rotation2d>);
static_assert(Message<geometry::Pose2d>);
static_assert(Message<geometry::Twist2d>);
static_assert(Message<geometry::OdometrySample>);
static_assert(Message<geometry::SwerveModulePosition>);
static_assert(Message<geometry::PoseObservation>);
static_assert(Message<geometry::PoseEstimate>);
static_assert(Message<control::VelocityTarget>);
static_assert(Message<control::VelocityState>);
static_assert(Message<control::PositionTarget>);
static_assert(Message<control::PositionState>);
static_assert(Message<control::ChassisTarget>);

constexpr const char* kToml = R"(
[robot]
period_us = 10000
command_timeout_us = 100000

[subsystems.wrist.motors.m]
can_id = 1
)";

struct DummyPositionNode {
  explicit DummyPositionNode(Node& n)
      : target{n},
        state{n.publish<control::PositionState>("/wrist/state")},
        odom{n.publish<geometry::OdometrySample>("/wrist/telemetry")} {}

  void update() {
    const control::PositionTarget* t = target.get();
    const double goal = t != nullptr ? t->position_rot() : 0.0;
    const bool active = t != nullptr;
    state.send(control::PositionState{goal, 0.0, goal, active, active, true});
    odom.send(geometry::OdometrySample{100u, geometry::Pose2d{1.5, -2.5, 0.25},
                                       geometry::Twist2d{3.0, 0.0, 0.5}});
  }

  Target<control::PositionTarget> target;
  Publisher<control::PositionState> state;
  Publisher<geometry::OdometrySample> odom;
};

TEST(StandardMessages, RoundTripControlAndGeometryStructsOnSim) {
  auto sim = Sim::FromToml(kToml);
  sim.add<DummyPositionNode>("wrist");
  SimHardware hw{sim};
  hw.enable();
  hw.set_driver_station(testing::DsPacket(ds::kEnabled | ds::kTeleop));
  sim.step(10ms);

  testing::PublishTarget(sim, "wrist", "teleop",
                         control::PositionTarget{0.375, 0.5});
  sim.step(20ms);

  const auto pos = sim.latest<control::PositionState>("/wrist/state");
  ASSERT_TRUE(pos.has_value());
  const control::PositionState p = pos.value_or(control::PositionState{});
  EXPECT_TRUE(p.enabled());
  EXPECT_TRUE(p.at_goal());
  EXPECT_DOUBLE_EQ(p.position_rot(), 0.375);
  EXPECT_DOUBLE_EQ(p.goal_rot(), 0.375);

  const auto sample = sim.latest<geometry::OdometrySample>("/wrist/telemetry");
  ASSERT_TRUE(sample.has_value());
  const geometry::OdometrySample s =
      sample.value_or(geometry::OdometrySample{});
  EXPECT_EQ(s.sample_time_ns(), 100u);
  EXPECT_DOUBLE_EQ(s.pose().x_m(), 1.5);
  EXPECT_DOUBLE_EQ(s.pose().y_m(), -2.5);
  EXPECT_DOUBLE_EQ(s.pose().yaw_rad(), 0.25);
  EXPECT_DOUBLE_EQ(s.velocity().vx_mps(), 3.0);
  EXPECT_DOUBLE_EQ(s.velocity().omega_radps(), 0.5);
}

}  // namespace
}  // namespace talos::msgs
