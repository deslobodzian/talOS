#include <gtest/gtest.h>

#include <chrono>

#include "control/control_generated.h"
#include "geometry/geometry_generated.h"
#include "talOS/node/node.h"
#include "talOS/node/testing.h"

namespace talos::msgs {
namespace {

using namespace std::chrono_literals;
using testing::Sim;

static_assert(Message<geometry::Translation2d>);
static_assert(Message<geometry::Rotation2d>);
static_assert(Message<geometry::Pose2d>);
static_assert(Message<geometry::Twist2d>);
static_assert(Message<geometry::OdometrySample>);
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
      : target{n.subscribe<control::PositionTarget>("/wrist/target",
                                                    {.optional = true})},
        state{n.publish<control::PositionState>("/wrist/state")},
        odom{n.publish<geometry::OdometrySample>("/wrist/telemetry")} {}

  void update() {
    const bool active = target.fresh() && target->enabled();
    const double goal = active ? target->position_rot() : 0.0;
    state.send(control::PositionState{goal, 0.0, goal, active, active, true});
    odom.send(geometry::OdometrySample{100u, geometry::Pose2d{1.5, -2.5, 0.25},
                                       geometry::Twist2d{3.0, 0.0, 0.5}});
  }

  Subscription<control::PositionTarget> target;
  Publisher<control::PositionState> state;
  Publisher<geometry::OdometrySample> odom;
};

TEST(StandardMessages, RoundTripControlAndGeometryStructsOnSim) {
  auto sim = Sim::FromToml(kToml);
  sim.add<DummyPositionNode>("wrist");

  sim.publish("/wrist/target", control::PositionTarget{0.375, 0.5, true});
  sim.step(20ms);

  const auto pos = sim.latest<control::PositionState>("/wrist/state");
  ASSERT_TRUE(pos.has_value());
  EXPECT_TRUE(pos->enabled());
  EXPECT_TRUE(pos->at_goal());
  EXPECT_DOUBLE_EQ(pos->position_rot(), 0.375);
  EXPECT_DOUBLE_EQ(pos->goal_rot(), 0.375);

  const auto sample = sim.latest<geometry::OdometrySample>("/wrist/telemetry");
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->timestamp_ns(), 100u);
  EXPECT_DOUBLE_EQ(sample->pose().x_m(), 1.5);
  EXPECT_DOUBLE_EQ(sample->pose().y_m(), -2.5);
  EXPECT_DOUBLE_EQ(sample->pose().yaw_rot(), 0.25);
  EXPECT_DOUBLE_EQ(sample->velocity().vx_mps(), 3.0);
  EXPECT_DOUBLE_EQ(sample->velocity().omega_radps(), 0.5);
}

}  // namespace
}  // namespace talos::msgs
