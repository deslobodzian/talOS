// Compile-time checks of the shared messages: building this file is the
// check, so a message that stops being one or changes layout fails the build.

#include "control/control_generated.h"
#include "geometry/geometry_generated.h"
#include "talOS/node/node.h"

namespace talos::msgs {

// Size and alignment contracts for every shared message (RULES.md 9;
// PoseObservation, PoseEstimate, and OdometrySample slots are recorded in
// docs/DATA_FLOW.md).
static_assert(sizeof(geometry::Translation2d) == 16);
static_assert(alignof(geometry::Translation2d) == 8);
static_assert(sizeof(geometry::Rotation2d) == 8);
static_assert(alignof(geometry::Rotation2d) == 8);
static_assert(sizeof(geometry::Pose2d) == 24);
static_assert(alignof(geometry::Pose2d) == 8);
static_assert(sizeof(geometry::Twist2d) == 24);
static_assert(alignof(geometry::Twist2d) == 8);
static_assert(sizeof(geometry::SwerveModulePosition) == 16);
static_assert(alignof(geometry::SwerveModulePosition) == 8);
static_assert(sizeof(geometry::OdometrySample) == 56,
              "OdometrySample size changed; update docs/DATA_FLOW.md");
static_assert(alignof(geometry::OdometrySample) == 8,
              "OdometrySample alignment changed; update docs/DATA_FLOW.md");
static_assert(sizeof(geometry::PoseObservation) == 64,
              "PoseObservation size changed; update docs/DATA_FLOW.md");
static_assert(alignof(geometry::PoseObservation) == 8,
              "PoseObservation alignment changed; update docs/DATA_FLOW.md");
static_assert(sizeof(geometry::PoseEstimate) == 72,
              "PoseEstimate size changed; update docs/DATA_FLOW.md");
static_assert(alignof(geometry::PoseEstimate) == 8,
              "PoseEstimate alignment changed; update docs/DATA_FLOW.md");
static_assert(sizeof(control::VelocityTarget) == 16);
static_assert(alignof(control::VelocityTarget) == 8);
static_assert(sizeof(control::VelocityState) == 32);
static_assert(alignof(control::VelocityState) == 8);
static_assert(sizeof(control::PositionTarget) == 16);
static_assert(alignof(control::PositionTarget) == 8);
static_assert(sizeof(control::PositionState) == 32);
static_assert(alignof(control::PositionState) == 8);

// Every shared message can cross a topic (talos::Message).
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

}  // namespace talos::msgs

int main() { return 0; }
