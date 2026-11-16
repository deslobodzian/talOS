// Compile-time checks of the shared messages: building this file is the
// check, so a message that stops being one fails the build.

#include "control/control_generated.h"
#include "geometry/geometry_generated.h"
#include "talOS/node/node.h"

namespace talos::msgs {

// PoseEstimate's shared-memory slot contract is recorded in docs/DATA_FLOW.md.
static_assert(sizeof(geometry::PoseEstimate) == 72,
              "PoseEstimate size changed; update docs/DATA_FLOW.md");
static_assert(alignof(geometry::PoseEstimate) == 8,
              "PoseEstimate alignment changed; update docs/DATA_FLOW.md");

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
