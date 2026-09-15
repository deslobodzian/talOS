#include <gtest/gtest.h>

#include <cstdint>

#include "talos_nodes/drivetrain/drive_message_generated.h"
#include "talos_nodes/drivetrain/packet.h"

namespace talos::drive {
namespace {

// Wire-compat sentinel: these structs cross every chassis topic on the robot,
// so a layout change is a new contract by policy (whole-bundle rollback),
// never a silent edit. FlatBuffers structs are tightly packed by field
// alignment: ChassisTarget is 3 doubles + int64 + bool padded to 40,
// DrivetrainState is uint64 + bool (7 pad) + 5 doubles = 56.
TEST(ChassisContract, WireSizesAreFrozen) {
  EXPECT_EQ(sizeof(ChassisTarget), 40u);
  EXPECT_EQ(sizeof(DrivetrainState), 56u);
}

TEST(ChassisContract, TopicsAreSpelledOnce) {
  EXPECT_STREQ(kTargetTopic, "/drivetrain/target");
  EXPECT_STREQ(kTeleopTargetTopic, "/drivetrain/target/teleop");
  EXPECT_STREQ(kAutoTargetTopic, "/drivetrain/target/auto");
  EXPECT_STREQ(kDrivetrainStateTopic, "/drivetrain/state");
  EXPECT_EQ(kDriveRequestTopic, "/hw/request/drivetrain");
}

}  // namespace
}  // namespace talos::drive
