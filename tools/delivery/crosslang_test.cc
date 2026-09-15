// C++ side of the cross-language wire test (item 50).
//
// Decodes the golden files produced by the real flatc-generated Python
// bindings (tools/delivery/gen_goldens.py) with the real flatc-generated
// C++ headers, then re-encodes the same values and requires byte-identical
// output. Python-bytes -> C++-decoder is the forward direction; C++-encoder
// -> Python-pinned-bytes is the reverse. Either generator drifting fails
// here without touching the other side's code.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "talos_nodes/drivetrain/drive_message_generated.h"

namespace talos::delivery {
namespace {

std::string TestDataDir() {
  const char* srcdir = std::getenv("TEST_SRCDIR");
  std::string base = srcdir ? srcdir : ".";
  return base + "/_main/tools/delivery/testdata/";
}

std::vector<uint8_t> LoadGolden(const std::string& name) {
  std::ifstream in(TestDataDir() + name, std::ios::binary);
  EXPECT_TRUE(in.is_open()) << "missing golden file: " << name;
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(in),
                              std::istreambuf_iterator<char>());
}

// The probe values from goldens.json. Duplicated by value (not by code) so
// this test and the Python side can only agree through the golden bytes.
constexpr double kVx = 1.5, kVy = -0.5, kOmega = 3.141592653589793;
constexpr int64_t kIssuedNs = 123456789;

TEST(CrossLangTest, PythonTargetDecodesInCpp) {
  const auto wire = LoadGolden("chassis_target_golden.bin");
  ASSERT_EQ(wire.size(), 48u);  // 4 root + 4 pad + 40 struct
  const auto* t =
      flatbuffers::GetRoot<drive::ChassisTarget>(wire.data());
  EXPECT_DOUBLE_EQ(t->vx_mps(), kVx);
  EXPECT_DOUBLE_EQ(t->vy_mps(), kVy);
  EXPECT_DOUBLE_EQ(t->omega_radps(), kOmega);
  EXPECT_EQ(t->issued_ns(), kIssuedNs);
  EXPECT_TRUE(t->enabled());
}

TEST(CrossLangTest, CppTargetEncodesToPythonBytes) {
  const auto golden = LoadGolden("chassis_target_golden.bin");
  flatbuffers::FlatBufferBuilder builder(64);
  auto off = builder.CreateStruct(
      drive::ChassisTarget(kVx, kVy, kOmega, kIssuedNs, true));
  builder.Finish(off);
  const uint8_t* buf = builder.GetBufferPointer();
  const std::vector<uint8_t> encoded(buf, buf + builder.GetSize());
  EXPECT_EQ(encoded, golden);
}

TEST(CrossLangTest, PythonStateDecodesInCpp) {
  const auto wire = LoadGolden("drivetrain_state_golden.bin");
  ASSERT_EQ(wire.size(), 64u);  // 4 root + 4 pad + 56 struct
  const auto* s =
      flatbuffers::GetRoot<drive::DrivetrainState>(wire.data());
  EXPECT_EQ(s->timestamp_ns(), 987654321u);
  EXPECT_TRUE(s->enabled());
  EXPECT_DOUBLE_EQ(s->vx_mps(), 2.0);
  EXPECT_DOUBLE_EQ(s->vy_mps(), -1.0);
  EXPECT_DOUBLE_EQ(s->omega_radps(), 0.5);
  EXPECT_DOUBLE_EQ(s->yaw_rot(), 1.25);
  EXPECT_DOUBLE_EQ(s->yaw_rate_rps(), -0.2);
}

TEST(CrossLangTest, CppStateEncodesToPythonBytes) {
  const auto golden = LoadGolden("drivetrain_state_golden.bin");
  flatbuffers::FlatBufferBuilder builder(80);
  auto off = builder.CreateStruct(drive::DrivetrainState(
      987654321u, true, 2.0, -1.0, 0.5, 1.25, -0.2));
  builder.Finish(off);
  const uint8_t* buf = builder.GetBufferPointer();
  const std::vector<uint8_t> encoded(buf, buf + builder.GetSize());
  EXPECT_EQ(encoded, golden);
}

}  // namespace
}  // namespace talos::delivery
