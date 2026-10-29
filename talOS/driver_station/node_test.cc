#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/driver_station/driver_station_node.h"
#include "talOS/hardware/packet.h"
#include "talOS/node/testing.h"

namespace driver_station {
namespace {

using namespace std::chrono_literals;
using namespace talos::driver_station;  // the codec
using talos::hardware::Packet;
using talos::testing::Sim;

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"runfile unavailable: "} + env + " " +
                             error);
  }
  return runfiles->Rlocation(location);
}

Packet Encoded(const DriverStationData& data) {
  Packet packet{};
  packet.size = static_cast<std::uint32_t>(Encode(data, packet.data));
  return packet;
}

class DriverStationTest : public ::testing::Test {
 protected:
  void Send(const Packet& packet) {
    sim.publish(talos::hardware::kDriverStationTopic, packet);
    sim.step(10ms);
  }
  ::driver_station::DriverStationState State() const {
    const auto state =
        sim.latest<::driver_station::DriverStationState>(topics::kState);
    EXPECT_TRUE(state.has_value());
    return state.value_or(::driver_station::DriverStationState{});
  }

  Sim sim{Runfile("TALOS_TEST_CONFIG")};
  DriverStation& node = sim.add<DriverStation>("driver_station");
};

TEST_F(DriverStationTest, DecodesThePacketAndRepublishesIt) {
  DriverStationData data{};
  data.sample_time_us = 12345000;
  data.fms.flags = kEnabled | kTeleop | kDsAttached;
  data.fms.alliance = Alliance::kRed;
  data.fms.station = 1;
  data.fms.match_type = MatchType::kQualification;
  data.fms.match_number = 42;
  data.fms.replay_number = 1;
  data.fms.match_time_s = 120.5f;
  data.joysticks[0].connected = true;
  data.joysticks[0].axis_count = 4;
  data.joysticks[0].button_count = 8;
  data.joysticks[0].pov_count = 1;
  data.joysticks[0].buttons = 1;
  data.joysticks[0].axes = {0.5f, -0.75f, 0.25f, 0.0f};
  data.joysticks[0].povs[0] = 90;
  data.joysticks[1].connected = true;
  data.joysticks[1].axis_count = 2;
  data.joysticks[1].button_count = 4;
  data.joysticks[1].buttons = 4;

  sim.step(50ms);
  Send(Encoded(data));
  const auto state = State();

  // timestamp_ns is the node clock; the Rio stamp rides separately.
  EXPECT_EQ(state.rio_sample_time_us(), 12345000u);
  EXPECT_GE(state.timestamp_ns(), 50'000'000u);
  EXPECT_LE(state.timestamp_ns(),
            static_cast<std::uint64_t>(sim.now().nanos()));
  EXPECT_EQ(state.fms().flags(), kEnabled | kTeleop | kDsAttached);
  EXPECT_EQ(state.fms().alliance(), static_cast<std::uint8_t>(Alliance::kRed));
  EXPECT_EQ(state.fms().station(), 1);
  EXPECT_EQ(state.fms().match_type(),
            static_cast<std::uint8_t>(MatchType::kQualification));
  EXPECT_EQ(state.fms().match_number(), 42);
  EXPECT_EQ(state.fms().replay_number(), 1);
  EXPECT_FLOAT_EQ(state.fms().match_time_s(), 120.5f);

  EXPECT_TRUE(state.stick0().connected());
  EXPECT_EQ(state.stick0().axis_count(), 4);
  EXPECT_EQ(state.stick0().button_count(), 8);
  EXPECT_EQ(state.stick0().buttons(), 1u);
  EXPECT_FLOAT_EQ(state.stick0().axis0(), 0.5f);
  EXPECT_FLOAT_EQ(state.stick0().axis1(), -0.75f);
  EXPECT_FLOAT_EQ(state.stick0().axis2(), 0.25f);
  EXPECT_EQ(state.stick0().pov0(), 90);
  EXPECT_TRUE(state.stick1().connected());
  EXPECT_EQ(state.stick1().buttons(), 4u);
}

TEST_F(DriverStationTest, PublishesOnlyTheFirstTwoJoysticks) {
  // The wire carries kMaxJoysticks sticks; the state keeps stick0 and stick1.
  static_assert(kMaxJoysticks == 6);
  DriverStationData data{};
  data.fms.flags = kEnabled | kTeleop;
  for (std::size_t i = 0; i < kMaxJoysticks; ++i) {
    data.joysticks[i].connected = true;
    data.joysticks[i].axis_count = 4;
    data.joysticks[i].buttons = 1u << i;
    data.joysticks[i].axes[1] = -0.1f * static_cast<float>(i + 1);
  }
  Send(Encoded(data));
  EXPECT_EQ(State().stick0().buttons(), 1u);
  EXPECT_EQ(State().stick1().buttons(), 2u);
  EXPECT_FLOAT_EQ(State().stick0().axis1(), -0.1f);
  EXPECT_FLOAT_EQ(State().stick1().axis1(), -0.2f);
}

TEST_F(DriverStationTest, IgnoresACorruptPacket) {
  Packet corrupt{};
  corrupt.size = 2;  // too small to decode
  corrupt.data[0] = 0xFF;
  corrupt.data[1] = 0xFF;
  Send(corrupt);
  EXPECT_FALSE(
      sim.latest<::driver_station::DriverStationState>(topics::kState));
}

struct Result {
  int status;
  std::string output;
};

Result RunNode(const std::string& args) {
  const std::string command = "'" + Runfile("TALOS_TEST_NODE") + "' " + args +
                              " --config '" + Runfile("TALOS_TEST_CONFIG") +
                              "' 2>&1";
  FILE* pipe = ::popen(command.c_str(), "r");
  if (pipe == nullptr) throw std::runtime_error("popen failed");
  Result result{0, {}};
  char buffer[4096];
  while (const std::size_t n = std::fread(buffer, 1, sizeof buffer, pipe)) {
    result.output.append(buffer, n);
  }
  const int status = ::pclose(pipe);
  result.status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
  return result;
}

TEST(DriverStationBinary, LoggedSimRunReplaysWithoutDivergence) {
  const std::string log =
      std::string{std::getenv("TEST_TMPDIR")} + "/driver_station_test.tlog";
  const Result run = RunNode("--sim --duration-s 1 --log '" + log + "'");
  ASSERT_EQ(run.status, 0) << run.output;
  const Result replay = RunNode("--replay '" + log + "'");
  EXPECT_EQ(replay.status, 0) << replay.output;
  EXPECT_NE(replay.output.find("diverged=0 clean_exit=1"), std::string::npos)
      << replay.output;
}

}  // namespace
}  // namespace driver_station
