#include <gtest/gtest.h>
#include <sys/wait.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "localizer/localizer.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/devices/testing.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/node/testing.h"

namespace localizer {
namespace {

using namespace std::chrono_literals;
using talos::testing::AllocationProbe;
using talos::testing::Sim;
using talos::testing::SimHardware;

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

// The node test matrix for an observer, docs/TESTING.md §7: the case names
// are the matrix's, which //tools:check_node_matrix looks for.
class LocalizerTest : public ::testing::Test {
 protected:
  using State = Localizer::State;
  using Observation = Localizer::Observation;

  // A camera's observation of (x, y), taken now.
  void Observe(const char* camera, double x, double y, bool valid = true,
               double std_m = 0.1) {
    sim.publish(
        std::string{"/"} + camera + "/state",
        Observation{static_cast<std::uint64_t>(sim.now().nanos()),
                    geometry::Pose2d{x, y, 0.5}, std_m, std_m, 0.05, 2, valid});
  }
  State Latest() const {
    const auto state = sim.latest<State>(topics::kState);
    EXPECT_TRUE(state.has_value());
    return state.value_or(State{});
  }

  Sim sim{Runfile("TALOS_TEST_CONFIG")};
  Localizer& node = sim.add<Localizer>("localizer");
  SimHardware hw{sim};
};

TEST_F(LocalizerTest, PublishesFromALiveInput) {
  sim.step(20ms);
  Observe("front_camera", 1.0, 2.0);
  sim.step(40ms);
  const State state = Latest();
  EXPECT_TRUE(state.valid());
  EXPECT_DOUBLE_EQ(state.pose().x_m(), 1.0);
  EXPECT_DOUBLE_EQ(state.pose().y_m(), 2.0);
  EXPECT_EQ(state.observation_accepted_count(), 1u);  // counted once
}

TEST_F(LocalizerTest, EstimatesWhileTheRobotIsDisabled) {
  hw.enable(false);  // nothing here is actuation-gated
  sim.step(20ms);
  Observe("back_camera", 3.0, 1.0);
  sim.step(40ms);
  EXPECT_TRUE(Latest().valid());
  EXPECT_DOUBLE_EQ(Latest().pose().x_m(), 3.0);
}

// REQ-CONTRACT-25
TEST_F(LocalizerTest, StaleInputClearsValidity) {
  sim.step(20ms);
  Observe("front_camera", 1.0, 2.0);
  sim.step(40ms);
  ASSERT_TRUE(Latest().valid());
  sim.step(600ms);  // past observation_timeout_us
  const State state = Latest();
  EXPECT_FALSE(state.valid());
  EXPECT_DOUBLE_EQ(state.pose().x_m(), 1.0);
}

TEST_F(LocalizerTest, RejectsUnusableObservations) {
  sim.step(20ms);
  Observe("front_camera", 1.0, 2.0, /*valid=*/false);
  sim.step(40ms);
  Observe("front_camera", 1.0, 2.0, true,
          std::numeric_limits<double>::quiet_NaN());
  sim.step(40ms);
  const State state = Latest();
  EXPECT_FALSE(state.valid());
  EXPECT_EQ(state.observation_rejected_count(), 2u);
  EXPECT_EQ(state.observation_accepted_count(), 0u);
}

TEST_F(LocalizerTest, ReadsOnlyTheListedCameras) {
  sim.step(20ms);
  Observe("side_camera", 1.0, 2.0);  // not in `cameras`
  sim.step(40ms);
  EXPECT_FALSE(Latest().valid());
  EXPECT_EQ(Latest().observation_accepted_count(), 0u);
}

TEST_F(LocalizerTest, CountsEveryObservationBetweenTicks) {
  sim.step(20ms);
  Observe("front_camera", 1.0, 2.0);
  Observe("front_camera", 1.0, 2.0, /*valid=*/false);
  Observe("front_camera", 1.5, 2.0);
  sim.step(40ms);
  const State state = Latest();
  EXPECT_EQ(state.observation_accepted_count(), 2u);
  EXPECT_EQ(state.observation_rejected_count(), 1u);
}

// REQ-CONTRACT-18
TEST(LocalizerParameters, BadParameterFailsConstruction) {
  const struct {
    const char* key;
    const char* settings;  // [subsystems.localizer]
  } rows[] = {
      {"cameras", "observation_timeout_us = 500000"},
      {"cameras", "cameras = 3"},
      {"observation_timeout_us",
       "cameras = ['front_camera']\nobservation_timeout_us = 0"},
  };
  for (const auto& row : rows) {
    SCOPED_TRACE(row.settings);
    Sim sim = Sim::FromToml(
        std::string{"[robot]\nperiod_us = 20000\ncommand_timeout_us = 100000\n"
                    "[subsystems.localizer]\n"} +
        row.settings +
        "\n[subsystems.chassis.motors.chassis_motor]\n"
        "type = 'TalonFX'\ncan_id = 1\n");
    try {
      sim.add<Localizer>("localizer");
      ADD_FAILURE() << "constructed";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string{error.what()}.find(row.key), std::string::npos)
          << error.what();
    }
  }
}

// REQ-RT-06: a tick that drains a queued observation, picks it and publishes
// the estimate allocates nothing.
TEST_F(LocalizerTest, UpdateDoesNotAllocate) {
  if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  sim.step(40ms);
  Observe("front_camera", 1.0, 2.0);
  AllocationProbe probe;
  probe.start();
  sim.step(60ms);
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  EXPECT_EQ(Latest().observation_accepted_count(), 1u);
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

TEST(LocalizerBinary, LoggedSimRunReplaysWithoutDivergence) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string log =
      std::string{tmp != nullptr ? tmp : "/tmp"} + "/node_test.tlog";
  const Result run = RunNode("--sim --duration-s 1 --log '" + log + "'");
  ASSERT_EQ(run.status, 0) << run.output;
  const Result replay = RunNode("--replay '" + log + "'");
  EXPECT_EQ(replay.status, 0) << replay.output;
  EXPECT_NE(replay.output.find("diverged=0 clean_exit=1"), std::string::npos)
      << replay.output;
}

}  // namespace
}  // namespace localizer
