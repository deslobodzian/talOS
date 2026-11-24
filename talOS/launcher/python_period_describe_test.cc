// REQ-RT-23: end-to-end Launcher --describe-only against real talos_py_node
// and hosted talos.cpp_node actuator-owner binaries.
#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/launcher/describe_graph.h"
#include "talOS/launcher/launcher.h"

namespace talos::launcher {
namespace {

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

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream input{path};
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

// REQ-RT-23: a real talos_py_node actuator owner faster than 20 ms is refused
// by Launcher --describe-only (naming the subsystem, its period_us, and the
// 20000 us floor), accepted at 20 ms, and a real hosted talos.cpp_node actuator
// owner is accepted at 5 ms because its update() runs C++.
TEST(LintPythonActuatorPeriods, DescribeOnlyEnforcesPythonFloorOnRealBinaries) {
  namespace fs = std::filesystem;
  const std::string bridge_bin = Runfile("TALOS_BRIDGE");
  const std::string py_motor_bin = Runfile("TALOS_PY_MOTOR");
  const std::string hosted_motor_bin = Runfile("TALOS_HOSTED_MOTOR");

  const fs::path root = fs::path{::testing::TempDir()} /
                        ("python_period_e2e_" + std::to_string(::getpid()));
  std::error_code ec;
  fs::remove_all(root, ec);
  fs::create_directories(root);

  uint64_t next_session_id = 1;
  const auto run_describe_only =
      [&](const std::string& label, const std::string& target,
          const std::string& binary, std::int64_t period_us,
          std::string& graph_json) {
        const fs::path case_dir = root / label;
        fs::create_directories(case_dir);
        const fs::path config = case_dir / "robot.toml";
        std::ofstream{config}
            << "[robot]\n"
            << "name = \"python_period_e2e\"\n"
            << "period_us = 5000\n"
            << "command_timeout_us = 100000\n"
            << "[subsystems.arm]\n"
            << "node = \"" << target << "\"\n"
            << "period_us = " << period_us << "\n"
            << "[subsystems.arm.motors.lift]\n"
            << "type = \"TalonFX\"\n"
            << "bus = \"rio\"\n"
            << "can_id = 1\n"
            << "slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, "
            << "a = 0.0, g = 0.0 }\n";

        const uint64_t session_id = next_session_id++;
        LauncherOptions options;
        options.config_path = config.string();
        options.output_dir = (case_dir / "sessions").string();
        options.session_id = session_id;
        options.describe_only = true;
        options.describe_timeout_ms = 10000;
        options.binary_overrides["//talOS/bridge:hardware_node"] = bridge_bin;
        options.binary_overrides[target] = binary;

        const int rc = Launcher{std::move(options)}.Run();
        graph_json = ReadFile(case_dir / "sessions" /
                              std::to_string(session_id) / "graph.json");
        return rc;
      };

  // Real talos_py_node actuator owner at 10 ms (< 20 ms floor) is refused,
  // naming the subsystem, its period_us, and the 20000 us minimum.
  {
    std::string graph_json;
    const int rc = run_describe_only("py_10ms", "//example/arm:node",
                                     py_motor_bin, 10000, graph_json);
    EXPECT_EQ(rc, 1) << graph_json;
    EXPECT_NE(graph_json.find("\"name\": \"arm\""), std::string::npos)
        << graph_json;
    EXPECT_NE(graph_json.find("\"described\": true"), std::string::npos)
        << graph_json;
    EXPECT_NE(graph_json.find("\"/hw/request/arm\""), std::string::npos)
        << graph_json;
    const std::size_t first_error = graph_json.find("\"severity\": \"error\"");
    ASSERT_NE(first_error, std::string::npos) << graph_json;
    // Exactly one error: the Python actuator period refusal itself.
    EXPECT_EQ(graph_json.find("\"severity\": \"error\"", first_error + 1),
              std::string::npos)
        << graph_json;
    EXPECT_NE(graph_json.find("\"subject\": \"arm\""), std::string::npos)
        << graph_json;
    EXPECT_NE(graph_json.find("period_us 10000"), std::string::npos)
        << graph_json;
    EXPECT_NE(
        graph_json.find("minimum of " +
                        std::to_string(kMinPythonActuatorPeriodUs) + " us"),
        std::string::npos)
        << graph_json;
  }

  // Same real talos_py_node actuator owner at 20 ms is accepted.
  {
    std::string graph_json;
    const int rc = run_describe_only("py_20ms", "//example/arm:node",
                                     py_motor_bin, 20000, graph_json);
    EXPECT_EQ(rc, 0) << graph_json;
    EXPECT_NE(graph_json.find("\"/hw/request/arm\""), std::string::npos)
        << graph_json;
    EXPECT_EQ(graph_json.find("\"severity\": \"error\""), std::string::npos)
        << graph_json;
  }

  // Real hosted talos.cpp_node actuator owner at 5 ms (< 20 ms) is accepted:
  // its update() is C++, so it reports abi_update = false.
  {
    std::string graph_json;
    const int rc = run_describe_only("hosted_5ms", "//example/arm:node",
                                     hosted_motor_bin, 5000, graph_json);
    EXPECT_EQ(rc, 0) << graph_json;
    EXPECT_NE(graph_json.find("\"/hw/request/arm\""), std::string::npos)
        << graph_json;
    EXPECT_EQ(graph_json.find("\"severity\": \"error\""), std::string::npos)
        << graph_json;
  }

  fs::remove_all(root, ec);
}

}  // namespace
}  // namespace talos::launcher
