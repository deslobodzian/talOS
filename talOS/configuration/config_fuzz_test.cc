#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/protocol/fuzz_corpus.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, std::size_t size) {
  // Empty vectors may have a null data pointer; string_view requires a range.
  const std::string_view input(size ? reinterpret_cast<const char*>(data) : "",
                               size);
  talos::config::RobotConfig parsed;
  try {
    parsed = talos::config::ParseRobotConfigString(input);
  } catch (const std::invalid_argument&) {
    // Syntax and semantic rejection is the parser's documented contract.
    return 0;
  }
  std::ostringstream encoded;
  encoded << toml::toml_formatter{parsed.toml_data};
  const auto again = talos::config::ParseRobotConfigString(encoded.str());
  std::ostringstream canonical;
  canonical << toml::toml_formatter{again.toml_data};
  EXPECT_EQ(canonical.str(), encoded.str());
  return 0;
}

#ifndef TALOS_LIBFUZZER
namespace {

constexpr const char* kRichConfigSeed = R"([robot]
name = "bench"
period_us = 5000
command_timeout_us = 100000
status_hz = 200.0
commissioned = false
require_target_coverage = true
controller_address = "127.0.0.1"
local_address = "10.0.0.2"
controller_port = 5802
local_port = 5803
log_keep_sessions = 10
log_max_bytes = 1048576

[robot.targets]
teleop = ["teleop"]
autonomous = ["auto"]

[studio]
http_port = 5800
max_clients = 4
topic = "/talos/telemetry"

[simulation]
wheel_radius_m = 0.0508

[simulation.battery]
nominal_voltage_v = 12.6
internal_resistance_ohm = 0.02
base_current_a = 1.5
brownout_voltage_v = 6.75

[simulation.chassis]
mass_kg = 54.0
moment_of_inertia_kg_m2 = 6.0
traction_coefficient = 1.1

[[simulation.swerve_modules]]
drive = "fl_drive"
steer = "fl_steer"
x_m = 0.3
y_m = 0.3

[[simulation.hard_stops]]
motor = "arm_motor"
min_rot = -0.5
max_rot = 2.5

[[simulation.mechanisms]]
motor = "arm_motor"
inertia_kg_m2 = 0.08
load = "flywheel"
viscous_damping_nm_per_radps = 0.01
static_friction_nm = 0.05

[subsystems.drive]
node = "//bench/main_processor/swerve_module:node"
critical = true

[subsystems.drive.motors.fl_drive]
type = "TalonFX"
can_id = 1
foc = true
max_torque_current_a = 40.0
slot0 = { p = 1.0, gravity_type = "Elevator" }

[subsystems.drive.motors.fl_steer]
can_id = 2
feedback = "RemoteCANcoder"
feedback_sensor = "fl_enc"
position_frame = "absolute"
soft_limits = true

[subsystems.drive.sensors.fl_enc]
type = "CANcoder"
can_id = 3
offset_rot = 0.125

[subsystems.drive.geometry]
modules = [{ drive = "fl_drive", steer = "fl_steer", encoder = "fl_enc" }]

[subsystems.arm.motors.arm_motor]
can_id = 4

[subsystems.arm.motors.arm_follower]
can_id = 5
follow = "arm_motor"
oppose_master = true

[subsystems.arm.digital_inputs.home]
dio = 0
sim_motor = "arm_motor"
sim_closed_at_rot = 0.0

[subsystems.arm.encoders.quad]
type = "QuadratureEncoder"
dio_a = 2
dio_b = 3

[subsystems.arm.teleop]
defaults = { velocity_rps = 0.0, enabled = false }

[[subsystems.arm.teleop.bindings]]
button = 1
while_held = { velocity_rps = 5.0 }

[[subsystems.arm.teleop.bindings]]
axis = 1
field = "velocity_rps"
scale = 10.0
deadband_frac = 0.05
)";

constexpr std::string_view kHostileValues[] = {
    "nan",
    "inf",
    "-inf",
    "1e308",
    "-1e308",
    "1e-320",
    "1e400",
    "9223372036854775807",
    "-9223372036854775808",
    "9223372036854775808",
    "-9223372036854775809",
    "0",
    "-1",
    "-0.0",
    "65536",
    "\"\"",
    "\"../escape\"",
    "\"a\\u0000b\"",
    "1979-05-27T:32:00Z",
    "[]",
    "{}",
};

constexpr std::string_view kHostileKeys[] = {
    "robot",
    "studio",
    "simulation",
    "subsystems",
    "include",
    "motors",
    "sensors",
    "can_id",
    "period_us",
    "command_timeout_us",
    "controller_address",
    "dashboard",
    "deadband_frac",
    "max_torque_current_a",
    "soft_limits",
    "follow",
    "battery",
    "chassis",
    "mechanisms",
    "nominal_voltage_v",
    "brownout_voltage_v",
    "mass_kg",
    "inertia_kg_m2",
    "load",
};

std::string MutateStructureAware(std::string input, std::mt19937_64& rng) {
  const int ops = 1 + static_cast<int>(rng() % 4);
  for (int op = 0; op < ops; ++op) {
    switch (rng() % 6) {
      case 0: {
        std::vector<std::size_t> eqs;
        for (std::size_t i = 0; i < input.size(); ++i) {
          if (input[i] == '=') eqs.push_back(i);
        }
        if (!eqs.empty()) {
          const std::size_t pos = eqs[rng() % eqs.size()] + 1;
          std::size_t end = input.find_first_of("\n,}", pos);
          if (end == std::string::npos) end = input.size();
          const auto val = kHostileValues[rng() % std::size(kHostileValues)];
          input.replace(pos, end - pos, " " + std::string(val));
        }
        break;
      }
      case 1: {
        if (!input.empty()) {
          const std::size_t p = rng() % input.size();
          const std::size_t prev = input.rfind('\n', p);
          const std::size_t start = (prev == std::string::npos) ? 0 : prev + 1;
          std::size_t end = input.find('\n', p);
          if (end == std::string::npos) end = input.size();
          input.insert(end, "\n" + input.substr(start, end - start));
        }
        break;
      }
      case 2: {
        static constexpr std::string_view kBadBytes[] = {
            std::string_view("\0", 1),
            "\x01",
            "\x7f",
            "\x80",
            "\xc0\xaf",
            "\xed\xa0\x80",
            "\xff\xfe",
        };
        const std::size_t pos = input.empty() ? 0 : rng() % (input.size() + 1);
        input.insert(pos, kBadBytes[rng() % std::size(kBadBytes)]);
        break;
      }
      case 3: {
        if (!input.empty()) {
          const std::size_t pos = rng() % input.size();
          if (rng() % 2 == 0) {
            input[pos] = static_cast<char>(
                static_cast<unsigned char>(input[pos]) ^ (1u << (rng() % 8)));
          } else {
            const std::size_t len =
                1 + (rng() % std::min<std::size_t>(16, input.size() - pos));
            input.erase(pos, len);
          }
        }
        break;
      }
      case 4: {
        const std::size_t pos = input.empty() ? 0 : rng() % (input.size() + 1);
        std::size_t nl = input.find('\n', pos);
        if (nl == std::string::npos) nl = input.size();
        const auto k = kHostileKeys[rng() % std::size(kHostileKeys)];
        const auto v = kHostileValues[rng() % std::size(kHostileValues)];
        input.insert(nl, "\n" + std::string(k) + " = " + std::string(v) + "\n");
        break;
      }
      default: {
        if (input.size() > 1) input.resize(rng() % input.size());
        break;
      }
    }
  }
  return input;
}

void RunOne(std::string_view text) {
  LLVMFuzzerTestOneInput(reinterpret_cast<const uint8_t*>(text.data()),
                         text.size());
}

}  // namespace

TEST(ConfigFuzzTest, BoundedMutations) {
  std::vector<std::vector<uint8_t>> seeds;
  // Every seed is a complete, accepted config, so mutations reach semantic
  // validation and not only the TOML syntax layer.
  for (const auto* text :
       {"[robot]\nperiod_us = 10000\n"
        "[subsystems.drive.motors.left]\ntype = 'TalonFX'\ncan_id = 1\n",
        "[robot]\nperiod_us = 10000\ncommand_timeout_us = 100000\n"
        "[subsystems.arm.motors.joint]\ncan_id = 2\nmax_velocity_rps = 10.0\n"
        "[subsystems.arm.sensors.encoder]\ncan_id = 3\n",
        "[robot.targets]\nteleop = ['teleop']\nautonomous = ['auto']\n"
        "[subsystems.drive.motors.left]\ncan_id = 1\n"}) {
    // Seeds come from TOML's encoder, not hand-crafted wire fragments.
    const auto table = toml::parse(text);
    std::ostringstream encoded;
    encoded << toml::toml_formatter{table};
    EXPECT_NO_THROW(talos::config::ParseRobotConfigString(encoded.str()))
        << encoded.str();
    const auto bytes = encoded.str();
    seeds.emplace_back(bytes.begin(), bytes.end());
  }
  talos::protocol::testing::RunCorpus(seeds, LLVMFuzzerTestOneInput);
}

TEST(ConfigFuzzTest, SeededStructureAwareMutationsAndEdgeCases) {
  ASSERT_NO_THROW(talos::config::ParseRobotConfigString(kRichConfigSeed));

  std::mt19937_64 rng(0x5eedcafe1234abcdULL);
  for (int i = 0; i < 1500; ++i) {
    RunOne(MutateStructureAware(kRichConfigSeed, rng));
  }

  // Hostile numeric, UTF-8, NUL, and duplicate-key cases.
  for (const std::string_view value : kHostileValues) {
    RunOne(std::string("[robot]\nperiod_us = ") + std::string(value) +
           "\n[subsystems.a.motors.m]\ncan_id = " + std::string(value) +
           "\n[subsystems.a.teleop]\ndefaults = { v = " + std::string(value) +
           " }\n");
  }
  for (const std::string_view bad : {
           std::string_view("a = \"x\0y\"\n", 9),
           std::string_view(
               "[subsystems.a]\nk = \"\x80\xc0\xaf\xed\xa0\x80\"\n"),
           std::string_view("[robot]\nperiod_us = 5000\nperiod_us = 5000\n"),
       }) {
    RunOne(bad);
  }

  // Deep nesting across arrays, inline tables, headers, and dotted keys
  // (including combined header + dotted key depth that previously broke
  // round-trip formatting or overflowed the stack).
  RunOne("a = " + std::string(300, '[') + "1" + std::string(300, ']') + "\n");
  {
    std::string inline_tbl = "a = ";
    for (int i = 0; i < 300; ++i) inline_tbl += "{k=";
    inline_tbl += "1" + std::string(300, '}');
    RunOne(inline_tbl);
  }
  {
    std::string deep_header = "[subsystems";
    for (int i = 0; i < 5000; ++i) deep_header += ".p";
    deep_header += "]\nk = 1\n";
    RunOne(deep_header);
  }
  {
    std::string split_depth = "[subsystems";
    for (int i = 0; i < 240; ++i) split_depth += ".a";
    split_depth += "]\n";
    for (int i = 0; i < 25; ++i) split_depth += "b.";
    split_depth += "k = 1\n";
    RunOne(split_depth);
  }
  {
    std::string deep_inline = "[subsystems";
    for (int i = 0; i < 254; ++i) deep_inline += ".a";
    deep_inline += "]\nx.y = { z = 1 }\n";
    RunOne(deep_inline);
  }

  // Huge strings and oversized device counts stay bounded and throw only
  // std::invalid_argument.
  RunOne("[subsystems.a]\nnode = \"" + std::string(65536, 'x') +
         "\"\n[subsystems.a.motors.m]\ncan_id = 1\n");
  {
    std::string many_motors;
    for (int i = 0; i < 200; ++i) {
      many_motors += "[subsystems.a.motors.m" + std::to_string(i) +
                     "]\ncan_id = " + std::to_string(i % 63) +
                     "\nfollow = \"m199\"\n";
    }
    RunOne(many_motors);
  }
}

TEST(ConfigFuzzTest, MalformedAndSymlinkPathsThrowInvalidArgument) {
  namespace fs = std::filesystem;
  const fs::path root = fs::path(::testing::TempDir()) / "config_fuzz_paths";
  std::error_code ec;
  fs::remove_all(root, ec);
  fs::create_directories(root / "subsystems", ec);
  {
    std::ofstream(root / "subsystems" / "drive.toml")
        << "[subsystem]\nname = \"drive\"\n[motors.m]\ncan_id = 1\n";
  }
  fs::create_symlink(root / "subsystems" / "loop_b.toml",
                     root / "subsystems" / "loop_a.toml", ec);
  fs::create_symlink(root / "subsystems" / "loop_a.toml",
                     root / "subsystems" / "loop_b.toml", ec);

  const fs::path robot = root / "robot.toml";
  for (const std::string& content : {
           std::string("[subsystems.drive.trajectories]\nt = "
                       "\"subsystems/loop_a.toml\"\n"
                       "[subsystems.drive.motors.m]\ncan_id = 1\n"),
           std::string("[studio]\ndashboard = \"subsystems/loop_a.toml\"\n"
                       "[subsystems.drive.motors.m]\ncan_id = 1\n"),
           std::string("[subsystems.drive.trajectories]\nt = \"") +
               std::string(5000, 'a') +
               "\"\n[subsystems.drive.motors.m]\ncan_id = 1\n",
           std::string("[studio]\ndashboard = \"") + std::string(5000, 'a') +
               "\"\n[subsystems.drive.motors.m]\ncan_id = 1\n",
           std::string("include = \"subsystems\"\n"
                       "[subsystems.drive.motors.m]\ncan_id = 1\n"),
           std::string("include = \"../outside.toml\"\n"),
       }) {
    {
      std::ofstream out(robot, std::ios::binary | std::ios::trunc);
      out.write(content.data(), static_cast<std::streamsize>(content.size()));
    }
    EXPECT_THROW(talos::config::ParseRobotConfig(robot.string()),
                 std::invalid_argument)
        << content;
  }

  // Embedded NUL byte in path must be rejected as std::invalid_argument.
  EXPECT_THROW(talos::config::ParseRobotConfigString(
                   std::string("[studio]\ndashboard = \"a\\u0000b\"\n"
                               "[subsystems.drive.motors.m]\ncan_id = 1\n")),
               std::invalid_argument);
  EXPECT_THROW(talos::config::ParseRobotConfigString(std::string(
                   "[subsystems.drive.trajectories]\nt = \"a\\u0000b\"\n"
                   "[subsystems.drive.motors.m]\ncan_id = 1\n")),
               std::invalid_argument);

  fs::remove_all(root, ec);
}

// toml++ 3.4.0 asserted (UB under NDEBUG) when a table header began with a
// character that cannot start a key, or when a date-time separator was not
// followed by an hour digit; third_party/tomlplusplus_parser_ub.patch makes
// these ordinary parse errors.
TEST(ConfigFuzzTest, ParserUbGuardsRejectMalformedInputsSafely) {
  for (const std::string_view text :
       {std::string_view{"[#motors]\n"}, std::string_view{"[=drive]\n"},
        std::string_view{"[,]\n"}, std::string_view{"[[#x]]\n"},
        std::string_view{"[\0]\n", 4}, std::string_view{"[\xc3\xa9]\n"},
        std::string_view{"d = 1979-05-27T:32:00Z\n"},
        std::string_view{"d = 1979-05-27  00:00\n"},
        std::string_view{"d = 1979-05-27t+00:00\n"}}) {
    EXPECT_THROW(talos::config::ParseRobotConfigString(text),
                 std::invalid_argument)
        << text;
  }
}

// toml++ 3.4.0's whitespace classifier reached TOML_UNREACHABLE for U+00A1..
// U+049A and other unfiltered buckets, so these comments (the 2026 robot's
// vision.toml has the first two) were UB on every parse. UBSan
// (--config=asan) fails this test without third_party/tomlplusplus_parser_
// ub.patch.
TEST(ConfigFuzzTest, NonAsciiCommentsParse) {
  EXPECT_NO_THROW(talos::config::ParseRobotConfigString(
      "# \xc2\xb1"
      "1 deg \xe2\x89\x88 0.0175 rad, 5 \xc2\xb5s, \xe3\x80\x81\n"
      "[subsystems.drive.motors.left] # \xc2\xb0\n"
      "can_id = 1\n"));
}
#endif
