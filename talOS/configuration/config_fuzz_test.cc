#include <gtest/gtest.h>

#include <sstream>
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

// toml++ 3.4.0 asserted (UB under NDEBUG) when a table header began with a
// character that cannot start a key; third_party/tomlplusplus_parser_ub.patch
// makes these ordinary parse errors. Plausible hand-edit typos.
TEST(ConfigFuzzTest, TableHeaderTyposAreRejectedNotFatal) {
  for (const std::string_view text :
       {std::string_view{"[#motors]\n"}, std::string_view{"[=drive]\n"},
        std::string_view{"[,]\n"}, std::string_view{"[[#x]]\n"},
        std::string_view{"[\0]\n", 4}, std::string_view{"[\xc3\xa9]\n"}}) {
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
