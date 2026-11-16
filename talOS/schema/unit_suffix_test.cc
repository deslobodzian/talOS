#include "talOS/schema/unit_suffix.h"

#include <gtest/gtest.h>

#include <string_view>

namespace talos::schema {
namespace {

TEST(UnitSuffixTest, SuffixTableMatchesMethodology) {
  const struct {
    const char* field;
    std::string_view unit;
  } kCases[] = {
      {"sample_time_ns", "ns"},   {"duration_us", "us"},
      {"distance_m", "m"},        {"velocity_mps", "m/s"},
      {"accel_mps2", "m/s²"},     {"heading_rad", "rad"},
      {"omega_radps", "rad/s"},   {"alpha_radps2", "rad/s²"},
      {"position_rot", "rot"},    {"speed_rps", "rot/s"},
      {"accel_rps2", "rot/s²"},   {"voltage_v", "V"},
      {"current_a", "A"},         {"power_w", "W"},
      {"temperature_degc", "°C"}, {"mass_kg", "kg"},
      {"force_n", "N"},           {"torque_nm", "N·m"},
      {"progress_frac", ""},      {"output_duty", ""},
      {"gear_ratio", ""},         {"tick_count", ""},
      {"slot_index", ""},         {"device_id", ""},
      {"fault_mask", ""},         {"pixel_std_px", "px"},
  };
  for (const auto& c : kCases) {
    EXPECT_TRUE(has_unit_suffix(c.field)) << c.field;
    EXPECT_EQ(unit_for_field(c.field), c.unit) << c.field;
  }
}

TEST(UnitSuffixTest, RejectsInvalidNames) {
  EXPECT_FALSE(has_unit_suffix(""));
  EXPECT_FALSE(has_unit_suffix("count"));
  EXPECT_FALSE(has_unit_suffix("_m"));
  EXPECT_FALSE(has_unit_suffix("foo"));
  EXPECT_EQ(unit_for_field("foo"), "");
  EXPECT_EQ(unit_for_field("enabled"), "");
}

}  // namespace
}  // namespace talos::schema
