#include "talOS/schema/unit_suffix.h"

#include <gtest/gtest.h>

namespace talos::schema {
namespace {

TEST(UnitSuffixTest, SuffixTableMatchesMethodology) {
  EXPECT_TRUE(has_unit_suffix("sample_time_ns"));
  EXPECT_EQ(unit_for_field("sample_time_ns"), "ns");

  EXPECT_TRUE(has_unit_suffix("duration_us"));
  EXPECT_EQ(unit_for_field("duration_us"), "us");

  EXPECT_TRUE(has_unit_suffix("distance_m"));
  EXPECT_EQ(unit_for_field("distance_m"), "m");

  EXPECT_TRUE(has_unit_suffix("velocity_mps"));
  EXPECT_EQ(unit_for_field("velocity_mps"), "m/s");

  EXPECT_TRUE(has_unit_suffix("accel_mps2"));
  EXPECT_EQ(unit_for_field("accel_mps2"), "m/s²");

  EXPECT_TRUE(has_unit_suffix("heading_rad"));
  EXPECT_EQ(unit_for_field("heading_rad"), "rad");

  EXPECT_TRUE(has_unit_suffix("omega_radps"));
  EXPECT_EQ(unit_for_field("omega_radps"), "rad/s");

  EXPECT_TRUE(has_unit_suffix("alpha_radps2"));
  EXPECT_EQ(unit_for_field("alpha_radps2"), "rad/s²");

  EXPECT_TRUE(has_unit_suffix("position_rot"));
  EXPECT_EQ(unit_for_field("position_rot"), "rot");

  EXPECT_TRUE(has_unit_suffix("speed_rps"));
  EXPECT_EQ(unit_for_field("speed_rps"), "rot/s");

  EXPECT_TRUE(has_unit_suffix("accel_rps2"));
  EXPECT_EQ(unit_for_field("accel_rps2"), "rot/s²");

  EXPECT_TRUE(has_unit_suffix("voltage_v"));
  EXPECT_EQ(unit_for_field("voltage_v"), "V");

  EXPECT_TRUE(has_unit_suffix("current_a"));
  EXPECT_EQ(unit_for_field("current_a"), "A");

  EXPECT_TRUE(has_unit_suffix("power_w"));
  EXPECT_EQ(unit_for_field("power_w"), "W");

  EXPECT_TRUE(has_unit_suffix("temperature_degc"));
  EXPECT_EQ(unit_for_field("temperature_degc"), "°C");

  EXPECT_TRUE(has_unit_suffix("mass_kg"));
  EXPECT_EQ(unit_for_field("mass_kg"), "kg");

  EXPECT_TRUE(has_unit_suffix("force_n"));
  EXPECT_EQ(unit_for_field("force_n"), "N");

  EXPECT_TRUE(has_unit_suffix("torque_nm"));
  EXPECT_EQ(unit_for_field("torque_nm"), "N·m");

  EXPECT_TRUE(has_unit_suffix("progress_frac"));
  EXPECT_EQ(unit_for_field("progress_frac"), "");

  EXPECT_TRUE(has_unit_suffix("output_duty"));
  EXPECT_EQ(unit_for_field("output_duty"), "");

  EXPECT_TRUE(has_unit_suffix("gear_ratio"));
  EXPECT_EQ(unit_for_field("gear_ratio"), "");

  EXPECT_TRUE(has_unit_suffix("tick_count"));
  EXPECT_EQ(unit_for_field("tick_count"), "");

  EXPECT_TRUE(has_unit_suffix("slot_index"));
  EXPECT_EQ(unit_for_field("slot_index"), "");

  EXPECT_TRUE(has_unit_suffix("device_id"));
  EXPECT_EQ(unit_for_field("device_id"), "");

  EXPECT_TRUE(has_unit_suffix("fault_mask"));
  EXPECT_EQ(unit_for_field("fault_mask"), "");

  for (const auto* name : {"pixel_std_px", "huber_delta_px", "min_tag_px"}) {
    EXPECT_TRUE(has_unit_suffix(name));
    EXPECT_EQ(unit_for_field(name), "px");
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
