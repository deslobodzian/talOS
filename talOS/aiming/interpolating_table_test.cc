// Tests for talOS/aiming InterpolatingTable and ShotTable: interpolation
// against hand-computed values, the out-of-range clamp and its flag, and
// every validation error.

#include "talOS/aiming/interpolating_table.h"

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "talOS/aiming/target_solver.h"

namespace talos::aiming {
namespace {

using Table = InterpolatingTable<2, 8>;
using Row = Table::Row;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

const std::array<Row, 3> kRows = {{
    {1.0, {10.0, 0.0}},
    {2.0, {20.0, -1.0}},
    {4.0, {30.0, 1.0}},
}};

// exact at a row's key, linear between rows.
TEST(InterpolatingTable, ExactAtRowsAndLinearBetween) {
  const Table table(kRows);
  ASSERT_EQ(table.size(), 3u);
  EXPECT_EQ(table.min_key(), 1.0);
  EXPECT_EQ(table.max_key(), 4.0);
  for (const Row& row : kRows) {
    const Table::Lookup at = table.at(row.key);
    EXPECT_TRUE(at.in_range);
    EXPECT_EQ(at.values, row.values);
  }
  const Table::Lookup a = table.at(1.25);  // a quarter of 1 -> 2
  EXPECT_TRUE(a.in_range);
  EXPECT_DOUBLE_EQ(a.values[0], 12.5);
  EXPECT_DOUBLE_EQ(a.values[1], -0.25);
  const Table::Lookup b = table.at(3.5);  // three quarters of 2 -> 4
  EXPECT_TRUE(b.in_range);
  EXPECT_DOUBLE_EQ(b.values[0], 27.5);
  EXPECT_DOUBLE_EQ(b.values[1], 0.5);
}

// outside the keys a lookup clamps to the end row and says so,
// so a caller can refuse to shoot.
TEST(InterpolatingTable, OutOfRangeClampsAndFlags) {
  const Table table(kRows);
  const Table::Lookup below = table.at(0.5);
  EXPECT_FALSE(below.in_range);
  EXPECT_EQ(below.values, kRows[0].values);
  const Table::Lookup above = table.at(9.0);
  EXPECT_FALSE(above.in_range);
  EXPECT_EQ(above.values, kRows[2].values);
  EXPECT_FALSE(table.at(kInf).in_range);
  EXPECT_EQ(table.at(kInf).values, kRows[2].values);
  EXPECT_FALSE(table.at(-kInf).in_range);
  EXPECT_EQ(table.at(-kInf).values, kRows[0].values);
}

// a NaN key never yields NaN values or an in-range answer.
TEST(InterpolatingTable, NaNKeyIsOutOfRangeWithFiniteValues) {
  const Table table(kRows);
  const Table::Lookup at = table.at(kNaN);
  EXPECT_FALSE(at.in_range);
  EXPECT_EQ(at.values, kRows[0].values);
}

// one row is a constant table, in range only at its key.
TEST(InterpolatingTable, SingleRowIsConstant) {
  const std::array<Row, 1> one = {{{3.0, {7.0, 8.0}}}};
  const Table table(one);
  EXPECT_TRUE(table.at(3.0).in_range);
  EXPECT_FALSE(table.at(2.0).in_range);
  EXPECT_EQ(table.at(5.0).values, one[0].values);
}

// bad tables are refused at construction, with the reason and
// the offending row.
TEST(InterpolatingTable, ValidationErrors) {
  EXPECT_EQ(Table::Validate({}).error, TableError::kEmpty);

  const std::array<Row, 3> unsorted = {
      {{1.0, {0.0, 0.0}}, {3.0, {0.0, 0.0}}, {2.0, {0.0, 0.0}}}};
  EXPECT_EQ(Table::Validate(unsorted).error, TableError::kUnsorted);
  EXPECT_EQ(Table::Validate(unsorted).row_index, 2u);

  const std::array<Row, 2> duplicate = {{{1.0, {0.0, 0.0}}, {1.0, {1.0, 1.0}}}};
  EXPECT_EQ(Table::Validate(duplicate).error, TableError::kDuplicateKey);
  EXPECT_EQ(Table::Validate(duplicate).row_index, 1u);

  const std::array<Row, 2> nan_key = {{{1.0, {0.0, 0.0}}, {kNaN, {0.0, 0.0}}}};
  EXPECT_EQ(Table::Validate(nan_key).error, TableError::kNonFiniteKey);

  const std::array<Row, 2> inf_value = {
      {{1.0, {0.0, 0.0}}, {2.0, {kInf, 0.0}}}};
  EXPECT_EQ(Table::Validate(inf_value).error, TableError::kNonFiniteValue);
  EXPECT_EQ(Table::Validate(inf_value).row_index, 1u);

  std::vector<Row> too_many(Table::capacity() + 1);
  for (std::size_t i = 0; i < too_many.size(); ++i) {
    too_many[i].key = static_cast<double>(i);
  }
  EXPECT_EQ(Table::Validate(too_many).error, TableError::kTooManyRows);

  EXPECT_THROW(Table(std::span<const Row>{}), std::invalid_argument);
  EXPECT_THROW(Table{unsorted}, std::invalid_argument);
  EXPECT_THROW(Table{duplicate}, std::invalid_argument);
  EXPECT_THROW(Table{too_many}, std::invalid_argument);
  try {
    const Table table(duplicate);
    FAIL() << "expected a throw";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("row 1"), std::string::npos);
  }
}

// the shot table names its columns and refuses a time of
// flight that is not positive.
TEST(ShotTable, NamedColumnsAndPositiveTimeOfFlight) {
  const std::array<ShotRow, 2> rows = {{
      {2.0, 40.0, 0.2, 0.6},
      {4.0, 60.0, 0.4, 1.0},
  }};
  const ShotTable table(rows);
  const Shot mid = table.at(3.0);
  EXPECT_TRUE(mid.in_range);
  EXPECT_DOUBLE_EQ(mid.flywheel_rps, 50.0);
  EXPECT_DOUBLE_EQ(mid.hood_rad, 0.3);
  EXPECT_DOUBLE_EQ(mid.time_of_flight_s, 0.8);
  EXPECT_FALSE(table.at(5.0).in_range);
  EXPECT_DOUBLE_EQ(table.at(5.0).flywheel_rps, 60.0);
  EXPECT_EQ(table.min_distance_m(), 2.0);
  EXPECT_EQ(table.max_distance_m(), 4.0);

  const std::array<ShotRow, 2> zero_flight = {{
      {2.0, 40.0, 0.2, 0.6},
      {4.0, 60.0, 0.4, 0.0},
  }};
  EXPECT_THROW(ShotTable{zero_flight}, std::invalid_argument);
  const std::vector<ShotRow> too_many(ShotTable::kCapacity + 1,
                                      ShotRow{1.0, 1.0, 1.0, 1.0});
  EXPECT_THROW(ShotTable{too_many}, std::invalid_argument);
}

}  // namespace
}  // namespace talos::aiming
