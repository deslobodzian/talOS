#pragma once

// A monotonic lookup table: strictly increasing keys, each with a fixed
// number of values, linearly interpolated between rows. Built once from
// config-like rows (storage is a fixed-capacity array inside the object), so
// a lookup never allocates. Outside the key range a lookup clamps to the end
// row and reports `in_range == false`, so a caller can refuse to act on an
// extrapolated answer. See talOS/aiming/README.md.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace talos::aiming {

enum class TableError {
  kNone,
  kEmpty,
  kTooManyRows,
  kNonFiniteKey,
  kNonFiniteValue,
  kUnsorted,
  kDuplicateKey,
};

constexpr std::string_view ToString(TableError error) {
  switch (error) {
    case TableError::kNone:
      return "ok";
    case TableError::kEmpty:
      return "no rows";
    case TableError::kTooManyRows:
      return "more rows than the table's capacity";
    case TableError::kNonFiniteKey:
      return "key is not finite";
    case TableError::kNonFiniteValue:
      return "value is not finite";
    case TableError::kUnsorted:
      return "keys are not increasing";
    case TableError::kDuplicateKey:
      return "duplicate key";
  }
  return "unknown";
}

// The first problem Validate found, and the index of the row it is in (0 for
// kEmpty and kTooManyRows).
struct TableCheck {
  TableError error = TableError::kNone;
  std::size_t row_index = 0;
};

template <std::size_t kValueCount, std::size_t kCapacity = 32>
class InterpolatingTable {
  static_assert(kValueCount > 0, "a table needs at least one value column");
  static_assert(kCapacity > 0, "a table needs room for at least one row");

 public:
  using Values = std::array<double, kValueCount>;

  struct Row {
    double key = 0.0;
    Values values{};
  };

  struct Lookup {
    Values values{};
    // False when the key is outside [min_key(), max_key()] or not a number;
    // `values` are then the nearest end row's (the first row's for NaN).
    bool in_range = false;
  };

  static constexpr std::size_t capacity() { return kCapacity; }

  // Rows must be non-empty, at most kCapacity, finite, with strictly
  // increasing keys. One row is a constant table.
  static TableCheck Validate(std::span<const Row> rows) {
    if (rows.empty()) return {TableError::kEmpty, 0};
    if (rows.size() > kCapacity) return {TableError::kTooManyRows, 0};
    for (std::size_t i = 0; i < rows.size(); ++i) {
      if (!std::isfinite(rows[i].key)) return {TableError::kNonFiniteKey, i};
      for (const double value : rows[i].values) {
        if (!std::isfinite(value)) return {TableError::kNonFiniteValue, i};
      }
      if (i == 0) continue;
      if (rows[i].key == rows[i - 1].key) {
        return {TableError::kDuplicateKey, i};
      }
      if (rows[i].key < rows[i - 1].key) return {TableError::kUnsorted, i};
    }
    return {};
  }

  // Throws std::invalid_argument naming the row and the problem.
  explicit InterpolatingTable(std::span<const Row> rows) {
    const TableCheck check = Validate(rows);
    if (check.error != TableError::kNone) {
      throw std::invalid_argument("interpolating table: row " +
                                  std::to_string(check.row_index) + ": " +
                                  std::string(ToString(check.error)));
    }
    std::copy(rows.begin(), rows.end(), rows_.begin());
    count_ = rows.size();
  }

  std::size_t size() const { return count_; }
  double min_key() const { return rows_[0].key; }
  double max_key() const { return rows_[count_ - 1].key; }
  const Row& row(std::size_t index) const { return rows_.at(index); }

  // Linear interpolation between the bracketing rows; exact at a row's key.
  // O(log n), no allocation.
  Lookup at(double key) const {
    const Row& first = rows_[0];
    const Row& last = rows_[count_ - 1];
    if (std::isnan(key)) return {first.values, false};
    if (key <= first.key) return {first.values, key == first.key};
    if (key >= last.key) return {last.values, key == last.key};
    // first.key < key < last.key, so count_ >= 2 and `upper` is a row after
    // the first and no later than the last.
    const auto end = rows_.begin() + static_cast<std::ptrdiff_t>(count_);
    const auto upper =
        std::upper_bound(rows_.begin(), end, key,
                         [](double k, const Row& row) { return k < row.key; });
    const Row& hi = *upper;
    const Row& lo = *(upper - 1);
    const double fraction = (key - lo.key) / (hi.key - lo.key);
    Lookup out;
    out.in_range = true;
    for (std::size_t i = 0; i < kValueCount; ++i) {
      out.values[i] = std::lerp(lo.values[i], hi.values[i], fraction);
    }
    return out;
  }

 private:
  std::array<Row, kCapacity> rows_{};
  std::size_t count_ = 0;
};

}  // namespace talos::aiming
