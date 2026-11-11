#pragma once

// Shared pieces of the mechanism building blocks: a parameter reader scoped to
// one mechanism's table, the motor's own config, and a time debounce.
// Construction-time helpers allocate; nothing here allocates once
// constructed.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "talOS/configuration/config_parser.h"
#include "talOS/devices/devices.h"
#include "talOS/hardware/config.h"
#include "talOS/node/node.h"

namespace talos::mechanisms {

// Reads `<name>.<key>` from the owning subsystem's table: a `[<name>]` table
// in a subsystem file, `[subsystems.<sub>.<name>]` in a single-file config.
// Errors name the node and the full key.
class Params {
 public:
  Params(const Node& node, std::string_view name) : node_{&node}, name_{name} {}

  std::string key(std::string_view key) const {
    std::string full{name_};
    full += '.';
    full += key;
    return full;
  }

  template <class T>
  T get(std::string_view k) const {
    return node_->param<T>(key(k));
  }
  template <class T>
  T get(std::string_view k, T fallback) const {
    return node_->param<T>(key(k), std::move(fallback));
  }

  // A finite number, required or with a fallback.
  double number(std::string_view k) const {
    return finite(k, node_->param<double>(key(k)));
  }
  double number(std::string_view k, double fallback) const {
    return finite(k, node_->param<double>(key(k), fallback));
  }
  // A finite number > 0.
  double positive(std::string_view k) const {
    const double value = number(k);
    if (!(value > 0)) fail(k, "must be > 0");
    return value;
  }
  double positive(std::string_view k, double fallback) const {
    const double value = number(k, fallback);
    if (!(value > 0)) fail(k, "must be > 0");
    return value;
  }
  // An optional finite number: nullopt when the key is absent.
  std::optional<double> optional_number(std::string_view k) const {
    const double value =
        node_->param<double>(key(k), std::numeric_limits<double>::quiet_NaN());
    if (std::isnan(value)) return std::nullopt;
    return finite(k, value);
  }
  // A limit: any number but NaN, so +-inf (the fallback for "none") passes.
  double limit(std::string_view k, double fallback) const {
    const double value = node_->param<double>(key(k), fallback);
    if (std::isnan(value)) fail(k, "must be a number");
    return value;
  }
  // Integer microseconds >= 0, as a Duration.
  Duration micros(std::string_view k, std::int64_t fallback_us) const {
    return to_duration(k, node_->param<std::int64_t>(key(k), fallback_us));
  }
  Duration micros(std::string_view k) const {
    return to_duration(k, node_->param<std::int64_t>(key(k)));
  }

  [[noreturn]] void fail(std::string_view k, std::string_view why) const {
    throw std::invalid_argument(std::string{node_->name()} + ": " + key(k) +
                                " " + std::string{why});
  }

 private:
  double finite(std::string_view k, double value) const {
    if (!std::isfinite(value)) fail(k, "must be finite");
    return value;
  }
  Duration to_duration(std::string_view k, std::int64_t us) const {
    if (us < 0) fail(k, "must be >= 0");
    return std::chrono::microseconds{us};
  }

  const Node* node_;
  std::string name_;
};

// The [motors.<name>] config of this node's motor (what the devices layer
// clamps to and how its position frame behaves). Throws
// std::invalid_argument when the node owns no motor of that name.
inline const hardware::MotorConfig& MotorConfigOf(Node& node,
                                                  std::string_view name) {
  const config::RobotConfig& config = node.extension<HardwareLink>().config();
  const hardware::Devices* owned = config.GetDevices(std::string{node.name()});
  if (owned != nullptr) {
    for (const hardware::MotorConfig& motor : config.hardware.motors) {
      if (motor.name != name) continue;
      for (const auto id : owned->motors) {
        if (id == motor.id) return motor;
      }
    }
  }
  throw std::invalid_argument(std::string{node.name()} + ": no motor named '" +
                              std::string{name} + "' in [motors]");
}

// True once a condition has held continuously for `delay` (at once for a
// zero delay); false, and restarted, the moment it does not hold.
class Debounce {
 public:
  explicit Debounce(Duration delay = Duration::zero()) : delay_{delay} {}

  bool update(bool condition, Time now) {
    if (!condition) {
      since_.reset();
      return false;
    }
    if (!since_) since_ = now;
    return now - *since_ >= delay_;
  }
  void reset() { since_.reset(); }

 private:
  Duration delay_;
  std::optional<Time> since_;
};

}  // namespace talos::mechanisms
