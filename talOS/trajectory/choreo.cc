#include "talOS/trajectory/choreo.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/trajectory/trajectory.h"

namespace talos::trajectory {
namespace {

using Json = nlohmann::json;

// A problem with the file's content; the message names the JSON location.
class FormatError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// "<where>.<key>: <problem>".
FormatError FieldError(std::string_view where, const char* key,
                       std::string_view problem) {
  std::string message{where};
  message.append(".").append(key).append(": ").append(problem);
  return FormatError(message);
}

// `where` is a string_view, not a std::string, so call sites passing a
// literal create no temporary for the returned reference to outlive (GCC's
// -Wdangling-reference).
const Json& Member(const Json& object, const char* key,
                   std::string_view where) {
  const auto it = object.find(key);
  if (it == object.end()) {
    std::string message{where};
    message.append(": missing \"").append(key).append("\"");
    throw FormatError(message);
  }
  return *it;
}

const Json& Object(const Json& object, const char* key,
                   std::string_view where) {
  const Json& value = Member(object, key, where);
  if (!value.is_object()) throw FieldError(where, key, "expected an object");
  return value;
}

const Json& Array(const Json& object, const char* key, std::string_view where) {
  const Json& value = Member(object, key, where);
  if (!value.is_array()) throw FieldError(where, key, "expected an array");
  return value;
}

double Number(const Json& object, const char* key, std::string_view where) {
  const Json& value = Member(object, key, where);
  if (!value.is_number()) throw FieldError(where, key, "expected a number");
  return value.get<double>();
}

// The number at `key`, nullopt when absent; a present non-number is an
// error.
std::optional<double> OptionalNumber(const Json& object, const char* key,
                                     std::string_view where) {
  const auto it = object.find(key);
  if (it == object.end()) return std::nullopt;
  if (!it->is_number()) throw FieldError(where, key, "expected a number");
  return it->get<double>();
}

void CheckVersion(const Json& root) {
  const std::string supported = std::to_string(kChoreoMinVersion) + " to " +
                                std::to_string(kChoreoMaxVersion);
  const Json& version = Member(root, "version", "file");
  if (!version.is_number_integer()) {
    throw FormatError("\"version\" is " + version.dump() +
                      ", not an integer (supported: " + supported + ")");
  }
  const auto major = version.get<std::int64_t>();
  if (major < kChoreoMinVersion || major > kChoreoMaxVersion) {
    throw FormatError("unsupported Choreo trajectory version " +
                      std::to_string(major) + " (supported: " + supported +
                      ")");
  }
}

struct Samples {
  std::vector<TrajectorySample> samples;
  Interpolation interpolation = Interpolation::kConstantAcceleration;
};

Samples ReadSamples(const Json& trajectory) {
  const Json& samples = Array(trajectory, "samples", "trajectory");
  if (samples.empty()) {
    throw FormatError(
        "trajectory.samples: empty (generate the path in Choreo first)");
  }
  Samples out;
  out.samples.reserve(samples.size());
  bool all_accelerations = true;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const std::string where = "trajectory.samples[" + std::to_string(i) + "]";
    const Json& s = samples[i];
    if (!s.is_object()) throw FormatError(where + ": expected an object");
    TrajectorySample sample;
    sample.t_s = Number(s, "t", where);
    sample.x_m = Number(s, "x", where);
    sample.y_m = Number(s, "y", where);
    sample.heading_rad = Number(s, "heading", where);
    sample.vx_mps = Number(s, "vx", where);
    sample.vy_mps = Number(s, "vy", where);
    sample.omega_radps = Number(s, "omega", where);
    const std::optional<double> ax = OptionalNumber(s, "ax", where);
    const std::optional<double> ay = OptionalNumber(s, "ay", where);
    const std::optional<double> alpha = OptionalNumber(s, "alpha", where);
    if (ax && ay && alpha) {
      sample.ax_mps2 = *ax;
      sample.ay_mps2 = *ay;
      sample.alpha_radps2 = *alpha;
    } else {
      all_accelerations = false;
    }
    out.samples.push_back(sample);
  }
  if (!all_accelerations) out.interpolation = Interpolation::kHermite;
  return out;
}

std::vector<std::size_t> ReadSplits(const Json& trajectory,
                                    std::size_t sample_count) {
  std::vector<std::size_t> splits;
  const auto it = trajectory.find("splits");
  if (it == trajectory.end() || it->is_null()) return splits;
  if (!it->is_array()) {
    throw FormatError("trajectory.splits: expected an array");
  }
  for (std::size_t i = 0; i < it->size(); ++i) {
    const Json& index = (*it)[i];
    const std::string where = "trajectory.splits[" + std::to_string(i) + "]";
    if (!index.is_number_unsigned() ||
        index.get<std::size_t>() >= sample_count) {
      throw FormatError(where + ": expected a sample index below " +
                        std::to_string(sample_count));
    }
    splits.push_back(index.get<std::size_t>());
  }
  return splits;
}

struct Events {
  std::vector<EventMarker> markers;
  std::size_t dropped = 0;
};

// A Choreo event is anchored to a waypoint: its time is from.targetTimestamp
// (the waypoint's time; null while the event is attached to none) plus
// from.offset.val. Like ChoreoLib, an event without a numeric time, with a
// negative time or with an empty name is dropped, not an error; a time past
// the end is kept.
Events ReadEvents(const Json& root) {
  Events out;
  const auto it = root.find("events");
  if (it == root.end() || it->is_null()) return out;
  if (!it->is_array()) throw FormatError("events: expected an array");
  for (const Json& event : *it) {
    if (!event.is_object()) {
      ++out.dropped;
      continue;
    }
    const auto name = event.find("name");
    const auto from = event.find("from");
    if (name == event.end() || !name->is_string() ||
        name->get_ref<const std::string&>().empty() || from == event.end() ||
        !from->is_object()) {
      ++out.dropped;
      continue;
    }
    const auto target = from->find("targetTimestamp");
    if (target == from->end() || !target->is_number()) {
      ++out.dropped;
      continue;
    }
    double t_s = target->get<double>();
    const auto offset = from->find("offset");
    if (offset != from->end() && offset->is_object()) {
      const auto val = offset->find("val");
      if (val != offset->end() && val->is_number()) t_s += val->get<double>();
    }
    if (!std::isfinite(t_s) || t_s < 0) {
      ++out.dropped;
      continue;
    }
    out.markers.push_back({name->get<std::string>(), t_s});
  }
  return out;
}

LoadResult Parse(const Json& root) {
  if (!root.is_object()) throw FormatError("expected a JSON object");
  CheckVersion(root);
  const Json& trajectory = Object(root, "trajectory", "file");
  if (const auto type = trajectory.find("sampleType");
      type != trajectory.end() && !type->is_null() && *type != "Swerve") {
    throw FormatError("trajectory.sampleType " + type->dump() +
                      " is not supported (only \"Swerve\")");
  }
  Samples samples = ReadSamples(trajectory);
  std::vector<std::size_t> splits =
      ReadSplits(trajectory, samples.samples.size());
  Events events = ReadEvents(root);
  LoadResult result;
  result.trajectory =
      Trajectory{std::move(samples.samples), samples.interpolation,
                 std::move(events.markers), std::move(splits)};
  result.dropped_event_count = events.dropped;
  return result;
}

LoadResult Fail(std::string_view source, std::string_view problem) {
  LoadResult result;
  result.error.append(source).append(": ").append(problem);
  return result;
}

}  // namespace

LoadResult ParseChoreo(std::string_view json, std::string_view source) {
  try {
    return Parse(Json::parse(json.begin(), json.end()));
  } catch (const Json::parse_error& e) {
    return Fail(source, std::string("invalid JSON: ") + e.what());
  } catch (const std::exception& e) {
    // FormatError, or Trajectory's own validation (std::invalid_argument).
    return Fail(source, e.what());
  }
}

LoadResult LoadChoreoFile(const std::filesystem::path& path) {
  const std::string source = path.string();
  try {
    std::ifstream in(path, std::ios::binary);
    if (!in) return Fail(source, "cannot open file");
    const std::string text{std::istreambuf_iterator<char>(in),
                           std::istreambuf_iterator<char>()};
    if (in.bad()) return Fail(source, "read failed");
    return ParseChoreo(text, source);
  } catch (const std::exception& e) {
    return Fail(source, e.what());
  }
}

}  // namespace talos::trajectory
