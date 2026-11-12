#include "talOS/launcher/log_budget.h"

#include <algorithm>
#include <cstddef>
#include <set>

#include "bridge/topics.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/manifest.h"
#include "talOS/hardware/config.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/names.h"
#include "talOS/schema/identity.h"

namespace talos::launcher {
namespace {

constexpr double kRecordHeaderBytes = sizeof(event::log::RecordHeader);
// A hardware::Packet's size word, logged before its encoded bytes; the zero
// fill after them is not stored.
constexpr double kPacketSizeBytes = sizeof(uint32_t);
// The bridge publishes /hw/status every HardwareNode::kStatusPeriodUs
// (20 ms).
constexpr double kBridgeStatusPerSecond = 50;

double PerSecond(int64_t period_us) {
  return 1e6 / static_cast<double>(std::max<int64_t>(period_us, 1));
}

bool IsByteTopic(const introspect::naming::SourceShape& source) {
  return source.kind != event::SourceKind::TIMER &&
         source.schema_hash == schema::BytesHash();
}

std::size_t StateBytes(const hardware::Config& c) {
  return hardware::kStateBaseSize +
         c.motors.size() * hardware::kMotorSampleSize +
         c.sensors.size() * hardware::kSensorSampleSize +
         c.digital_inputs.size() * hardware::kDigitalInputSampleSize +
         c.digital_outputs.size() * hardware::kDigitalOutputSampleSize +
         c.analog_inputs.size() * hardware::kAnalogInputSampleSize +
         c.encoders.size() * hardware::kEncoderSampleSize +
         c.pwm_outputs.size() * hardware::kPwmSampleSize;
}

std::size_t CommandBytes(std::size_t motors, std::size_t digital_outputs,
                         std::size_t pwm_outputs) {
  return hardware::kCommandBaseSize + motors * hardware::kMotorRequestSize +
         digital_outputs * hardware::kDigitalOutputRequestSize +
         pwm_outputs * hardware::kPwmRequestSize;
}

}  // namespace

int64_t MemberPeriodUs(const config::RobotConfig& config,
                       std::string_view member) {
  return config.toml_data["subsystems"][member]["period_us"].value_or<int64_t>(
      config.toml_data["robot"]["period_us"].value_or<int64_t>(5000));
}

TopicLoads DeclaredTopicLoads(const std::vector<NodeDescription>& described,
                              const config::RobotConfig& config) {
  TopicLoads loads;
  // Every end's declared size first, so a topic whose writer did not
  // describe still has one; then each writer's rate.
  for (const auto& node : described) {
    if (!node.answered) continue;
    for (const auto& source : node.declared.sources) {
      if (source.kind == event::SourceKind::TIMER) continue;
      loads.try_emplace(
          source.name, TopicLoad{0, static_cast<double>(source.message_bytes)});
    }
  }
  double command_per_second = 0;
  std::size_t motors = 0;
  std::size_t digital_outputs = 0;
  std::size_t pwm_outputs = 0;
  for (const auto& node : described) {
    if (!node.answered) continue;
    const double rate = PerSecond(MemberPeriodUs(config, node.roster_name));
    for (const auto& source : node.declared.sources) {
      if (source.kind != event::SourceKind::SENDER) continue;
      TopicLoad& load = loads[source.name];
      load.per_second = rate;
      if (!source.name.starts_with(hardware::kRequestTopicPrefix)) continue;
      // A request carries its owner's outputs; the bridge sends one command
      // per accepted request that changes the merge, at most one per owner
      // request.
      const std::string_view owner = std::string_view{source.name}.substr(
          std::string_view{hardware::kRequestTopicPrefix}.size());
      const hardware::Devices* devices = config.GetDevices(std::string{owner});
      if (devices == nullptr) continue;
      load.stored_bytes =
          kPacketSizeBytes +
          static_cast<double>(CommandBytes(devices->motors.size(),
                                           devices->digital_outputs.size(),
                                           devices->pwm_outputs.size()));
      if (!devices->owns_actuators()) continue;
      command_per_second += rate;
      motors += devices->motors.size();
      digital_outputs += devices->digital_outputs.size();
      pwm_outputs += devices->pwm_outputs.size();
    }
  }
  // The bridge's topics, from the config: whatever the bridge declared, these
  // are what reach the logs.
  loads[hardware::kStateTopic] = {
      PerSecond(config.hardware.period_us),
      kPacketSizeBytes + static_cast<double>(StateBytes(config.hardware))};
  loads[hardware::kDriverStationTopic] = {
      PerSecond(static_cast<int64_t>(driver_station::kSamplePeriodUs)),
      kPacketSizeBytes +
          static_cast<double>(driver_station::kFixedEncodedSize +
                              driver_station::kMaxJoystickEncodedSize)};
  loads[hardware::kCommandTopic] = {
      command_per_second,
      kPacketSizeBytes + static_cast<double>(CommandBytes(
                             motors, digital_outputs, pwm_outputs))};
  if (auto status = loads.find(::bridge::topics::kStatus);
      status != loads.end()) {
    status->second.per_second = kBridgeStatusPerSecond;
  }
  return loads;
}

double ExpectedLogRate(const introspect::Description& declared,
                       int64_t period_us, bool hardware_bridge,
                       const TopicLoads& loads) {
  const double member = PerSecond(period_us);
  const auto load_of = [&loads](const introspect::naming::SourceShape& source) {
    const auto it = loads.find(source.name);
    return it != loads.end()
               ? it->second
               : TopicLoad{0, static_cast<double>(source.message_bytes)};
  };
  double bytes = 0;
  if (hardware_bridge) {
    // It logs the State it publishes and each command it sends, nothing else.
    for (const std::string_view topic :
         {std::string_view{hardware::kStateTopic},
          std::string_view{hardware::kCommandTopic}}) {
      if (const auto it = loads.find(topic); it != loads.end()) {
        bytes += it->second.per_second *
                 (kRecordHeaderBytes + it->second.stored_bytes);
      }
    }
    return bytes;
  }
  const std::string lanes = "/" + declared.name + "/target/";
  bool reads_lanes = false;
  for (const auto& source : declared.sources) {
    if (IsByteTopic(source)) continue;
    switch (source.kind) {
      case event::SourceKind::TIMER:
        bytes += member * kRecordHeaderBytes;
        break;
      case event::SourceKind::SENDER:
        bytes += member * (kRecordHeaderBytes + load_of(source).stored_bytes);
        break;
      case event::SourceKind::WATCHER: {
        const TopicLoad load = load_of(source);
        // A latest-only watcher records at most the newest message per
        // drain; a sequence watcher records every one (an older describe,
        // without the flag, reads as one: an upper bound).
        const double records =
            (source.flags & introspect::naming::kSourceFlagLatestOnly) != 0
                ? std::min(load.per_second, member)
                : load.per_second;
        bytes += records * (kRecordHeaderBytes + load.stored_bytes);
        if (introspect::naming::IsTargetLane(source.name) &&
            source.name.starts_with(lanes)) {
          reads_lanes = true;
        }
        break;
      }
      case event::SourceKind::FETCHER:
        break;  // A fetch is not a record.
    }
  }
  // Target::get's payload-free READ, once per tick.
  if (reads_lanes) bytes += member * kRecordHeaderBytes;
  return bytes;
}

uint64_t SmallestRotatingShare(const introspect::Description& declared) {
  uint64_t prefix = sizeof(event::log::FileHeader);
  uint64_t largest = sizeof(event::log::RecordHeader);
  for (const auto& source : declared.sources) {
    if (IsByteTopic(source)) continue;  // Not in the loop's manifest.
    prefix += sizeof(event::log::ManifestEntry) + source.schema.size() +
              source.bfbs.size();
    largest = std::max<uint64_t>(
        largest, sizeof(event::log::RecordHeader) + source.message_bytes);
  }
  return 2 * (prefix + largest);
}

LogSplit SplitLogBudget(uint64_t budget, const std::vector<bool>& logs,
                        const std::vector<double>& rates,
                        const std::vector<uint64_t>& floors) {
  LogSplit split;
  std::vector<uint64_t>& shares = split.shares;
  shares.assign(logs.size(), 0);
  std::size_t known = 0;
  double known_sum = 0;
  std::vector<std::size_t> loggers;
  for (std::size_t i = 0; i < logs.size(); ++i) {
    if (!logs[i]) continue;
    loggers.push_back(i);
    if (rates[i] >= 0) {
      ++known;
      known_sum += rates[i];
    }
  }
  if (loggers.empty()) return split;
  const double mean = known > 0 ? known_sum / static_cast<double>(known) : 1;
  const auto weight = [&](std::size_t i) {
    return rates[i] >= 0 ? rates[i] : mean;
  };

  // Proportional shares; a member whose share would fall below its floor is
  // held at the floor and the rest is shared again among the others, until
  // no share falls below its floor.
  std::set<std::size_t> held;
  double floor_sum = 0;
  std::vector<double> share(logs.size(), 0);
  double rest = 0;
  double total = 0;
  for (;;) {
    rest = static_cast<double>(budget) - floor_sum;
    total = 0;
    std::size_t free = 0;
    for (const std::size_t i : loggers) {
      if (held.contains(i)) continue;
      total += weight(i);
      ++free;
    }
    bool changed = false;
    for (const std::size_t i : loggers) {
      if (held.contains(i)) continue;
      share[i] = total > 0 ? std::max(rest, 0.0) * weight(i) / total
                           : std::max(rest, 0.0) / static_cast<double>(free);
      if (share[i] < static_cast<double>(floors[i])) {
        held.insert(i);
        floor_sum += static_cast<double>(floors[i]);
        changed = true;
      }
    }
    if (!changed) break;
  }
  for (const std::size_t i : held) share[i] = static_cast<double>(floors[i]);
  if (floor_sum > static_cast<double>(budget)) {
    // The floors alone do not fit: share the budget in their proportion.
    for (const std::size_t i : loggers) {
      share[i] = static_cast<double>(budget) * static_cast<double>(floors[i]) /
                 floor_sum;
    }
  } else if (known > 0 && total > 0 && rest > 0) {
    // T = rest / total: what every free member's share holds at its weight,
    // before its segments' prefixes. It never falls as the budget grows: the
    // shares are max(floor, T x weight) summing to the budget. A member's
    // prefix cost is the time its kLogSegments segments' prefixes take of
    // its share at its weight (half a floor bounds a prefix and the largest
    // record): it does not depend on the budget. The window is T less the
    // largest prefix cost, but never less than 7/8 of T, so it never falls
    // as the budget grows either, and one member with large prefixes
    // (a member held at its floor among them) cannot shrink every log's
    // window: its own segments end on bytes, as a member that stores more
    // than predicted does, and it keeps a shorter tail.
    const double t = rest / total;
    double cost = 0;
    for (const std::size_t i : loggers) {
      const double w = weight(i);
      if (w <= 0) continue;
      cost = std::max(cost, static_cast<double>(event::log::kLogSegments) *
                                (static_cast<double>(floors[i]) / 2) / w);
    }
    const double window_s = t - std::min(t / 8, cost);
    // Whole microseconds, rounded down, within a log writer's nanoseconds.
    constexpr int64_t kMaxWindowUs =
        std::chrono::nanoseconds::max().count() / 1000 / 2;
    const double window_us =
        std::min(window_s * 1e6, static_cast<double>(kMaxWindowUs));
    split.window = std::chrono::microseconds{
        static_cast<int64_t>(std::max(window_us, 0.0))};
  }
  for (const std::size_t i : loggers) {
    shares[i] = std::max<uint64_t>(1, static_cast<uint64_t>(share[i]));
  }
  return split;
}

}  // namespace talos::launcher
