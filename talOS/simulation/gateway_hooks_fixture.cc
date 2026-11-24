// Test-only: sim_gateway with an extension that uses the BeforeTick, Stamp,
// Topics and Counts hooks (gateway_hooks_test.cc). An optional first argument
// picks a variant and is not passed on:
//   --plain            an extension that overrides none of the three hooks;
//   --topics=N         Topics() declares N topics /test/state/t<i> instead;
//   --topics=twice     Topics() declares kTopic twice;
//   --topics=state     Topics() declares /sim/state;
//   --topics=empty     Topics() declares a topic with an empty name;
//   --topics=timer     Topics() declares a TIMER entry instead of SENDER;
//   --topics=long      Topics() declares a name longer than MAX_SOURCE_NAME;
//   --slots=N          TopicSlots(0) returns N instead of 0;
//   --nonfinite-stamp  Stamp() writes quiet_NaN() to match_time_s;
//   --throw-after-tick AfterTick() throws std::runtime_error on the first tick.
//
// The stamp it writes in Stamp, from what the same tick's BeforeTick saw:
// match_time_s is the schedule time in seconds (the float of schedule_us /
// 1e6), or -1 before the schedules' time zero; game_specific_message is
// "<flags> <schedule_us>", the decimal Driver Station flags the tick uses and
// its schedule time, "-" before time zero. Each tick it writes one message on
// kTopic. Finish prints how often BeforeTick, Stamp and AfterTick ran.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "geometry/geometry_descriptor.h"
#include "geometry/geometry_generated.h"
#include "talOS/events/manifest.h"
#include "talOS/ipc/publisher.h"
#include "talOS/simulation/gateway_main.h"

namespace {

constexpr const char* kTopic = "/test/state/hooks";

talos::event::Registration Topic(std::string name) {
  return {
      .kind = talos::event::SourceKind::SENDER,
      .name = std::move(name),
      .message_bytes = sizeof(geometry::SwerveModulePosition),
      .alignment = alignof(geometry::SwerveModulePosition),
      .schema = talos::schema::Of<geometry::SwerveModulePosition>(),
  };
}

class PlainExtension : public talos::simulation::GatewayExtension {
 public:
  void Configure(
      const talos::config::RobotConfig& /*config*/,
      const talos::hardware::sim::PhysicsOptions& /*options*/,
      talos::hardware::sim::PhysicsSimBackend& /*backend*/) override {}
  void AfterTick(uint64_t /*now_us*/) override {}
  void Finish() override {}
};

struct HooksVariant {
  talos::event::Manifest topics;
  uint32_t topic_slots{0};
  bool nonfinite_stamp{false};
  bool throw_after_tick{false};
};

class HooksExtension : public PlainExtension {
 public:
  explicit HooksExtension(HooksVariant variant)
      : topics_(std::move(variant.topics)),
        topic_slots_(variant.topic_slots),
        nonfinite_stamp_(variant.nonfinite_stamp),
        throw_after_tick_(variant.throw_after_tick) {}

  void Configure(
      const talos::config::RobotConfig& /*config*/,
      const talos::hardware::sim::PhysicsOptions& /*options*/,
      talos::hardware::sim::PhysicsSimBackend& /*backend*/) override {
    publisher_.emplace(kTopic);
  }

  void BeforeTick(const talos::simulation::TickInput& tick) override {
    ++before_tick_calls_;
    flags_ = tick.driver_station->fms.flags;
    schedule_us_ = tick.schedule_us;
    now_us_ = tick.now_us;
  }

  void Stamp(talos::simulation::FmsStamp& fms) override {
    ++stamp_calls_;
    fms.match_time_s =
        nonfinite_stamp_
            ? std::numeric_limits<float>::quiet_NaN()
            : (schedule_us_ ? static_cast<float>(*schedule_us_) / 1e6F : -1.0F);
    const std::string message =
        std::to_string(flags_) + " " +
        (schedule_us_ ? std::to_string(*schedule_us_) : "-");
    fms.game_specific_message.fill('\0');
    message.copy(fms.game_specific_message.data(),
                 fms.game_specific_message.size() - 1);
  }

  void AfterTick(uint64_t /*now_us*/) override {
    ++after_tick_calls_;
    if (throw_after_tick_) {
      throw std::runtime_error("injected after_tick failure");
    }
    if (!publisher_) return;
    const auto status = publisher_->write(geometry::SwerveModulePosition{
        schedule_us_ ? static_cast<double>(*schedule_us_) / 1e6 : -1.0, 0.0});
    if (status == WriteResult::SUCCESS) {
      ++count_.samples;
      count_.sequence = status.sequence;
      count_.last_ns = static_cast<int64_t>(now_us_) * 1000;
    } else {
      ++count_.dropped;
    }
  }

  void Finish() override {
    std::printf(
        "hooks: before_tick_calls=%llu stamp_calls=%llu "
        "after_tick_calls=%llu\n",
        static_cast<unsigned long long>(before_tick_calls_),
        static_cast<unsigned long long>(stamp_calls_),
        static_cast<unsigned long long>(after_tick_calls_));
  }

  talos::event::Manifest Topics() const override { return topics_; }

  uint32_t TopicSlots(std::size_t index) const override {
    return index == 0 ? topic_slots_ : 0;
  }

  void Counts(std::span<talos::simulation::TopicCount> out) const override {
    if (!out.empty()) out[0] = count_;
  }

 private:
  talos::event::Manifest topics_;
  uint32_t topic_slots_{0};
  bool nonfinite_stamp_{false};
  bool throw_after_tick_{false};
  std::optional<ipc::Publisher<geometry::SwerveModulePosition>> publisher_;
  uint32_t flags_{0};
  std::optional<uint64_t> schedule_us_;
  uint64_t now_us_{0};
  uint64_t before_tick_calls_{0};
  uint64_t stamp_calls_{0};
  uint64_t after_tick_calls_{0};
  talos::simulation::TopicCount count_;
};

talos::event::Manifest Topics(std::string_view variant) {
  if (variant == "twice") return {Topic(kTopic), Topic(kTopic)};
  if (variant == "state") return {Topic("/sim/state")};
  if (variant == "empty") return {Topic("")};
  if (variant == "timer") {
    talos::event::Registration entry = Topic(kTopic);
    entry.kind = talos::event::SourceKind::TIMER;
    return {std::move(entry)};
  }
  if (variant == "long") {
    return {Topic("/test/state/" +
                  std::string(talos::event::MAX_SOURCE_NAME, 'a'))};
  }
  talos::event::Manifest topics;
  const int count = std::stoi(std::string{variant});
  for (int i = 0; i < count; ++i) {
    topics.push_back(Topic("/test/state/t" + std::to_string(i)));
  }
  return topics;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<char*> args(argv, argv + argc);
  const std::string_view variant = argc > 1 ? argv[1] : "";
  constexpr std::string_view kTopicsFlag = "--topics=";
  constexpr std::string_view kSlotsFlag = "--slots=";
  const bool plain = variant == "--plain";
  const bool topics = variant.starts_with(kTopicsFlag);
  const bool slots = variant.starts_with(kSlotsFlag);
  const bool nonfinite_stamp = variant == "--nonfinite-stamp";
  const bool throw_after_tick = variant == "--throw-after-tick";
  if (plain || topics || slots || nonfinite_stamp || throw_after_tick) {
    args.erase(args.begin() + 1);
  }
  PlainExtension plain_extension;
  HooksVariant hooks_variant{
      .topics = topics ? Topics(variant.substr(kTopicsFlag.size()))
                       : talos::event::Manifest{Topic(kTopic)},
      .topic_slots = slots ? static_cast<uint32_t>(std::stoul(std::string{
                                 variant.substr(kSlotsFlag.size())}))
                           : 0U,
      .nonfinite_stamp = nonfinite_stamp,
      .throw_after_tick = throw_after_tick,
  };
  HooksExtension hooks_extension{std::move(hooks_variant)};
  talos::simulation::GatewayExtension& extension =
      plain ? static_cast<talos::simulation::GatewayExtension&>(plain_extension)
            : hooks_extension;
  return talos::simulation::RunSimGateway(
      static_cast<int>(args.size()), args.data(),
      {"//talOS/simulation:gateway_hooks_fixture", "physics", true},
      &extension);
}
