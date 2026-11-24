// Test-only: sim_gateway with an extension that uses the BeforeTick, Topics
// and Counts hooks (gateway_hooks_test.cc). An optional first argument picks
// a variant and is not passed on:
//   --plain           an extension that overrides none of the three hooks;
//   --topics=N        Topics() declares N topics /test/state/t<i> instead;
//   --topics=twice    Topics() declares kTopic twice;
//   --topics=state    Topics() declares /sim/state.
//
// The stamp it writes: match_time_s is the schedule time in seconds (the
// float of schedule_us / 1e6), or -1 before the schedules' time zero;
// game_specific_message is "<flags> <schedule_us>", the decimal Driver Station
// flags the tick uses and its schedule time, "-" before time zero. Each tick
// it writes one message on kTopic. Finish prints how often BeforeTick and
// AfterTick ran.

#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>
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

class HooksExtension : public PlainExtension {
 public:
  explicit HooksExtension(talos::event::Manifest topics)
      : topics_(std::move(topics)) {}

  void Configure(
      const talos::config::RobotConfig& /*config*/,
      const talos::hardware::sim::PhysicsOptions& /*options*/,
      talos::hardware::sim::PhysicsSimBackend& /*backend*/) override {
    publisher_.emplace(kTopic);
  }

  void BeforeTick(const talos::simulation::TickInput& tick,
                  talos::simulation::FmsStamp& fms) override {
    ++before_tick_calls_;
    fms.match_time_s =
        tick.schedule_us ? static_cast<float>(*tick.schedule_us) / 1e6F : -1.0F;
    const std::string message =
        std::to_string(tick.driver_station->fms.flags) + " " +
        (tick.schedule_us ? std::to_string(*tick.schedule_us) : "-");
    fms.game_specific_message.fill('\0');
    message.copy(fms.game_specific_message.data(),
                 fms.game_specific_message.size() - 1);
    schedule_us_ = tick.schedule_us;
    now_us_ = tick.now_us;
  }

  void AfterTick(uint64_t /*now_us*/) override {
    ++after_tick_calls_;
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
    std::printf("hooks: before_tick_calls=%llu after_tick_calls=%llu\n",
                static_cast<unsigned long long>(before_tick_calls_),
                static_cast<unsigned long long>(after_tick_calls_));
  }

  talos::event::Manifest Topics() const override { return topics_; }

  void Counts(std::span<talos::simulation::TopicCount> out) const override {
    out[0] = count_;
  }

 private:
  talos::event::Manifest topics_;
  std::optional<ipc::Publisher<geometry::SwerveModulePosition>> publisher_;
  std::optional<uint64_t> schedule_us_;
  uint64_t now_us_{0};
  uint64_t before_tick_calls_{0};
  uint64_t after_tick_calls_{0};
  talos::simulation::TopicCount count_;
};

talos::event::Manifest Topics(std::string_view variant) {
  if (variant == "twice") return {Topic(kTopic), Topic(kTopic)};
  if (variant == "state") return {Topic("/sim/state")};
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
  const bool plain = variant == "--plain";
  const bool topics = variant.starts_with(kTopicsFlag);
  if (plain || topics) args.erase(args.begin() + 1);
  PlainExtension plain_extension;
  HooksExtension hooks_extension{
      topics ? Topics(variant.substr(kTopicsFlag.size()))
             : talos::event::Manifest{Topic(kTopic)}};
  talos::simulation::GatewayExtension& extension =
      plain ? static_cast<talos::simulation::GatewayExtension&>(plain_extension)
            : hooks_extension;
  return talos::simulation::RunSimGateway(
      static_cast<int>(args.size()), args.data(),
      {"//talOS/simulation:gateway_hooks_fixture", "physics", true},
      &extension);
}
