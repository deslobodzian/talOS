#include "talOS/telemetry/testing.h"

#include <cstdint>
#include <string>
#include <vector>

#include "talOS/events/time.h"
#include "talOS/node/node.h"
#include "talOS/node/testing.h"
#include "talOS/telemetry/node.h"

namespace talos::telemetry {
namespace {

class TimedTelemetry {
 public:
  explicit TimedTelemetry(talos::Node& node) : inner_{node} {
    node.after_update([this] {
      const auto elapsed_ns =
          (talos::event::steady_now() - tick_start_).count();
      if (recording_ != nullptr && *recording_ && samples_ns_ != nullptr) {
        samples_ns_->push_back(elapsed_ns);
      }
    });
  }

  void bind(std::vector<std::int64_t>* samples_ns, const bool* recording) {
    samples_ns_ = samples_ns;
    recording_ = recording;
  }

  void update() {
    tick_start_ = talos::event::steady_now();
    inner_.update();
  }

 private:
  Telemetry inner_;
  talos::event::MonotonicTime tick_start_{};
  std::vector<std::int64_t>* samples_ns_ = nullptr;
  const bool* recording_ = nullptr;
};

}  // namespace

void AddTelemetry(talos::testing::Sim& sim, const std::string& name) {
  sim.add<Telemetry>(name);
}

void AddTimedTelemetry(talos::testing::Sim& sim,
                       std::vector<std::int64_t>& samples_ns,
                       const bool& recording, const std::string& name) {
  auto& timed = sim.add<TimedTelemetry>(name);
  timed.bind(&samples_ns, &recording);
}

}  // namespace talos::telemetry
