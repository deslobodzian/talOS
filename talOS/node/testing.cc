#include "talOS/node/testing.h"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/node/internal.h"

namespace talos::testing {
namespace {

using namespace std::chrono_literals;

// The loops' message tick: a slice never spans two deliveries.
constexpr Duration kSlice = 1ms;

// An inline config written to a file, so config_path() means the same thing
// in a test as under talos::Run. Removed with the Sim.
std::string WriteTempConfig(std::string_view toml) {
  const char* dir = std::getenv("TEST_TMPDIR");
  std::string pattern = (std::filesystem::path{dir != nullptr ? dir : "/tmp"} /
                         "talos_sim_XXXXXX.toml")
                            .string();
  const int fd = ::mkstemps(pattern.data(), 5);
  if (fd < 0) throw std::runtime_error("Sim: cannot create " + pattern);
  ::close(fd);
  std::ofstream{pattern} << toml;
  return pattern;
}

// Byte topics by name: the newest message on each, once there is one.
using ByteTopics =
    std::map<std::string, std::optional<std::vector<std::byte>>, std::less<>>;

class LatestBytes final : public detail::ByteTopic {
 public:
  explicit LatestBytes(std::optional<std::vector<std::byte>>& latest)
      : latest_{latest} {}
  bool write(std::span<const std::byte> message) override {
    latest_.emplace(message.begin(), message.end());
    return true;
  }

 private:
  std::optional<std::vector<std::byte>>& latest_;
};

// A node's port in the Sim: byte topics stay in memory, where latest_bytes
// reads them, rather than in shared memory.
class SimPort final : public detail::LoopPortFor<event::SimulatedEventLoop<>> {
 public:
  SimPort(event::SimulatedEventLoop<>& loop, ByteTopics& byte_topics)
      : LoopPortFor{loop}, byte_topics_{byte_topics} {}

  std::unique_ptr<detail::ByteTopic> open_byte_topic(
      std::string_view topic, std::size_t, const BytePublishOptions&) override {
    return std::make_unique<LatestBytes>(
        byte_topics_.try_emplace(std::string{topic}).first->second);
  }

 private:
  ByteTopics& byte_topics_;
};

}  // namespace

struct Sim::Impl {
  struct Hosted {
    std::string name;
    std::unique_ptr<event::SimulatedEventLoop<>> loop;
    std::unique_ptr<detail::LoopPort> port;
    std::unique_ptr<Node> node;
    detail::Instance user;
    ~Hosted() {
      user.reset();
      node.reset();
    }
  };
  struct Periodic {
    Duration period;
    Duration next;
    std::function<bool()> fn;
  };

  std::string config_path;
  bool owns_config_file = false;
  std::shared_ptr<const config::RobotConfig> config;
  // Declared before the nodes: their loops and byte topics write here until
  // they are gone.
  event::SimulationEnvironment environment;
  ByteTopics byte_topics;
  std::vector<std::unique_ptr<Hosted>> nodes;
  std::vector<Periodic> periodic;
  Duration now{0};
  bool started = false;

  ~Impl() {
    nodes.clear();
    if (owns_config_file) std::filesystem::remove(config_path);
  }

  void RunPeriodic() {
    for (std::size_t i = 0; i < periodic.size();) {
      Periodic& p = periodic[i];
      bool keep = true;
      while (keep && now >= p.next) {
        keep = p.fn();
        p.next += p.period;
      }
      if (keep) {
        ++i;
      } else {
        periodic.erase(periodic.begin() + static_cast<std::ptrdiff_t>(i));
      }
    }
  }
};

Sim::Sim() : impl_{std::make_unique<Impl>()} {}

Sim::Sim(std::string config_path) : Sim{} {
  impl_->config = std::make_shared<const config::RobotConfig>(
      config::ParseRobotConfig(config_path));
  impl_->config_path = std::move(config_path);
}

Sim Sim::FromToml(std::string_view robot_toml) {
  Sim sim;
  sim.impl_->config_path = WriteTempConfig(robot_toml);
  sim.impl_->owns_config_file = true;
  sim.impl_->config = std::make_shared<const config::RobotConfig>(
      config::ParseRobotConfig(sim.impl_->config_path));
  return sim;
}

Sim::~Sim() = default;
Sim::Sim(Sim&&) noexcept = default;
Sim& Sim::operator=(Sim&&) noexcept = default;

Time Sim::now() const { return Time::from_nanos(impl_->now.count()); }

Node& Sim::node(std::string_view node_name) {
  for (const auto& hosted : impl_->nodes) {
    if (hosted->name == node_name) return *hosted->node;
  }
  throw std::invalid_argument("Sim: no node named '" + std::string{node_name} +
                              "'");
}

std::optional<std::vector<std::byte>> Sim::latest_bytes(
    std::string_view topic) const {
  const auto found = impl_->byte_topics.find(topic);
  if (found == impl_->byte_topics.end()) return std::nullopt;
  return found->second;
}

void Sim::step(Duration d) {
  if (d < Duration::zero()) throw std::invalid_argument("Sim: negative step");
  Impl& impl = *impl_;
  impl.started = true;
  const Duration end = impl.now + d;
  for (;;) {
    impl.RunPeriodic();
    if (impl.now >= end) break;
    Duration next = std::min(impl.now + kSlice, end);
    for (const auto& p : impl.periodic) next = std::min(next, p.next);
    const Time until = Time::from_nanos(next.count());
    for (const auto& hosted : impl.nodes) hosted->loop->run_until(until);
    impl.now = next;
  }
}

}  // namespace talos::testing

namespace talos::detail {

using testing::Sim;

void* SimAccess::add_instance(Sim& sim, std::string_view node_name,
                              const InstanceFactory& make) {
  Sim::Impl& impl = *sim.impl_;
  if (impl.started) {
    throw std::logic_error("Sim: add every node before the first step");
  }
  for (const auto& hosted : impl.nodes) {
    if (hosted->name == node_name) {
      throw std::invalid_argument("Sim: node '" + std::string{node_name} +
                                  "' added twice");
    }
  }
  auto hosted = std::make_unique<Sim::Impl::Hosted>();
  hosted->name = std::string{node_name};
  hosted->loop =
      std::make_unique<event::SimulatedEventLoop<>>(impl.environment);
  hosted->port =
      std::make_unique<testing::SimPort>(*hosted->loop, impl.byte_topics);
  hosted->node = NodeAccess::create(NodeSetup{
      .name = hosted->name,
      .config_path = impl.config_path,
      .config = impl.config,
      .simulation = true,
      .loop = hosted->port.get(),
  });
  hosted->user = make(*hosted->node);
  NodeAccess::start(*hosted->node);
  void* user = hosted->user.get();
  impl.nodes.push_back(std::move(hosted));
  return user;
}

const std::string& SimAccess::config_path(const Sim& sim) {
  return sim.impl_->config_path;
}

void SimAccess::every(Sim& sim, Duration period, std::function<bool()> fn) {
  if (period <= Duration::zero() || !fn) {
    throw std::invalid_argument("Sim::every needs a positive period and a fn");
  }
  sim.impl_->periodic.push_back({period, sim.impl_->now, std::move(fn)});
}

void SimAccess::publish_raw(Sim& sim, std::string_view topic, const void* data,
                            std::size_t bytes) {
  sim.impl_->environment.channel(topic, static_cast<std::uint32_t>(bytes))
      .publish(std::span<const std::byte>{static_cast<const std::byte*>(data),
                                          bytes});
}

std::uint64_t SimAccess::message_count(const Sim& sim, std::string_view topic,
                                       std::size_t bytes) {
  return sim.impl_->environment
      .channel(topic, static_cast<std::uint32_t>(bytes))
      .next_sequence();
}

void SimAccess::read_raw(const Sim& sim, std::string_view topic,
                         std::size_t bytes, std::uint64_t sequence, void* out) {
  auto& channel =
      sim.impl_->environment.channel(topic, static_cast<std::uint32_t>(bytes));
  if (sequence >= channel.next_sequence()) {
    throw std::out_of_range("Sim: no message " + std::to_string(sequence) +
                            " on '" + std::string{topic} + "'");
  }
  channel.copy_to(sequence,
                  std::span<std::byte>{static_cast<std::byte*>(out), bytes});
}

}  // namespace talos::detail
