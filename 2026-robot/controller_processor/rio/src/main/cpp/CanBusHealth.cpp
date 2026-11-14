#include "CanBusHealth.h"

#include <chrono>
#include <system_error>
#include <utility>

#ifndef _WIN32
#include <pthread.h>
#endif
#ifdef __linux__
#include <sched.h>
#endif

namespace talos::rio {

CanBusSampler::CanBusSampler(CanBusHealth& health, Read read, Clock clock,
                             uint64_t period_us)
    : health_{health},
      read_{std::move(read)},
      clock_{clock},
      period_us_{period_us} {}

CanBusSampler::~CanBusSampler() { Stop(); }

void CanBusSampler::Reconfigure(const hardware::Config& config,
                                bool simulation) {
  Stop();
  health_.Configure(config, simulation);
  if (health_.bus_count() == 0) return;
  stop_ = false;
  try {
    thread_ = std::thread([this] { Run(); });
  } catch (const std::system_error&) {
    // No reads: Poll reports every bus stale (see the header).
  }
}

void CanBusSampler::Stop() {
  if (!thread_.joinable()) return;
  {
    const std::lock_guard lock{mutex_};
    stop_ = true;
  }
  wake_.notify_one();
  thread_.join();
}

void CanBusSampler::Run() {
#ifdef __linux__
  pthread_setname_np(pthread_self(), "talos-canbus");
  const sched_param param{};
  pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
#elif defined(__APPLE__)
  pthread_setname_np("talos-canbus");
#endif
  std::unique_lock lock{mutex_};
  while (!stop_) {
    lock.unlock();
    health_.SampleNext(clock_(), read_);
    lock.lock();
    wake_.wait_for(lock, std::chrono::microseconds{period_us_},
                   [this] { return stop_; });
  }
}

}  // namespace talos::rio
