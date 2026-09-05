// Latency/jitter benchmark for RealtimeEventLoop.
//
// Runs a control timer at --rate-hz alongside --publishers background
// threads that publish onto topics the loop watches, then reports the
// dispatch-latency, interval and handler-time distributions LoopMetrics
// collected plus the end-to-end IPC latency measured here.

#include <unistd.h>

#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <vector>

#include "talOS/events/events_perf_message_generated.h"
#include "talOS/events/handles.h"
#include "talOS/events/latency_stats.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/metrics.h"
#include "talOS/events/os/poller.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/recorder.h"
#include "talOS/events/time.h"
#include "talOS/ipc/publisher.h"

namespace {

using talos::event::Context;
using talos::event::Duration;
using talos::event::format_micros;
using talos::event::LatencyStats;
using talos::event::LoopMetrics;
using talos::event::MonotonicTime;
using talos::event::NullRecorder;
using talos::event::Poller;
using talos::event::RealtimeEventLoop;
using talos::event::log::LogWriter;

struct Config {
  int rate_hz{1000};
  int duration_ms{5000};
  int publishers{1};
  int publish_hz{1000};
  int tick_us{1000};
  std::string log_path;
  std::string csv_path;
};

constexpr const char* kUsage =
    "usage: loop_perf [options]\n"
    "  --rate-hz N        control timer frequency (default 1000)\n"
    "  --duration-ms N    run length in milliseconds (default 5000)\n"
    "  --publishers N     background publisher threads (default 1, may be "
    "0)\n"
    "  --publish-hz N     per-publisher send rate (default 1000)\n"
    "  --tick-us N        message-drain tick period, microseconds (default "
    "1000)\n"
    "  --log PATH         also record a dispatch log (default off)\n"
    "  --csv PATH         write raw per-cycle latency samples to a CSV "
    "(default off)\n"
    "  --help             print this message\n";

[[noreturn]] void UsageError(std::string_view message) {
  std::fprintf(stderr, "loop_perf: %.*s\n", static_cast<int>(message.size()),
               message.data());
  std::fputs(kUsage, stderr);
  std::exit(1);
}

// Parses a bounded integer flag value. A benchmark that silently clamped a
// typo (e.g. "--rate-hz 1oo0") would report numbers for a configuration
// nobody asked for, so anything but a clean integer above `minimum` is fatal.
int ParseIntOrDie(std::string_view flag, std::string_view value, int minimum) {
  int parsed = 0;
  const auto [ptr, ec] =
      std::from_chars(value.data(), value.data() + value.size(), parsed);

  if (ec != std::errc{} || ptr != value.data() + value.size() ||
      parsed < minimum) {
    UsageError("bad value '" + std::string{value} + "' for " +
               std::string{flag});
  }
  return parsed;
}

Config ParseArgs(int argc, char** argv) {
  Config config{};
  const std::vector<std::string_view> args{argv + 1, argv + argc};

  const auto next_value = [&](std::size_t& i,
                              std::string_view flag) -> std::string_view {
    if (i + 1 >= args.size()) {
      UsageError("missing value for " + std::string{flag});
    }
    return args[++i];
  };

  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string_view arg = args[i];

    if (arg == "--help" || arg == "-h") {
      std::fputs(kUsage, stdout);
      std::exit(0);
    } else if (arg == "--rate-hz") {
      config.rate_hz = ParseIntOrDie(arg, next_value(i, arg), 1);
    } else if (arg == "--duration-ms") {
      config.duration_ms = ParseIntOrDie(arg, next_value(i, arg), 1);
    } else if (arg == "--publishers") {
      config.publishers = ParseIntOrDie(arg, next_value(i, arg), 0);
    } else if (arg == "--publish-hz") {
      config.publish_hz = ParseIntOrDie(arg, next_value(i, arg), 1);
    } else if (arg == "--tick-us") {
      config.tick_us = ParseIntOrDie(arg, next_value(i, arg), 1);
    } else if (arg == "--log") {
      config.log_path = std::string{next_value(i, arg)};
    } else if (arg == "--csv") {
      config.csv_path = std::string{next_value(i, arg)};
    } else {
      UsageError("unrecognized flag '" + std::string{arg} + "'");
    }
  }

  return config;
}

// macOS caps shm_open names at 31 characters including the leading slash, so
// the topic name has to stay short even once a pid and an index are folded
// in; this comfortably fits under that limit.
std::string TopicName(::pid_t pid, int index) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "/lp_s_%d_%d", static_cast<int>(pid),
                index);
  return std::string{buffer};
}

// Registers the control timer plus one watcher per publisher, and turns
// every watched PerfMessage into an end-to-end latency sample.
template <typename Loop>
class Harness {
 public:
  Harness(Loop& loop, int watcher_count, ::pid_t pid, LatencyStats& ipc_latency,
          std::atomic<std::uint64_t>& received,
          std::vector<std::int64_t>* control_samples,
          std::vector<std::int64_t>* ipc_samples)
      : loop_{&loop},
        ipc_latency_{&ipc_latency},
        received_{&received},
        control_samples_{control_samples},
        ipc_samples_{ipc_samples} {
    control_ =
        talos::event::make_timer<&Harness::OnControl>(loop, "control", this);

    topics_.reserve(static_cast<std::size_t>(watcher_count));
    for (int index = 0; index < watcher_count; ++index) {
      topics_.push_back(TopicName(pid, index));
      talos::event::watch<EventsPerf::PerfMessage, &Harness::OnMessage>(
          loop, topics_.back(), this);
    }
  }

  talos::event::Timer<Loop>& control_timer() { return control_; }
  const std::vector<std::string>& topics() const { return topics_; }

 private:
  void OnControl(const Context& context) {
    // Deliberately trivial: this loop measures the platform's scheduling and
    // transport, not application work, so the handler should not add its own
    // noise to the handler-time histogram.
    if (control_samples_ != nullptr) {
      control_samples_->push_back(context.latency().count());
    }
  }

  void OnMessage(const Context&, const EventsPerf::PerfMessage& message) {
    // The publisher threads and the loop live in the same process, so
    // Poller::now() on one side is directly comparable to monotonic_now() on
    // the other: both read the same clock, with no cross-machine offset to
    // correct for.
    const MonotonicTime sent = MonotonicTime::from_nanos(message.sent_ns());
    const Duration latency = loop_->monotonic_now() - sent;

    ipc_latency_->add(latency);
    received_->fetch_add(1, std::memory_order_relaxed);

    if (ipc_samples_ != nullptr) {
      ipc_samples_->push_back(latency.count());
    }
  }

  Loop* loop_;
  LatencyStats* ipc_latency_;
  std::atomic<std::uint64_t>* received_;
  std::vector<std::int64_t>* control_samples_;
  std::vector<std::int64_t>* ipc_samples_;
  talos::event::Timer<Loop> control_{};
  std::vector<std::string> topics_;
};

// Publishes at a fixed rate on a grid anchored to `start`, so a slow send
// does not push every later send back by the same amount the way accumulated
// sleep_for(period) calls would.
void PublisherThread(std::string topic, int rate_hz,
                     const std::atomic<bool>& stop,
                     std::atomic<std::uint64_t>& published) {
  ipc::Publisher<EventsPerf::PerfMessage> publisher{topic};

  const Duration period{1'000'000'000LL / rate_hz};
  const MonotonicTime start = Poller::now();
  std::uint64_t sequence = 0;

  while (!stop.load(std::memory_order_relaxed)) {
    const MonotonicTime deadline =
        start + period * static_cast<std::int64_t>(sequence);

    // Sleep in short slices rather than for the whole remaining gap so a
    // stop request lands within a couple of milliseconds instead of at the
    // next scheduled send.
    MonotonicTime now = Poller::now();
    while (now < deadline) {
      if (stop.load(std::memory_order_relaxed)) {
        return;
      }
      const Duration remaining = deadline - now;
      std::this_thread::sleep_for(
          std::min(remaining, Duration{std::chrono::milliseconds{2}}));
      now = Poller::now();
    }

    const EventsPerf::PerfMessage message{sequence, Poller::now().nanos()};
    if (publisher.write(message).result == WriteResult::SUCCESS) {
      published.fetch_add(1, std::memory_order_relaxed);
    }
    ++sequence;
  }
}

// count / mean / stddev / p50 / p90 / p99 / p99.9 / max / jitter: the full
// set LoopMetrics::report() does not itself print (it omits stddev and p90),
// so the two latencies the prompt calls out by name -- control-timer dispatch
// latency and end-to-end IPC latency -- get their own detailed line here.
void PrintLatencyDetail(const char* label, const LatencyStats& stats) {
  std::printf(
      "  %-24s %8llu %10s %10s %10s %10s %10s %10s %10s\n", label,
      static_cast<unsigned long long>(stats.count()),
      format_micros(stats.mean_ns()).c_str(),
      format_micros(static_cast<std::int64_t>(std::llround(stats.stddev_ns())))
          .c_str(),
      format_micros(stats.percentile_ns(0.50)).c_str(),
      format_micros(stats.percentile_ns(0.90)).c_str(),
      format_micros(stats.percentile_ns(0.99)).c_str(),
      format_micros(stats.percentile_ns(0.999)).c_str(),
      format_micros(stats.max_ns()).c_str());
  std::printf("  %-24s %8s %10s %10s (jitter, peak-to-peak: %s us)\n", "", "",
              "", "", format_micros(stats.jitter_ns()).c_str());
}

void PrintLatencyHeader() {
  std::printf("  %-24s %8s %10s %10s %10s %10s %10s %10s %10s\n", "metric (us)",
              "count", "mean", "stddev", "p50", "p90", "p99", "p99.9", "max");
}

void WriteCsv(const std::string& path,
              const std::vector<std::int64_t>& control_samples,
              const std::vector<std::int64_t>& ipc_samples) {
  std::ofstream out{path};
  if (!out) {
    std::fprintf(stderr, "loop_perf: could not open --csv path '%s'\n",
                 path.c_str());
    return;
  }

  out << "kind,index,latency_ns\n";
  for (std::size_t i = 0; i < control_samples.size(); ++i) {
    out << "control," << i << ',' << control_samples[i] << '\n';
  }
  for (std::size_t i = 0; i < ipc_samples.size(); ++i) {
    out << "ipc," << i << ',' << ipc_samples[i] << '\n';
  }
}

// Runs the whole benchmark on an already-configured loop. Templated on the
// loop type (rather than just the recorder) so the same body drives both the
// NullRecorder and LogWriter instantiations built in main().
template <typename Recorder>
int RunBenchmark(const Config& config,
                 RealtimeEventLoop<Recorder, LoopMetrics>& loop) {
  using Loop = RealtimeEventLoop<Recorder, LoopMetrics>;

  const ::pid_t pid = ::getpid();

  LatencyStats ipc_latency;
  std::atomic<std::uint64_t> received{0};
  std::atomic<std::uint64_t> published{0};

  // Reserved up front, never grown mid-run: an allocation on the sample path
  // would show up in the very histograms this tool is trying to measure.
  const bool want_csv = !config.csv_path.empty();
  std::vector<std::int64_t> control_samples;
  std::vector<std::int64_t> ipc_samples;
  if (want_csv) {
    const auto seconds = static_cast<double>(config.duration_ms) / 1000.0;
    control_samples.reserve(static_cast<std::size_t>(config.rate_hz * seconds) +
                            64);
    ipc_samples.reserve(static_cast<std::size_t>(config.publish_hz *
                                                 config.publishers * seconds) +
                        64);
  }

  Harness<Loop> harness{loop,
                        config.publishers,
                        pid,
                        ipc_latency,
                        received,
                        want_csv ? &control_samples : nullptr,
                        want_csv ? &ipc_samples : nullptr};

  const Duration control_period{1'000'000'000LL / config.rate_hz};
  harness.control_timer().setup_periodic(Poller::now() + control_period,
                                         control_period);

  std::atomic<bool> stop{false};
  std::vector<std::thread> publisher_threads;
  publisher_threads.reserve(harness.topics().size());
  for (const std::string& topic : harness.topics()) {
    publisher_threads.emplace_back(PublisherThread, topic, config.publish_hz,
                                   std::cref(stop), std::ref(published));
  }

  std::printf("configuration:\n");
  std::printf("  rate-hz:      %d\n", config.rate_hz);
  std::printf("  duration-ms:  %d\n", config.duration_ms);
  std::printf("  publishers:   %d\n", config.publishers);
  std::printf("  publish-hz:   %d\n", config.publish_hz);
  std::printf("  tick-us:      %d\n", config.tick_us);
  std::printf("  logging:      %s\n",
              config.log_path.empty() ? "off" : config.log_path.c_str());
  std::printf("  csv:          %s\n",
              config.csv_path.empty() ? "off" : config.csv_path.c_str());
  std::printf("\n");

  loop.run_for(std::chrono::milliseconds{config.duration_ms});

  // Publishers pace themselves off Poller::now(), not off the loop, so they
  // keep sending until told to stop; join before touching any of the counts
  // they write so the report reflects a quiescent state.
  stop.store(true, std::memory_order_relaxed);
  for (std::thread& thread : publisher_threads) {
    thread.join();
  }

  const std::uint16_t control_id = harness.control_timer().id();
  const LoopMetrics& metrics = loop.metrics();

  std::printf("=== loop metrics (LoopMetrics::report) ===\n");
  std::printf("%s\n", metrics.report(loop.manifest()).c_str());

  std::printf("=== control-timer dispatch latency (now - event_time) ===\n");
  PrintLatencyHeader();
  PrintLatencyDetail("dispatch latency", metrics.source(control_id).latency);
  const std::uint64_t overruns = metrics.source(control_id).overruns;
  std::printf("  overruns (missed periods): %llu\n",
              static_cast<unsigned long long>(overruns));
  std::printf("\n");

  std::printf(
      "=== end-to-end IPC latency (monotonic_now() - PerfMessage.sent_ns) "
      "===\n");
  std::printf(
      "this is the number that says whether the tick-poll design is fast "
      "enough:\n");
  PrintLatencyHeader();
  PrintLatencyDetail("ipc latency", ipc_latency);
  std::printf("\n");

  std::uint64_t dropped_total = 0;
  // Watcher source ids are assigned right after the control timer, in
  // registration order, so they occupy a contiguous range.
  for (std::size_t offset = 0; offset < harness.topics().size(); ++offset) {
    const auto id = static_cast<std::uint16_t>(control_id + 1 + offset);
    dropped_total += metrics.source(id).dropped;
  }

  std::printf("=== messages ===\n");
  std::printf("  published:        %llu\n",
              static_cast<unsigned long long>(
                  published.load(std::memory_order_relaxed)));
  std::printf("  received:         %llu\n",
              static_cast<unsigned long long>(
                  received.load(std::memory_order_relaxed)));
  std::printf("  dropped (lapped): %llu\n",
              static_cast<unsigned long long>(dropped_total));
  std::printf("\n");

  std::printf("=== loop utilization ===\n");
  std::printf("  %.2f%% of wall time spent inside handlers\n",
              metrics.utilization() * 100.0);
  std::printf("\n");

  if constexpr (std::is_same_v<Recorder, LogWriter>) {
    std::printf("=== logging overhead ===\n");
    std::printf("  logging:  on (%s)\n", config.log_path.c_str());
    std::printf("  stalls:   %llu (loop thread waits for a free chunk)\n",
                static_cast<unsigned long long>(loop.recorder().stalls()));
    std::printf("\n");
  } else {
    std::printf("=== logging overhead ===\n");
    std::printf("  logging:  off\n");
    std::printf("\n");
  }

  if (want_csv) {
    WriteCsv(config.csv_path, control_samples, ipc_samples);
  }

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const Config config = ParseArgs(argc, argv);

  if (!config.log_path.empty()) {
    LogWriter recorder{config.log_path, "loop_perf"};

    RealtimeEventLoop<LogWriter, LoopMetrics>::Options options{};
    options.tick_period = std::chrono::microseconds{config.tick_us};

    RealtimeEventLoop<LogWriter, LoopMetrics> loop{std::move(recorder),
                                                   options};
    return RunBenchmark(config, loop);
  }

  RealtimeEventLoop<NullRecorder, LoopMetrics>::Options options{};
  options.tick_period = std::chrono::microseconds{config.tick_us};

  RealtimeEventLoop<NullRecorder, LoopMetrics> loop{options};
  return RunBenchmark(config, loop);
}
