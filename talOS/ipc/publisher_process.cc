#include <charconv>
#include <chrono>
#include <iostream>
#include <numeric>
#include <string_view>
#include <vector>
#include <thread>

#include "publisher.h"
#include "talOS/ipc/ipc_test_message_generated.h"

struct IPCTestParameters {
  std::string topic{"/process_test"};
  std::chrono::seconds duration{10};
};

void calculate_metrics(
    const std::vector<std::chrono::nanoseconds>& time_deltas) {
  if (time_deltas.empty()) {
    std::cout << "No time deltas!\n";
    return;
  }

  const auto total_duration = std::accumulate(
      time_deltas.begin(), time_deltas.end(), std::chrono::nanoseconds{0});

  const auto average_duration =
      total_duration /
      static_cast<std::chrono::nanoseconds::rep>(time_deltas.size());

  const auto avg_ns = average_duration.count();
  const auto avg_s = std::chrono::duration<double>(average_duration).count();

  std::cout << "Average time (ns): " << avg_ns << '\n';
  std::cout << "Average time (s):  " << avg_s << '\n';
}

IPCTestParameters args_parse(int argc, char** argv) {
  std::vector<std::string_view> args{argv + 1, argv + argc};
  IPCTestParameters params{};

  for (std::size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "-d" || args[i] == "--duration") {
      if (i + 1 >= args.size()) {
        std::cerr << "Error: Missing value for " << args[i] << "\n";
        return params;
      }

      std::string_view duration_str = args[++i];

      int duration_val = 0;
      auto [ptr, err] = std::from_chars(
          duration_str.data(), duration_str.data() + duration_str.size(),
          duration_val);

      if (err != std::errc{} ||
          ptr != duration_str.data() + duration_str.size()) {
        std::cerr << "Error: '" << duration_str
                  << "' is not a valid integer.\n";
        return params;
      }

      params.duration = std::chrono::seconds{duration_val};
    } else if (args[i] == "-t" || args[i] == "--topic") {
      if (i + 1 >= args.size()) {
        std::cerr << "Error: Missing value for " << args[i] << "\n";
        return params;
      }
      params.topic = args[++i];
    }
  }
  return params;
}

int main(int argc, char** argv) {
  auto params = args_parse(argc, argv);
  using Clock = std::chrono::steady_clock;

  std::printf("Running publisher process:\n  Time: %ld\n  Topic: %s\n",
              params.duration.count(), params.topic.c_str());

  ipc::Publisher<IPCMessage::TimeMessage> pub{params.topic};
  std::vector<std::chrono::nanoseconds> write_durations;

  auto start = Clock::now();
  auto run_duration = start + params.duration;

  while (Clock::now() < run_duration) {
    start = Clock::now();


    auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    start.time_since_epoch())
                    .count();

    IPCMessage::TimeMessage msg{0, static_cast<uint64_t>(nsec)};

    pub.write(msg);

    auto stop = Clock::now();

    auto duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

    write_durations.push_back(duration);
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1us);
  }

  calculate_metrics(write_durations);
}
