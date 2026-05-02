#include <algorithm>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "talos/protocol/runtime_udp.h"

namespace {

using talos::protocol::DecodedFrame;
using talos::protocol::FrameType;
using talos::protocol::RuntimeUdpPeer;
using talos::protocol::RuntimeValuesHeader;
using talos::protocol::SignalValue;
using talos::protocol::UdpSocketOptions;
using talos::protocol::UdpStatus;
using talos::protocol::UdpStatusName;
using talos::protocol::ValueType;
using Clock = std::chrono::steady_clock;

constexpr uint64_t kSimManifestId = 1;

struct Payload {
  RuntimeValuesHeader header;
  SignalValue values[2];
};

struct Options {
  int iterations = 100;
  int timeout_ms = 3000;
  uint16_t talos_port = 5803;
  uint16_t rio_port = 5802;
};

uint64_t NowUs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          Clock::now().time_since_epoch())
          .count());
}

uint64_t NowNs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          Clock::now().time_since_epoch())
          .count());
}

double Percentile(const std::vector<uint64_t>& sorted_values, double pct) {
  if (sorted_values.empty()) {
    return 0.0;
  }
  const double index =
      (pct / 100.0) * static_cast<double>(sorted_values.size() - 1);
  const auto lower = static_cast<std::size_t>(index);
  const auto upper = std::min(lower + 1, sorted_values.size() - 1);
  const double fraction = index - static_cast<double>(lower);
  return static_cast<double>(sorted_values[lower]) * (1.0 - fraction) +
         static_cast<double>(sorted_values[upper]) * fraction;
}

bool ParseIntArg(const char* arg, const char* prefix, int* out) {
  const std::size_t prefix_len = std::strlen(prefix);
  if (std::strncmp(arg, prefix, prefix_len) != 0) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  const int64_t value = std::strtoll(arg + prefix_len, &end, 10);
  if (errno != 0 || end == arg + prefix_len || *end != '\0' ||
      value < INT_MIN || value > INT_MAX) {
    return false;
  }
  *out = static_cast<int>(value);
  return true;
}

Options ParseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    int value = 0;
    if (ParseIntArg(argv[i], "--iterations=", &value)) {
      options.iterations = value;
    } else if (ParseIntArg(argv[i], "--timeout_ms=", &value)) {
      options.timeout_ms = value;
    } else if (ParseIntArg(argv[i], "--talos_port=", &value)) {
      options.talos_port = static_cast<uint16_t>(value);
    } else if (ParseIntArg(argv[i], "--rio_port=", &value)) {
      options.rio_port = static_cast<uint16_t>(value);
    }
  }
  return options;
}

bool SendCommand(RuntimeUdpPeer* peer, double value) {
  auto* payload =
      reinterpret_cast<Payload*>(peer->MutablePayloadBuffer(sizeof(Payload)));
  if (payload == nullptr) {
    return false;
  }
  payload->header.manifest_id = kSimManifestId;
  payload->header.value_count = 1;
  payload->header.reserved = 0;
  payload->values[0].id = 1;
  payload->values[0].value_type = ValueType::kFloat64;
  payload->values[0].reserved = 0;
  payload->values[0].float64_value = value;
  payload->values[1] = {};

  const UdpStatus status = peer->SendBufferedFrame(
      FrameType::kRioCommand, talos::protocol::kFlagNone, NowUs(),
      sizeof(RuntimeValuesHeader) + sizeof(SignalValue));
  if (status != UdpStatus::kOk) {
    std::fprintf(stderr, "send command failed: %s\n", UdpStatusName(status));
    return false;
  }
  return true;
}

bool WaitForState(RuntimeUdpPeer* peer, double expected_value,
                  std::chrono::milliseconds timeout) {
  const auto deadline = Clock::now() + timeout;
  while (Clock::now() < deadline) {
    DecodedFrame frame;
    const UdpStatus status = peer->TryReceive(&frame);
    if (status == UdpStatus::kWouldBlock) {
      std::this_thread::yield();
      continue;
    }
    if (status != UdpStatus::kOk) {
      std::fprintf(stderr, "receive state failed: %s\n", UdpStatusName(status));
      return false;
    }
    if (frame.header.type != FrameType::kRioState ||
        frame.payload_size <
            sizeof(RuntimeValuesHeader) + sizeof(SignalValue)) {
      continue;
    }

    Payload payload{};
    const std::size_t copy_size = frame.payload_size < sizeof(payload)
                                      ? frame.payload_size
                                      : sizeof(payload);
    std::memcpy(&payload, frame.payload, copy_size);
    if (payload.header.manifest_id != kSimManifestId ||
        payload.header.value_count < 1 ||
        payload.values[0].value_type != ValueType::kFloat64) {
      continue;
    }
    if (payload.values[0].float64_value != expected_value) {
      std::fprintf(stderr, "state echoed %.3f, expected %.3f\n",
                   payload.values[0].float64_value, expected_value);
      return false;
    }
    return true;
  }

  std::fprintf(stderr,
               "timed out waiting for Rio sim state. Is ./gradlew "
               "simulateNative running?\n");
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  const Options options = ParseOptions(argc, argv);
  if (options.iterations <= 0 || options.timeout_ms <= 0) {
    std::fprintf(stderr,
                 "usage: sim_udp_smoke [--iterations=N] [--timeout_ms=N] "
                 "[--talos_port=N] [--rio_port=N]\n");
    return 2;
  }

  UdpSocketOptions socket_options;
  socket_options.non_blocking = true;
  RuntimeUdpPeer peer;
  const UdpStatus status =
      peer.Open("127.0.0.1", options.talos_port, "127.0.0.1", options.rio_port,
                socket_options);
  if (status != UdpStatus::kOk) {
    std::fprintf(stderr, "open failed: %s\n", UdpStatusName(status));
    return 1;
  }

  std::vector<uint64_t> round_trip_ns;
  round_trip_ns.reserve(static_cast<std::size_t>(options.iterations));
  const uint64_t bench_start_ns = NowNs();
  for (int i = 0; i < options.iterations; ++i) {
    const double command = 1000.0 + static_cast<double>(i);
    const uint64_t exchange_start_ns = NowNs();
    if (!SendCommand(&peer, command)) {
      return 1;
    }
    if (!WaitForState(&peer, command,
                      std::chrono::milliseconds(options.timeout_ms))) {
      return 1;
    }
    round_trip_ns.push_back(NowNs() - exchange_start_ns);
  }
  const uint64_t elapsed_ns = NowNs() - bench_start_ns;
  std::sort(round_trip_ns.begin(), round_trip_ns.end());

  const double elapsed_s = static_cast<double>(elapsed_ns) / 1.0e9;
  const double exchanges_per_s =
      static_cast<double>(options.iterations) / elapsed_s;
  std::printf("TalOS <-> WPILib simulation UDP smoke passed\n");
  std::printf("  command/state exchanges: %d\n", options.iterations);
  std::printf("  elapsed:                 %.6f s\n", elapsed_s);
  std::printf("  exchanges/sec:           %.2f\n", exchanges_per_s);
  std::printf("  RTT p50:                 %.0f ns\n",
              Percentile(round_trip_ns, 50.0));
  std::printf("  RTT p90:                 %.0f ns\n",
              Percentile(round_trip_ns, 90.0));
  std::printf("  RTT p99:                 %.0f ns\n",
              Percentile(round_trip_ns, 99.0));
  std::printf("  RTT max:                 %llu ns\n",
              static_cast<unsigned long long>(round_trip_ns.back()));
  std::printf("  approx one-way p50:      %.0f ns\n",
              Percentile(round_trip_ns, 50.0) / 2.0);
  std::printf("  sent/received frames:    %llu / %llu\n",
              static_cast<unsigned long long>(peer.stats().sent_frames),
              static_cast<unsigned long long>(peer.stats().received_frames));
  std::printf("  sequence gaps:           %llu\n",
              static_cast<unsigned long long>(peer.stats().sequence_gaps));
  return 0;
}
