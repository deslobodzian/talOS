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
using talos::protocol::UdpSocketOptions;
using talos::protocol::UdpStatus;
using talos::protocol::UdpStatusName;
using Clock = std::chrono::steady_clock;

struct Options {
  int iterations = 10000;
  int warmup = 1000;
  int payload_bytes = 128;
  uint16_t mac_port = 59200;
  uint16_t rio_port = 59201;
};

uint64_t NowNs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          Clock::now().time_since_epoch())
          .count());
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
    } else if (ParseIntArg(argv[i], "--warmup=", &value)) {
      options.warmup = value;
    } else if (ParseIntArg(argv[i], "--payload_bytes=", &value)) {
      options.payload_bytes = value;
    } else if (ParseIntArg(argv[i], "--mac_port=", &value)) {
      options.mac_port = static_cast<uint16_t>(value);
    } else if (ParseIntArg(argv[i], "--rio_port=", &value)) {
      options.rio_port = static_cast<uint16_t>(value);
    }
  }
  return options;
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

bool SendPayload(RuntimeUdpPeer* peer, FrameType type, uint64_t timestamp_ns,
                 int payload_bytes, uint8_t fill) {
  uint8_t* payload = peer->MutablePayloadBuffer(payload_bytes);
  if (payload == nullptr) {
    std::fprintf(stderr, "payload_bytes exceeds max payload\n");
    return false;
  }
  std::memset(payload, fill, static_cast<std::size_t>(payload_bytes));
  const UdpStatus status = peer->SendBufferedFrame(
      type, talos::protocol::kFlagNone, timestamp_ns / 1000u, payload_bytes);
  if (status != UdpStatus::kOk) {
    std::fprintf(stderr, "send failed: %s\n", UdpStatusName(status));
    return false;
  }
  return true;
}

bool ReceiveBlocking(RuntimeUdpPeer* peer, DecodedFrame* frame) {
  for (;;) {
    const UdpStatus status = peer->TryReceive(frame);
    if (status == UdpStatus::kOk) {
      return true;
    }
    if (status != UdpStatus::kWouldBlock) {
      std::fprintf(stderr, "receive failed: %s\n", UdpStatusName(status));
      return false;
    }
    std::this_thread::yield();
  }
}

bool RunExchange(RuntimeUdpPeer* mac, RuntimeUdpPeer* rio, int payload_bytes,
                 uint64_t* rtt_ns) {
  const uint64_t start_ns = NowNs();
  if (!SendPayload(mac, FrameType::kRioCommand, start_ns, payload_bytes,
                   0xA5)) {
    return false;
  }

  DecodedFrame rio_rx;
  if (!ReceiveBlocking(rio, &rio_rx)) {
    return false;
  }
  if (static_cast<int>(rio_rx.payload_size) != payload_bytes) {
    std::fprintf(stderr, "rio received wrong payload size\n");
    return false;
  }

  if (!SendPayload(rio, FrameType::kRioState, NowNs(), payload_bytes, 0x5A)) {
    return false;
  }

  DecodedFrame mac_rx;
  if (!ReceiveBlocking(mac, &mac_rx)) {
    return false;
  }
  if (static_cast<int>(mac_rx.payload_size) != payload_bytes) {
    std::fprintf(stderr, "mac received wrong payload size\n");
    return false;
  }

  *rtt_ns = NowNs() - start_ns;
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  const Options options = ParseOptions(argc, argv);
  if (options.iterations <= 0 || options.warmup < 0 ||
      options.payload_bytes < 0 ||
      options.payload_bytes >
          static_cast<int>(talos::protocol::kMaxPayloadSize)) {
    std::fprintf(stderr,
                 "usage: udp_perf [--iterations=N] [--warmup=N] "
                 "[--payload_bytes=N] [--mac_port=N] [--rio_port=N]\n");
    return 2;
  }

  UdpSocketOptions socket_options;
  socket_options.non_blocking = true;
  socket_options.receive_buffer_bytes = 1024 * 1024;
  socket_options.send_buffer_bytes = 1024 * 1024;

  RuntimeUdpPeer mac;
  RuntimeUdpPeer rio;
  UdpStatus status = rio.Open("127.0.0.1", options.rio_port, "127.0.0.1",
                              options.mac_port, socket_options);
  if (status != UdpStatus::kOk) {
    std::fprintf(stderr, "rio open failed: %s\n", UdpStatusName(status));
    return 1;
  }
  status = mac.Open("127.0.0.1", options.mac_port, "127.0.0.1",
                    options.rio_port, socket_options);
  if (status != UdpStatus::kOk) {
    std::fprintf(stderr, "mac open failed: %s\n", UdpStatusName(status));
    return 1;
  }

  uint64_t rtt_ns = 0;
  for (int i = 0; i < options.warmup; ++i) {
    if (!RunExchange(&mac, &rio, options.payload_bytes, &rtt_ns)) {
      return 1;
    }
  }

  std::vector<uint64_t> round_trip_ns;
  round_trip_ns.reserve(static_cast<std::size_t>(options.iterations));
  const uint64_t bench_start_ns = NowNs();
  for (int i = 0; i < options.iterations; ++i) {
    if (!RunExchange(&mac, &rio, options.payload_bytes, &rtt_ns)) {
      return 1;
    }
    round_trip_ns.push_back(rtt_ns);
  }
  const uint64_t bench_elapsed_ns = NowNs() - bench_start_ns;

  std::sort(round_trip_ns.begin(), round_trip_ns.end());
  const double elapsed_s = static_cast<double>(bench_elapsed_ns) / 1.0e9;
  const double exchanges_per_s =
      static_cast<double>(options.iterations) / elapsed_s;
  const double frames_per_s = exchanges_per_s * 2.0;
  const double wire_bytes_per_exchange =
      static_cast<double>((talos::protocol::kFrameHeaderSize +
                           static_cast<std::size_t>(options.payload_bytes)) *
                          2u);
  const double mib_per_s =
      (wire_bytes_per_exchange * exchanges_per_s) / (1024.0 * 1024.0);

  std::printf("TalOS UDP loopback perf\n");
  std::printf("  iterations:       %d\n", options.iterations);
  std::printf("  warmup:           %d\n", options.warmup);
  std::printf("  payload bytes:    %d\n", options.payload_bytes);
  std::printf("  elapsed:          %.6f s\n", elapsed_s);
  std::printf("  exchanges/sec:    %.2f\n", exchanges_per_s);
  std::printf("  frames/sec:       %.2f\n", frames_per_s);
  std::printf("  payload+header:   %.2f MiB/s\n", mib_per_s);
  std::printf("  RTT p50:          %.0f ns\n", Percentile(round_trip_ns, 50.0));
  std::printf("  RTT p90:          %.0f ns\n", Percentile(round_trip_ns, 90.0));
  std::printf("  RTT p99:          %.0f ns\n", Percentile(round_trip_ns, 99.0));
  std::printf("  RTT max:          %llu ns\n",
              static_cast<unsigned long long>(round_trip_ns.back()));
  std::printf("  approx one-way p50: %.0f ns\n",
              Percentile(round_trip_ns, 50.0) / 2.0);
  std::printf("  mac sent/recv:    %llu / %llu\n",
              static_cast<unsigned long long>(mac.stats().sent_frames),
              static_cast<unsigned long long>(mac.stats().received_frames));
  std::printf("  rio sent/recv:    %llu / %llu\n",
              static_cast<unsigned long long>(rio.stats().sent_frames),
              static_cast<unsigned long long>(rio.stats().received_frames));
  std::printf("  sequence gaps:    mac=%llu rio=%llu\n",
              static_cast<unsigned long long>(mac.stats().sequence_gaps),
              static_cast<unsigned long long>(rio.stats().sequence_gaps));
  return 0;
}
