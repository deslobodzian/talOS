#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "talOS/rtms/rtms.h"
#include "talOS/telemetry/frames.h"
#include "talOS/telemetry/wire.h"

namespace wire = talos::telemetry_wire;
using talos::telemetry::ChannelDef;
namespace {

volatile std::sig_atomic_t stop = 0;
void signal_handler(int) { stop = 1; }

}  // namespace

int main(int argc, char** argv) try {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  RTMSQueue queue(argc > 1 ? argv[1] : "/talos/telemetry", wire::slot_bytes, 8,
                  talos::schema::Bytes(), wire::slot_count,
                  {.overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
                   .read_mode = ReadMode::SEQUENCE,
                   .publisher = true});
  // The segment exists now; tests wait for this line.
  std::cout << "ready\n" << std::flush;

  std::vector<ChannelDef> dictionary = {
      {"drive/velocity", "m/s", {}, Talos::Telemetry::ChannelKind_Double},
      {"intake/inhibit",
       "",
       {"DRIVE", "GATEWAY_NOT_READY", "TARGET_DISABLED", "TARGET_STALE",
        "FEEDBACK_INVALID", "DEMAND_NONFINITE"},
       Talos::Telemetry::ChannelKind_UInt},
      {"pose/valid", "", {}, Talos::Telemetry::ChannelKind_Bool},
  };
  const auto channels = argc > 2 ? std::stoul(argv[2]) : dictionary.size();
  if (channels < dictionary.size() ||
      channels > talos::telemetry_wire::kMaxChannels) {
    throw std::runtime_error(
        "mock channel count must be in [3, " +
        std::to_string(talos::telemetry_wire::kMaxChannels) + "]");
  }
  // Extra channels all change every tick, like a robot in motion: the
  // heaviest change-only load per channel.
  while (dictionary.size() < channels) {
    dictionary.push_back({"test/channel_" + std::to_string(dictionary.size()),
                          "unit",
                          {},
                          Talos::Telemetry::ChannelKind_Double});
  }
  const std::uint32_t dictionary_id =
      talos::telemetry::DictionaryId(dictionary);
  std::random_device device;
  const std::uint64_t stream_id =
      (static_cast<std::uint64_t>(device()) << 32) | device();

  std::array<std::byte, wire::slot_bytes> slot{};
  // The buffer already starts with its size prefix; the slot is that prefix
  // plus buffer followed by unused padding.
  const auto publish = [&](std::span<const std::uint8_t> page) {
    if (page.size() > slot.size())
      throw std::runtime_error("page exceeds slot");
    std::memcpy(slot.data(), page.data(), page.size());
    queue.write(std::span<const std::byte>{slot.data(), slot.size()});
  };
  flatbuffers::FlatBufferBuilder builder;
  std::vector<std::uint32_t> indices(dictionary.size());
  std::vector<std::uint64_t> samples(dictionary.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    indices[i] = static_cast<std::uint32_t>(i);
  }
  auto next = std::chrono::steady_clock::now();
  for (std::uint64_t seq = 0; !stop; ++seq) {
    const double t = static_cast<double>(seq) * 0.005;
    const Talos::Telemetry::Pose pose(8 + 3 * std::cos(t * .25),
                                      4 + 2 * std::sin(t * .25), 0, 0, 0,
                                      t * .25 + 1.5707963267948966);
    const talos::telemetry::PageHeader header{
        .stream_id = stream_id,
        .sequence_id = seq,
        .timestamp_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count()),
        .dictionary_id = dictionary_id,
        .channel_count = static_cast<std::uint32_t>(dictionary.size())};
    // Dictionary on the first tick and every 100th, never otherwise.
    if (seq % 100 == 0) {
      talos::telemetry::EncodeDictionaryPages(builder, header, dictionary,
                                              publish);
    }
    samples[0] = talos::telemetry::RawFromDouble(0.75 + 0.25 * std::sin(t));
    samples[1] = (seq / 200) % dictionary[1].labels.size();
    samples[2] = 1;
    for (std::size_t i = 3; i < samples.size(); ++i) {
      samples[i] =
          talos::telemetry::RawFromDouble(std::sin(t + static_cast<double>(i)));
    }
    if (!talos::telemetry::EncodeValuePages(
            builder, header, pose, true, indices, samples, {},
            [&publish](std::span<const std::uint8_t> page,
                       const talos::telemetry::PageSpan&) { publish(page); })) {
      throw std::logic_error("mock: a value page outgrew its bound");
    }
    next += std::chrono::milliseconds(5);
    std::this_thread::sleep_until(next);
  }
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
