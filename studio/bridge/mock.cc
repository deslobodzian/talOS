#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "talOS/rtms/rtms.h"
#include "talOS/telemetry/telemetry_generated.h"
#include "talOS/telemetry/wire.h"

namespace wire = talos::telemetry_wire;
namespace {

volatile std::sig_atomic_t stop = 0;
void signal_handler(int) { stop = 1; }

// The mock's fixed dictionary, mirroring the producer's FNV-1a scheme
// (talOS/telemetry/README.md) so its frames carry a matching dictionary_id.
struct ChannelDef {
  std::string name;
  std::string unit;
  std::vector<std::string> labels;
};

std::uint32_t DictionaryId(const std::vector<ChannelDef>& dictionary) {
  std::uint32_t hash = 2166136261u;
  auto mix = [&hash](const std::string& text) {
    for (const char c : text) {
      hash ^= static_cast<std::uint8_t>(c);
      hash *= 16777619u;
    }
  };
  auto separator = [&hash](std::uint8_t byte) {
    hash ^= byte;
    hash *= 16777619u;
  };
  for (const ChannelDef& channel : dictionary) {
    mix(channel.name);
    separator(0);
    mix(channel.unit);
    separator(0);
    for (const std::string& label : channel.labels) {
      mix(label);
      separator(0);
    }
    separator(0xff);
  }
  return hash;
}

}  // namespace

int main(int argc, char** argv) try {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  RTMSQueue queue(argc > 1 ? argv[1] : "/talos/telemetry", wire::slot_bytes, 8,
                  wire::slot_count,
                  {OverflowPolicy::DROP_NEWEST, ReadMode::SEQUENCE});
  // The segment exists now; tests wait for this line before starting a bridge.
  std::cout << "ready\n" << std::flush;

  const std::vector<ChannelDef> dictionary = {
      {"drive/velocity", "m/s", {}},
      {"intake/inhibit",
       "",
       {"DRIVE", "GATEWAY_NOT_READY", "TARGET_DISABLED", "TARGET_STALE",
        "FEEDBACK_INVALID", "DEMAND_NONFINITE"}},
      {"pose/valid", "", {}},
  };
  const std::uint32_t dictionary_id = DictionaryId(dictionary);
  std::random_device device;
  const std::uint64_t stream_id =
      (static_cast<std::uint64_t>(device()) << 32) | device();

  std::array<std::byte, wire::slot_bytes> slot{};
  flatbuffers::FlatBufferBuilder builder;
  auto next = std::chrono::steady_clock::now();
  for (std::uint64_t seq = 0; !stop; ++seq) {
    builder.Clear();
    const double t = static_cast<double>(seq) * 0.005;
    Talos::Telemetry::Pose pose(8 + 3 * std::cos(t * .25),
                                4 + 2 * std::sin(t * .25), 0, 0, 0,
                                t * .25 + 1.5707963267948966);
    flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<Talos::Telemetry::ChannelInfo>>>
        dictionary_offset = 0;
    // Dictionary on the first frame and every 100th, never otherwise.
    if (seq % 100 == 0) {
      std::vector<flatbuffers::Offset<Talos::Telemetry::ChannelInfo>> entries;
      for (const ChannelDef& channel : dictionary) {
        std::vector<flatbuffers::Offset<flatbuffers::String>> labels;
        for (const std::string& label : channel.labels) {
          labels.push_back(builder.CreateString(label));
        }
        entries.push_back(Talos::Telemetry::CreateChannelInfo(
            builder, builder.CreateString(channel.name),
            channel.unit.empty() ? 0 : builder.CreateString(channel.unit),
            channel.labels.empty() ? 0 : builder.CreateVector(labels)));
      }
      dictionary_offset = builder.CreateVector(entries);
    }
    const double inhibit =
        static_cast<double>((seq / 200) % dictionary[1].labels.size());
    auto values = builder.CreateVector(
        std::vector<double>{0.75 + 0.25 * std::sin(t), inhibit, 1.0});
    auto frame = Talos::Telemetry::CreateFrame(
        builder, stream_id, seq, seq * 5000000ULL, dictionary_id, &pose, values,
        dictionary_offset);
    Talos::Telemetry::FinishSizePrefixedFrameBuffer(builder, frame);
    const auto size = builder.GetSize();
    if (size > slot.size()) throw std::runtime_error("frame exceeds slot");
    // The buffer already starts with its size prefix; the slot is that
    // prefix plus buffer followed by unused padding.
    std::memcpy(slot.data(), builder.GetBufferPointer(), size);
    queue.write(std::span<const std::byte>{slot.data(), slot.size()});
    next += std::chrono::milliseconds(5);
    std::this_thread::sleep_until(next);
  }
} catch (const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 1;
}
