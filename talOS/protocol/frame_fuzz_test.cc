#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <vector>

#include "talOS/protocol/frame.h"
#include "talOS/protocol/fuzz_corpus.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, std::size_t size) {
  talos::protocol::DecodedFrame decoded;
  if (talos::protocol::DecodeFrame(data, size, &decoded) ==
      talos::protocol::DecodeStatus::kOk) {
    if (decoded.payload < data || decoded.payload_size > size ||
        decoded.payload + decoded.payload_size > data + size) {
      std::abort();
    }
    std::vector<uint8_t> encoded(talos::protocol::kMaxFrameSize);
    const auto written = talos::protocol::EncodeFrame(
        decoded.header.type, decoded.header.flags, decoded.header.sequence,
        decoded.header.monotonic_time_us, decoded.payload, decoded.payload_size,
        encoded.data(), encoded.size());
    talos::protocol::DecodedFrame again;
    if (written == 0 ||
        talos::protocol::DecodeFrame(encoded.data(), written, &again) !=
            talos::protocol::DecodeStatus::kOk)
      std::abort();
    std::vector<uint8_t> canonical(encoded.begin(), encoded.begin() + written);
    std::vector<uint8_t> roundtrip(talos::protocol::kMaxFrameSize);
    roundtrip.resize(talos::protocol::EncodeFrame(
        again.header.type, again.header.flags, again.header.sequence,
        again.header.monotonic_time_us, again.payload, again.payload_size,
        roundtrip.data(), roundtrip.size()));
    EXPECT_EQ(roundtrip, canonical);
  }
  return 0;
}

#ifndef TALOS_LIBFUZZER
TEST(FrameFuzzTest, BoundedMutations) {
  using namespace talos::protocol;
  std::array<uint8_t, kMaxPayloadSize> payload{};
  std::vector<std::vector<uint8_t>> seeds;
  for (const auto size : {std::size_t{0}, std::size_t{17}, kMaxPayloadSize}) {
    std::vector<uint8_t> frame(kMaxFrameSize);
    frame.resize(EncodeFrame(static_cast<FrameType>(1), 0, 42, 123,
                             payload.data(), size, frame.data(), frame.size()));
    seeds.push_back(frame);
  }
  talos::protocol::testing::RunCorpus(seeds, LLVMFuzzerTestOneInput);
}
#endif
