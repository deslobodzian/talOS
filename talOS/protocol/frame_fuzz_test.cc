#ifndef TALOS_LIBFUZZER
#include <gtest/gtest.h>
#endif

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#include "talOS/protocol/frame.h"
#include "talOS/protocol/fuzz_corpus.h"

#ifdef TALOS_LIBFUZZER
#define TALOS_FRAME_FUZZ_CHECK(cond) \
  do {                               \
    if (!(cond)) std::abort();       \
  } while (false)
#else
#define TALOS_FRAME_FUZZ_CHECK(cond) \
  do {                               \
    if (!(cond)) {                   \
      ADD_FAILURE() << #cond;        \
      return 0;                      \
    }                                \
  } while (false)
#endif

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, std::size_t size) {
  using namespace talos::protocol;

  DecodedFrame sentinel{};
  sentinel.header.magic = 0x1122334455667788ull;
  sentinel.header.version = 0xBEEFu;
  sentinel.header.header_size = 0xCAFEu;
  sentinel.header.type = static_cast<FrameType>(0xDEADu);
  sentinel.header.flags = 0xF00Du;
  sentinel.header.sequence = 0x123456789ABCDEF0ull;
  sentinel.header.monotonic_time_us = 0x0FEDCBA987654321ull;
  sentinel.header.payload_size = 0xAABBCCDDu;
  sentinel.header.reserved = 0x11223344u;
  sentinel.payload_size = 999;

  DecodedFrame decoded = sentinel;
  const DecodeStatus status = DecodeFrame(data, size, &decoded);
  TALOS_FRAME_FUZZ_CHECK(DecodeStatusName(status) != nullptr);

  if (status == DecodeStatus::kOk) {
    TALOS_FRAME_FUZZ_CHECK(data != nullptr);
    TALOS_FRAME_FUZZ_CHECK(decoded.payload == data + kFrameHeaderSize);
    TALOS_FRAME_FUZZ_CHECK(decoded.payload_size <= kMaxPayloadSize);
    TALOS_FRAME_FUZZ_CHECK(decoded.payload_size == decoded.header.payload_size);
    TALOS_FRAME_FUZZ_CHECK(size == kFrameHeaderSize + decoded.payload_size);
    TALOS_FRAME_FUZZ_CHECK(decoded.header.magic == kFrameMagic);
    TALOS_FRAME_FUZZ_CHECK(decoded.header.version == kProtocolVersion);
    TALOS_FRAME_FUZZ_CHECK(decoded.header.header_size == kFrameHeaderSize);

    std::vector<uint8_t> encoded(kMaxFrameSize);
    const auto written = EncodeFrame(
        decoded.header.type, decoded.header.flags, decoded.header.sequence,
        decoded.header.monotonic_time_us, decoded.payload, decoded.payload_size,
        encoded.data(), encoded.size());
    TALOS_FRAME_FUZZ_CHECK(written == size);

    // EncodeFrameInPlace must produce identical bytes to EncodeFrame.
    std::vector<uint8_t> inplace(written);
    if (decoded.payload_size > 0) {
      std::memcpy(inplace.data() + kFrameHeaderSize, decoded.payload,
                  decoded.payload_size);
    }
    const auto inplace_written = EncodeFrameInPlace(
        decoded.header.type, decoded.header.flags, decoded.header.sequence,
        decoded.header.monotonic_time_us, decoded.payload_size, inplace.data(),
        inplace.size());
    TALOS_FRAME_FUZZ_CHECK(inplace_written == written);
    TALOS_FRAME_FUZZ_CHECK(
        std::memcmp(inplace.data(), encoded.data(), written) == 0);

    // One-byte-short output capacity must be rejected without writing OOB.
    std::vector<uint8_t> too_small(written - 1);
    TALOS_FRAME_FUZZ_CHECK(
        EncodeFrame(decoded.header.type, decoded.header.flags,
                    decoded.header.sequence, decoded.header.monotonic_time_us,
                    decoded.payload, decoded.payload_size, too_small.data(),
                    too_small.size()) == 0);

    DecodedFrame again{};
    TALOS_FRAME_FUZZ_CHECK(DecodeFrame(encoded.data(), written, &again) ==
                           DecodeStatus::kOk);
    TALOS_FRAME_FUZZ_CHECK(again.header.reserved == 0);

    std::vector<uint8_t> canonical(
        encoded.begin(),
        encoded.begin() + static_cast<std::ptrdiff_t>(written));
    std::vector<uint8_t> roundtrip(kMaxFrameSize);
    roundtrip.resize(EncodeFrame(
        again.header.type, again.header.flags, again.header.sequence,
        again.header.monotonic_time_us, again.payload, again.payload_size,
        roundtrip.data(), roundtrip.size()));
    TALOS_FRAME_FUZZ_CHECK(roundtrip == canonical);
  } else if (status == DecodeStatus::kUnsupportedVersion) {
    // Contract (frame.h): only out->header.version changes on version mismatch.
    TALOS_FRAME_FUZZ_CHECK(decoded.header.version != kProtocolVersion);
    decoded.header.version = sentinel.header.version;
    TALOS_FRAME_FUZZ_CHECK(std::memcmp(&decoded, &sentinel, sizeof(decoded)) ==
                           0);
  } else {
    TALOS_FRAME_FUZZ_CHECK(std::memcmp(&decoded, &sentinel, sizeof(decoded)) ==
                           0);
  }
  return 0;
}

#ifndef TALOS_LIBFUZZER
// REQ-CONTRACT-01
TEST(FrameFuzzTest, SeededMutationsAndRandomInputs) {
  using namespace talos::protocol;
  std::array<uint8_t, kMaxPayloadSize> payload{};
  for (std::size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<uint8_t>((i * 131u) ^ 0x5Au);
  }
  std::vector<std::vector<uint8_t>> seeds;
  for (const FrameType type :
       {FrameType::kHardwareState, FrameType::kHardwareCommand,
        FrameType::kHardwareConfig, FrameType::kHardwareConfigAck,
        FrameType::kDriverStation, FrameType::kClockProbe}) {
    for (const auto size :
         {std::size_t{0}, std::size_t{1}, std::size_t{17}, kMaxPayloadSize}) {
      std::vector<uint8_t> frame(kMaxFrameSize);
      frame.resize(EncodeFrame(type, 0xA55Au, 0x1122334455667788ull,
                               0xFEDCBA9876543210ull, payload.data(), size,
                               frame.data(), frame.size()));
      seeds.push_back(std::move(frame));
    }
  }
  // Seed with non-zero reserved bytes and unsupported version so mutations
  // explore both normalization and kUnsupportedVersion paths.
  {
    auto with_reserved = seeds.front();
    with_reserved[36] = 0x42;
    with_reserved[39] = 0xFF;
    seeds.push_back(std::move(with_reserved));

    auto bad_version = seeds.front();
    bad_version[8] = static_cast<uint8_t>((kProtocolVersion + 1) & 0xFF);
    bad_version[9] = static_cast<uint8_t>((kProtocolVersion + 1) >> 8);
    seeds.push_back(std::move(bad_version));
  }

  talos::protocol::testing::RunMutatedCampaign(
      seeds, LLVMFuzzerTestOneInput, /*random_iterations=*/8192,
      /*max_random_bytes=*/kMaxFrameSize + 64, /*rng_seed=*/0x4652414Du);
}

TEST(FrameFuzzTest, HeaderAndEncodeBoundaryMutations) {
  using namespace talos::protocol;

  // Explicit boundary checks discovered/targeted around header limits.
  DecodedFrame decoded{};
  uint8_t non_null = 0;
  EXPECT_EQ(DecodeFrame(nullptr, 0, &decoded), DecodeStatus::kTooSmall);
  EXPECT_EQ(DecodeFrame(nullptr, kFrameHeaderSize, &decoded),
            DecodeStatus::kTooSmall);
  EXPECT_EQ(DecodeFrame(&non_null, 0, &decoded), DecodeStatus::kTooSmall);
  EXPECT_EQ(DecodeFrame(&non_null, kFrameHeaderSize, nullptr),
            DecodeStatus::kTooSmall);

  // Header-preserved mutations: keep magic + version valid so every iteration
  // exercises header_size, payload_size (including 0xFFFFFFFF wraparound),
  // type, flags, reserved, and frame length checks.
  std::mt19937 rng(0x48445231u);
  for (int i = 0; i < 4096; ++i) {
    const std::size_t buf_size = rng() % (kMaxFrameSize + 64);
    std::vector<uint8_t> buf(buf_size);
    for (auto& b : buf) b = static_cast<uint8_t>(rng());
    if (buf.size() >= kFrameHeaderSize) {
      for (std::size_t b = 0; b < 8; ++b) {
        buf[b] = static_cast<uint8_t>(kFrameMagic >> (8 * b));
      }
      if ((i % 8) != 0) {
        buf[8] = static_cast<uint8_t>(kProtocolVersion & 0xFF);
        buf[9] = static_cast<uint8_t>(kProtocolVersion >> 8);
      }
      if ((i % 4) != 0) {
        buf[10] = static_cast<uint8_t>(kFrameHeaderSize & 0xFF);
        buf[11] = static_cast<uint8_t>(kFrameHeaderSize >> 8);
      }
      if ((i % 3) == 0 && buf.size() >= kFrameHeaderSize) {
        // Test exact matching payload_size, +1, or 0xFFFFFFFF wraparound.
        const uint32_t declared =
            (i % 6 == 0) ? std::numeric_limits<uint32_t>::max()
                         : static_cast<uint32_t>(buf.size() - kFrameHeaderSize);
        for (std::size_t b = 0; b < 4; ++b) {
          buf[32 + b] = static_cast<uint8_t>(declared >> (8 * b));
        }
      }
    }
    LLVMFuzzerTestOneInput(buf.empty() ? nullptr : buf.data(), buf.size());
  }

  // Randomized EncodeFrame / EncodeFrameInPlace contract checks.
  for (int i = 0; i < 2048; ++i) {
    const auto type = static_cast<FrameType>(static_cast<uint16_t>(rng()));
    const uint16_t flags = static_cast<uint16_t>(rng());
    const uint64_t seq = (uint64_t{rng()} << 32) | rng();
    const uint64_t time_us = (uint64_t{rng()} << 32) | rng();
    const std::size_t payload_sz = rng() % (kMaxPayloadSize + 32);
    const std::size_t out_cap = rng() % (kMaxFrameSize + 32);

    std::vector<uint8_t> payload(payload_sz);
    for (auto& b : payload) b = static_cast<uint8_t>(rng());
    std::vector<uint8_t> out(out_cap);

    const bool null_payload =
        (payload_sz == 0 && (i % 2 == 0)) || (i % 37 == 0);
    const bool null_out = (out_cap == 0 && (i % 2 == 0)) || (i % 41 == 0);

    const std::size_t written = EncodeFrame(
        type, flags, seq, time_us, null_payload ? nullptr : payload.data(),
        payload_sz, null_out ? nullptr : out.data(), out_cap);

    const bool should_fail = payload_sz > kMaxPayloadSize ||
                             out_cap < kFrameHeaderSize + payload_sz ||
                             null_out || (payload_sz > 0 && null_payload);
    if (should_fail) {
      EXPECT_EQ(written, 0u);
    } else {
      ASSERT_EQ(written, kFrameHeaderSize + payload_sz);
      LLVMFuzzerTestOneInput(out.data(), written);
    }
  }
}
#endif
