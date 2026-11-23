#ifndef TALOS_LIBFUZZER
#include <gtest/gtest.h>
#endif

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

#include "talOS/driver_station/driver_station.h"
#include "talOS/protocol/fuzz_corpus.h"

#ifdef TALOS_LIBFUZZER
#define TALOS_DS_FUZZ_CHECK(cond) \
  do {                            \
    if (!(cond)) std::abort();    \
  } while (false)
#else
#define TALOS_DS_FUZZ_CHECK(cond) \
  do {                            \
    if (!(cond)) {                \
      ADD_FAILURE() << #cond;     \
      return 0;                   \
    }                             \
  } while (false)
#endif

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, std::size_t size) {
  using namespace talos::driver_station;

  DriverStationData sentinel{};
  sentinel.sample_time_us = 0x1122334455667788ull;
  sentinel.mode_since_us = 0x8877665544332211ull;
  sentinel.boot_id = 0xCAFEBABEDEADBEEFull;
  sentinel.fms.flags = 0x12345678u;
  sentinel.fms.station = 9;
  sentinel.fms.match_number = 42;
  sentinel.joysticks[0].connected = true;
  sentinel.joysticks[0].axis_count = 2;

  DriverStationData decoded = sentinel;
  if (!Decode({data, size}, decoded)) {
    TALOS_DS_FUZZ_CHECK(decoded == sentinel);
    return 0;
  }

  // Verify decoded invariants before re-encoding.
  TALOS_DS_FUZZ_CHECK(std::isfinite(decoded.fms.match_time_s));
  TALOS_DS_FUZZ_CHECK(strnlen(decoded.fms.game_specific_message,
                              kMaxGameMessageLength) < kMaxGameMessageLength);
  TALOS_DS_FUZZ_CHECK(strnlen(decoded.fms.event_name, kMaxEventNameLength) <
                      kMaxEventNameLength);
  for (const auto& stick : decoded.joysticks) {
    if (!stick.connected) {
      TALOS_DS_FUZZ_CHECK(stick == JoystickData{});
      continue;
    }
    TALOS_DS_FUZZ_CHECK(stick.axis_count <= kMaxAxes);
    TALOS_DS_FUZZ_CHECK(stick.button_count <= kMaxButtons);
    TALOS_DS_FUZZ_CHECK(stick.pov_count <= kMaxPovs);
    for (uint8_t a = 0; a < stick.axis_count; ++a) {
      TALOS_DS_FUZZ_CHECK(std::isfinite(stick.axes[a]));
    }
    TALOS_DS_FUZZ_CHECK(strnlen(stick.name, kMaxNameLength) < kMaxNameLength);
  }

  std::vector<uint8_t> encoded(kMaxEncodedSize);
  const auto written = Encode(decoded, encoded);
  TALOS_DS_FUZZ_CHECK(written > 0 && written <= kMaxEncodedSize);

  // One-byte-short buffer must fail cleanly.
  std::vector<uint8_t> short_buf(written - 1);
  TALOS_DS_FUZZ_CHECK(Encode(decoded, short_buf) == 0);

  DriverStationData again{};
  TALOS_DS_FUZZ_CHECK(Decode({encoded.data(), written}, again));
  encoded.resize(written);
  std::vector<uint8_t> roundtrip(kMaxEncodedSize);
  roundtrip.resize(Encode(again, roundtrip));
  TALOS_DS_FUZZ_CHECK(roundtrip == encoded);
  return 0;
}

#ifndef TALOS_LIBFUZZER
TEST(DriverStationCodecFuzzTest, SeededMutationsAndRandomInputs) {
  using namespace talos::driver_station;
  std::vector<std::vector<uint8_t>> seeds;

  // 1. Minimal empty packet.
  {
    DriverStationData ds{};
    std::vector<uint8_t> encoded(kMaxEncodedSize);
    encoded.resize(Encode(ds, encoded));
    seeds.push_back(std::move(encoded));
  }

  // 2. Mixed connected joysticks, non-empty strings, -0.0f and extreme POVs.
  {
    DriverStationData ds{};
    ds.sample_time_us = 1000;
    ds.mode_since_us = 900;
    ds.boot_id = 0x0102030405060708ull;
    ds.fms.flags = kDsAttached | kFmsAttached | kEnabled | kAutonomous;
    ds.fms.alliance = Alliance::kRed;
    ds.fms.station = 2;
    ds.fms.match_type = MatchType::kElimination;
    ds.fms.match_number = 99;
    ds.fms.replay_number = 1;
    ds.fms.match_time_s = -0.0f;
    std::memcpy(ds.fms.game_specific_message, "LRL", 3);
    std::memcpy(ds.fms.event_name, "2026cmp", 7);
    for (std::size_t i = 0; i < kMaxJoysticks; ++i) {
      if ((i % 2) == 0) {
        auto& stick = ds.joysticks[i];
        stick.connected = true;
        stick.axis_count = static_cast<uint8_t>(i + 1);
        stick.button_count = 16;
        stick.pov_count = 2;
        stick.buttons = 0xA5A55A5Au;
        stick.axes[0] = -1.0f;
        stick.povs[0] = -1;
        stick.povs[1] = 270;
        std::memcpy(stick.name, "gamepad", 7);
      }
    }
    std::vector<uint8_t> encoded(kMaxEncodedSize);
    encoded.resize(Encode(ds, encoded));
    seeds.push_back(std::move(encoded));
  }

  // 3. Fully populated kMaxEncodedSize (678-byte) packet.
  {
    DriverStationData ds{};
    ds.sample_time_us = UINT64_MAX;
    ds.mode_since_us = UINT64_MAX - 1;
    ds.boot_id = 0xFEDCBA9876543210ull;
    ds.fms.flags = kDsAttached | kEnabled | kTeleop;
    ds.fms.alliance = Alliance::kBlue;
    ds.fms.station = 3;
    ds.fms.match_type = MatchType::kQualification;
    ds.fms.match_number = 1234;
    ds.fms.replay_number = 2;
    ds.fms.match_time_s = 135.5f;
    std::memset(ds.fms.game_specific_message, 'G', kMaxGameMessageLength - 1);
    std::memset(ds.fms.event_name, 'E', kMaxEventNameLength - 1);
    for (auto& stick : ds.joysticks) {
      stick.connected = true;
      stick.axis_count = kMaxAxes;
      stick.button_count = kMaxButtons;
      stick.pov_count = kMaxPovs;
      stick.buttons = 0xFFFFFFFFu;
      stick.axes.fill(0.5f);
      stick.povs.fill(180);
      std::memset(stick.name, 'N', kMaxNameLength - 1);
    }
    std::vector<uint8_t> encoded(kMaxEncodedSize);
    encoded.resize(Encode(ds, encoded));
    ASSERT_EQ(encoded.size(), kMaxEncodedSize);
    seeds.push_back(std::move(encoded));
  }

  talos::protocol::testing::RunMutatedCampaign(
      seeds, LLVMFuzzerTestOneInput, /*random_iterations=*/8192,
      /*max_random_bytes=*/kMaxEncodedSize + 64, /*rng_seed=*/0x44534344u);
}

TEST(DriverStationCodecFuzzTest, RandomizedStructEncodeAndWireBoundaries) {
  using namespace talos::driver_station;

  // Boundary checks around non-canonical wire values and float bit patterns.
  {
    std::vector<uint8_t> base(kMaxEncodedSize);
    base.resize(Encode(DriverStationData{}, base));
    // Non-1 connected byte (e.g. 2) is accepted as disconnected and
    // canonicalized.
    auto non_one = base;
    non_one[40] = 2;
    LLVMFuzzerTestOneInput(non_one.data(), non_one.size());

    // Embedded NUL in event_name is accepted and canonicalized by Encode.
    auto embedded_nul = base;
    embedded_nul[39] = 3;
    embedded_nul.insert(embedded_nul.begin() + 40, {'a', '\0', 'b'});
    LLVMFuzzerTestOneInput(embedded_nul.data(), embedded_nul.size());

    // Non-finite match_time_s bit patterns (qNaN, +Inf, -Inf) must be rejected.
    for (const uint32_t bad_bits :
         {0x7fc00000u, 0x7f800000u, 0xff800000u, 0x7f800001u}) {
      auto bad_float = base;
      for (std::size_t b = 0; b < 4; ++b) {
        bad_float[34 + b] = static_cast<uint8_t>(bad_bits >> (8 * b));
      }
      DriverStationData out{};
      EXPECT_FALSE(Decode(bad_float, out));
    }
  }

  // Randomized Encode fuzzing: non-finite floats, over-cap counts, unterminated
  // 32-byte strings, and arbitrary output capacities 0..kMaxEncodedSize + 32.
  constexpr std::array<uint32_t, 6> kFloatBits{0x00000000u, 0x80000000u,
                                               0x3f000000u, 0x7f800000u,
                                               0xff800000u, 0x7fc00001u};
  std::mt19937 rng(0x4453454Eu);
  for (int i = 0; i < 2048; ++i) {
    DriverStationData ds{};
    ds.sample_time_us = (uint64_t{rng()} << 32) | rng();
    ds.mode_since_us = (uint64_t{rng()} << 32) | rng();
    ds.boot_id = (uint64_t{rng()} << 32) | rng();
    ds.fms.flags = rng();
    ds.fms.alliance = static_cast<Alliance>(static_cast<uint8_t>(rng()));
    ds.fms.station = static_cast<uint8_t>(rng());
    ds.fms.match_type = static_cast<MatchType>(static_cast<uint8_t>(rng()));
    ds.fms.match_number = static_cast<uint16_t>(rng());
    ds.fms.replay_number = static_cast<uint8_t>(rng());
    ds.fms.match_time_s =
        std::bit_cast<float>(kFloatBits[rng() % kFloatBits.size()]);

    auto fill_str = [&](char* dst, std::size_t len) {
      const bool unterminated = (rng() % 4) == 0;
      const std::size_t stop = unterminated ? len : (rng() % len);
      for (std::size_t k = 0; k < stop; ++k) {
        dst[k] = static_cast<char>('A' + (rng() % 26));
      }
      if (!unterminated) dst[stop] = '\0';
    };
    fill_str(ds.fms.game_specific_message, kMaxGameMessageLength);
    fill_str(ds.fms.event_name, kMaxEventNameLength);

    for (auto& stick : ds.joysticks) {
      stick.connected = (rng() % 2) == 1;
      stick.axis_count = static_cast<uint8_t>(rng() % 32);
      stick.button_count = static_cast<uint8_t>(rng() % 64);
      stick.pov_count = static_cast<uint8_t>(rng() % 16);
      stick.buttons = rng();
      for (auto& a : stick.axes) {
        a = std::bit_cast<float>((rng() % 4 == 0)
                                     ? kFloatBits[rng() % kFloatBits.size()]
                                     : static_cast<uint32_t>(rng()));
      }
      for (auto& p : stick.povs) p = static_cast<int16_t>(rng());
      fill_str(stick.name, kMaxNameLength);
    }

    const std::size_t cap = rng() % (kMaxEncodedSize + 32);
    std::vector<uint8_t> out(cap);
    const std::size_t written = Encode(ds, out);
    EXPECT_LE(written, cap);
    EXPECT_LE(written, kMaxEncodedSize);
    if (written > 0) {
      LLVMFuzzerTestOneInput(out.data(), written);
    }
  }
}
#endif
