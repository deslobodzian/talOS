#include <gtest/gtest.h>

#include <cstdlib>
#include <vector>

#include "talOS/driver_station/driver_station.h"
#include "talOS/protocol/fuzz_corpus.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, std::size_t size) {
  using namespace talos::driver_station;
  DriverStationData decoded;
  if (Decode({data, size}, decoded)) {
    std::vector<uint8_t> encoded(kMaxEncodedSize);
    const auto written = Encode(decoded, encoded);
    DriverStationData again;
    if (written == 0 || !Decode({encoded.data(), written}, again)) std::abort();
    encoded.resize(written);
    std::vector<uint8_t> roundtrip(kMaxEncodedSize);
    roundtrip.resize(Encode(again, roundtrip));
    EXPECT_EQ(roundtrip, encoded);
  }
  return 0;
}

#ifndef TALOS_LIBFUZZER
TEST(DriverStationCodecFuzzTest, BoundedMutations) {
  using namespace talos::driver_station;
  std::vector<std::vector<uint8_t>> seeds;
  DriverStationData ds;
  for (int populated = 0; populated < 2; ++populated) {
    if (populated) {
      ds.fms.flags = kDsAttached | kEnabled | kTeleop;
      for (auto& joystick : ds.joysticks) {
        joystick.connected = true;
        joystick.axis_count = kMaxAxes;
        joystick.pov_count = kMaxPovs;
        joystick.button_count = kMaxButtons;
        joystick.axes.fill(0.5f);
      }
    }
    std::vector<uint8_t> encoded(kMaxEncodedSize);
    encoded.resize(Encode(ds, encoded));
    seeds.push_back(encoded);
  }
  talos::protocol::testing::RunCorpus(seeds, LLVMFuzzerTestOneInput);
}
#endif
