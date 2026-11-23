#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "talOS/vision/apriltag/cpu_apriltag_backend.h"

namespace talos::vision::apriltag {
namespace {

std::string FixturePath() {
  const char* root = std::getenv("TEST_SRCDIR");
  const char* workspace = std::getenv("TEST_WORKSPACE");
  return std::string(root ? root : ".") + "/" +
         (workspace ? workspace : "_main") +
         "/talOS/vision/testdata/33369213973_9d9bb4cc96_c.jpg";
}

std::string TempDir() {
  const char* tmp = std::getenv("TEST_TMPDIR");
  return tmp != nullptr ? std::string(tmp) : std::string("/tmp");
}

std::vector<uint8_t> ReadBytes(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  EXPECT_TRUE(in);
  return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                              std::istreambuf_iterator<char>());
}

void WriteBytes(const std::string& path, const std::vector<uint8_t>& bytes) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(out);
  if (!bytes.empty()) {
    out.write(reinterpret_cast<const char*>(bytes.data()),
              static_cast<std::streamsize>(bytes.size()));
  }
  out.close();
  ASSERT_TRUE(out);
}

CpuAprilTagBackend MakeBackend(float quad_decimate = 2.0f) {
  DetectorOptions options;
  options.threads = 1;
  options.quad_decimate = quad_decimate;
  return CpuAprilTagBackend(options);
}

// Corrupt entropy-coded data must be refused or decoded within bounds:
// a Huffman run past coefficient 63 used to index ZZ, qtab and the block
// out of range. Deterministic byte edits after the start-of-scan marker;
// the outcome (empty or not) is unspecified, only memory safety is checked,
// which the asan config enforces.
TEST(JpegLoader, CorruptScanDataStaysInBounds) {
  const std::vector<uint8_t> original = ReadBytes(FixturePath());
  ASSERT_FALSE(original.empty());
  std::size_t scan = 0;
  for (std::size_t i = 0; i + 1 < original.size(); ++i) {
    if (original[i] == 0xFF && original[i + 1] == 0xDA) {
      scan = i + 2;
      break;
    }
  }
  ASSERT_GT(scan, 0u);
  ASSERT_LT(scan + 64, original.size());

  const std::string path = TempDir() + "/corrupt.jpg";
  CpuAprilTagBackend backend = MakeBackend();

  uint32_t state = 12345u;  // LCG: reproducible edits
  const auto next = [&state] {
    state = state * 1664525u + 1013904223u;
    return state >> 8;
  };
  const std::size_t span = original.size() - scan - 2;  // keep the EOI marker
  for (int variant = 0; variant < 64; ++variant) {
    std::vector<uint8_t> bytes = original;
    for (int edit = 0; edit < 8; ++edit) {
      bytes[scan + next() % span] = static_cast<uint8_t>(next());
    }
    WriteBytes(path, bytes);
    (void)backend.DetectImageFile(path);
  }
}

// Empty files and frame headers without a Start-of-Scan used to return
// PJPEG_OKAY with ncomponents == 0 or components[i].data == NULL, aborting
// or dereferencing NULL in pjpeg_to_u8_baseline.
TEST(JpegLoader, RejectsEmptyFileAndMissingScan) {
  CpuAprilTagBackend backend = MakeBackend();

  const std::string empty_path = TempDir() + "/empty.jpg";
  WriteBytes(empty_path, {});
  EXPECT_TRUE(backend.DetectImageFile(empty_path).empty());

  // SOI + SOF0 (16x16, 1 component) + EOI, with no SOS segment.
  const std::string no_sos_path = TempDir() + "/no_sos.jpg";
  const std::vector<uint8_t> no_sos = {
      0xFF, 0xD8,  // SOI
      0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0x10, 0x00, 0x10, 0x01, 0x01, 0x11,
      0x00,        // SOF0
      0xFF, 0xD9,  // EOI
  };
  WriteBytes(no_sos_path, no_sos);
  EXPECT_TRUE(backend.DetectImageFile(no_sos_path).empty());
}

// APP/COM and unknown markers with length < 2 used to pass a negative count
// to bd_discard_bytes, rewinding bd.inpos instead of rejecting the marker.
TEST(JpegLoader, RejectsMarkerWithLengthLessThanTwo) {
  const std::vector<uint8_t> original = ReadBytes(FixturePath());
  ASSERT_GE(original.size(), 4u);
  ASSERT_EQ(original[0], 0xFF);
  ASSERT_EQ(original[1], 0xD8);

  CpuAprilTagBackend backend = MakeBackend();
  ASSERT_FALSE(backend.DetectImageFile(FixturePath()).empty());

  // Standalone malformed APP0 marker must return cleanly.
  const std::string standalone_path = TempDir() + "/short_app0_only.jpg";
  WriteBytes(standalone_path, {0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x00, 0xFF, 0xD9});
  EXPECT_TRUE(backend.DetectImageFile(standalone_path).empty());

  // Injected after SOI of a valid fixture: APP0 (0xE0) and unknown marker
  // (0xE9) with length 0 or 1 must fail decode rather than rewinding and
  // continuing into the fixture.
  const std::array<std::array<uint8_t, 4>, 2> bad_markers = {{
      {0xFF, 0xE0, 0x00, 0x00},
      {0xFF, 0xE9, 0x00, 0x01},
  }};
  for (std::size_t i = 0; i < bad_markers.size(); ++i) {
    std::vector<uint8_t> mutated;
    mutated.reserve(original.size() + 4);
    mutated.push_back(0xFF);
    mutated.push_back(0xD8);
    mutated.insert(mutated.end(), bad_markers[i].begin(), bad_markers[i].end());
    mutated.insert(mutated.end(), original.begin() + 2, original.end());
    const std::string path =
        TempDir() + "/short_marker_" + std::to_string(i) + ".jpg";
    WriteBytes(path, mutated);
    EXPECT_TRUE(backend.DetectImageFile(path).empty());
  }
}

// DHT (0xC4) segments shorter than 2 + 17 + sum(L) used to underflow uint16_t
// length to ~65535 and keep consuming subsequent bytes as Huffman tables.
TEST(JpegLoader, RejectsTruncatedDhtWithoutUnderflow) {
  const std::vector<uint8_t> original = ReadBytes(FixturePath());
  ASSERT_GE(original.size(), 4u);

  CpuAprilTagBackend backend = MakeBackend();
  ASSERT_FALSE(backend.DetectImageFile(FixturePath()).empty());

  // Truncated DHT on its own.
  const std::string short_dht_path = TempDir() + "/short_dht.jpg";
  WriteBytes(short_dht_path,
             {0xFF, 0xD8, 0xFF, 0xC4, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00});
  EXPECT_TRUE(backend.DetectImageFile(short_dht_path).empty());

  // Crafted DHT where length = 19 (remaining = 17) declares 1 symbol
  // (needs 18 bytes): uint16_t underflows to 65535 = 3855 * 17, which
  // previously consumed 65535 zero bytes as empty tables and decoded the
  // trailing fixture.
  std::vector<uint8_t> wrapped;
  wrapped.reserve(2 + 22 + 65535 + original.size());
  wrapped.push_back(0xFF);
  wrapped.push_back(0xD8);
  wrapped.insert(
      wrapped.end(),
      {0xFF, 0xC4, 0x00, 0x13,  // DHT, length = 19 -> 17 payload
       0x00,                    // Tc=0, Th=0
       0,    0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,  // sum(L)=1
       0x00});  // 1 symbol value
  wrapped.insert(wrapped.end(), 65535u, 0x00);
  wrapped.insert(wrapped.end(), original.begin() + 2, original.end());

  const std::string wrapped_path = TempDir() + "/dht_underflow.jpg";
  WriteBytes(wrapped_path, wrapped);
  EXPECT_TRUE(backend.DetectImageFile(wrapped_path).empty());
}

// Passing a directory with a .jpg suffix used to let fopen succeed, get
// LONG_MAX or -1 from ftell, and call malloc/fread with that size.
TEST(JpegLoader, RejectsDirectoryPath) {
  const std::string dir_path = TempDir() + "/not_a_file.jpg";
  std::filesystem::create_directories(dir_path);

  CpuAprilTagBackend backend = MakeBackend();
  EXPECT_TRUE(backend.DetectImageFile(dir_path).empty());
}

// Restart markers (RST0..RST7) must reset DC predictors for every component in
// the scan, not just index 0. Put the luminance component second in SOS (ns=2)
// with DRI=1 over a 12x12-MCU tag36h11 id 0 image: without resetting dcpred[1],
// DC accumulates across MCUs and washes out the tag.
TEST(JpegLoader, RestartMarkerResetsAllComponentDcPredictors) {
  // tag36h11 id 0 bit pattern and coordinates (from tag36h11.c).
  constexpr uint64_t kTag0Code = 0x0000000d7e00984bULL;
  constexpr std::array<int, 36> kBitX = {1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6,
                                         6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5,
                                         4, 3, 4, 1, 1, 1, 1, 1, 2, 2, 2, 3};
  constexpr std::array<int, 36> kBitY = {1, 1, 1, 1, 1, 2, 2, 2, 3, 1, 2, 3,
                                         4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5,
                                         5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4};

  // 12x12 grid of 8x8 MCUs (96x96 px): 2-cell white quiet zone around the
  // 8x8 black-bordered tag (inner 6x6 data at grid coords [3..8]).
  std::array<std::array<bool, 12>, 12> white{};
  for (auto& row : white) {
    row.fill(true);
  }
  for (int y = 2; y < 10; ++y) {
    for (int x = 2; x < 10; ++x) {
      white[y][x] = false;
    }
  }
  for (int i = 0; i < 36; ++i) {
    const bool is_white = ((kTag0Code >> (35 - i)) & 1ULL) != 0;
    white[2 + kBitY[i]][2 + kBitX[i]] = is_white;
  }

  std::vector<uint8_t> jpeg;
  jpeg.reserve(1024);
  jpeg.push_back(0xFF);
  jpeg.push_back(0xD8);  // SOI
  // DQT table 0: all entries = 64 so DC +/-16 maps to white (255) / black (0).
  jpeg.insert(jpeg.end(), {0xFF, 0xDB, 0x00, 0x43, 0x00});
  jpeg.insert(jpeg.end(), 64u, 64u);
  // SOF0: 96x96, 2 components (id=1 is component[0]/Y, id=2 is dummy).
  jpeg.insert(jpeg.end(), {0xFF, 0xC0, 0x00, 0x0E, 0x08, 0x00, 0x60, 0x00, 0x60,
                           0x02, 0x01, 0x11, 0x00, 0x02, 0x11, 0x00});
  // DHT DC table 0: '0' -> ssss=0, '10' -> ssss=5.
  jpeg.insert(jpeg.end(),
              {0xFF, 0xC4, 0x00, 0x15, 0x00, 1, 1, 0, 0, 0,    0,   0,
               0,    0,    0,    0,    0,    0, 0, 0, 0, 0x00, 0x05});
  // DHT AC table 0: '0' -> EOB (0x00).
  jpeg.insert(jpeg.end(), {0xFF, 0xC4, 0x00, 0x14, 0x10, 1, 0, 0, 0, 0, 0,
                           0,    0,    0,    0,    0,    0, 0, 0, 0, 0, 0x00});
  // DRI: reset_interval = 1 MCU.
  jpeg.insert(jpeg.end(), {0xFF, 0xDD, 0x00, 0x04, 0x00, 0x01});
  // SOS: ns=2, listing dummy (id=2) at nsidx=0 and Y (id=1) at nsidx=1.
  jpeg.insert(jpeg.end(), {0xFF, 0xDA, 0x00, 0x0A, 0x02, 0x02, 0x00, 0x01, 0x00,
                           0x00, 0x3F, 0x00});

  int mcu = 0;
  for (int y = 0; y < 12; ++y) {
    for (int x = 0; x < 12; ++x) {
      if (mcu > 0) {
        jpeg.push_back(0xFF);
        jpeg.push_back(static_cast<uint8_t>(0xD0 + ((mcu - 1) & 7)));
      }
      // nsidx=0 (dummy): DC=0 ('0'), AC=EOB ('0').
      // nsidx=1 (Y): DC=+16 ('10' + 10000) or -16 ('10' + 01111), AC=EOB ('0'),
      // plus 6 trailing 1-bits to byte-align.
      if (white[y][x]) {
        jpeg.push_back(0x28);
        jpeg.push_back(0x3F);
      } else {
        jpeg.push_back(0x27);
        jpeg.push_back(0xBF);
      }
      ++mcu;
    }
  }
  jpeg.push_back(0xFF);
  jpeg.push_back(0xD9);  // EOI

  const std::string path = TempDir() + "/rst_dcpred.jpg";
  WriteBytes(path, jpeg);

  CpuAprilTagBackend backend = MakeBackend(/*quad_decimate=*/1.0f);
  const std::vector<Detection> detections = backend.DetectImageFile(path);
  ASSERT_EQ(detections.size(), 1u);
  EXPECT_EQ(detections[0].id, 0);
}

}  // namespace
}  // namespace talos::vision::apriltag
