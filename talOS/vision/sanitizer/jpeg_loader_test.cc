#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
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

// The offline DetectImageFile API decodes JPEGs with AprilTag's pjpeg; live
// camera detection passes raw buffers and never reaches it. Under the asan
// config (UBSan included, pjpeg no longer exempt) this decodes a photograph.
TEST(JpegLoader, PhotographicFixture) {
  DetectorOptions options;
  options.threads = 2;
  options.quad_decimate = 2.0f;
  CpuAprilTagBackend backend(options);
  EXPECT_FALSE(backend.DetectImageFile(FixturePath()).empty());
}

// Corrupt entropy-coded data must be refused or decoded within bounds:
// a Huffman run past coefficient 63 used to index ZZ, qtab and the block
// out of range. Deterministic byte edits after the start-of-scan marker;
// the outcome (empty or not) is unspecified, only memory safety is checked,
// which the asan config enforces.
TEST(JpegLoader, CorruptScanDataStaysInBounds) {
  std::ifstream in(FixturePath(), std::ios::binary);
  ASSERT_TRUE(in);
  const std::vector<uint8_t> original((std::istreambuf_iterator<char>(in)),
                                      std::istreambuf_iterator<char>());
  std::size_t scan = 0;
  for (std::size_t i = 0; i + 1 < original.size(); ++i) {
    if (original[i] == 0xFF && original[i + 1] == 0xDA) {
      scan = i + 2;
      break;
    }
  }
  ASSERT_GT(scan, 0u);
  ASSERT_LT(scan + 64, original.size());

  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string path = std::string(tmp ? tmp : "/tmp") + "/corrupt.jpg";
  DetectorOptions options;
  options.threads = 1;
  options.quad_decimate = 2.0f;
  CpuAprilTagBackend backend(options);

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
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(bytes.data()),
              static_cast<std::streamsize>(bytes.size()));
    out.close();
    (void)backend.DetectImageFile(path);
  }
}

}  // namespace
}  // namespace talos::vision::apriltag
