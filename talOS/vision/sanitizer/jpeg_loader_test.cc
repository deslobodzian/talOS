#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

#include "talOS/vision/apriltag/cpu_apriltag_backend.h"

namespace talos::vision::apriltag {
namespace {

// This offline API is not called by live camera detection. Retain a direct
// decoder probe so the remaining pjpeg UBSan exemption can be reproduced by
// appending
// --per_file_copt='external/.*apriltag.*/common/pjpeg.*[.]c@-fsanitize=shift'.
TEST(JpegLoader, PhotographicFixture) {
  const char* root = std::getenv("TEST_SRCDIR");
  const char* workspace = std::getenv("TEST_WORKSPACE");
  const std::string path =
      std::string(root ? root : ".") + "/" + (workspace ? workspace : "_main") +
      "/talOS/vision/testdata/33369213973_9d9bb4cc96_c.jpg";
  DetectorOptions options;
  options.threads = 2;
  options.quad_decimate = 2.0f;
  CpuAprilTagBackend backend(options);
  EXPECT_FALSE(backend.DetectImageFile(path).empty());
}

}  // namespace
}  // namespace talos::vision::apriltag
