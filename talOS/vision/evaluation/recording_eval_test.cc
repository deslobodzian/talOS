#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace talos::vision::evaluation {
namespace {

namespace fs = std::filesystem;

std::string FindRunfile(const std::string& rel_path) {
  if (const char* srcdir = std::getenv("TEST_SRCDIR")) {
    const char* ws = std::getenv("TEST_WORKSPACE");
    const std::string workspace = ws != nullptr ? ws : "_main";
    const fs::path p = fs::path(srcdir) / workspace / rel_path;
    if (fs::exists(p)) return p.string();
    const fs::path p2 = fs::path(srcdir) / rel_path;
    if (fs::exists(p2)) return p2.string();
  }
  return rel_path;
}

struct ProcessResult {
  int exit_code = -1;
  bool signaled = false;
  std::string output;
};

ProcessResult RunBinary(const std::string& bin,
                        const std::vector<std::string>& args) {
  int pipefd[2];
  if (::pipe(pipefd) != 0) return {};
  const pid_t pid = ::fork();
  if (pid == 0) {
    ::close(pipefd[0]);
    ::dup2(pipefd[1], STDOUT_FILENO);
    ::dup2(pipefd[1], STDERR_FILENO);
    ::close(pipefd[1]);
    std::vector<char*> argv;
    argv.reserve(args.size() + 2);
    argv.push_back(const_cast<char*>(bin.c_str()));
    for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);
    ::execv(bin.c_str(), argv.data());
    ::_exit(127);
  }
  ::close(pipefd[1]);
  ProcessResult res;
  char buf[4096];
  ssize_t n = 0;
  while ((n = ::read(pipefd[0], buf, sizeof(buf))) > 0) {
    res.output.append(buf, static_cast<std::size_t>(n));
  }
  ::close(pipefd[0]);
  int status = 0;
  ::waitpid(pid, &status, 0);
  if (WIFEXITED(status)) {
    res.exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    res.signaled = true;
    res.exit_code = 128 + WTERMSIG(status);
  }
  return res;
}

class RecordingEvalFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    const char* tmpdir = std::getenv("TEST_TMPDIR");
    base_dir_ = fs::path(tmpdir != nullptr ? tmpdir : "/tmp") /
                ("recording_eval_test_" +
                 std::to_string(static_cast<long>(::getpid())));
    frames_dir_ = base_dir_ / "frames";
    fs::remove_all(base_dir_);
    fs::create_directories(frames_dir_);

    const fs::path src_png =
        FindRunfile("talOS/vision/testdata/scene_single_tag0.png");
    ASSERT_TRUE(fs::exists(src_png)) << "Missing: " << src_png;
    fs::copy_file(src_png, frames_dir_ / "frame_0.png");
    fs::copy_file(src_png, frames_dir_ / "frame_1.png");

    std::ofstream ts(frames_dir_ / "timestamps.csv");
    ts << "filename,pts_ns\n"
       << "frame_0.png,1000000000\n"
       << "frame_1.png,1010000000\n";

    const char* bin_env = std::getenv("RECORDING_EVAL_BIN");
    ASSERT_NE(bin_env, nullptr);
    bin_ = FindRunfile(bin_env);
    ASSERT_TRUE(fs::exists(bin_)) << "Missing binary: " << bin_;
  }

  void TearDown() override { fs::remove_all(base_dir_); }

  ProcessResult RunWithMocap(const std::string& csv_content) {
    const fs::path mocap_path = base_dir_ / "mocap.csv";
    {
      std::ofstream out(mocap_path);
      out << csv_content;
    }
    return RunBinary(
        bin_, {"--frames", frames_dir_.string(), "--intrinsics",
               "500,500,320,240", "--aprilgrid", "1,1,0.16,0.3", "--decimate",
               "1", "--threads", "1", "--mocap", mocap_path.string(),
               "--t-cam-imu", "1,0,0,0,0,1,0,0,0,0,1,0"});
  }

  fs::path base_dir_;
  fs::path frames_dir_;
  std::string bin_;
};

TEST_F(RecordingEvalFixture, MocapCsvRejectsNonNumericFieldsWithoutCrashing) {
  const std::string csv =
      "timestamp,px,py,pz,qw,qx,qy,qz\n"
      "not_a_number,0,0,0,1,0,0,0\n"
      "995000000,bad_x,0,0,1,0,0,0\n"
      "995000000,0,0,0,1,0,0,0\n"
      "1005000000,0,0,0,1,0,0,0\n"
      "1015000000,0,0,0,1,0,0,0\n";
  const auto res = RunWithMocap(csv);
  EXPECT_FALSE(res.signaled) << res.output;
  EXPECT_EQ(res.exit_code, 0) << res.output;
  const std::size_t gt_pos = res.output.find("ground truth (mocap");
  ASSERT_NE(gt_pos, std::string::npos) << res.output;
  EXPECT_EQ(res.output.substr(gt_pos).find("nan"), std::string::npos)
      << res.output;
}

TEST_F(RecordingEvalFixture, MocapCsvRejectsZeroAndNonFiniteQuaternions) {
  const std::string csv =
      "995000000,0,0,0,0,0,0,0\n"
      "1005000000,0,0,0,0,0,0,0\n"
      "1015000000,0,0,0,nan,0,0,0\n";
  const auto res = RunWithMocap(csv);
  EXPECT_FALSE(res.signaled) << res.output;
  EXPECT_EQ(res.exit_code, 0) << res.output;
  const std::size_t gt_pos =
      res.output.find("ground truth: fewer than 2 frames with mocap");
  ASSERT_NE(gt_pos, std::string::npos) << res.output;
  EXPECT_EQ(res.output.substr(gt_pos).find("nan"), std::string::npos)
      << res.output;
}

TEST_F(RecordingEvalFixture, RejectsHostileCliAndCalibrationInputsCleanly) {
  for (const auto& extra_args : std::vector<std::vector<std::string>>{
           {"--frames", frames_dir_.string(), "--intrinsics", "0,500,320,240",
            "--aprilgrid", "1,1,0.16,0.3"},
           {"--frames", frames_dir_.string(), "--intrinsics", "500,500,320,240",
            "--aprilgrid", "1e20,1,0.16,0.3"},
           {"--frames", frames_dir_.string(), "--intrinsics", "500,500,320,240",
            "--aprilgrid", "1,1,0.16,0.3", "--threads", "1e20"},
           {"--frames", frames_dir_.string(), "--intrinsics", "500,500,320,240",
            "--aprilgrid", "1,1,0.16,0.3", "--max-frames", "-1"},
       }) {
    const auto res = RunBinary(bin_, extra_args);
    EXPECT_FALSE(res.signaled) << res.output;
    EXPECT_NE(res.exit_code, 0) << res.output;
  }

  const fs::path bad_pv = base_dir_ / "bad_pv.json";
  {
    std::ofstream out(bad_pv);
    out << R"({"cameraIntrinsics":{"data":["x",0,320,0,500,240,0,0,1]},"distCoeffs":{"data":[0,0,0,0,0]}})";
  }
  const auto pv_res = RunBinary(
      bin_, {"--frames", frames_dir_.string(), "--photonvision-calibration",
             bad_pv.string(), "--aprilgrid", "1,1,0.16,0.3"});
  EXPECT_FALSE(pv_res.signaled) << pv_res.output;
  EXPECT_NE(pv_res.exit_code, 0) << pv_res.output;
}

TEST_F(RecordingEvalFixture,
       MocapCsvParsesSubnormalDoublesExactStartAndUnsortedRows) {
  // Unsorted timestamps, exact match at times_.front() (1000000000), and
  // subnormal double (1e-320) that sets errno=ERANGE on glibc strtod.
  const std::string csv =
      "1010000000,1e-320,0,0,1,0,0,0\n"
      "1000000000,1e-320,0,0,1,0,0,0\n";
  const auto res = RunWithMocap(csv);
  EXPECT_FALSE(res.signaled) << res.output;
  EXPECT_EQ(res.exit_code, 0) << res.output;
  EXPECT_NE(res.output.find("ground truth (mocap, layout pose held fixed in "
                            "the world): 2 frames"),
            std::string::npos)
      << res.output;
}

TEST_F(RecordingEvalFixture,
       PhotonvisionCalibrationClearsPriorOpencvDistortion) {
  const fs::path pv_path = base_dir_ / "pv8.json";
  {
    std::ofstream out(pv_path);
    out << R"({"cameraIntrinsics":{"data":[500,0,320,0,500,240,0,0,1]},)"
        << R"("distCoeffs":{"data":[0,0,0,0,0,0,0,0]}})";
  }
  const auto res =
      RunBinary(bin_, {"--frames", frames_dir_.string(), "--opencv-distortion",
                       "0,0,0,0,0", "--photonvision-calibration",
                       pv_path.string(), "--aprilgrid", "1,1,0.16,0.3",
                       "--decimate", "1", "--threads", "1"});
  EXPECT_FALSE(res.signaled) << res.output;
  EXPECT_EQ(res.exit_code, 0) << res.output;
}

TEST_F(RecordingEvalFixture, ProjectBehindCameraDoesNotPolluteCornerMetrics) {
  constexpr int kW = 640;
  constexpr int kH = 480;
  std::vector<unsigned char> img(static_cast<std::size_t>(kW) * kH, 200);
  auto paint_tag = [&](const char* const rows[10], int cx, int cy, int cell) {
    const int x0 = cx - 5 * cell;
    const int y0 = cy - 5 * cell;
    for (int r = 0; r < 10; ++r) {
      for (int c = 0; c < 10; ++c) {
        const unsigned char val = rows[r][c] == '#' ? 255 : 0;
        for (int dy = 0; dy < cell; ++dy) {
          for (int dx = 0; dx < cell; ++dx) {
            img[static_cast<std::size_t>(y0 + r * cell + dy) * kW +
                static_cast<std::size_t>(x0 + c * cell + dx)] = val;
          }
        }
      }
    }
  };
  static const char* const kTag0[10] = {
      "##########", "#........#", "#.##.#.#.#", "#..###.#.#", "#..##....#",
      "#.#.#....#", "#..#.##..#", "#....#...#", "#........#", "##########",
  };
  static const char* const kTag1[10] = {
      "##########", "#........#", "#.##.##..#", "#..#.###.#", "#.####...#",
      "#..##....#", "#.#.##.#.#", "#...#..#.#", "#........#", "##########",
  };
  static const char* const kTag2[10] = {
      "##########", "#........#", "#.##.###.#", "#..#..#..#", "#.#......#",
      "#...#..#.#", "#....#...#", "#...###..#", "#........#", "##########",
  };

  // Tags 0 and 1 have 128 px black squares -> range ~3.91 m (<= 8 m, used by
  // MultiTagPoseSolver). Tag 2 has a 48 px black square -> range ~10.4 m
  // (> 8 m, excluded from solver but still valid and in layout).
  paint_tag(kTag0, 180, 200, 16);
  paint_tag(kTag1, 460, 200, 16);
  paint_tag(kTag2, 320, 410, 6);

  const fs::path multi_dir = base_dir_ / "multi_frames";
  fs::create_directories(multi_dir);
  {
    // Uncompressed 24-bit BMP (built into OpenCV with no external codec).
    constexpr std::uint32_t kHeaderSize = 54;
    constexpr std::uint32_t kPixelBytes = kW * kH * 3;
    constexpr std::uint32_t kFileSize = kHeaderSize + kPixelBytes;
    unsigned char hdr[kHeaderSize] = {};
    hdr[0] = 'B';
    hdr[1] = 'M';
    auto put_u32 = [&](std::size_t off, std::uint32_t v) {
      hdr[off] = static_cast<unsigned char>(v & 0xffu);
      hdr[off + 1] = static_cast<unsigned char>((v >> 8) & 0xffu);
      hdr[off + 2] = static_cast<unsigned char>((v >> 16) & 0xffu);
      hdr[off + 3] = static_cast<unsigned char>((v >> 24) & 0xffu);
    };
    put_u32(2, kFileSize);
    put_u32(10, kHeaderSize);
    put_u32(14, 40);
    put_u32(18, static_cast<std::uint32_t>(kW));
    put_u32(22, static_cast<std::uint32_t>(kH));
    hdr[26] = 1;
    hdr[28] = 24;
    put_u32(34, kPixelBytes);
    std::ofstream bmp(multi_dir / "frame_0.bmp", std::ios::binary);
    bmp.write(reinterpret_cast<const char*>(hdr), kHeaderSize);
    std::vector<unsigned char> row(static_cast<std::size_t>(kW) * 3);
    for (int y = kH - 1; y >= 0; --y) {
      for (int x = 0; x < kW; ++x) {
        const unsigned char v =
            img[static_cast<std::size_t>(y) * kW + static_cast<std::size_t>(x)];
        row[static_cast<std::size_t>(x) * 3] = v;
        row[static_cast<std::size_t>(x) * 3 + 1] = v;
        row[static_cast<std::size_t>(x) * 3 + 2] = v;
      }
      bmp.write(reinterpret_cast<const char*>(row.data()),
                static_cast<std::streamsize>(row.size()));
    }
  }
  {
    std::ofstream ts(multi_dir / "timestamps.csv");
    ts << "filename,pts_ns\nframe_0.bmp,1000000000\n";
  }

  // Place tags 0 and 1 in front of the camera (at x=6.0 m) matching their
  // image geometry, and place tag 2 behind the camera (at x=0.5 m) so
  // Project() returns early on p.z() <= 0.05.
  const fs::path layout_path = base_dir_ / "layout.json";
  {
    std::ofstream out(layout_path);
    out << R"({
      "field": {"length": 16.0, "width": 8.0},
      "tags": [
        {"ID": 0, "pose": {"translation": {"x": 6.0, "y": 4.546875, "z": 0.65625},
          "rotation": {"quaternion": {"W": 0.0, "X": 0.0, "Y": 0.0, "Z": 1.0}}}},
        {"ID": 1, "pose": {"translation": {"x": 6.0, "y": 3.453125, "z": 0.65625},
          "rotation": {"quaternion": {"W": 0.0, "X": 0.0, "Y": 0.0, "Z": 1.0}}}},
        {"ID": 2, "pose": {"translation": {"x": 0.5, "y": 4.0, "z": 0.5},
          "rotation": {"quaternion": {"W": 0.0, "X": 0.0, "Y": 0.0, "Z": 1.0}}}}
      ]
    })";
  }

  const auto res =
      RunBinary(bin_, {"--frames", multi_dir.string(), "--intrinsics",
                       "1000,1000,320,240", "--layout", layout_path.string(),
                       "--tag-size-m", "0.5", "--mount", "0,0,0.5,0,0,0",
                       "--decimate", "1", "--threads", "1"});
  EXPECT_FALSE(res.signaled) << res.output;
  EXPECT_EQ(res.exit_code, 0) << res.output;
  EXPECT_NE(res.output.find("multi-tag 1"), std::string::npos) << res.output;
  // Only tags 0 and 1 project in front of the camera; tag 2 must be skipped
  // instead of reading uninitialized Projected::corners.
  EXPECT_NE(
      res.output.find(
          "detected corner vs multi-tag reprojection, worst corner px: n=2 "),
      std::string::npos)
      << res.output;
  EXPECT_NE(res.output.find("; misplaced (> 1/4 side) 0"), std::string::npos)
      << res.output;
}

TEST_F(RecordingEvalFixture, ExpectedInViewCountsTagIdsBeyond999) {
  constexpr int kW = 640;
  constexpr int kH = 480;
  std::vector<unsigned char> img(static_cast<std::size_t>(kW) * kH, 200);
  auto paint_tag = [&](const char* const rows[10], int cx, int cy, int cell) {
    const int x0 = cx - 5 * cell;
    const int y0 = cy - 5 * cell;
    for (int r = 0; r < 10; ++r) {
      for (int c = 0; c < 10; ++c) {
        const unsigned char val = rows[r][c] == '#' ? 255 : 0;
        for (int dy = 0; dy < cell; ++dy) {
          for (int dx = 0; dx < cell; ++dx) {
            img[static_cast<std::size_t>(y0 + r * cell + dy) * kW +
                static_cast<std::size_t>(x0 + c * cell + dx)] = val;
          }
        }
      }
    }
  };
  static const char* const kTag0[10] = {
      "##########", "#........#", "#.##.#.#.#", "#..###.#.#", "#..##....#",
      "#.#.#....#", "#..#.##..#", "#....#...#", "#........#", "##########",
  };
  static const char* const kTag1[10] = {
      "##########", "#........#", "#.##.##..#", "#..#.###.#", "#.####...#",
      "#..##....#", "#.#.##.#.#", "#...#..#.#", "#........#", "##########",
  };
  paint_tag(kTag0, 180, 200, 16);
  paint_tag(kTag1, 460, 200, 16);

  const fs::path multi_dir = base_dir_ / "high_id_frames";
  fs::create_directories(multi_dir);
  {
    constexpr std::uint32_t kHeaderSize = 54;
    constexpr std::uint32_t kPixelBytes = kW * kH * 3;
    constexpr std::uint32_t kFileSize = kHeaderSize + kPixelBytes;
    unsigned char hdr[kHeaderSize] = {};
    hdr[0] = 'B';
    hdr[1] = 'M';
    auto put_u32 = [&](std::size_t off, std::uint32_t v) {
      hdr[off] = static_cast<unsigned char>(v & 0xffu);
      hdr[off + 1] = static_cast<unsigned char>((v >> 8) & 0xffu);
      hdr[off + 2] = static_cast<unsigned char>((v >> 16) & 0xffu);
      hdr[off + 3] = static_cast<unsigned char>((v >> 24) & 0xffu);
    };
    put_u32(2, kFileSize);
    put_u32(10, kHeaderSize);
    put_u32(14, 40);
    put_u32(18, static_cast<std::uint32_t>(kW));
    put_u32(22, static_cast<std::uint32_t>(kH));
    hdr[26] = 1;
    hdr[28] = 24;
    put_u32(34, kPixelBytes);
    std::ofstream bmp(multi_dir / "frame_0.bmp", std::ios::binary);
    bmp.write(reinterpret_cast<const char*>(hdr), kHeaderSize);
    std::vector<unsigned char> row(static_cast<std::size_t>(kW) * 3);
    for (int y = kH - 1; y >= 0; --y) {
      for (int x = 0; x < kW; ++x) {
        const unsigned char v =
            img[static_cast<std::size_t>(y) * kW + static_cast<std::size_t>(x)];
        row[static_cast<std::size_t>(x) * 3] = v;
        row[static_cast<std::size_t>(x) * 3 + 1] = v;
        row[static_cast<std::size_t>(x) * 3 + 2] = v;
      }
      bmp.write(reinterpret_cast<const char*>(row.data()),
                static_cast<std::streamsize>(row.size()));
    }
  }
  {
    std::ofstream ts(multi_dir / "timestamps.csv");
    ts << "filename,pts_ns\nframe_0.bmp,1000000000\n";
  }

  // Tags 0 and 1 establish the multi-tag camera pose; tag 1005 sits in view
  // between them (undetected) so expected-in-view must count it as 2 / 3.
  const fs::path layout_path = base_dir_ / "high_id_layout.json";
  {
    std::ofstream out(layout_path);
    out << R"({
      "field": {"length": 16.0, "width": 8.0},
      "tags": [
        {"ID": 0, "pose": {"translation": {"x": 6.0, "y": 4.546875, "z": 0.65625},
          "rotation": {"quaternion": {"W": 0.0, "X": 0.0, "Y": 0.0, "Z": 1.0}}}},
        {"ID": 1, "pose": {"translation": {"x": 6.0, "y": 3.453125, "z": 0.65625},
          "rotation": {"quaternion": {"W": 0.0, "X": 0.0, "Y": 0.0, "Z": 1.0}}}},
        {"ID": 1005, "pose": {"translation": {"x": 6.0, "y": 4.0, "z": 0.65625},
          "rotation": {"quaternion": {"W": 0.0, "X": 0.0, "Y": 0.0, "Z": 1.0}}}}
      ]
    })";
  }

  const auto res =
      RunBinary(bin_, {"--frames", multi_dir.string(), "--intrinsics",
                       "1000,1000,320,240", "--layout", layout_path.string(),
                       "--tag-size-m", "0.5", "--mount", "0,0,0.5,0,0,0",
                       "--decimate", "1", "--threads", "1"});
  EXPECT_FALSE(res.signaled) << res.output;
  EXPECT_EQ(res.exit_code, 0) << res.output;
  EXPECT_NE(res.output.find("multi-tag 1"), std::string::npos) << res.output;
  EXPECT_NE(res.output.find("expected-in-view detection rate (tags the "
                            "multi-tag pose puts in view, >=12 px): 2 / 3"),
            std::string::npos)
      << res.output;
  EXPECT_NE(res.output.find("1005      0      1"), std::string::npos)
      << res.output;
}

}  // namespace
}  // namespace talos::vision::evaluation
