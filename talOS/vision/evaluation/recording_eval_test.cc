#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

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

}  // namespace
}  // namespace talos::vision::evaluation
