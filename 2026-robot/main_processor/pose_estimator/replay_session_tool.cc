#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <span>
#include <string>
#include <string_view>

#include "2026-robot/main_processor/drivetrain/drive_message_generated.h"
#include "geometry/geometry_generated.h"
#include "talOS/events/context.h"
#include "talOS/events/handles.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/manifest.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/events/time.h"

namespace {

using namespace std::chrono_literals;
using Loop = talos::event::SimulatedEventLoop<talos::event::log::LogWriter>;

constexpr std::uint64_t kSessionId = 42;
constexpr talos::event::MonotonicTime kOrigin =
    talos::event::MonotonicTime::from_nanos(10'000'000'000);
constexpr auto kStep = 20ms;
constexpr int kSteps = 50;

struct ModuleOffset {
  double x;
  double y;
};

// 2026 robot swerve geometry: BL, BR, FL, FR
constexpr ModuleOffset kModules[4] = {
    {-0.276225, 0.276225},
    {-0.276225, -0.276225},
    {0.276225, 0.276225},
    {0.276225, -0.276225},
};

class SessionGenerator {
 public:
  SessionGenerator(Loop& dt_loop, Loop& cam_loop)
      : dt_sender_{talos::event::make_sender<drivetrain::DrivetrainState>(
            dt_loop, "/drivetrain/state")},
        cam_sender_{talos::event::make_sender<geometry::PoseObservation>(
            cam_loop, "/front_camera/state")},
        dt_tick_{talos::event::make_timer<&SessionGenerator::on_dt_tick>(
            dt_loop, "dt_tick", this)},
        cam_tick_{talos::event::make_timer<&SessionGenerator::on_cam_tick>(
            cam_loop, "cam_tick", this)} {
    dt_tick_.setup_periodic(kOrigin + kStep, kStep);
    cam_tick_.setup_periodic(kOrigin + kStep, kStep);
  }

 private:
  void on_dt_tick(const talos::event::Context& ctx) {
    ++step_;
    const double dt_s = 0.02;
    const double vx = 1.0;
    const double vy = 0.0;
    const double omega = (step_ > 20) ? 1.0 : 0.0;  // Turn after step 20
    const double slip = 0.85;  // Wheel odometry falls behind true x

    // Advance ground truth pose
    if (omega == 0.0) {
      true_x_ += vx * dt_s;
    } else {
      const double yaw_next = true_yaw_ + omega * dt_s;
      true_x_ += (vx * (std::sin(yaw_next) - std::sin(true_yaw_))) / omega;
      true_y_ += (vx * (std::cos(true_yaw_) - std::cos(yaw_next))) / omega;
      true_yaw_ = yaw_next;
    }

    for (int i = 0; i < 4; ++i) {
      const double wx = vx - omega * kModules[i].y;
      const double wy = vy + omega * kModules[i].x;
      const double speed = std::hypot(wx, wy);
      if (speed > 0.0) {
        mod_ang_[i] = std::atan2(wy, wx);
      }
      mod_dist_[i] += speed * dt_s * slip;
    }

    const std::uint64_t sample_time_ns =
        static_cast<std::uint64_t>(ctx.now.nanos());

    // Record sample time at step 10 for the delayed observation
    if (step_ == 10) {
      delayed_obs_time_ns_ = sample_time_ns;
      delayed_obs_truth_x_ = true_x_;
      delayed_obs_truth_y_ = true_y_;
    }

    dt_sender_.send(drivetrain::DrivetrainState{
        sample_time_ns,
        /*enabled=*/false,
        /*modules_valid=*/true,
        /*yaw_valid=*/true,
        vx,
        vy,
        omega,
        true_yaw_,
        geometry::SwerveModulePosition{mod_dist_[0], mod_ang_[0]},
        geometry::SwerveModulePosition{mod_dist_[1], mod_ang_[1]},
        geometry::SwerveModulePosition{mod_dist_[2], mod_ang_[2]},
        geometry::SwerveModulePosition{mod_dist_[3], mod_ang_[3]},
    });
  }

  void on_cam_tick(const talos::event::Context& ctx) {
    if (step_ == 18) {
      // 1. Delayed valid observation of the true pose from step 10 (160ms ago)
      cam_sender_.send(geometry::PoseObservation{
          delayed_obs_time_ns_,
          geometry::Pose2d{delayed_obs_truth_x_, delayed_obs_truth_y_, 0.0},
          /*x_std_m=*/0.01,
          /*y_std_m=*/0.01,
          /*yaw_std_rad=*/0.01,
          /*tag_count=*/2,
          /*valid=*/true,
      });
    } else if (step_ == 19) {
      // 2. Stale observation (2 seconds in the past, older than
      // max_observation_age_us)
      cam_sender_.send(geometry::PoseObservation{
          static_cast<std::uint64_t>((ctx.now - 2s).nanos()),
          geometry::Pose2d{5.0, 5.0, 1.0},
          /*x_std_m=*/0.01,
          /*y_std_m=*/0.01,
          /*yaw_std_rad=*/0.01,
          /*tag_count=*/2,
          /*valid=*/true,
      });
    } else if (step_ == 20) {
      // 3. Invalid observation
      cam_sender_.send(geometry::PoseObservation{
          static_cast<std::uint64_t>(ctx.now.nanos()),
          geometry::Pose2d{10.0, 10.0, 2.0},
          /*x_std_m=*/0.01,
          /*y_std_m=*/0.01,
          /*yaw_std_rad=*/0.01,
          /*tag_count=*/2,
          /*valid=*/false,
      });
    }
  }

  talos::event::Sender<Loop, drivetrain::DrivetrainState> dt_sender_;
  talos::event::Sender<Loop, geometry::PoseObservation> cam_sender_;
  talos::event::Timer<Loop> dt_tick_;
  talos::event::Timer<Loop> cam_tick_;

  int step_{0};
  double true_x_{0.0};
  double true_y_{0.0};
  double true_yaw_{0.0};
  double mod_dist_[4]{0.0, 0.0, 0.0, 0.0};
  double mod_ang_[4]{0.0, 0.0, 0.0, 0.0};

  std::uint64_t delayed_obs_time_ns_{0};
  double delayed_obs_truth_x_{0.0};
  double delayed_obs_truth_y_{0.0};
};

int CommandGenerate(std::string_view session_dir) {
  std::filesystem::create_directories(session_dir);

  talos::event::SimulationEnvironment env;
  const Loop::Options options{.start = kOrigin};

  Loop dt_loop{env,
               talos::event::log::LogWriter{
                   std::string(session_dir) + "/drivetrain.tlog",
                   "drivetrain",
                   {},
                   kSessionId},
               options};
  Loop cam_loop{env,
                talos::event::log::LogWriter{
                    std::string(session_dir) + "/front_camera.tlog",
                    "front_camera",
                    {},
                    kSessionId},
                options};

  SessionGenerator generator{dt_loop, cam_loop};

  for (int step = 1; step <= kSteps; ++step) {
    const auto now = kOrigin + step * kStep;
    dt_loop.run_until(now);
    cam_loop.run_until(now);
  }

  dt_loop.finish();
  cam_loop.finish();
  std::cout << "generated session in " << session_dir << '\n';
  return 0;
}

int CommandEstimates(std::string_view log_path) {
  talos::event::log::LogReader reader{log_path};
  std::optional<std::uint16_t> state_source_id;

  for (const auto& reg : reader.manifest()) {
    if (reg.kind == talos::event::SourceKind::SENDER &&
        reg.name == "/pose_estimator/state") {
      state_source_id = reg.id;
      break;
    }
  }

  if (!state_source_id.has_value()) {
    std::cerr << "error: /pose_estimator/state sender not found in manifest\n";
    return 1;
  }

  talos::event::log::LogReader::Record record;
  while (reader.next(record)) {
    if (record.header.kind !=
            static_cast<std::uint16_t>(talos::event::EventKind::SEND) ||
        record.header.source_id != *state_source_id) {
      continue;
    }

    if (record.payload.size() < sizeof(geometry::PoseEstimate)) {
      std::cerr << "error: payload too small for PoseEstimate: "
                << record.payload.size() << " vs "
                << sizeof(geometry::PoseEstimate) << '\n';
      return 1;
    }

    const auto* est =
        reinterpret_cast<const geometry::PoseEstimate*>(record.payload.data());

    std::cout << std::setprecision(17)
              << "{\"sample_time_ns\": " << est->sample_time_ns()
              << ", \"pose_x_m\": " << est->pose().x_m()
              << ", \"pose_y_m\": " << est->pose().y_m()
              << ", \"pose_yaw_rad\": " << est->pose().yaw_rad()
              << ", \"velocity_vx_mps\": " << est->velocity().vx_mps()
              << ", \"velocity_vy_mps\": " << est->velocity().vy_mps()
              << ", \"velocity_omega_radps\": " << est->velocity().omega_radps()
              << ", \"valid\": " << (est->valid() ? "true" : "false")
              << ", \"observation_accepted_count\": "
              << est->observation_accepted_count()
              << ", \"observation_rejected_count\": "
              << est->observation_rejected_count() << "}\n";
  }

  return 0;
}

int CommandRecords(std::string_view log_path) {
  talos::event::log::LogReader reader{log_path};
  talos::event::log::LogReader::Record record;

  while (reader.next(record)) {
    std::cout << "rec " << record.header.kind << " " << record.header.source_id
              << " " << record.header.dispatch_index << " "
              << record.header.event_time_ns << " " << record.header.now_ns
              << " " << record.header.sequence << " " << record.header.aux
              << " " << record.header.payload_bytes << " hex=";
    for (std::byte b : record.payload) {
      std::cout << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<unsigned int>(static_cast<unsigned char>(b));
    }
    std::cout << std::dec << '\n';
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: replay_session_tool <generate DIR|estimates "
                 "LOG|records LOG>\n";
    return 1;
  }

  const std::string_view cmd = argv[1];
  const std::string_view arg = argv[2];

  if (cmd == "generate") {
    return CommandGenerate(arg);
  }
  if (cmd == "estimates") {
    return CommandEstimates(arg);
  }
  if (cmd == "records") {
    return CommandRecords(arg);
  }

  std::cerr << "unknown subcommand: " << cmd << '\n';
  return 1;
}
