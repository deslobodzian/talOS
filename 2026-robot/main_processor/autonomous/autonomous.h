#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "autonomous/autonomous_message_generated.h"
#include "autonomous/progress.h"
#include "autonomous/selection.h"
#include "autonomous/topics.h"
#include "drivetrain/drive_message_generated.h"
#include "geometry/geometry_generated.h"
#include "intake/intake_message_generated.h"
#include "intake/topics.h"
#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/devices.h"
#include "talOS/geometry/se2.h"
#include "talOS/node/node.h"
#include "talOS/trajectory/holonomic_follower.h"
#include "talOS/trajectory/straight_line.h"
#include "talOS/trajectory/trajectory.h"

namespace autonomous {

// Shared-memory slot contracts: docs/DATA_FLOW.md. /autonomous/state is this
// node's; /intake/state comes from a Python node, so its C++ consumer pins it.
static_assert(sizeof(AutonomousState) == 2,
              "AutonomousState size changed; update docs/DATA_FLOW.md");
static_assert(sizeof(intake::IntakeState) == 20,
              "IntakeState size changed; update docs/DATA_FLOW.md");

class Autonomous {
 public:
  explicit Autonomous(talos::Node& node)
      : node_{node},
        shooter_velocity_{node.param("shooter_velocity_rps", 60.0)},
        intake_velocity_{node.param("intake_velocity_rps", 30.0)},
        feed_velocity_{node.param("feed_velocity_rps", -20.0)},
        target_x_{node.param("target_x_m", 2.0)},
        target_y_{node.param("target_y_m", 0.0)},
        target_yaw_{node.param("target_yaw_rad", 0.0)},
        position_tolerance_{node.param("position_tolerance_m", 0.10)},
        yaw_tolerance_{node.param("yaw_tolerance_rad", 0.05)},
        max_linear_{node.param("max_linear_mps", 2.0)},
        max_angular_{node.param("max_angular_radps", 3.0)},
        max_linear_accel_{node.param("max_linear_accel_mps2", 4.0)},
        max_angular_accel_{node.param("max_angular_accel_radps2", 6.0)},
        replan_step_m_{node.param("replan_step_m", 0.15)},
        replan_step_rad_{node.param("replan_step_rad", 0.1)},
        field_length_{node.param("field_length_m", 16.541)},
        field_width_{node.param("field_width_m", 8.069)},
        kp_xy_{node.param("xy_feedback_per_m_mps", 2.0)},
        kp_yaw_{node.param("yaw_feedback_per_rad_radps", 1.0)},
        feed_duration_{std::chrono::microseconds{
            node.param<int>("feed_duration_us", 500000)}},
        spinup_timeout_s_{PositiveSeconds(node, "spinup_timeout_s", 2.0)},
        drive_timeout_s_{PositiveSeconds(node, "drive_timeout_s", 10.0)},
        autonomous_period_{std::chrono::microseconds{
            node.param<int>("autonomous_period_us", 20000000)}},
        command_timeout_{CommandTimeout(node)},
        routines_{LoadRoutines(node)},
        selection_{node.session_dir(), RoutineSignature(routines_),
                   static_cast<unsigned>(routines_.size()),
                   DefaultRoutine(node, routines_)},
        selection_port_{node.param("selection_port_index", 0)},
        selection_pov_{node.param("selection_pov_index", 0)},
        selection_next_{node.param("selection_next_direction_index", 2)},
        selection_previous_{
            node.param("selection_previous_direction_index", 6)},
        progress_{node.session_dir()},
        pose_{node.subscribe<geometry::PoseEstimate>("/pose_estimator/state")},
        shooter_state_{
            node.subscribe<shooter::ShooterState>(shooter::topics::kState)},
        intake_state_{
            node.subscribe<intake::IntakeState>(intake::topics::kState)},
        drive_{node, "drivetrain", "auto"},
        shooter_{node, "shooter", "auto"},
        intake_{node, "intake", "auto"},
        ds_{node},
        phase_{node, Phase::kInhibited},
        state_{node.publish<AutonomousState>(topics::kState)} {
    // feed_duration_us, the timeouts and every routine key are checked by
    // LoadRoutines and PositiveSeconds above.
    if (autonomous_period_ <= talos::Duration::zero())
      BadParameter(node, "autonomous_period_us", "must be positive");
    if (selection_port_ < 0 || selection_port_ > 5)
      BadParameter(node, "selection_port_index", "must be in [0, 5]");
    if (selection_pov_ < 0 || selection_pov_ > 3)
      BadParameter(node, "selection_pov_index", "must be in [0, 3]");
    if (!PovAngle(selection_next_))
      BadParameter(node, "selection_next_direction_index", "must be in [0, 7]");
    if (!PovAngle(selection_previous_))
      BadParameter(node, "selection_previous_direction_index",
                   "must be in [0, 7]");
    if (selection_next_ == selection_previous_)
      BadParameter(node, "selection_previous_direction_index",
                   "must differ from selection_next_direction_index");
    ValidateController();
    // After ValidateController, so a bad gain or tolerance names its key.
    talos::trajectory::FollowerConfig follower;
    follower.translation = {.kp = kp_xy_};
    follower.heading = {.kp = kp_yaw_};
    follower.max_linear_mps = max_linear_;
    follower.max_angular_radps = max_angular_;
    follower.position_tolerance_m = position_tolerance_;
    follower.heading_tolerance_rad = yaw_tolerance_;
    follower_ = talos::trajectory::HolonomicFollower{follower};
    // A plan inside update() rebuilds the path without allocating.
    path_.reserve(talos::trajectory::kStraightLineSamples);
    signature_ = RoutineSignature(routines_);
  }

  void update() {
    const auto now = node_.now();
    if (feeding_) feed_elapsed_ += now - feed_last_update_;
    feeding_ = false;
    // A drive tick continues the path only if the previous tick sent a
    // path-following target in the same generation. After any other tick
    // (released, stopped, another phase, a resume) it plans again from the
    // live pose: the robot braked while its lane was released, so the old
    // path's reference would ask for its cruise speed at once.
    continuing_ = following_;
    following_ = false;
    // Silence is not a mode transition: preserve progress and wall deadlines,
    // but never keep commanding lanes or advance feed time.
    if (!ds_.connected()) {
      selection_seeded_ = false;
      CheckDeadline(now);
      Stop();
      return;
    }
    // A restarted process learns from its predecessor's record what this
    // match has already done, against the first Driver Station packet.
    if (!restore_checked_) {
      restore_checked_ = true;
      Restore(now);
    }
    // An armed routine adopts the first FMS match data it sees, so data that
    // arrives after the arm does not read as another match.
    if (Armed() && match_ == MatchKey{}) match_ = MatchKey::Of(ds_.data().fms);
    SelectRoutine();
    if (!ds_.enabled() || !ds_.autonomous()) {
      // Leaving enabled auto pauses an armed routine for the rest of its
      // match; only another match (NewMatch) starts over.
      if (!Armed() || NewMatch(now)) ResetRoutine();
      CheckDeadline(now);
      Stop();
      return;
    }
    // A new process without a record of this match must observe
    // disabled/non-auto before it can arm. Merely receiving an already-running
    // auto generation must never fire a piece.
    if (phase_.is(Phase::kInhibited)) {
      Stop();
      return;
    }
    // A new generation in the same match resumes the step it paused on: a
    // fired action never fires again in that match.
    if (generation_ != ds_.data().mode_since_us && Armed()) {
      if (NewMatch(now)) {
        ResetRoutine();
      } else {
        Resume();
      }
    }
    if (phase_.is(Phase::kIdle)) {
      generation_ = ds_.data().mode_since_us;
      armed_at_ = now;
      match_ = MatchKey::Of(ds_.data().fms);
      red_ = ds_.alliance() == talos::driver_station::Alliance::kRed;
      latched_ = selection_.get();
      ApplyRoutine();
      action_ = 0;
      spinup_started_ = now;
      Enter(now);
    }
    CheckDeadline(now);
    if (phase_.is(Phase::kAborted) || phase_.is(Phase::kHold)) {
      Stop();
      return;
    }
    const bool shooter_ready =
        shooter_state_.fresh() && shooter_state_->at_speed() &&
        std::abs(shooter_state_->target_velocity_rps() - shooter_velocity_) <=
            1e-6;
    // Prerequisites and completions belong to action descriptors, not to the
    // routine name. FEED retries SPINUP without resetting the preload budget.
    if (CurrentAction().prerequisite == Prerequisite::kShooterReady &&
        !PrerequisiteMet(CurrentAction(), shooter_ready)) {
      --action_;
      phase_.go(CurrentAction().phase);
    }
    if (CurrentAction().completion == Completion::kShooterReady &&
        shooter_ready) {
      Advance(now);
    }
    if (CurrentAction().completion == Completion::kFeedDuration &&
        feed_elapsed_ >= feed_duration_) {
      Advance(now);
    }
    drivetrain::ChassisTarget drive{};
    double intake = 0;
    double shooter = shooter_velocity_;
    if (phase_.is(Phase::kFeed)) {
      intake = feed_velocity_;
      feeding_ = true;
      feed_last_update_ = now;
    }
    if (CurrentAction().completion == Completion::kWaypoint) {
      shooter = 0;
      // A pose that is not fresh, valid, localized and usable releases the
      // drive lane on this tick (below), and the next usable one plans
      // again. A step in a usable estimate plans again on this tick without
      // releasing: a release would neutralize the drivetrain at once,
      // skipping its slew limit.
      if (PrerequisiteMet(CurrentAction(), shooter_ready)) {
        const auto& estimate = *pose_;
        const talos::Time estimate_time = now - pose_.age();
        if (!continuing_ || Stepped(estimate, estimate_time)) {
          Plan(estimate.pose(), now);
        }
        const auto out =
            planned_ ? follower_.step_pose(
                           path_, Seconds(now - plan_started_), estimate.pose(),
                           continuing_ ? Seconds(now - followed_at_) : 0.0,
                           talos::trajectory::SpeedsFrame::kRobot)
                     : talos::trajectory::FollowerOutput{};
        if (out.finished) {
          Advance(now);
        } else if (out.valid) {
          drive = out.speeds.as<drivetrain::ChassisTarget>();
          following_ = true;
          followed_at_ = now;
          previous_estimate_ = estimate;
          previous_estimate_time_ = estimate_time;
          if (routines_[latched_].collect && intake_state_.fresh() &&
              !intake_state_->beam_broken()) {
            intake = intake_velocity_;
          }
        }
      }
    }
    if (phase_.is(Phase::kHold)) {
      Stop();
      return;
    }
    // Write-ahead: the record holds this step before its commands go out.
    Persist();
    if (phase_.is(Phase::kDrive) && !following_) {
      drive_.release();
    } else {
      drive_.send(drive);
    }
    if (phase_.is(Phase::kDrive) && !routines_[latched_].collect) {
      shooter_.release();
      intake_.release();
    } else {
      shooter_.send(shooter::ShooterTarget{shooter});
      intake_.send(intake::IntakeTarget{static_cast<float>(intake)});
    }
    PublishState();
  }

 private:
  enum class Phase {
    kIdle,
    kSpinup,
    kFeed,
    kDrive,
    kHold,
    kAborted,
    kInhibited
  };
  enum class Prerequisite { kNone, kShooterReady, kLocalizedPose };
  enum class Completion { kShooterReady, kFeedDuration, kWaypoint, kNever };
  enum class Timeout { kPreload, kDrive, kNone };
  enum class Cancel { kReleaseAll };
  struct Action {
    Phase phase;
    Prerequisite prerequisite;
    Completion completion;
    Timeout timeout;
    Cancel cancel = Cancel::kReleaseAll;
  };
  // The FMS match identity; all zero without FMS match data.
  static_assert(sizeof(Progress::event_name) ==
                    talos::driver_station::kMaxEventNameLength &&
                sizeof(talos::driver_station::FmsData::event_name) ==
                    talos::driver_station::kMaxEventNameLength);
  struct MatchKey {
    talos::driver_station::MatchType type{};
    std::uint16_t number = 0;
    std::uint8_t replay = 0;
    std::array<char, talos::driver_station::kMaxEventNameLength> event{};
    bool operator==(const MatchKey&) const = default;
    static MatchKey Of(const talos::driver_station::FmsData& fms) {
      MatchKey key{fms.match_type, fms.match_number, fms.replay_number, {}};
      std::copy_n(fms.event_name,
                  ::strnlen(fms.event_name, sizeof fms.event_name),
                  key.event.begin());
      return key;
    }
    static MatchKey Of(const Progress& progress) {
      MatchKey key{
          static_cast<talos::driver_station::MatchType>(progress.match_type),
          progress.match_number,
          progress.replay_number,
          {}};
      std::copy_n(progress.event_name,
                  ::strnlen(progress.event_name, sizeof progress.event_name),
                  key.event.begin());
      return key;
    }
  };
  struct Routine {
    std::string name;
    std::vector<Action> actions;
    bool collect;
    double shooter, intake, feed, x, y, yaw, feed_us, spinup_s, drive_s;
  };
  talos::Node& node_;
  double shooter_velocity_, intake_velocity_, feed_velocity_;
  double target_x_, target_y_, target_yaw_;
  double position_tolerance_, yaw_tolerance_, max_linear_, max_angular_;
  double max_linear_accel_, max_angular_accel_;
  double replan_step_m_, replan_step_rad_;
  double field_length_, field_width_, kp_xy_, kp_yaw_;
  // The drive action's path, a straight line planned from the live pose,
  // and its follower; planned_ is false while no plan could be built.
  talos::trajectory::Trajectory path_;
  talos::trajectory::HolonomicFollower follower_;
  bool planned_ = false;
  bool following_ = false;   // this tick sent a path-following target
  bool continuing_ = false;  // ... and so did the previous one
  talos::Time plan_started_{};
  talos::Time followed_at_{};
  // The estimate the last following tick drove on, and its publication time.
  geometry::PoseEstimate previous_estimate_{};
  talos::Time previous_estimate_time_{};
  talos::Duration feed_duration_;
  double spinup_timeout_s_, drive_timeout_s_;
  talos::Duration autonomous_period_;
  talos::Duration command_timeout_;  // [robot] command_timeout_us
  std::vector<Routine> routines_;
  Selection selection_;
  int selection_port_, selection_pov_, selection_next_, selection_previous_;
  bool selection_seeded_ = false, next_was_pressed_ = false;
  bool previous_was_pressed_ = false;
  std::uint8_t latched_ = 0;
  std::size_t action_ = 0;
  std::uint64_t generation_ = 0;
  talos::Time armed_at_{};
  MatchKey match_{};
  talos::Time spinup_started_{};
  talos::Time drive_started_{};
  ProgressStore progress_;
  bool restore_checked_ = false;
  std::uint64_t signature_ = 0;
  talos::Duration feed_elapsed_{};
  talos::Time feed_last_update_{};
  bool feeding_ = false;
  talos::Subscription<geometry::PoseEstimate> pose_;
  talos::Subscription<shooter::ShooterState> shooter_state_;
  talos::Subscription<intake::IntakeState> intake_state_;
  talos::TargetLane<drivetrain::ChassisTarget> drive_;
  talos::TargetLane<shooter::ShooterTarget> shooter_;
  talos::TargetLane<intake::IntakeTarget> intake_;
  talos::DriverStation ds_;
  talos::StateMachine<Phase> phase_;
  talos::Publisher<AutonomousState> state_;
  bool red_ = false;

  static talos::Duration CommandTimeout(talos::Node& node) {
    return std::chrono::microseconds{
        talos::config::ParseRobotConfig(node.config_path())
            .hardware.command_timeout_us};
  }

  // Every construction error names its key as the config spells it, in the
  // form talos::Node::param uses for a wrong type.
  [[noreturn]] static void BadParameter(const talos::Node& node,
                                        std::string_view key,
                                        std::string_view problem) {
    const std::string name{node.name()};
    throw std::invalid_argument(name + ": parameter '" + std::string{key} +
                                "' " + std::string{problem} +
                                " in [subsystems." + name + "]");
  }

  static double PositiveSeconds(talos::Node& node, const char* key,
                                double fallback) {
    const double value = node.param(key, fallback);
    if (!std::isfinite(value) || value <= 0)
      BadParameter(node, key, "must be finite and positive");
    return value;
  }

  void ResetRoutine() {
    phase_.go(Phase::kIdle);
    generation_ = 0;
    continuing_ = false;
    feeding_ = false;
    feed_elapsed_ = talos::Duration::zero();
  }

  bool Armed() const {
    return !phase_.is(Phase::kIdle) && !phase_.is(Phase::kInhibited);
  }

  // Whether the routine armed at armed_at_ belongs to an earlier match:
  //  - the Driver Station reported teleop or test (FMS runs teleop after
  //    auto). Only with a DS attached: WPILib reads a lost DS as teleop;
  //  - FMS match data (type, number, replay, event) differs from the
  //    routine's. Only when both are present: missing data on either side is
  //    not a new match, so an FMS blip, or data arriving after the arm,
  //    cannot re-fire;
  //  - autonomous_period_us has passed since the arm, so no autonomous
  //    period that included it is still running.
  bool NewMatch(talos::Time now) const {
    const auto& fms = ds_.data().fms;
    const bool attached = (fms.flags & talos::driver_station::kDsAttached) != 0;
    const auto key = MatchKey::Of(fms);
    return (attached && (ds_.teleop() || ds_.test())) ||
           (key != MatchKey{} && match_ != MatchKey{} && key != match_) ||
           now - armed_at_ >= autonomous_period_;
  }

  // The predecessor's record, against the first Driver Station packet:
  //  - none (or cleared): stay INHIBITED, as a process that knows nothing;
  //  - from the same match (NewMatch false for its arm time and match key,
  //    on the same boot's monotonic clock): restore its paused step. The next
  //    enabled-auto tick Resume()s it like any new generation;
  //  - from another match: ignore it and stay INHIBITED;
  //  - unreadable (both slots damaged, another record version, or indices
  //    this routine list does not have): HoldForMatch.
  void Restore(talos::Time now) {
    using Loaded = ProgressStore::Loaded;
    if (progress_.loaded() == Loaded::kNone) return;
    const Progress& p = progress_.progress();
    if (progress_.loaded() == Loaded::kCorrupt || p.signature != signature_ ||
        p.routine >= routines_.size() ||
        p.action >= routines_[p.routine].actions.size()) {
      HoldForMatch(now);
      return;
    }
    armed_at_ = talos::Time::from_nanos(p.armed_at_ns);
    match_ = MatchKey::Of(p);
    if (now < armed_at_ || NewMatch(now)) return;
    latched_ = p.routine;
    ApplyRoutine();
    action_ = p.action;
    spinup_started_ = talos::Time::from_nanos(p.spinup_started_ns);
    drive_started_ = talos::Time::from_nanos(p.drive_started_ns);
    feed_elapsed_ = talos::Duration{p.feed_elapsed_ns};
    red_ = ds_.alliance() == talos::driver_station::Alliance::kRed;
    phase_.go(p.aborted ? Phase::kAborted : CurrentAction().phase);
    generation_ = 0;
  }

  // A record that cannot be trusted means some routine may have fired in
  // this match at a step nobody can name. Starting fresh could fire it again,
  // and INHIBITED would arm fresh at the next disable/re-enable; so treat
  // everything as fired: HOLD, armed now in the current match, until NewMatch.
  void HoldForMatch(talos::Time now) {
    latched_ = selection_.get();
    ApplyRoutine();
    action_ = routines_[latched_].actions.size() - 1;  // every routine's HOLD
    armed_at_ = now;
    match_ = MatchKey::Of(ds_.data().fms);
    generation_ = 0;
    phase_.go(Phase::kHold);
  }

  // Record the routine's progress (write-ahead, before the tick's commands).
  // INHIBITED writes nothing: the predecessor's record stays until this
  // process knows better.
  void Persist() {
    if (phase_.is(Phase::kInhibited)) return;
    Progress p{};
    if (Armed()) {
      p.armed = 1;
      p.aborted = phase_.is(Phase::kAborted) ? 1 : 0;
      p.signature = signature_;
      p.armed_at_ns = armed_at_.nanos();
      p.spinup_started_ns = spinup_started_.nanos();
      p.drive_started_ns = drive_started_.nanos();
      // A feed commanded this tick counts as fed until its lane goes stale:
      // after a kill, the intake obeys it for up to command_timeout_us. So a
      // restarted feed runs at most the remaining time, never more.
      p.feed_elapsed_ns =
          (feed_elapsed_ + (feeding_ ? command_timeout_ : talos::Duration{}))
              .count();
      p.action = static_cast<std::uint32_t>(action_);
      p.routine = latched_;
      p.match_type = static_cast<std::uint8_t>(match_.type);
      p.match_number = match_.number;
      p.replay_number = match_.replay;
      std::copy(match_.event.begin(), match_.event.end(), p.event_name);
    }
    progress_.Write(p);
  }

  void Stop() {
    Persist();
    ReleaseTargets();
    PublishState();
  }

  // Continue the paused step under the new generation. Deadlines keep their
  // wall-clock start (spinup_started_, the DRIVE entry), feed keeps its
  // accumulated time, and a drive re-plans from the current pose: the robot
  // stopped, and may have been moved, while disabled.
  void Resume() {
    generation_ = ds_.data().mode_since_us;
    continuing_ = false;
  }

  static double Seconds(talos::Duration duration) {
    return std::chrono::duration<double>(duration).count();
  }

  // A straight line from `start` to the routine's waypoint (rotated about
  // the field centre for Red), rest to rest within the speed and
  // acceleration limits, followed from its start. A line that cannot be
  // built (UsablePose keeps the start finite and near the field, but a
  // waypoint the config puts beyond the double range overflows) leaves
  // planned_ false: the lane is released, as for an unusable pose, and the
  // next tick plans again.
  void Plan(const geometry::Pose2d& start, talos::Time now) {
    using talos::geometry::SE2;
    SE2<double> goal{target_yaw_, SE2<double>::Vector2{target_x_, target_y_}};
    if (red_) {
      goal = talos::trajectory::Flip(
          goal, talos::trajectory::AllianceFlip::kRotate,
          talos::trajectory::FieldDimensions{field_length_, field_width_});
    }
    const SE2<double> from{start.yaw_rad(),
                           SE2<double>::Vector2{start.x_m(), start.y_m()}};
    planned_ = static_cast<bool>(talos::trajectory::MakeStraightLine(
        from, goal,
        {max_linear_, max_linear_accel_, max_angular_, max_angular_accel_},
        path_));
    follower_.reset();
    plan_started_ = now;
  }

  // Whether `estimate` moved from the one the last following tick drove on
  // by more than its velocity explains plus replan_step_m or
  // replan_step_rad: a correction (vision returning, a re-seed), not motion.
  // The estimate's velocity is the chassis's, robot frame
  // (/drivetrain/state's); the motion it explains is the mean of the two
  // estimates' field-frame velocities over the time between their
  // publications, so a re-read of the same estimate explains, and moves,
  // nothing.
  bool Stepped(const geometry::PoseEstimate& estimate,
               talos::Time estimate_time) const {
    const geometry::PoseEstimate& before = previous_estimate_;
    const double dt_s =
        std::max(Seconds(estimate_time - previous_estimate_time_), 0.0);
    const auto field_velocity = [](const geometry::PoseEstimate& e) {
      const double c = std::cos(e.pose().yaw_rad());
      const double s = std::sin(e.pose().yaw_rad());
      return std::array<double, 2>{
          c * e.velocity().vx_mps() - s * e.velocity().vy_mps(),
          s * e.velocity().vx_mps() + c * e.velocity().vy_mps()};
    };
    const auto v0 = field_velocity(before);
    const auto v1 = field_velocity(estimate);
    const double dx_m = estimate.pose().x_m() - before.pose().x_m() -
                        0.5 * (v0[0] + v1[0]) * dt_s;
    const double dy_m = estimate.pose().y_m() - before.pose().y_m() -
                        0.5 * (v0[1] + v1[1]) * dt_s;
    const double omega_radps = 0.5 * (before.velocity().omega_radps() +
                                      estimate.velocity().omega_radps());
    const double dyaw_rad = talos::trajectory::WrapAngle(
        estimate.pose().yaw_rad() - before.pose().yaw_rad() -
        omega_radps * dt_s);
    return std::hypot(dx_m, dy_m) > replan_step_m_ ||
           std::abs(dyaw_rad) > replan_step_rad_;
  }

  void CheckDeadline(talos::Time now) {
    // One wall-clock preload budget from the FIRST spinup entry, including
    // feed and readiness retries. Flapping readiness, a DS outage or a pause
    // cannot postpone the deadline. Drive likewise includes every outage.
    if (((phase_.is(Phase::kSpinup) || phase_.is(Phase::kFeed)) &&
         CurrentAction().timeout == Timeout::kPreload &&
         now - spinup_started_ >=
             std::chrono::duration<double>{spinup_timeout_s_}) ||
        (phase_.is(Phase::kDrive) &&
         CurrentAction().timeout == Timeout::kDrive &&
         now - drive_started_ >=
             std::chrono::duration<double>{drive_timeout_s_})) {
      phase_.go(Phase::kAborted);
    }
  }

  void ReleaseTargets() {
    // Every action's explicit cancellation policy is fail-closed. Repeat
    // releases even in HOLD/none so the lane is observably released all period.
    switch (CurrentAction().cancel) {
      case Cancel::kReleaseAll:
        drive_.release();
        shooter_.release();
        intake_.release();
        break;
    }
  }

  static bool PovAngle(int value) { return value >= 0 && value < 8; }
  static std::vector<Routine> LoadRoutines(talos::Node& node) {
    const auto names = node.param<std::vector<std::string>>(
        "routines", {"preload", "none", "shoot_only", "taxi"});
    if (names.empty() || names.size() > 256)
      BadParameter(node, "routines", "must list 1..256 names");
    std::vector<Routine> result;
    result.reserve(names.size());
    for (const auto& name : names) {
      if (name != "preload" && name != "none" && name != "shoot_only" &&
          name != "taxi")
        BadParameter(node, "routines",
                     "names unknown routine '" + name +
                         "' (known: preload, none, shoot_only, taxi)");
      if (std::any_of(result.begin(), result.end(),
                      [&](const Routine& r) { return r.name == name; }))
        BadParameter(node, "routines", "names '" + name + "' twice");
      Routine r{};
      r.name = name;
      r.collect = name == "preload";
      if (name == "preload" || name == "shoot_only") {
        r.actions.push_back({Phase::kSpinup, Prerequisite::kNone,
                             Completion::kShooterReady, Timeout::kPreload});
        r.actions.push_back({Phase::kFeed, Prerequisite::kShooterReady,
                             Completion::kFeedDuration, Timeout::kPreload});
      }
      if (name == "preload" || name == "taxi")
        r.actions.push_back({Phase::kDrive, Prerequisite::kLocalizedPose,
                             Completion::kWaypoint, Timeout::kDrive});
      r.actions.push_back({Phase::kHold, Prerequisite::kNone,
                           Completion::kNever, Timeout::kNone});
      // A routine key overrides the common one. The common value is checked
      // first, so a bad effective value names the routine key only when the
      // routine sets it.
      const auto param = [&](const char* key, double fallback, bool positive) {
        const auto check = [&](std::string_view where, double value) {
          if (!std::isfinite(value) || (positive && value <= 0))
            BadParameter(
                node, where,
                positive ? "must be finite and positive" : "must be finite");
        };
        const double common = node.param(key, fallback);
        check(key, common);
        const std::string routine_key = "routine." + name + "." + key;
        const double value = node.param(routine_key, common);
        check(routine_key, value);
        return value;
      };
      r.shooter = param("shooter_velocity_rps", 60, false);
      r.intake = param("intake_velocity_rps", 30, false);
      r.feed = param("feed_velocity_rps", -20, false);
      r.x = param("target_x_m", 2, false);
      r.y = param("target_y_m", 0, false);
      r.yaw = param("target_yaw_rad", 0, false);
      // feed_duration_us is an integer in the existing public configuration.
      const int common_feed_us = node.param<int>("feed_duration_us", 500000);
      if (common_feed_us <= 0)
        BadParameter(node, "feed_duration_us", "must be positive");
      const std::string feed_key = "routine." + name + ".feed_duration_us";
      r.feed_us = node.param<int>(feed_key, common_feed_us);
      if (r.feed_us <= 0) BadParameter(node, feed_key, "must be positive");
      r.spinup_s = param("spinup_timeout_s", 2, true);
      r.drive_s = param("drive_timeout_s", 10, true);
      result.push_back(std::move(r));
    }
    return result;
  }
  static std::uint8_t DefaultRoutine(talos::Node& node,
                                     const std::vector<Routine>& routines) {
    const auto name = node.param<std::string>("default_routine", "preload");
    for (std::size_t i = 0; i < routines.size(); ++i)
      if (routines[i].name == name) return static_cast<std::uint8_t>(i);
    BadParameter(node, "default_routine",
                 "names '" + name + "', which routines does not list");
  }
  static std::uint64_t RoutineSignature(const std::vector<Routine>& routines) {
    std::uint64_t hash = 14695981039346656037ULL;
    for (const auto& r : routines) {
      for (unsigned char c : r.name) hash = (hash ^ c) * 1099511628211ULL;
      hash = (hash ^ 0xff) * 1099511628211ULL;
    }
    return hash;
  }
  void ValidateController() const {
    const std::pair<const char*, double> checked[] = {
        {"position_tolerance_m", position_tolerance_},
        {"yaw_tolerance_rad", yaw_tolerance_},
        {"max_linear_mps", max_linear_},
        {"max_angular_radps", max_angular_},
        {"max_linear_accel_mps2", max_linear_accel_},
        {"max_angular_accel_radps2", max_angular_accel_},
        {"replan_step_m", replan_step_m_},
        {"replan_step_rad", replan_step_rad_},
        {"field_length_m", field_length_},
        {"field_width_m", field_width_},
        {"xy_feedback_per_m_mps", kp_xy_},
        {"yaw_feedback_per_rad_radps", kp_yaw_},
    };
    for (const auto& [key, value] : checked) {
      if (!std::isfinite(value) || value <= 0)
        BadParameter(node_, key, "must be finite and positive");
    }
  }
  void ApplyRoutine() {
    const auto& r = routines_[latched_];
    shooter_velocity_ = r.shooter;
    intake_velocity_ = r.intake;
    feed_velocity_ = r.feed;
    target_x_ = r.x;
    target_y_ = r.y;
    target_yaw_ = r.yaw;
    feed_duration_ = std::chrono::microseconds{static_cast<int>(r.feed_us)};
    spinup_timeout_s_ = r.spinup_s;
    drive_timeout_s_ = r.drive_s;
  }
  bool PrerequisiteMet(const Action& action, bool shooter_ready) const {
    switch (action.prerequisite) {
      case Prerequisite::kNone:
        return true;
      case Prerequisite::kShooterReady:
        return shooter_ready;
      case Prerequisite::kLocalizedPose:
        return pose_.fresh() && pose_->valid() && pose_->localized() &&
               UsablePose(pose_->pose());
    }
    return false;
  }
  // The one test of a pose for planning and following alike: finite, and at
  // most one field length (width) beyond the field's ends (sides), so in
  // [-L, 2L] x [-W, 2W]. Vision accepts tag estimates only on the field, so
  // that margin never refuses a real estimate's drift near a wall, while a
  // pose that is no position on this field (NaN, inf, 1e308) is unusable,
  // whatever its flags say: the drive lane is released and the DRIVE
  // deadline keeps counting. Coordinates within tens of metres also keep
  // the planner's distance and the controller's error finite.
  bool UsablePose(const geometry::Pose2d& p) const {
    return std::isfinite(p.yaw_rad()) &&
           std::abs(p.x_m() - field_length_ / 2) <= 1.5 * field_length_ &&
           std::abs(p.y_m() - field_width_ / 2) <= 1.5 * field_width_;
  }
  const Action& CurrentAction() const {
    return routines_[latched_].actions[action_];
  }
  void Advance(talos::Time now) {
    ++action_;
    Enter(now);
  }
  // Enter the current action's phase; DRIVE's deadline counts from here.
  void Enter(talos::Time now) {
    phase_.go(CurrentAction().phase);
    if (phase_.is(Phase::kDrive)) drive_started_ = now;
  }
  void SelectRoutine() {
    const auto stick = ds_.joystick(selection_port_);
    if (!stick.connected()) {
      selection_seeded_ = false;
      return;
    }
    const bool next = stick.pov(selection_pov_) == selection_next_ * 45;
    const bool previous = stick.pov(selection_pov_) == selection_previous_ * 45;
    // Any disabled mode: FMS holds the robot disabled-autonomous pre-match.
    if (selection_seeded_ && !ds_.enabled()) {
      int step =
          (next && !next_was_pressed_) - (previous && !previous_was_pressed_);
      if (step != 0) {
        const int count = static_cast<int>(routines_.size());
        selection_.set(static_cast<std::uint8_t>(
            (selection_.get() + step + count) % count));
      }
    }
    next_was_pressed_ = next;
    previous_was_pressed_ = previous;
    selection_seeded_ = true;
  }
  void PublishState() {
    // The latched routine from its arm until a new match resets it, so a
    // paused routine reports what a re-enable resumes.
    state_.send(
        AutonomousState{WirePhase(), Armed() ? latched_ : selection_.get()});
  }

  autonomous::Phase WirePhase() const {
    switch (phase_.state()) {
      case Phase::kSpinup:
        return autonomous::Phase::SPINUP;
      case Phase::kFeed:
        return autonomous::Phase::FEED;
      case Phase::kDrive:
        return autonomous::Phase::DRIVE;
      case Phase::kHold:
        return autonomous::Phase::HOLD;
      case Phase::kAborted:
        return autonomous::Phase::ABORTED;
      case Phase::kInhibited:
        return autonomous::Phase::INHIBITED;
      case Phase::kIdle:
        return autonomous::Phase::IDLE;
    }
    return autonomous::Phase::IDLE;
  }
};

}  // namespace autonomous
