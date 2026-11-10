#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "autonomous/autonomous_message_generated.h"
#include "autonomous/progress.h"
#include "autonomous/selection.h"
#include "autonomous/topics.h"
#include "autonomous/trajectory.h"
#include "drivetrain/drive_message_generated.h"
#include "geometry/geometry_generated.h"
#include "intake/intake_message_generated.h"
#include "intake/topics.h"
#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

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
        field_length_{node.param("field_length_m", 16.541)},
        field_width_{node.param("field_width_m", 8.069)},
        kp_xy_{node.param("xy_feedback_per_m_mps", 1.0)},
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
    if (feed_duration_ <= talos::Duration::zero()) {
      throw std::invalid_argument(
          "autonomous: feed_duration_us must be positive");
    }
    if (autonomous_period_ <= talos::Duration::zero()) {
      throw std::invalid_argument(
          "autonomous: autonomous_period_us must be positive");
    }
    if (selection_port_ < 0 || selection_port_ > 5 || selection_pov_ < 0 ||
        selection_pov_ > 3 || !PovAngle(selection_next_) ||
        !PovAngle(selection_previous_) ||
        selection_next_ == selection_previous_) {
      throw std::invalid_argument("autonomous: invalid selection input");
    }
    ValidateFinite();
    signature_ = RoutineSignature(routines_);
  }

  void update() {
    const auto now = node_.now();
    if (feeding_) feed_elapsed_ += now - feed_last_update_;
    feeding_ = false;
    // Silence is not a mode transition: preserve progress and wall deadlines,
    // but never keep commanding lanes or advance feed/trajectory time.
    if (!ds_.connected()) {
      trajectory_pose_available_ = false;
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
      if (PrerequisiteMet(CurrentAction(), shooter_ready)) {
        const auto& p = pose_->pose();
        if (!trajectory_started_) {
          auto target = TrajectorySample{0, target_x_, target_y_, target_yaw_};
          if (red_) {
            target.x_m = field_length_ - target.x_m;
            target.y_m = field_width_ - target.y_m;
            target.yaw_rad = WrapAngle(target.yaw_rad + std::numbers::pi);
          }
          trajectory_ = Trajectory::FromPoses(
              p, geometry::Pose2d{target.x_m, target.y_m, target.yaw_rad},
              max_linear_, max_angular_);
          trajectory_started_ = true;
          trajectory_elapsed_ = talos::Duration::zero();
          trajectory_last_update_ = phase_.time_in_state();
          trajectory_pose_available_ = false;
        }
        if (trajectory_pose_available_) {
          trajectory_elapsed_ +=
              phase_.time_in_state() - trajectory_last_update_;
        }
        trajectory_last_update_ = phase_.time_in_state();
        trajectory_pose_available_ = true;
        const double elapsed =
            std::chrono::duration<double>(trajectory_elapsed_).count();
        const auto desired = trajectory_.SampleAt(elapsed);
        const double dx = desired.x_m - p.x_m(), dy = desired.y_m - p.y_m();
        const double da = WrapAngle(desired.yaw_rad - p.yaw_rad());
        if (elapsed >= trajectory_.total_time_s() &&
            std::hypot(dx, dy) <= position_tolerance_ &&
            std::abs(da) <= yaw_tolerance_) {
          Advance(now);
        } else {
          drive =
              HolonomicController{kp_xy_, kp_yaw_, max_linear_, max_angular_}
                  .Calculate(desired, p);
          if (routines_[latched_].collect && intake_state_.fresh() &&
              !intake_state_->beam_broken()) {
            intake = intake_velocity_;
          }
        }
      } else {
        trajectory_pose_available_ = false;
      }
    }
    if (phase_.is(Phase::kHold)) {
      Stop();
      return;
    }
    // Write-ahead: the record holds this step before its commands go out.
    Persist();
    if (phase_.is(Phase::kDrive) && !trajectory_pose_available_) {
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
  double field_length_, field_width_, kp_xy_, kp_yaw_;
  Trajectory trajectory_;
  talos::Duration trajectory_elapsed_{};
  talos::Duration trajectory_last_update_{};
  bool trajectory_pose_available_ = false;
  bool trajectory_started_ = false;
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

  static double PositiveSeconds(talos::Node& node, const char* key,
                                double fallback) {
    const double value = node.param(key, fallback);
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument(std::string{"autonomous: "} + key +
                                  " must be positive and finite");
    }
    return value;
  }

  void ResetRoutine() {
    phase_.go(Phase::kIdle);
    generation_ = 0;
    trajectory_started_ = false;
    trajectory_pose_available_ = false;
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
    trajectory_started_ = false;
    trajectory_pose_available_ = false;
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
      throw std::invalid_argument(
          "autonomous: routines must contain 1..256 names");
    std::vector<Routine> result;
    for (const auto& name : names) {
      if (name != "preload" && name != "none" && name != "shoot_only" &&
          name != "taxi")
        throw std::invalid_argument("autonomous: unknown routine " + name);
      if (std::any_of(result.begin(), result.end(),
                      [&](const Routine& r) { return r.name == name; }))
        throw std::invalid_argument("autonomous: duplicate routine " + name);
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
      const auto param = [&](const char* key, double fallback) {
        const double value = node.param("routine." + name + "." + key,
                                        node.param(key, fallback));
        if (!std::isfinite(value))
          throw std::invalid_argument(
              "autonomous: nonfinite routine parameter");
        return value;
      };
      r.shooter = param("shooter_velocity_rps", 60);
      r.intake = param("intake_velocity_rps", 30);
      r.feed = param("feed_velocity_rps", -20);
      r.x = param("target_x_m", 2);
      r.y = param("target_y_m", 0);
      r.yaw = param("target_yaw_rad", 0);
      // feed_duration_us is an integer in the existing public configuration.
      r.feed_us = node.param<int>("routine." + name + ".feed_duration_us",
                                  node.param<int>("feed_duration_us", 500000));
      r.spinup_s = param("spinup_timeout_s", 2);
      r.drive_s = param("drive_timeout_s", 10);
      if (r.feed_us <= 0 || r.spinup_s <= 0 || r.drive_s <= 0)
        throw std::invalid_argument("autonomous: invalid routine duration");
      result.push_back(std::move(r));
    }
    return result;
  }
  static std::uint8_t DefaultRoutine(talos::Node& node,
                                     const std::vector<Routine>& routines) {
    const auto name = node.param<std::string>("default_routine", "preload");
    for (std::size_t i = 0; i < routines.size(); ++i)
      if (routines[i].name == name) return static_cast<std::uint8_t>(i);
    throw std::invalid_argument("autonomous: default routine not in routines");
  }
  static std::uint64_t RoutineSignature(const std::vector<Routine>& routines) {
    std::uint64_t hash = 14695981039346656037ULL;
    for (const auto& r : routines) {
      for (unsigned char c : r.name) hash = (hash ^ c) * 1099511628211ULL;
      hash = (hash ^ 0xff) * 1099511628211ULL;
    }
    return hash;
  }
  void ValidateFinite() const {
    for (double value :
         {position_tolerance_, yaw_tolerance_, max_linear_, max_angular_,
          field_length_, field_width_, kp_xy_, kp_yaw_}) {
      if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument("autonomous: invalid controller parameter");
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
        return pose_.fresh() && pose_->valid() && pose_->localized();
    }
    return false;
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
