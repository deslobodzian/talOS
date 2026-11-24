#include "talOS/simulation/mujoco/fields/rebuilt_2026/rules.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace talos::simulation::mujoco {

namespace {

// README.md "Rules" gives each number's source.
constexpr uint64_t kSecondUs = 1'000'000;
constexpr uint64_t kAutoUs = 20 * kSecondUs;
constexpr uint64_t kTeleopUs = 140 * kSecondUs;
constexpr uint64_t kTransitionUs = 10 * kSecondUs;
constexpr uint64_t kShiftUs = 25 * kSecondUs;
constexpr int kShifts = 4;
constexpr int kEndGameStage = kShifts + 1;
constexpr uint64_t kGraceUs = 3 * kSecondUs;
constexpr double kReleaseDelayS = 1.0;
constexpr double kMinReleaseMps = 3.0;
constexpr double kMaxReleaseMps = 5.0;
constexpr double kMaxReleaseYawRad = 20.0 * std::numbers::pi / 180.0;
constexpr int kMaxPreloads = 8;
constexpr int kExits = 4;
constexpr uint32_t kAllExits = (1U << kExits) - 1U;
// Parking slots, outside the field beyond the reserve row.
constexpr double kParkX0M = 0.5;
constexpr double kParkY0M = -2.5;
constexpr double kParkPitchM = 0.2;
constexpr int kParkPerRow = 80;
constexpr double kParkLiftM = 0.0005;

constexpr std::array<std::string_view, 6> kRanges = {
    "neutral", "blue_depot", "red_depot", "blue_chute", "red_chute", "reserve"};
constexpr std::string_view kReserve = "reserve";
constexpr std::array<std::string_view, 2> kAlliances = {"blue", "red"};
constexpr std::size_t kBlue = 0;
constexpr std::size_t kRed = 1;

template <typename T>
T* At(T* array, int stride, int index) {
  return array + static_cast<std::ptrdiff_t>(stride) * index;
}

[[noreturn]] void Refuse(const std::string& message) {
  throw std::invalid_argument("rebuilt_2026 rules: " + message);
}

int Site(const mjModel& m, const std::string& name) {
  const int id = mj_name2id(&m, mjOBJ_SITE, name.c_str());
  if (id < 0) Refuse("the model has no site '" + name + "'");
  if (m.site_bodyid[id] != 0) {
    Refuse("site '" + name + "' is not on the field's world body");
  }
  return id;
}

// A FUEL range's `fuel_<range>_<what>` numeric: one non-negative integer.
int RangeNumeric(const mjModel& m, std::string_view range,
                 std::string_view what) {
  const std::string name =
      "fuel_" + std::string(range) + "_" + std::string(what);
  const int id = mj_name2id(&m, mjOBJ_NUMERIC, name.c_str());
  if (id < 0) Refuse("the model has no numeric '" + name + "'");
  const double value = m.numeric_data[m.numeric_adr[id]];
  if (m.numeric_size[id] != 1 || !(value >= 0.0) || value > 65535.0 ||
      value != std::floor(value)) {
    Refuse("numeric '" + name + "' is not one integer in 0 to 65535");
  }
  return static_cast<int>(value);
}

enum class Status : uint8_t { kInPlay, kHeld, kReserve };
enum class Phase : uint8_t { kBefore, kAuto, kBetween, kTeleop, kEnded };

class Rebuilt2026Rules final : public GameRules {
 public:
  explicit Rebuilt2026Rules(const GameOptions& options)
      : random_(options.seed) {}

  void Attach(const mjModel& model, mjData& data) override;
  bool AfterStep(mjModel& model, mjData& data, uint64_t step,
                 const MatchInput& input) override;
  // Never fails.
  std::string_view failure() const override { return {}; }
  void Report(::sim::MatchState& match) const override;
  FmsReport Fms() const override;

 private:
  struct Fuel {
    int qpos{-1};  // its free joint's qpos and dof addresses
    int dof{-1};
  };
  struct Hub {
    std::array<double, 3> score_at{};  // the scoring cylinder, world frame
    std::array<double, 9> score_axes{};
    double score_radius_m{};
    double score_half_height_m{};
    std::array<std::array<double, 3>, kExits> exit_at{};
    // Each exit's +x in the horizontal plane, unit length.
    std::array<std::array<double, 2>, kExits> exit_heading{};
    // Scored FUEL awaiting release, oldest first: a ring of FUEL indices as
    // long as there are FUEL, so it never overflows.
    std::vector<uint16_t> waiting;
    std::size_t head{};
    std::size_t size{};
    uint32_t scored{};
    uint32_t points{};
    uint32_t auto_points{};
  };

  bool InScoringSite(const Hub& hub, const mjtNum* centre) const;
  void Advance(const MatchInput& input);
  void StartTeleop(uint64_t now_us);
  int Stage(uint64_t now_us) const;
  bool InactiveFirst(std::size_t alliance) const {
    return alliance == kBlue ? blue_inactive_first_ : !blue_inactive_first_;
  }
  // Whether a FUEL scored now in `alliance`'s HUB earns a point.
  bool Counts(std::size_t alliance) const;
  void Park(mjData& data, std::size_t fuel) const;
  void Score(mjData& data, std::size_t alliance, std::size_t fuel,
             uint64_t step);
  uint32_t Occupied(const Hub& hub, const mjData& data) const;
  void Release(Hub& hub, mjData& data, uint64_t step);
  void SetStatus(std::size_t fuel, Status status);

  SplitMix64 random_;
  std::vector<Fuel> fuel_;
  std::vector<Status> status_;
  std::vector<uint64_t> release_step_;  // by FUEL index, while held
  std::array<Hub, 2> hubs_{};
  double radius_m_{};
  uint64_t delay_steps_{};
  uint32_t in_play_{};
  uint32_t held_{};

  Phase phase_{Phase::kBefore};
  uint64_t now_us_{};
  uint64_t auto_start_us_{};
  uint64_t auto_end_us_{};
  uint64_t teleop_start_us_{};
  bool teleop_started_{false};
  bool blue_inactive_first_{false};
  bool tie_drawn_{false};
};

void Rebuilt2026Rules::Attach(const mjModel& model, mjData& data) {
  // The FUEL, by index, from the field's range numerics.
  int count = 0;
  int reserve_first = 0;
  int reserve_count = 0;
  for (const std::string_view range : kRanges) {
    const int first = RangeNumeric(model, range, "first");
    const int size = RangeNumeric(model, range, "count");
    count = std::max(count, first + size);
    if (range == kReserve) {
      reserve_first = first;
      reserve_count = size;
    }
  }
  if (count > 65535) Refuse("more than 65535 FUEL");
  fuel_.assign(static_cast<std::size_t>(count), Fuel{});
  status_.assign(fuel_.size(), Status::kInPlay);
  release_step_.assign(fuel_.size(), 0);
  for (int k = 0; k < count; ++k) {
    std::array<char, 16> name{};
    std::snprintf(name.data(), name.size(), "fuel_%03d", k);
    const int body = mj_name2id(&model, mjOBJ_BODY, name.data());
    if (body < 0)
      Refuse("the model has no FUEL body '" + std::string(name.data()) + "'");
    if (model.body_jntnum[body] != 1 ||
        model.jnt_type[model.body_jntadr[body]] != mjJNT_FREE ||
        model.body_geomnum[body] < 1) {
      Refuse("FUEL body '" + std::string(name.data()) +
             "' is not a free body with one free joint and a geom");
    }
    const int joint = model.body_jntadr[body];
    fuel_[static_cast<std::size_t>(k)] = {model.jnt_qposadr[joint],
                                          model.jnt_dofadr[joint]};
    if (k == 0) radius_m_ = model.geom_rbound[model.body_geomadr[body]];
  }
  for (int k = reserve_first; k < reserve_first + reserve_count; ++k) {
    status_[static_cast<std::size_t>(k)] = Status::kReserve;
  }

  for (std::size_t a = 0; a < hubs_.size(); ++a) {
    Hub& hub = hubs_[a];
    const std::string side(kAlliances[a]);
    const int score = Site(model, side + "_hub_score");
    if (model.site_type[score] != mjGEOM_CYLINDER) {
      Refuse("site '" + side + "_hub_score' is not a cylinder");
    }
    std::copy_n(At(data.site_xpos, 3, score), 3, hub.score_at.begin());
    std::copy_n(At(data.site_xmat, 9, score), 9, hub.score_axes.begin());
    hub.score_radius_m = At(model.site_size, 3, score)[0];
    hub.score_half_height_m = At(model.site_size, 3, score)[1];
    for (int e = 0; e < kExits; ++e) {
      const std::string name = side + "_hub_exit_" + std::to_string(e);
      const int exit = Site(model, name);
      const mjtNum* at = At(data.site_xpos, 3, exit);
      const mjtNum* axes = At(data.site_xmat, 9, exit);
      const double hx = axes[0];
      const double hy = axes[3];
      const double norm = std::hypot(hx, hy);
      if (!(norm > 1e-6)) Refuse("site '" + name + "' has a vertical +x axis");
      const auto index = static_cast<std::size_t>(e);
      hub.exit_at[index] = {at[0], at[1], at[2]};
      hub.exit_heading[index] = {hx / norm, hy / norm};
    }
    hub.waiting.assign(fuel_.size(), 0);
  }
  const double step_s = model.opt.timestep;
  delay_steps_ =
      static_cast<uint64_t>(std::ceil(kReleaseDelayS / step_s - 1e-9));

  // Preloads: reserve FUEL, in order, at the robot's preload_0..7, at rest.
  int placed = 0;
  for (int i = 0; i < kMaxPreloads && placed < reserve_count; ++i) {
    const std::string name = "preload_" + std::to_string(i);
    const int site = mj_name2id(&model, mjOBJ_SITE, name.c_str());
    if (site < 0) continue;
    const auto k = static_cast<std::size_t>(reserve_first) +
                   static_cast<std::size_t>(placed);
    mjtNum* q = data.qpos + fuel_[k].qpos;
    const mjtNum* at = At(data.site_xpos, 3, site);
    std::copy_n(at, 3, q);
    q[3] = 1.0;
    q[4] = q[5] = q[6] = 0.0;
    std::fill_n(data.qvel + fuel_[k].dof, 6, 0.0);
    status_[k] = Status::kInPlay;
    ++placed;
  }
  in_play_ = static_cast<uint32_t>(
      std::count(status_.begin(), status_.end(), Status::kInPlay));
  held_ = 0;
}

bool Rebuilt2026Rules::InScoringSite(const Hub& hub,
                                     const mjtNum* centre) const {
  const std::array<double, 3> d = {centre[0] - hub.score_at[0],
                                   centre[1] - hub.score_at[1],
                                   centre[2] - hub.score_at[2]};
  const auto& r = hub.score_axes;  // row-major, local to world
  const double x = r[0] * d[0] + r[3] * d[1] + r[6] * d[2];
  const double y = r[1] * d[0] + r[4] * d[1] + r[7] * d[2];
  const double z = r[2] * d[0] + r[5] * d[1] + r[8] * d[2];
  return std::fabs(z) <= hub.score_half_height_m &&
         x * x + y * y < hub.score_radius_m * hub.score_radius_m;
}

void Rebuilt2026Rules::StartTeleop(uint64_t now_us) {
  phase_ = Phase::kTeleop;
  teleop_start_us_ = now_us;
  teleop_started_ = true;
  const uint32_t blue = hubs_[kBlue].auto_points;
  const uint32_t red = hubs_[kRed].auto_points;
  if (blue == red) {
    // A tie: one draw, at TELEOP start.
    tie_drawn_ = true;
    blue_inactive_first_ = random_.Uniform() < 0.5;
  } else {
    blue_inactive_first_ = blue > red;
  }
}

void Rebuilt2026Rules::Advance(const MatchInput& input) {
  now_us_ = input.schedule_us;
  const bool starts = input.scheduled && input.enabled;
  using Mode = MatchInput::Mode;
  switch (phase_) {
    case Phase::kBefore:
      if (starts && input.mode == Mode::kAutonomous) {
        phase_ = Phase::kAuto;
        auto_start_us_ = now_us_;
      } else if (starts && input.mode == Mode::kTeleop) {
        StartTeleop(now_us_);
      }
      break;
    case Phase::kAuto:
      if (input.mode != Mode::kAutonomous ||
          now_us_ >= auto_start_us_ + kAutoUs) {
        auto_end_us_ = std::min(now_us_, auto_start_us_ + kAutoUs);
        phase_ = Phase::kBetween;
        if (starts && input.mode == Mode::kTeleop) StartTeleop(now_us_);
      }
      break;
    case Phase::kBetween:
      if (starts && input.mode == Mode::kTeleop) StartTeleop(now_us_);
      break;
    case Phase::kTeleop:
      if (now_us_ >= teleop_start_us_ + kTeleopUs) phase_ = Phase::kEnded;
      break;
    case Phase::kEnded:
      break;
  }
}

int Rebuilt2026Rules::Stage(uint64_t now_us) const {
  const uint64_t elapsed =
      now_us > teleop_start_us_ ? now_us - teleop_start_us_ : 0;
  if (elapsed < kTransitionUs) return 0;
  const uint64_t into_shifts = elapsed - kTransitionUs;
  if (into_shifts >= kShifts * kShiftUs) return kEndGameStage;
  return 1 + static_cast<int>(into_shifts / kShiftUs);
}

bool Rebuilt2026Rules::Counts(std::size_t alliance) const {
  switch (phase_) {
    case Phase::kBefore:
      return false;
    case Phase::kAuto:
      return true;
    case Phase::kBetween:
      return now_us_ < auto_end_us_ + kGraceUs;
    case Phase::kTeleop: {
      const int stage = Stage(now_us_);
      const bool inactive = stage >= 1 && stage <= kShifts &&
                            (stage % 2 == 1) == InactiveFirst(alliance);
      if (!inactive) return true;
      // The HUB deactivated when this shift began, active before it.
      const uint64_t deactivated = teleop_start_us_ + kTransitionUs +
                                   static_cast<uint64_t>(stage - 1) * kShiftUs;
      return now_us_ < deactivated + kGraceUs;
    }
    case Phase::kEnded:
      return now_us_ < teleop_start_us_ + kTeleopUs + kGraceUs;
  }
  return false;
}

void Rebuilt2026Rules::SetStatus(std::size_t fuel, Status status) {
  const Status was = status_[fuel];
  if (was == status) return;
  if (was == Status::kInPlay) --in_play_;
  if (was == Status::kHeld) --held_;
  if (status == Status::kInPlay) ++in_play_;
  if (status == Status::kHeld) ++held_;
  status_[fuel] = status;
}

void Rebuilt2026Rules::Park(mjData& data, std::size_t fuel) const {
  const int k = static_cast<int>(fuel);
  mjtNum* q = data.qpos + fuel_[fuel].qpos;
  const int column = k % kParkPerRow;
  const int row = k / kParkPerRow;
  q[0] = kParkX0M + kParkPitchM * column;
  q[1] = kParkY0M - kParkPitchM * row;
  q[2] = radius_m_ + kParkLiftM;
  q[3] = 1.0;
  q[4] = q[5] = q[6] = 0.0;
  std::fill_n(data.qvel + fuel_[fuel].dof, 6, 0.0);
}

void Rebuilt2026Rules::Score(mjData& data, std::size_t alliance,
                             std::size_t fuel, uint64_t step) {
  Hub& hub = hubs_[alliance];
  ++hub.scored;
  if (Counts(alliance)) {
    ++hub.points;
    // AUTO's points: in AUTO and its grace, before TELEOP starts.
    if (phase_ == Phase::kAuto || phase_ == Phase::kBetween) {
      ++hub.auto_points;
    }
  }
  Park(data, fuel);
  SetStatus(fuel, Status::kHeld);
  release_step_[fuel] = step + delay_steps_;
  hub.waiting[(hub.head + hub.size) % hub.waiting.size()] =
      static_cast<uint16_t>(fuel);
  ++hub.size;
}

uint32_t Rebuilt2026Rules::Occupied(const Hub& hub, const mjData& data) const {
  const double clear = 2.0 * radius_m_;
  uint32_t occupied = 0;
  for (const Fuel& fuel : fuel_) {
    const mjtNum* p = data.qpos + fuel.qpos;
    for (int e = 0; e < kExits; ++e) {
      const auto& at = hub.exit_at[static_cast<std::size_t>(e)];
      const double dx = p[0] - at[0];
      const double dy = p[1] - at[1];
      const double dz = p[2] - at[2];
      if (dx * dx + dy * dy + dz * dz < clear * clear) occupied |= 1U << e;
    }
  }
  return occupied;
}

void Rebuilt2026Rules::Release(Hub& hub, mjData& data, uint64_t step) {
  uint32_t occupied = 0;
  bool looked = false;
  while (hub.size > 0 && release_step_[hub.waiting[hub.head]] <= step) {
    if (!looked) {
      occupied = Occupied(hub, data);
      looked = true;
    }
    const uint32_t empty = ~occupied & kAllExits;
    const int choices = std::popcount(empty);
    if (choices == 0) return;  // waits for a pocket to empty
    // Draws, in this order: the exit among the empty ones (in exit order),
    // the speed, the heading.
    int pick =
        std::min(static_cast<int>(random_.Uniform() * choices), choices - 1);
    int exit = 0;
    for (; exit < kExits; ++exit) {
      if ((empty & (1U << exit)) != 0 && pick-- == 0) break;
    }
    const double speed =
        kMinReleaseMps + (kMaxReleaseMps - kMinReleaseMps) * random_.Uniform();
    const double yaw = (2.0 * random_.Uniform() - 1.0) * kMaxReleaseYawRad;

    const std::size_t fuel = hub.waiting[hub.head];
    hub.head = (hub.head + 1) % hub.waiting.size();
    --hub.size;
    const auto e = static_cast<std::size_t>(exit);
    const auto& at = hub.exit_at[e];
    const auto& heading = hub.exit_heading[e];
    mjtNum* q = data.qpos + fuel_[fuel].qpos;
    q[0] = at[0];
    q[1] = at[1];
    q[2] = at[2];
    q[3] = 1.0;
    q[4] = q[5] = q[6] = 0.0;
    mjtNum* v = data.qvel + fuel_[fuel].dof;
    std::fill_n(v, 6, 0.0);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    v[0] = speed * (c * heading[0] - s * heading[1]);
    v[1] = speed * (s * heading[0] + c * heading[1]);
    SetStatus(fuel, Status::kInPlay);
    occupied |= 1U << exit;
  }
}

bool Rebuilt2026Rules::AfterStep(mjModel& /*model*/, mjData& data,
                                 uint64_t step, const MatchInput& input) {
  Advance(input);
  // Scoring: a FUEL not held whose centre is in a scoring site, moving down,
  // in FUEL index order; the qpos and qvel a rule before this one wrote.
  for (std::size_t k = 0; k < fuel_.size(); ++k) {
    if (status_[k] == Status::kHeld) continue;
    if (!(data.qvel[fuel_[k].dof + 2] < 0.0)) continue;
    const mjtNum* centre = data.qpos + fuel_[k].qpos;
    for (std::size_t a = 0; a < hubs_.size(); ++a) {
      if (InScoringSite(hubs_[a], centre)) {
        Score(data, a, k, step);
        break;
      }
    }
  }
  for (Hub& hub : hubs_) Release(hub, data, step);
  return true;
}

void Rebuilt2026Rules::Report(::sim::MatchState& match) const {
  uint64_t remaining_us = 0;
  ::sim::MatchPeriod period = ::sim::MatchPeriod::None;
  switch (phase_) {
    case Phase::kBefore:
    case Phase::kBetween:
      break;
    case Phase::kAuto:
      period = ::sim::MatchPeriod::Autonomous;
      remaining_us = auto_start_us_ + kAutoUs - now_us_;
      break;
    case Phase::kTeleop:
      period = ::sim::MatchPeriod::Teleop;
      remaining_us = teleop_start_us_ + kTeleopUs - now_us_;
      break;
    case Phase::kEnded:
      period = ::sim::MatchPeriod::Ended;
      break;
  }
  const Hub& blue = hubs_[kBlue];
  const Hub& red = hubs_[kRed];
  match = ::sim::MatchState(
      match.sample_time_ns(), now_us_, remaining_us, blue.points, red.points,
      blue.auto_points, red.auto_points, blue.scored, red.scored,
      /*shots_count=*/0, static_cast<uint16_t>(in_play_),
      static_cast<uint16_t>(held_), period,
      static_cast<uint8_t>(phase_ == Phase::kTeleop ? Stage(now_us_) : 0),
      Counts(kBlue), Counts(kRed), teleop_started_ && blue_inactive_first_,
      tie_drawn_);
}

FmsReport Rebuilt2026Rules::Fms() const {
  ::sim::MatchState match{};
  Report(match);
  FmsReport report;
  report.match_time_s =
      static_cast<float>(static_cast<double>(match.period_remaining_us()) /
                         static_cast<double>(kSecondUs));
  if (teleop_started_) {
    report.game_specific_message[0] = blue_inactive_first_ ? 'B' : 'R';
  }
  return report;
}

}  // namespace

std::unique_ptr<GameRules> MakeRebuilt2026Rules(const GameOptions& options) {
  return std::make_unique<Rebuilt2026Rules>(options);
}

}  // namespace talos::simulation::mujoco
