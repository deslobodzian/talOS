// The 2026 game's rules (REQ-DEV-40) on the generated rebuilt_2026 field,
// through MujocoPlant: scoring once through a HUB's scoring site, the HUBs
// returning FUEL from empty exit pockets under the field's release contract,
// the shift order and grace, preloads, the match report and the Driver
// Station stamp, the random stream, repeatability and no allocation.
//
// FUEL is put where a case needs it by Placer, a test rule added before the
// game's, so the game judges what it wrote in the same step.

#include "talOS/simulation/mujoco/fields/rebuilt_2026/rules.h"

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "sim/sim_generated.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/simulation/mujoco/game.h"
#include "talOS/simulation/mujoco/plant.h"
#include "talOS/simulation/mujoco/world_rules.h"

namespace talos::simulation::mujoco {
namespace {

using Vec3 = std::array<double, 3>;
using Mode = MatchInput::Mode;

constexpr uint64_t kSecondUs = 1'000'000;
constexpr int kFuel = 504;
constexpr int kReserveFirst = 456;  // README.md "Names": reserve 456-503
constexpr int kStaged = 456;        // every FUEL but the reserve
constexpr int kDelaySteps = 200;    // 1.0 s of 5 ms steps (README.md "Rules")

std::string FieldXml() {
  std::string error;
  const std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* path = std::getenv("FIELD_XML");
  if (!runfiles || path == nullptr) {
    throw std::runtime_error("runfiles: " + error + ", FIELD_XML unset");
  }
  return runfiles->Rlocation(path);
}

// A welded robot with no geoms and the sites `preload_0` .. `preload_<n-1>`,
// 0.2 m apart, so a preload touches nothing at Attach.
std::string Robot(int preloads) {
  std::string sites;
  for (int i = 0; i < preloads; ++i) {
    const int column = i % 4;
    const int row = i / 4;
    sites += "<site name=\"preload_" + std::to_string(i) + "\" pos=\"" +
             std::to_string(0.2 * column) + " " + std::to_string(0.2 * row) +
             " 0.4\"/>";
  }
  return R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="robot" pos="0 0 0.1">
    <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>)" +
         sites + "</body></worldbody></mujoco>";
}

// The full field with the robot at (2, 1.5), in the blue alliance zone away
// from every HUB, exit and staged FUEL.
std::unique_ptr<MujocoPlant> Make(const std::string& name, int preloads = 0) {
  InstallMujocoProcessHandlers();
  PlantModel model;
  model.field_xml = FieldXml();
  model.robot_xml = ::testing::TempDir() + "/" + name + "_robot.xml";
  std::ofstream(model.robot_xml) << Robot(preloads);
  model.start_x_m = 2.0;
  model.start_y_m = 1.5;
  return MakePlant(model, {});
}

int Id(const mjModel* m, mjtObj type, const std::string& name) {
  const int id = mj_name2id(m, type, name.c_str());
  if (id < 0) throw std::logic_error("no element '" + name + "'");
  return id;
}

// The offset of row `index` in a MuJoCo array of rows `width` long.
std::ptrdiff_t Row(int width, int index) {
  return static_cast<std::ptrdiff_t>(width) * index;
}

std::string FuelName(int k) {
  std::array<char, 16> name{};
  std::snprintf(name.data(), name.size(), "fuel_%03d", k);
  return name.data();
}

int FuelQpos(const mjModel* m, int k) {
  return m->jnt_qposadr[m->body_jntadr[Id(m, mjOBJ_BODY, FuelName(k))]];
}
int FuelDof(const mjModel* m, int k) {
  return m->jnt_dofadr[m->body_jntadr[Id(m, mjOBJ_BODY, FuelName(k))]];
}
double FuelRadius(const mjModel* m) {
  return m->geom_size[Row(3, Id(m, mjOBJ_GEOM, "fuel_000_geom"))];
}

// A field site's world position: field sites are on the world body.
Vec3 SiteAt(const mjModel* m, const std::string& name) {
  const mjtNum* p = m->site_pos + Row(3, Id(m, mjOBJ_SITE, name));
  return {p[0], p[1], p[2]};
}
// A site's world position after the plant's last kinematics.
Vec3 SiteNow(const mjModel* m, const mjData* d, const std::string& name) {
  const mjtNum* p = d->site_xpos + Row(3, Id(m, mjOBJ_SITE, name));
  return {p[0], p[1], p[2]};
}
// A field site's +x axis in the world.
Vec3 SiteX(const mjModel* m, const std::string& name) {
  Vec3 x{};
  const Vec3 unit = {1.0, 0.0, 0.0};
  mju_rotVecQuat(x.data(), unit.data(),
                 m->site_quat + Row(4, Id(m, mjOBJ_SITE, name)));
  return x;
}

double Distance(const mjtNum* a, const Vec3& b) {
  return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

// Writes FUEL poses after the step it names (or after every step), at rest
// or at a velocity.
class Placer final : public WorldRules {
 public:
  struct Placement {
    uint64_t step{};  // 0: every step
    int fuel{};
    Vec3 at{};
    Vec3 velocity{};
  };
  void Add(const Placement& placement) { placements_.push_back(placement); }
  void Attach(const mjModel& model, mjData& /*data*/) override {
    resolved_.clear();
    for (const Placement& p : placements_) {
      resolved_.push_back({FuelQpos(&model, p.fuel), FuelDof(&model, p.fuel)});
    }
  }
  bool AfterStep(mjModel& /*model*/, mjData& data, uint64_t step,
                 const MatchInput& /*input*/) override {
    for (std::size_t i = 0; i < placements_.size(); ++i) {
      const Placement& p = placements_[i];
      if (p.step != 0 && p.step != step) continue;
      mjtNum* q = data.qpos + resolved_[i].qpos;
      mjtNum* v = data.qvel + resolved_[i].dof;
      for (int k = 0; k < 3; ++k) {
        q[k] = p.at[static_cast<std::size_t>(k)];
        v[k] = p.velocity[static_cast<std::size_t>(k)];
        v[3 + k] = 0.0;
      }
      q[3] = 1.0;
      q[4] = q[5] = q[6] = 0.0;
    }
    return true;
  }
  std::string_view failure() const override { return {}; }

 private:
  struct Resolved {
    int qpos{};
    int dof{};
  };
  std::vector<Placement> placements_;
  std::vector<Resolved> resolved_;
};

MatchInput Input(Mode mode, double schedule_s, bool enabled = true,
                 bool scheduled = true) {
  MatchInput input;
  input.mode = mode;
  input.enabled = enabled;
  input.scheduled = scheduled;
  input.schedule_us = static_cast<uint64_t>(std::llround(schedule_s * 1e6));
  return input;
}

sim::MatchState ReportOf(const GameRules& rules) {
  sim::MatchState match{};
  rules.Report(match);
  return match;
}

std::string MessageOf(const FmsReport& report) {
  return std::string(report.game_specific_message.data(),
                     strnlen(report.game_specific_message.data(),
                             report.game_specific_message.size()));
}

// A plant with the game and a Placer, attached in that order of running.
struct Game {
  std::unique_ptr<MujocoPlant> plant;
  Placer placer;
  std::unique_ptr<GameRules> rules;
};
void Attach(Game& game, uint64_t seed) {
  game.rules = MakeRebuilt2026Rules(GameOptions{seed});
  game.plant->AddRules(game.placer);
  game.plant->AddRules(*game.rules);
}
bool Step(Game& game, const MatchInput& input) {
  game.plant->SetMatchInput(input);
  return game.plant->Step(1, {});
}

// Three FUEL dropped into the blue HUB's funnel at step 1, while the
// Placer holds a FUEL in each of blue's exit pockets 0 to 2, so only pocket
// 3 is empty; and one FUEL put in the red scoring site moving up. Before the
// match: the HUBs score, park and return FUEL but earn no points.
constexpr std::array<int, 3> kDropped = {0, 1, 2};
constexpr int kRising = 3;
constexpr std::array<int, 3> kPinned = {10, 11, 12};
constexpr int kOpenExit = 3;
void DropScenario(Game& game) {
  const mjModel* m = game.plant->model();
  const Vec3 score = SiteAt(m, "blue_hub_score");
  for (std::size_t i = 0; i < kDropped.size(); ++i) {
    const double dy = (static_cast<double>(i) - 1.0) * 0.25;
    game.placer.Add({1, kDropped[i], {score[0], score[1] + dy, 2.0}, {}});
  }
  game.placer.Add({1, kRising, SiteAt(m, "red_hub_score"), {0.0, 0.0, 1.5}});
  for (std::size_t i = 0; i < kPinned.size(); ++i) {
    game.placer.Add(
        {0, kPinned[i], SiteAt(m, "blue_hub_exit_" + std::to_string(i)), {}});
  }
}
const MatchInput kBeforeTheMatch = Input(Mode::kNone, 0.0, false, false);

// REQ-DEV-40: a FUEL that passes down through a HUB's scoring site counts
// once for that HUB's alliance (not while it moves up, not again while it is
// in the site), is parked out of play with no velocity, and leaves the HUB
// 1.0 s later from an exit whose pocket is empty, at the exit, at 3 to 5 m/s
// within 20 degrees of the exit's +x in the horizontal plane. Before the
// match it earns no point.
TEST(Rebuilt2026Rules, ScoresOnceParksAndReleasesFromAnEmptyPocket) {
  Game game{.plant = Make("release")};
  DropScenario(game);
  Attach(game, 1);
  const mjModel* m = game.plant->model();
  const mjData* d = game.plant->data();
  const double radius = FuelRadius(m);
  const Vec3 open = SiteAt(m, "blue_hub_exit_" + std::to_string(kOpenExit));
  const Vec3 open_x = SiteX(m, "blue_hub_exit_" + std::to_string(kOpenExit));
  std::vector<Vec3> exits;
  exits.reserve(4);
  for (int e = 0; e < 4; ++e) {
    exits.push_back(SiteAt(m, "blue_hub_exit_" + std::to_string(e)));
  }

  std::array<int, kDropped.size()> scored_at{};
  std::array<int, kDropped.size()> released_at{};
  uint32_t red_scored_first_step = 1;
  for (int step = 1; step <= 340; ++step) {
    ASSERT_TRUE(Step(game, kBeforeTheMatch)) << game.plant->failure();
    const sim::MatchState match = ReportOf(*game.rules);
    if (step == 1) red_scored_first_step = match.red_scored_count();
    EXPECT_EQ(match.blue_points_count() + match.red_points_count(), 0u)
        << "step " << step;
    for (std::size_t i = 0; i < kDropped.size(); ++i) {
      SCOPED_TRACE("fuel " + std::to_string(kDropped[i]) + ", step " +
                   std::to_string(step));
      const int k = kDropped[i];
      const mjtNum* q = d->qpos + FuelQpos(m, k);
      const mjtNum* v = d->qvel + FuelDof(m, k);
      const int column = k % 80;
      const int row = k / 80;
      const Vec3 slot = {0.5 + 0.2 * column, -2.5 - 0.2 * row, radius + 0.0005};
      if (scored_at[i] == 0 && Distance(q, slot) < 1e-9) {
        scored_at[i] = step;
        for (int j = 0; j < 6; ++j) EXPECT_EQ(v[j], 0.0);
        EXPECT_GE(match.pieces_held_count(), 1u);
      }
      // Held, or in play: the reserve is untouched.
      EXPECT_EQ(match.pieces_in_play_count() + match.pieces_held_count(),
                kStaged);
      bool at_exit = false;
      for (const Vec3& exit : exits)
        at_exit = at_exit || Distance(q, exit) < 1e-9;
      if (released_at[i] != 0 || !at_exit) continue;
      released_at[i] = step;
      ASSERT_NE(scored_at[i], 0) << "released before it was scored";
      EXPECT_GE(step, scored_at[i] + kDelaySteps);
      EXPECT_LT(Distance(q, open), 1e-9) << "released into an occupied pocket";
      // The pocket was empty: no other FUEL centre within a diameter.
      for (int j = 0; j < kFuel; ++j) {
        if (j == k) continue;
        EXPECT_GE(Distance(d->qpos + FuelQpos(m, j), open), 2 * radius)
            << "fuel " << j << " was in the pocket";
      }
      const double speed = std::hypot(v[0], v[1], v[2]);
      EXPECT_GE(speed, 3.0);
      EXPECT_LE(speed, 5.0);
      EXPECT_EQ(v[2], 0.0);
      const double off_axis = std::acos(
          std::clamp((v[0] * open_x[0] + v[1] * open_x[1]) / speed, -1.0, 1.0));
      EXPECT_LE(off_axis, 20.0 * std::numbers::pi / 180.0 + 1e-12);
      for (int j = 3; j < 6; ++j) EXPECT_EQ(v[j], 0.0) << "spin";
    }
  }
  const sim::MatchState match = ReportOf(*game.rules);
  EXPECT_EQ(red_scored_first_step, 0u) << "counted moving up";
  EXPECT_EQ(match.red_scored_count(), 1u) << "the rising FUEL on its way down";
  EXPECT_EQ(match.blue_scored_count(), kDropped.size());
  for (std::size_t i = 0; i < kDropped.size(); ++i) {
    EXPECT_NE(released_at[i], 0) << "fuel " << kDropped[i] << " never released";
  }
  EXPECT_EQ(*std::min_element(released_at.begin(), released_at.end()),
            *std::min_element(scored_at.begin(), scored_at.end()) + kDelaySteps)
      << "the first release is not 1.0 s after its score";
  EXPECT_EQ(match.pieces_held_count(), 0u) << "every scored FUEL released";
  EXPECT_EQ(match.pieces_in_play_count(), kStaged);
}

// The shift a hub is inactive in, by stage (0 TRANSITION, 1-4 SHIFT, 5 END
// GAME), for the alliance inactive first (GM Table 6-3).
bool InactiveFirstIsActive(int stage) {
  return stage == 0 || stage == 2 || stage == 4 || stage == 5;
}
bool InactiveSecondIsActive(int stage) {
  return stage == 0 || stage == 1 || stage == 3 || stage == 5;
}

// REQ-DEV-40: the match's periods on the schedule clock, the HUB states of
// GM Table 6-3 and the Driver Station stamp: the alliance with more AUTO
// points is inactive in SHIFTS 1 and 3, the other in 2 and 4, and a tie is
// drawn from the seed (seed 1 draws red first, seed 3 blue); both are
// active in TRANSITION and END GAME. Test mode and an unscheduled tick start
// nothing, a disable mid-AUTO does not end it; the match time counts down
// AUTO's 20 s and TELEOP's 140 s and is 0 outside them, and from TELEOP
// start the game data names the alliance inactive first.
TEST(Rebuilt2026Rules, ShiftOrderFollowsAutoPoints) {
  struct Case {
    const char* what;
    const char* scoring_site;  // AUTO's one FUEL; null: none
    uint64_t seed;
    bool blue_inactive_first;
    bool tie;
  };
  for (const Case& row : {Case{"blue more", "blue_hub_score", 1, true, false},
                          Case{"red more", "red_hub_score", 1, false, false},
                          Case{"tie, seed 1", nullptr, 1, false, true},
                          Case{"tie, seed 3", nullptr, 3, true, true}}) {
    SCOPED_TRACE(row.what);
    Game game{.plant = Make("shifts")};
    if (row.scoring_site != nullptr) {
      game.placer.Add(
          {4, 0, SiteAt(game.plant->model(), row.scoring_site), {0, 0, -1.0}});
    }
    Attach(game, row.seed);
    const GameRules& rules = *game.rules;

    for (const MatchInput& idle :
         {Input(Mode::kAutonomous, 0.0, true, false), Input(Mode::kTest, 0.0),
          Input(Mode::kTeleop, 0.0, false)}) {
      ASSERT_TRUE(Step(game, idle));
      EXPECT_EQ(ReportOf(rules).period(), sim::MatchPeriod::None);
      EXPECT_EQ(rules.Fms().match_time_s, 0.0f);
    }
    ASSERT_TRUE(Step(game, Input(Mode::kAutonomous, 1.0)));  // AUTO at 1 s
    EXPECT_EQ(ReportOf(rules).period(), sim::MatchPeriod::Autonomous);
    ASSERT_TRUE(Step(game, Input(Mode::kAutonomous, 6.0, false)));
    sim::MatchState match = ReportOf(rules);
    EXPECT_EQ(match.period(), sim::MatchPeriod::Autonomous) << "disabled";
    EXPECT_EQ(match.period_remaining_us(), 15 * kSecondUs);
    EXPECT_EQ(rules.Fms().match_time_s, 15.0f);
    EXPECT_EQ(MessageOf(rules.Fms()), "");
    const uint32_t blue_auto = row.blue_inactive_first && !row.tie ? 1 : 0;
    const uint32_t red_auto = !row.blue_inactive_first && !row.tie ? 1 : 0;
    EXPECT_EQ(match.blue_auto_points_count(), blue_auto);
    EXPECT_EQ(match.red_auto_points_count(), red_auto);

    constexpr double kTeleopStart = 22.0;
    ASSERT_TRUE(Step(game, Input(Mode::kTeleop, kTeleopStart)));
    match = ReportOf(rules);
    EXPECT_EQ(match.period(), sim::MatchPeriod::Teleop);
    EXPECT_EQ(match.period_remaining_us(), 140 * kSecondUs);
    EXPECT_EQ(match.blue_inactive_first(), row.blue_inactive_first);
    EXPECT_EQ(match.tie_drawn(), row.tie);
    const std::string letter = row.blue_inactive_first ? "B" : "R";
    EXPECT_EQ(MessageOf(rules.Fms()), letter);

    // 5 s into each stage, past any grace.
    const std::array<double, 6> stage_start = {0, 10, 35, 60, 85, 110};
    for (int stage = 0; stage < 6; ++stage) {
      SCOPED_TRACE("stage " + std::to_string(stage));
      const double t = kTeleopStart + stage_start[stage] + 5.0;
      ASSERT_TRUE(Step(game, Input(Mode::kTeleop, t)));
      match = ReportOf(rules);
      EXPECT_EQ(match.stage_index(), stage);
      const bool first = InactiveFirstIsActive(stage);
      const bool second = InactiveSecondIsActive(stage);
      EXPECT_EQ(match.blue_scoring_active(),
                row.blue_inactive_first ? first : second);
      EXPECT_EQ(match.red_scoring_active(),
                row.blue_inactive_first ? second : first);
      EXPECT_FLOAT_EQ(rules.Fms().match_time_s,
                      static_cast<float>(140.0 - stage_start[stage] - 5.0));
    }
    ASSERT_TRUE(Step(game, Input(Mode::kTeleop, kTeleopStart + 140.0)));
    match = ReportOf(rules);
    EXPECT_EQ(match.period(), sim::MatchPeriod::Ended);
    EXPECT_EQ(rules.Fms().match_time_s, 0.0f);
    EXPECT_EQ(MessageOf(rules.Fms()), letter);
  }
}

// The points, AUTO points and scored count of each alliance.
struct Tally {
  uint32_t points;
  uint32_t auto_points;
  uint32_t scored;
};
std::pair<Tally, Tally> TallyOf(const GameRules& rules) {
  const sim::MatchState s = ReportOf(rules);
  return {
      {s.blue_points_count(), s.blue_auto_points_count(),
       s.blue_scored_count()},
      {s.red_points_count(), s.red_auto_points_count(), s.red_scored_count()}};
}

// REQ-DEV-40: a FUEL earns a point within 3 s after its HUB deactivates and
// not later, after each of the three deactivations: AUTO's end (the point is
// an AUTO point, and it decides the shift order), the start of the HUB's
// inactive shift, and TELEOP's end; scored later, it counts in `scored` and
// earns nothing. Seed 3 would draw blue inactive first on a tie, so red's
// one AUTO point, earned 2.9 s after AUTO, is what puts red first.
TEST(Rebuilt2026Rules, GraceEndsThreeSecondsAfterAHubDeactivates) {
  Game game{.plant = Make("grace")};
  const mjModel* m = game.plant->model();
  const Vec3 down = {0.0, 0.0, -1.0};
  const Vec3 blue = SiteAt(m, "blue_hub_score");
  const Vec3 red = SiteAt(m, "red_hub_score");
  game.placer.Add({3, 0, red, down});   // 2.9 s after AUTO
  game.placer.Add({4, 1, blue, down});  // 3.1 s after AUTO
  game.placer.Add({6, 2, red, down});   // 2.9 s into SHIFT 1
  game.placer.Add({7, 3, red, down});   // 3.1 s into SHIFT 1
  game.placer.Add({7, 4, blue, down});  // blue, active in SHIFT 1
  game.placer.Add({8, 5, blue, down});  // 2.9 s after TELEOP
  game.placer.Add({9, 6, red, down});   // 3.1 s after TELEOP
  Attach(game, 3);
  const GameRules& rules = *game.rules;

  ASSERT_TRUE(Step(game, Input(Mode::kAutonomous, 0.0)));
  ASSERT_TRUE(Step(game, Input(Mode::kAutonomous, 20.0)));  // AUTO's end
  ASSERT_EQ(ReportOf(rules).period(), sim::MatchPeriod::None);
  ASSERT_TRUE(Step(game, Input(Mode::kNone, 22.9, false)));
  auto [b, r] = TallyOf(rules);
  EXPECT_EQ(r.points, 1u) << "2.9 s after AUTO";
  EXPECT_EQ(r.auto_points, 1u) << "2.9 s after AUTO";
  ASSERT_TRUE(Step(game, Input(Mode::kNone, 23.1, false)));
  std::tie(b, r) = TallyOf(rules);
  EXPECT_EQ(b.scored, 1u);
  EXPECT_EQ(b.points, 0u) << "3.1 s after AUTO";
  EXPECT_EQ(b.auto_points, 0u) << "3.1 s after AUTO";

  constexpr double kTeleopStart = 25.0;
  ASSERT_TRUE(Step(game, Input(Mode::kTeleop, kTeleopStart)));
  sim::MatchState match = ReportOf(rules);
  EXPECT_FALSE(match.blue_inactive_first()) << "red's grace AUTO point";
  EXPECT_FALSE(match.tie_drawn());

  // Red, inactive first, deactivates at SHIFT 1, 10 s in.
  ASSERT_TRUE(Step(game, Input(Mode::kTeleop, kTeleopStart + 12.9)));
  std::tie(b, r) = TallyOf(rules);
  EXPECT_EQ(r.points, 2u) << "2.9 s into SHIFT 1";
  EXPECT_TRUE(ReportOf(rules).red_scoring_active());
  ASSERT_TRUE(Step(game, Input(Mode::kTeleop, kTeleopStart + 13.1)));
  std::tie(b, r) = TallyOf(rules);
  EXPECT_EQ(r.scored, 3u);
  EXPECT_EQ(r.points, 2u) << "3.1 s into SHIFT 1";
  EXPECT_FALSE(ReportOf(rules).red_scoring_active());
  EXPECT_EQ(b.points, 1u) << "the active HUB";

  ASSERT_TRUE(Step(game, Input(Mode::kTeleop, kTeleopStart + 142.9)));
  match = ReportOf(rules);
  EXPECT_EQ(match.period(), sim::MatchPeriod::Ended);
  EXPECT_EQ(match.blue_points_count(), 2u) << "2.9 s after TELEOP";
  ASSERT_TRUE(Step(game, Input(Mode::kTeleop, kTeleopStart + 143.1)));
  std::tie(b, r) = TallyOf(rules);
  EXPECT_EQ(r.scored, 4u);
  EXPECT_EQ(r.points, 2u) << "3.1 s after TELEOP";
}

// REQ-DEV-40: periods are half-open on the schedule clock: AUTO's last step
// is the one before 20 s; a step exactly 10 s after TELEOP starts is SHIFT 1
// and one exactly 140 s after is Ended; the time left counts down to the
// last step. TELEOP starting 1 s after AUTO ends AUTO's assessment: a FUEL
// 2 s after AUTO's end is a TELEOP point. An Ended match never restarts.
TEST(Rebuilt2026Rules, PeriodBoundariesAndNoRestart) {
  Game game{.plant = Make("boundaries")};
  game.placer.Add(
      {5, 0, SiteAt(game.plant->model(), "blue_hub_score"), {0, 0, -1.0}});
  Attach(game, 1);
  const GameRules& rules = *game.rules;
  struct Row {
    const char* what;
    MatchInput input;
    sim::MatchPeriod period;
    uint8_t stage;
    uint64_t remaining_us;
  };
  using P = sim::MatchPeriod;
  const std::vector<Row> rows = {
      {"AUTO starts", Input(Mode::kAutonomous, 0.0), P::Autonomous, 0,
       20 * kSecondUs},
      {"AUTO's last step", Input(Mode::kAutonomous, 19.995), P::Autonomous, 0,
       5000},
      {"AUTO's end", Input(Mode::kAutonomous, 20.0), P::None, 0, 0},
      {"TELEOP 1 s later", Input(Mode::kTeleop, 21.0), P::Teleop, 0,
       140 * kSecondUs},
      {"a FUEL 2 s after AUTO", Input(Mode::kTeleop, 22.0), P::Teleop, 0,
       139 * kSecondUs},
      {"TRANSITION's last step", Input(Mode::kTeleop, 30.995), P::Teleop, 0,
       130'005'000},
      {"SHIFT 1's first", Input(Mode::kTeleop, 31.0), P::Teleop, 1,
       130 * kSecondUs},
      {"TELEOP's last step", Input(Mode::kTeleop, 160.995), P::Teleop, 5, 5000},
      {"Ended", Input(Mode::kTeleop, 161.0), P::Ended, 0, 0},
      {"AUTO again", Input(Mode::kAutonomous, 170.0), P::Ended, 0, 0},
      {"TELEOP again", Input(Mode::kTeleop, 171.0), P::Ended, 0, 0},
  };
  for (const Row& row : rows) {
    SCOPED_TRACE(row.what);
    ASSERT_TRUE(Step(game, row.input));
    const sim::MatchState match = ReportOf(rules);
    EXPECT_EQ(match.period(), row.period);
    EXPECT_EQ(match.stage_index(), row.stage);
    EXPECT_EQ(match.period_remaining_us(), row.remaining_us);
    EXPECT_FLOAT_EQ(rules.Fms().match_time_s,
                    static_cast<float>(static_cast<double>(row.remaining_us) /
                                       static_cast<double>(kSecondUs)));
  }
  const auto [b, r] = TallyOf(rules);
  EXPECT_EQ(b.points, 1u);
  EXPECT_EQ(b.auto_points, 0u) << "AUTO's assessment ended at TELEOP start";
}

// One release's draws: the exit, the speed and the heading off the exit's +x.
struct Release {
  int exit;
  double speed_mps;
  double heading_deg;
};
// The release of `fuel`, which must be at one of `side`'s exits.
Release ReleaseOf(const MujocoPlant& plant, const std::string& side, int fuel) {
  const mjModel* m = plant.model();
  const mjtNum* q = plant.data()->qpos + FuelQpos(m, fuel);
  const mjtNum* v = plant.data()->qvel + FuelDof(m, fuel);
  for (int e = 0; e < 4; ++e) {
    const std::string exit = side + "_hub_exit_" + std::to_string(e);
    if (Distance(q, SiteAt(m, exit)) > 1e-9) continue;
    const Vec3 x = SiteX(m, exit);
    const double along = v[0] * x[0] + v[1] * x[1];
    const double across = x[0] * v[1] - x[1] * v[0];
    return {e, std::hypot(v[0], v[1], v[2]),
            std::atan2(across, along) * 180.0 / std::numbers::pi};
  }
  return {-1, 0.0, 0.0};
}

// REQ-DEV-40: the draws follow the documented order. Seed 1, blue 2 - red 1
// in AUTO (no tie, so TELEOP start draws nothing); 200 steps later both HUBs
// release in one step, blue's two FUEL (in score order) before red's, each
// the exit among the empty pockets, the speed, the heading: the first nine
// draws of seed 1's stream.
TEST(Rebuilt2026Rules, ReleaseDrawsFollowTheDocumentedOrder) {
  Game game{.plant = Make("draws")};
  const mjModel* m = game.plant->model();
  const Vec3 blue = SiteAt(m, "blue_hub_score");
  const Vec3 down = {0.0, 0.0, -1.0};
  game.placer.Add({1, 0, {blue[0] - 0.08, blue[1], blue[2]}, down});
  game.placer.Add({1, 1, {blue[0] + 0.08, blue[1], blue[2]}, down});
  game.placer.Add({1, 2, SiteAt(m, "red_hub_score"), down});
  Attach(game, 1);
  ASSERT_TRUE(Step(game, Input(Mode::kAutonomous, 0.0)));
  ASSERT_TRUE(Step(game, Input(Mode::kTeleop, 20.5)));
  const sim::MatchState match = ReportOf(*game.rules);
  ASSERT_EQ(match.blue_auto_points_count(), 2u);
  ASSERT_EQ(match.red_auto_points_count(), 1u);
  ASSERT_FALSE(match.tie_drawn());
  for (int step = 3; step <= 201; ++step) {
    ASSERT_TRUE(Step(game, Input(Mode::kTeleop, 20.5 + 0.005 * (step - 2))));
  }
  struct Golden {
    const char* side;
    int fuel;
    Release release;
  };
  for (const Golden& golden :
       {Golden{"blue", 0, {2, 4.491563514525402, 18.840110143471847}},
        Golden{"blue", 1, {1, 3.888529401652716, 10.51577567647044}},
        Golden{"red", 2, {3, 4.046134359701963, -8.579652624121334}}}) {
    SCOPED_TRACE(std::string(golden.side) + " fuel " +
                 std::to_string(golden.fuel));
    const Release got = ReleaseOf(*game.plant, golden.side, golden.fuel);
    EXPECT_EQ(got.exit, golden.release.exit);
    EXPECT_NEAR(got.speed_mps, golden.release.speed_mps, 1e-12);
    EXPECT_NEAR(got.heading_deg, golden.release.heading_deg, 1e-9);
  }
}

// REQ-DEV-40: up to 8 reserve FUEL start at the robot's preload_0 to
// preload_7, at rest, in reserve order (preload_8 and preload_9 are not
// sites the rules use); the rest stay in the reserve row, out of play.
TEST(Rebuilt2026Rules, PreloadsAtTheRobotsSites) {
  for (const auto& [sites, placed] :
       {std::pair{10, 8}, std::pair{3, 3}, std::pair{0, 0}}) {
    SCOPED_TRACE(std::to_string(sites) + " preload sites");
    Game game{.plant = Make("preload_" + std::to_string(sites), sites)};
    const mjModel* m = game.plant->model();
    std::vector<Vec3> reserve_row;
    for (int k = kReserveFirst; k < kFuel; ++k) {
      const mjtNum* q = game.plant->data()->qpos + FuelQpos(m, k);
      reserve_row.push_back({q[0], q[1], q[2]});
    }
    Attach(game, 1);
    const mjData* d = game.plant->data();
    for (int i = 0; i < kFuel - kReserveFirst; ++i) {
      SCOPED_TRACE("reserve fuel " + std::to_string(kReserveFirst + i));
      const mjtNum* q = d->qpos + FuelQpos(m, kReserveFirst + i);
      const mjtNum* v = d->qvel + FuelDof(m, kReserveFirst + i);
      const Vec3 at = i < placed ? SiteNow(m, d, "preload_" + std::to_string(i))
                                 : reserve_row[static_cast<std::size_t>(i)];
      EXPECT_LT(Distance(q, at), 1e-12);
      for (int j = 0; j < 6; ++j) EXPECT_EQ(v[j], 0.0);
    }
    const sim::MatchState match = ReportOf(*game.rules);
    EXPECT_EQ(match.pieces_in_play_count(), kStaged + placed);
    EXPECT_EQ(match.pieces_held_count(), 0u);
  }
}

// REQ-DEV-40: the game's draws are splitmix64 from the seed, whatever the
// standard library: the first outputs for seed 1, and a uniform double is
// the top 53 bits times 2^-53.
TEST(Rebuilt2026Rules, RandomStreamIsSplitMix64) {
  SplitMix64 stream(1);
  EXPECT_EQ(stream.Next(), 0x910a2dec89025cc1ULL);
  EXPECT_EQ(stream.Next(), 0xbeeb8da1658eec67ULL);
  EXPECT_EQ(stream.Next(), 0xf893a2eefb32555eULL);
  SplitMix64 uniform(1);
  EXPECT_EQ(uniform.Uniform(), 0x1.22145bd91204bp-1);
  EXPECT_EQ(uniform.Uniform(), 0x1.7dd71b42cb1ddp-1);
}

// The drop scenario under AUTO from step 1, 5 ms of schedule time a step:
// every step's MatchState bytes and the final qpos and qvel.
struct Recording {
  std::vector<sim::MatchState> matches;
  std::vector<mjtNum> qpos;
  std::vector<mjtNum> qvel;
};
constexpr int kRunSteps = 300;  // past the first release
Recording RunScenario(uint64_t seed) {
  Game game{.plant = Make("run")};
  DropScenario(game);
  Attach(game, seed);
  Recording run;
  for (int step = 0; step < kRunSteps; ++step) {
    if (!Step(game, Input(Mode::kAutonomous, 0.005 * step))) {
      throw std::logic_error(std::string(game.plant->failure()));
    }
    run.matches.push_back(ReportOf(*game.rules));
  }
  const mjModel* m = game.plant->model();
  const mjData* d = game.plant->data();
  run.qpos.assign(d->qpos, d->qpos + m->nq);
  run.qvel.assign(d->qvel, d->qvel + m->nv);
  return run;
}

// REQ-DEV-40: the same model, seed and inputs give the same match and the
// same world, bit for bit; another seed releases differently.
TEST(Rebuilt2026Rules, RunsAreBitIdenticalAndTheSeedMatters) {
  const Recording a = RunScenario(1);
  const Recording b = RunScenario(1);
  const Recording c = RunScenario(2);
  ASSERT_GE(a.matches.back().blue_points_count(), 3u);
  ASSERT_LT(a.matches.back().pieces_held_count(), 3u) << "nothing released";
  for (std::size_t i = 0; i < a.matches.size(); ++i) {
    ASSERT_EQ(std::memcmp(&a.matches[i], &b.matches[i], sizeof(a.matches[i])),
              0)
        << "step " << i + 1;
  }
  const std::size_t qpos = sizeof(mjtNum) * a.qpos.size();
  const std::size_t qvel = sizeof(mjtNum) * a.qvel.size();
  EXPECT_EQ(std::memcmp(a.qpos.data(), b.qpos.data(), qpos), 0);
  EXPECT_EQ(std::memcmp(a.qvel.data(), b.qvel.data(), qvel), 0);
  EXPECT_NE(std::memcmp(a.qvel.data(), c.qvel.data(), qvel), 0)
      << "seed 2 released as seed 1 did";
}

// Step with the game attached runs inside the gateway's tick: it allocates
// nothing, scoring, parking and releasing included.
TEST(Rebuilt2026Rules, StepAllocatesNothing) {
  if (const auto why = testing::AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  Game game{.plant = Make("allocation")};
  DropScenario(game);
  Attach(game, 1);
  const MatchInput input = Input(Mode::kAutonomous, 0.0);
  game.plant->SetMatchInput(input);
  ASSERT_TRUE(game.plant->Step(1, {}));
  testing::AllocationProbe probe;
  probe.start();
  for (int step = 1; step < kRunSteps; ++step) game.plant->Step(1, {});
  EXPECT_EQ(probe.stop(), 0u) << probe.sites();
  const sim::MatchState match = ReportOf(*game.rules);
  EXPECT_EQ(match.blue_scored_count(), kDropped.size());
  EXPECT_LT(match.pieces_held_count(), 3u) << "nothing released";
  EXPECT_TRUE(game.plant->failure().empty()) << game.plant->failure();
}

}  // namespace
}  // namespace talos::simulation::mujoco
