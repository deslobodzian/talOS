// MuJoCo as this repository builds it steps repeatably, and a run restarts
// bit-exactly from a saved state: a precondition of REQ-DEV-37, whose own test
// is the gateway's. What is asserted holds on one host, for one binary, on one
// thread, in one floating-point environment; nothing here compares hosts.

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace {

struct ModelDeleter {
  void operator()(mjModel* m) const { mj_deleteModel(m); }
};
struct DataDeleter {
  void operator()(mjData* d) const { mj_deleteData(d); }
};
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

ModelPtr Compile(const std::string& xml) {
  std::array<char, 1024> error{};
  mjSpec* spec = mj_parseXMLString(xml.c_str(), nullptr, error.data(),
                                   static_cast<int>(error.size()));
  if (spec == nullptr) {
    ADD_FAILURE() << "parse: " << error.data();
    return nullptr;
  }
  ModelPtr model(mj_compile(spec, nullptr));
  if (!model) {
    ADD_FAILURE() << "compile: " << mjs_getError(spec);
  }
  mj_deleteSpec(spec);
  return model;
}

// Six spheres on a walled carpet with rolling resistance, thrown at one
// another and at the walls, with the solver settings the gateway uses (CG,
// 5 ms, 20 iterations, elliptic cone, implicitfast); `sleep` enables
// sleeping.
std::string ContactScene(bool sleep) {
  return std::string(R"(
<mujoco>
  <option timestep="0.005" solver="CG" iterations="20" cone="elliptic"
          integrator="implicitfast" sleep_tolerance="0.03">
    <flag sleep=")") +
         (sleep ? "enable" : "disable") + R"("/>
  </option>
  <worldbody>
    <geom type="plane" size="2 2 0.1" condim="6" friction="0.8 0.001 0.01"/>
    <geom type="box" pos=" 1.05 0 0.25" size="0.05 1 0.25"/>
    <geom type="box" pos="-1.05 0 0.25" size="0.05 1 0.25"/>
    <geom type="box" pos="0  1.05 0.25" size="1 0.05 0.25"/>
    <geom type="box" pos="0 -1.05 0.25" size="1 0.05 0.25"/>
    <body pos="-0.5 0.02 0.3"><freejoint/><geom type="sphere" size="0.075" mass="0.215"/></body>
    <body pos="0.5 -0.03 0.2"><freejoint/><geom type="sphere" size="0.075" mass="0.215"/></body>
    <body pos="0 0.5 0.4"><freejoint/><geom type="sphere" size="0.075" mass="0.215"/></body>
    <body pos="0.01 -0.5 0.1"><freejoint/><geom type="sphere" size="0.075" mass="0.215"/></body>
    <body pos="0.3 0.3 0.6"><freejoint/><geom type="sphere" size="0.075" mass="0.215"/></body>
    <body pos="-0.3 -0.28 0.5"><freejoint/><geom type="sphere" size="0.075" mass="0.215"/></body>
  </worldbody>
</mujoco>)";
}

// A compiled model and its data, the spheres given their initial throw.
struct Sim {
  ModelPtr model;
  DataPtr data;
  int most_contacts = 0;
  int fewest_awake_trees = 0;
};

// Compiles the scene afresh: every Sim has its own mjModel.
Sim Start(bool sleep) {
  Sim sim;
  sim.model = Compile(ContactScene(sleep));
  if (!sim.model) {
    return sim;
  }
  sim.data.reset(mj_makeData(sim.model.get()));
  if (!sim.data) {
    ADD_FAILURE() << "mj_makeData returned null";
    return sim;
  }
  sim.fewest_awake_trees = static_cast<int>(sim.model->ntree);
  constexpr std::array<std::array<double, 2>, 6> kVelocity = {{
      {2.0, 0.1},
      {-2.0, 0.0},
      {0.0, -2.5},
      {0.1, 2.5},
      {-1.5, -1.5},
      {1.5, 1.4},
  }};
  for (int ball = 0; ball < 6; ++ball) {
    sim.data->qvel[6 * ball + 0] = kVelocity[ball][0];
    sim.data->qvel[6 * ball + 1] = kVelocity[ball][1];
  }
  return sim;
}

void Step(Sim& sim, int steps) {
  for (int i = 0; i < steps; ++i) {
    mj_step(sim.model.get(), sim.data.get());
    sim.most_contacts = std::max(sim.most_contacts, sim.data->ncon);
    sim.fewest_awake_trees =
        std::min(sim.fewest_awake_trees, sim.data->ntree_awake);
  }
}

std::vector<mjtNum> State(const Sim& sim, int signature) {
  std::vector<mjtNum> state(
      static_cast<size_t>(mj_stateSize(sim.model.get(), signature)));
  mj_getState(sim.model.get(), sim.data.get(), state.data(), signature);
  return state;
}

bool SameBytes(const std::vector<mjtNum>& a, const std::vector<mjtNum>& b) {
  if (a.size() != b.size()) return false;
  if (a.empty()) return true;
  return std::memcmp(a.data(), b.data(), a.size() * sizeof(mjtNum)) == 0;
}

bool AllFinite(const std::vector<mjtNum>& state) {
  return std::all_of(state.begin(), state.end(),
                     [](mjtNum v) { return std::isfinite(v); });
}

void ExpectHealthy(const Sim& sim) {
  EXPECT_GT(sim.data->time, 0.0);
  for (const int warning :
       {mjWARN_CONTACTFULL, mjWARN_CNSTRFULL, mjWARN_BADQPOS, mjWARN_BADQVEL,
        mjWARN_BADQACC, mjWARN_BADCTRL}) {
    EXPECT_EQ(sim.data->warning[warning].number, 0)
        << "MuJoCo warning " << warning;
  }
}

// Two compiles of one scene, stepped from one initial state, end in the same
// integration state (mjSTATE_INTEGRATION: time, qpos, qvel, act, history,
// controls and applied forces, warm start) to the last bit, and, with
// sleeping on, the same trees asleep. Sleeping is off by default in the
// gateway; the sleep-on case is kept as the harder variant, since sleeping
// adds state (which trees sleep, and when) that mjSTATE_INTEGRATION does not
// carry.
TEST(MujocoDeterminism, TwoCompilesStepIdentically) {
  for (const bool sleep : {false, true}) {
    SCOPED_TRACE(sleep ? "sleep on" : "sleep off");
    Sim first = Start(sleep);
    Sim second = Start(sleep);
    ASSERT_TRUE(first.data && second.data);
    // 5 s: the spheres collide, roll and, with sleeping on, fall asleep.
    constexpr int kSteps = 1000;
    Step(first, kSteps);
    Step(second, kSteps);
    ExpectHealthy(first);
    ExpectHealthy(second);
    // Without a collision the comparison proves only free flight; with
    // sleeping on, without a sleeping tree it proves nothing about sleeping.
    ASSERT_GT(first.most_contacts, 6)
        << "no collision in " << kSteps << " steps";
    if (sleep) {
      ASSERT_GT(first.model->ntree, 0);
      ASSERT_EQ(first.model->ntree, second.model->ntree);
      ASSERT_LT(first.fewest_awake_trees, first.model->ntree)
          << "no tree slept in " << kSteps << " steps";
      EXPECT_EQ(
          std::memcmp(first.data->tree_asleep, second.data->tree_asleep,
                      sizeof(int) * static_cast<size_t>(first.model->ntree)),
          0)
          << "sleep state differs after " << kSteps << " steps";
    }
    const std::vector<mjtNum> first_state = State(first, mjSTATE_INTEGRATION);
    ASSERT_FALSE(first_state.empty());
    ASSERT_TRUE(AllFinite(first_state))
        << "non-finite integration state after " << kSteps << " steps";
    EXPECT_TRUE(SameBytes(first_state, State(second, mjSTATE_INTEGRATION)))
        << "integration state differs after " << kSteps << " steps";
  }
}

// A run saved mid-collision with mj_getState(mjSTATE_INTEGRATION) and
// restored with mj_setState into a fresh mjData of a second compile goes on
// to the same integration state, to the last bit, as the run it was saved
// from. Restoring only mjSTATE_PHYSICS (qpos, qvel, act, history) does not:
// without the solver's warm start (qacc_warmstart) CG, stopped at 20
// iterations, takes a different path.
TEST(MujocoDeterminism, RestartFromIntegrationStateContinuesIdentically) {
  constexpr int kAfter = 400;
  Sim original = Start(false);
  ASSERT_TRUE(original.data);
  // Save at the first step with a contact beyond the six on the floor.
  int before = 0;
  while (original.data->ncon <= 6 && before < 400) {
    Step(original, 1);
    ++before;
  }
  ASSERT_GT(original.data->ncon, 6) << "no collision in " << before << " steps";
  const std::vector<mjtNum> saved = State(original, mjSTATE_INTEGRATION);
  const std::vector<mjtNum> physics = State(original, mjSTATE_PHYSICS);
  ASSERT_FALSE(saved.empty());
  ASSERT_TRUE(AllFinite(saved));

  Sim restarted = Start(false);
  Sim physics_only = Start(false);
  ASSERT_TRUE(restarted.data && physics_only.data);
  mj_setState(restarted.model.get(), restarted.data.get(), saved.data(),
              mjSTATE_INTEGRATION);
  mj_setState(physics_only.model.get(), physics_only.data.get(), physics.data(),
              mjSTATE_PHYSICS);
  physics_only.data->time = original.data->time;

  Step(original, kAfter);
  Step(restarted, kAfter);
  Step(physics_only, kAfter);
  ExpectHealthy(original);
  ExpectHealthy(restarted);
  const std::vector<mjtNum> expected = State(original, mjSTATE_INTEGRATION);
  ASSERT_FALSE(expected.empty());
  ASSERT_TRUE(AllFinite(expected));
  EXPECT_TRUE(SameBytes(State(restarted, mjSTATE_INTEGRATION), expected))
      << "restart from mjSTATE_INTEGRATION diverged after " << kAfter
      << " steps";
  // The negative control: the comparison above can fail.
  EXPECT_FALSE(SameBytes(State(physics_only, mjSTATE_INTEGRATION), expected))
      << "restart without the warm start matched; the test above would not "
         "notice a lost warm start";
}

}  // namespace
