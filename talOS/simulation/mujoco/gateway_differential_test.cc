// REQ-DEV-37: a motor a model actuator binds moves as PhysicsSimBackend's
// analytic rigid-body mechanism of the same inertia does. Each case runs two
// Gateways tick for tick on explicit 5 ms sample times with the same
// commands: one over the analytic backend with a [[simulation.mechanisms]]
// inertia, one over the backend with a MujocoPlant holding one hinge of that
// inertia (armature 0), and compares the States' motor samples.
//
// Both sides share the backend's controller (modes, gains, follower and
// neutral handling), the motor type's constants (k_t, k_e, R, b_m) and the
// supply and stator-limit solve, so these cases do not check those. What
// differs is the integrator (MuJoCo's implicit 5 ms step against the
// analytic exact solution with regime events) and what the plant adds on its
// side of the seam: the gear G through the actuator, the compiled no-load
// drag G^2*b_m, the torque clamp at every step and the rotor's direction.
//
// Speeds are compared in rotor rps against the Kraken X60's 12 V free speed
// (100 rps), positions in rotor rotations; J is the rotor-side inertia, the
// hinge's being G^2 * J. The bounds are the design note's (plant-design.md,
// slice E): a voltage transient lags by about e^-1 * h / (2 tau) of a step,
// and a position leads by |w| h / 2. What is different by design is listed
// in README.md "Intended differences from --physics" and not compared here.

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include "talOS/hardware/config.h"
#include "talOS/hardware/gateway.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/sim_physics.h"
#include "talOS/hardware/sim_plant.h"
#include "talOS/simulation/mujoco/plant.h"

namespace talos::simulation::mujoco {
namespace {

using hardware::Config;
using hardware::Gateway;
using hardware::Mode;
using hardware::MotorRequest;
using hardware::MotorSample;
using hardware::sim::PhysicsSimBackend;

constexpr uint64_t kTickUs = 5000;
constexpr double kH = static_cast<double>(kTickUs) * 1e-6;  // seconds
constexpr double kFreeRps = 100.0;  // Kraken X60 at 12 V, rotor rps

Config OneMotor(double gear, bool inverted) {
  Config config;
  config.commissioned = true;
  config.command_timeout_us = 100000;
  config.period_us = kTickUs;
  hardware::MotorConfig m;
  m.id = 1;
  m.name = "m";
  m.can_id = 1;
  m.bus = "rio";
  m.sensor_to_mechanism_ratio = gear;
  m.inverted = inverted;
  // Out of the way unless a case sets them: only the winding law differs.
  m.supply_limit_a = 1000.0;
  m.stator_limit_a = 1000.0;
  config.motors.push_back(m);
  return config;
}

std::string HingeModel(double inertia, const std::string& joint_extra) {
  const std::string path =
      ::testing::TempDir() + "/differential_" +
      std::to_string(static_cast<long long>(inertia * 1e9)) + ".xml";
  std::ofstream(path) << R"(<mujoco><compiler angle="radian"/><worldbody>
  <body name="base" pos="0 0 0.5">
    <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    <body name="shaft">
      <joint name="shaft_joint" type="hinge" axis="0 0 1" )"
                      << joint_extra << R"(/>
      <inertial pos="0 0 0" mass="1" diaginertia=")"
                      << inertia << " " << inertia << " " << inertia << R"("/>
    </body>
  </body>
</worldbody>
<actuator><motor name="m" joint="shaft_joint"/></actuator></mujoco>)";
  return path;
}

// One Gateway over one backend, ticked and commanded every 5 ms.
struct Rig {
  PhysicsSimBackend backend;
  std::unique_ptr<MujocoPlant> plant;
  std::unique_ptr<Gateway> gateway;
  uint64_t now_us = 1000;
  uint64_t sequence = 1;

  void Start(const Config& config) {
    backend.SetLocalMotorTypes(config);
    if (plant) backend.SetPlant(plant.get());
    gateway = std::make_unique<Gateway>(config, backend, 7);
    ASSERT_FALSE(gateway->config_error().has_value());
    gateway->Tick(now_us, true);
  }

  // Sends `request` for the next tick, then ticks.
  MotorSample Step(const MotorRequest& request) {
    const hardware::State snap = gateway->snapshot();
    hardware::Command command{};
    command.config_id = snap.config_id;
    command.boot_id = snap.boot_id;
    command.epoch = snap.epoch;
    command.observed_time_us = snap.sample_time_us;
    command.count = 1;
    command.motors[0] = request;
    EXPECT_TRUE(gateway->Accept(command, sequence++, now_us));
    now_us += kTickUs;
    gateway->Tick(now_us, true);
    return gateway->snapshot().motors[0];
  }
};

struct Phase {
  MotorRequest request;
  int ticks;
};

struct Trace {
  std::vector<MotorSample> analytic;
  std::vector<MotorSample> plant;
  // The force MuJoCo applied through the actuator in the tick's last step
  // (mechanism N*m).
  std::vector<double> actuator_force;
};

// Runs `phases` on both rigs. J is the rotor-side inertia.
Trace RunBoth(const Config& config, double rotor_inertia,
              const std::vector<Phase>& phases,
              const std::string& joint_extra = "") {
  const double gear = config.motors[0].sensor_to_mechanism_ratio;
  const double mechanism_inertia = gear * gear * rotor_inertia;
  Rig analytic;
  analytic.backend.SetMechanisms({{.motor = "m",
                                   .inertia_kg_m2 = mechanism_inertia,
                                   .load = hardware::sim::LoadKind::kRoller}});
  analytic.Start(config);
  Rig plant;
  const hardware::sim::MotorModel motor =
      hardware::sim::MotorModelFor(config.motors[0].motor_type.value);
  const hardware::sim::PlantMotor plant_motor{
      "m",
      gear,
      motor.torque_constant_nm_per_a(),
      motor.back_emf_v_per_radps(),
      motor.resistance_ohm(),
      motor.no_load_damping_nm_per_radps()};
  InstallMujocoProcessHandlers();
  plant.plant =
      MakePlant({.robot_xml = HingeModel(mechanism_inertia, joint_extra)},
                std::span(&plant_motor, 1));
  plant.Start(config);
  Trace trace;
  for (const Phase& phase : phases) {
    for (int t = 0; t < phase.ticks; ++t) {
      trace.analytic.push_back(analytic.Step(phase.request));
      trace.plant.push_back(plant.Step(phase.request));
      trace.actuator_force.push_back(plant.plant->data()->actuator_force[0]);
    }
  }
  return trace;
}

// In the rotor's frame: mechanism units times the gear.
double RotorRps(const MotorSample& s, double gear) {
  return s.velocity_rps * gear;
}
double RotorRot(const MotorSample& s, double gear) {
  return s.position_rot * gear;
}

MotorRequest Volts(double volts) { return {1, Mode::kVoltage, 0, volts, 0.0}; }
MotorRequest Neutral() { return {1, Mode::kNeutral, 0, 0.0, 0.0}; }

// Each tick's rotor speeds within `bound` rps, from tick `from` on.
void ExpectSpeedsWithin(const Trace& trace, double gear, double bound,
                        std::size_t from = 0) {
  for (std::size_t t = from; t < trace.analytic.size(); ++t) {
    ASSERT_TRUE(trace.plant[t].valid) << "tick " << t;
    ASSERT_NEAR(RotorRps(trace.plant[t], gear),
                RotorRps(trace.analytic[t], gear), bound)
        << "tick " << t << ", bound " << bound << " rps";
  }
}

// 12 V from rest: every tick within 0.5 % of free speed and the steady
// state within 0.1 % (a plant missing the no-load drag G^2*b_m is 0.55 %
// fast there); positions within |w| h / 2 + 0.5 %. G = 5 tells G from G^2,
// and an inverted motor its direction.
TEST(MujocoGatewayDifferential, VoltageSpinUpMatchesTheAnalyticModel) {
  constexpr double kJ = 0.005;
  struct Case {
    double gear;
    bool inverted;
  };
  for (const Case c : {Case{1.0, false}, Case{5.0, false}, Case{1.0, true}}) {
    SCOPED_TRACE("G = " + std::to_string(c.gear) +
                 (c.inverted ? ", inverted" : ""));
    const Trace trace = RunBoth(OneMotor(c.gear, c.inverted), kJ,
                                {{Volts(12.0), 600}});  // 3 s, about 7 tau
    ExpectSpeedsWithin(trace, c.gear, 0.005 * kFreeRps);
    EXPECT_NEAR(RotorRps(trace.plant.back(), c.gear),
                RotorRps(trace.analytic.back(), c.gear), 0.001 * kFreeRps);
    EXPECT_GT(RotorRps(trace.plant.back(), c.gear), 0.99 * kFreeRps);
    for (std::size_t t = 0; t < trace.analytic.size(); ++t) {
      const double rps = RotorRps(trace.analytic[t], c.gear);
      const double rot = RotorRot(trace.analytic[t], c.gear);
      ASSERT_NEAR(RotorRot(trace.plant[t], c.gear), rot,
                  std::abs(rps) * kH / 2.0 + 0.005 * std::abs(rot))
          << "tick " << t;
    }
  }
}

// The backend's velocity loop closes on the plant's speed as on the
// analytic one's: within 0.5 % of free speed every tick.
TEST(MujocoGatewayDifferential, VelocityLoopMatchesTheAnalyticModel) {
  for (const double gear : {1.0, 5.0}) {
    SCOPED_TRACE("G = " + std::to_string(gear));
    Config config = OneMotor(gear, false);
    config.motors[0].slots[0].p = 0.5 * gear;
    config.motors[0].slots[0].v = 0.12 * gear;  // 12 V per 100 rotor rps
    const Trace trace = RunBoth(
        config, 0.005, {{{1, Mode::kVelocity, 0, 50.0 / gear, 0.0}, 600}});
    ExpectSpeedsWithin(trace, gear, 0.005 * kFreeRps);
    EXPECT_NEAR(RotorRps(trace.plant.back(), gear), 50.0, 0.05 * 50.0);
  }
}

// 40 A of torque current accelerates at k_t*40/J on both: the slope within
// 1 % at J = 0.005 and 0.0005 (a plant that held the tick-start voltage
// instead of clamping each step is 5.6 % short at 0.0005).
TEST(MujocoGatewayDifferential, TorqueCurrentSlopeMatchesTheAnalyticModel) {
  struct Case {
    double inertia;
    int ticks;  // well below the 40 A ceiling exit, ~90 rps
  };
  for (const Case c : {Case{0.005, 200}, Case{0.0005, 40}}) {
    SCOPED_TRACE("J = " + std::to_string(c.inertia));
    Config config = OneMotor(1.0, false);
    config.motors[0].foc = true;
    config.motors[0].max_torque_current_a = 80.0;
    const Trace trace =
        RunBoth(config, c.inertia,
                {{{1, Mode::kTorqueCurrent, 0, 40.0, 0.0}, c.ticks}});
    const std::size_t a = 1;
    const std::size_t b = trace.plant.size() - 1;
    const double plant_slope =
        RotorRps(trace.plant[b], 1.0) - RotorRps(trace.plant[a], 1.0);
    const double analytic_slope =
        RotorRps(trace.analytic[b], 1.0) - RotorRps(trace.analytic[a], 1.0);
    ASSERT_GT(analytic_slope, 0.0);
    EXPECT_LT(RotorRps(trace.analytic[b], 1.0), 80.0);
    EXPECT_NEAR(plant_slope / analytic_slope, 1.0, 0.01)
        << "plant " << plant_slope << " rps, analytic " << analytic_slope
        << " rps over ticks " << a << ".." << b;
  }
}

// From free speed to neutral: a shorted winding (brake) within 0.5 % of free
// speed every tick, an open one (coast) within 0.1 %; swapped, they differ
// by most of the free speed.
TEST(MujocoGatewayDifferential, BrakeAndCoastMatchTheAnalyticModel) {
  for (const bool brake : {true, false}) {
    SCOPED_TRACE(brake ? "brake" : "coast");
    Config config = OneMotor(1.0, false);
    config.motors[0].brake = brake;
    const Trace trace =
        RunBoth(config, 0.005, {{Volts(12.0), 600}, {Neutral(), 200}});
    ExpectSpeedsWithin(trace, 1.0, (brake ? 0.005 : 0.001) * kFreeRps, 600);
  }
}

// A J = 2e-5 roller with armature 0 is below h * b_em / 2: its transient is
// not the analytic one, but it rises monotonically (an explicit torque held
// for the tick would oscillate and diverge) to the analytic steady state
// within 0.1 %.
TEST(MujocoGatewayDifferential, LightRollerIsStableAndSettles) {
  const Trace trace = RunBoth(OneMotor(1.0, false), 2e-5, {{Volts(12.0), 100}});
  for (std::size_t t = 1; t < trace.plant.size(); ++t) {
    ASSERT_GE(RotorRps(trace.plant[t], 1.0), RotorRps(trace.plant[t - 1], 1.0))
        << "tick " << t;
  }
  EXPECT_NEAR(RotorRps(trace.plant.back(), 1.0),
              RotorRps(trace.analytic.back(), 1.0), 0.001 * kFreeRps);
}

// The speed changes within a tick while the regime holds the tick-start
// decision: under a binding 40 A supply limit the limited voltage is held
// for the tick, so the current falls by k_e*dw/R within it. The design
// note's bound for that regime is h / (2 tau) of free speed.
TEST(MujocoGatewayDifferential, SupplyLimitedSpinUpMatchesTheAnalyticModel) {
  constexpr double kJ = 0.005;
  constexpr double kLimitA = 40.0;
  Config config = OneMotor(1.0, false);
  config.motors[0].supply_limit_a = kLimitA;
  const Trace trace = RunBoth(config, kJ, {{Volts(12.0), 600}});
  // In the supply-limited regime: the analytic side draws the limit for the
  // first half second of the spin-up.
  for (std::size_t t = 1; t <= 100; ++t) {
    ASSERT_NEAR(trace.analytic[t].supply_current_a, kLimitA, 0.01 * kLimitA)
        << "tick " << t;
  }
  const hardware::sim::MotorModel motor =
      hardware::sim::MotorModelFor(hardware::MotorType::kKrakenX60);
  const double drag = motor.torque_constant_nm_per_a() *
                          motor.back_emf_v_per_radps() /
                          motor.resistance_ohm() +
                      motor.no_load_damping_nm_per_radps();
  const double tau_s = kJ / drag;
  ExpectSpeedsWithin(trace, 1.0, kH / (2.0 * tau_s) * kFreeRps);
}

// A torque-current demand drives the joint into its range and holds it
// there: the torque MuJoCo applies never exceeds k_t times the demand,
// accelerating or stalled against the limit (where the unclamped winding
// law would give the stall torque), nor does the reported stator current,
// and the joint stops at its limit.
TEST(MujocoGatewayDifferential, TorqueCurrentNeverExceedsItsDemand) {
  constexpr double kDemandA = 20.0;
  const double kt =
      hardware::sim::MotorModelFor(hardware::MotorType::kKrakenX60)
          .torque_constant_nm_per_a();
  Config config = OneMotor(1.0, false);
  config.motors[0].foc = true;
  config.motors[0].max_torque_current_a = 80.0;
  const Trace trace = RunBoth(
      config, 0.005, {{{1, Mode::kTorqueCurrent, 0, kDemandA, 0.0}, 200}},
      R"(range="0 3")");
  for (std::size_t t = 0; t < trace.plant.size(); ++t) {
    ASSERT_LE(trace.plant[t].stator_current_a, kDemandA + 1e-9) << "tick " << t;
    ASSERT_LE(trace.actuator_force[t], kDemandA * kt * (1.0 + 1e-12))
        << "tick " << t;
  }
  // 3 rad is 0.477 rotations: reached, and held there.
  EXPECT_NEAR(trace.plant.back().position_rot, 3.0 / (2.0 * std::numbers::pi),
              0.01);
  EXPECT_NEAR(trace.plant.back().velocity_rps, 0.0, 0.05);
}

}  // namespace
}  // namespace talos::simulation::mujoco
