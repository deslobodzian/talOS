#include "2026-robot/main_processor/shooter/node.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <string>

#include "talOS/hardware/sim_backend.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/simulated_event_loop.h"

namespace talos::shooter {
namespace {
using namespace std::chrono_literals;

config::RobotConfig TestRobotConfig() {
  auto cfg = config::ParseRobotConfig("2026-robot/main_processor/configuration/robot.toml");
  cfg.hardware.commissioned = true;
  return cfg;
}

hardware::State InitialState(hardware::SimBackend& backend,
                             const config::RobotConfig& robot_cfg,
                             uint64_t now_us = 1000, bool enabled = true) {
  hardware::Gateway gateway{robot_cfg.hardware, backend, 42};
  gateway.Tick(now_us, enabled);
  return gateway.snapshot();
}

Packet StatePacket(const hardware::State& state) {
  Packet p{};
  p.size = static_cast<uint32_t>(hardware::Encode(state, p.data));
  return p;
}

hardware::Command LastCommand(event::SimulationEnvironment& env) {
  auto& channel = env.channel(kShooterRequestTopic, sizeof(Packet));
  EXPECT_GT(channel.next_sequence(), 0);
  Packet p{};
  if (!channel.next_sequence()) return {};
  channel.copy_to(channel.next_sequence() - 1,
                  {reinterpret_cast<std::byte*>(&p), sizeof(p)});
  hardware::Command command;
  EXPECT_TRUE(hardware::Decode(p.bytes(), command));
  return command;
}

ShooterState LastShooterState(event::SimulationEnvironment& env) {
  auto& channel = env.channel(kShooterStateTopic, sizeof(ShooterState));
  EXPECT_GT(channel.next_sequence(), 0);
  ShooterState s{};
  if (!channel.next_sequence()) return {};
  channel.copy_to(channel.next_sequence() - 1,
                  {reinterpret_cast<std::byte*>(&s), sizeof(s)});
  return s;
}

TEST(Shooter, VelocityControlPublishesCommandSliceAndState) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  const auto robot_cfg = TestRobotConfig();
  const auto* devs = robot_cfg.GetDevices("shooter");
  ASSERT_NE(devs, nullptr);

  ShooterNode node{loop, robot_cfg.hardware, *devs};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  hardware::SimBackend backend;
  auto state = InitialState(backend, robot_cfg, 1000, true);

  loop.inject(kHwStateTopic, StatePacket(state));
  loop.inject(kShooterTargetTopic, ShooterTarget{80.0, 1'000'000, true});
  loop.run_for(10ms);

  auto cmd = LastCommand(env);
  EXPECT_EQ(cmd.count, 1);
  EXPECT_EQ(cmd.motors[0].id, node.flywheel_id());
  EXPECT_EQ(cmd.motors[0].mode, hardware::Mode::kVelocity);
  EXPECT_DOUBLE_EQ(cmd.motors[0].demand, 80.0);

  auto shooter_st = LastShooterState(env);
  EXPECT_DOUBLE_EQ(shooter_st.target_velocity_rps(), 80.0);
  EXPECT_TRUE(shooter_st.enabled());
}

TEST(Shooter, NeutralizationOnStaleTargetAndDisabledGateway) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  const auto robot_cfg = TestRobotConfig();
  const auto* devs = robot_cfg.GetDevices("shooter");
  ASSERT_NE(devs, nullptr);

  ShooterNode node{loop, robot_cfg.hardware, *devs};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  hardware::SimBackend backend;
  auto state = InitialState(backend, robot_cfg, 1000, true);

  loop.inject(kHwStateTopic, StatePacket(state));
  // Target issued at 1ms (1'000'000 ns)
  loop.inject(kShooterTargetTopic, ShooterTarget{80.0, 1'000'000, true});
  loop.run_for(10ms);

  // Active command in velocity mode
  EXPECT_EQ(LastCommand(env).motors[0].mode, hardware::Mode::kVelocity);

  // Advance by 110ms, surpassing command_timeout_us (100ms)
  // Keep state updated so state is not stale, but target is stale
  for (int i = 0; i < 11; ++i) {
    state.sample_time_us += 10000;
    loop.inject(kHwStateTopic, StatePacket(state));
    loop.run_for(10ms);
  }

  // Motor must now be neutralized
  auto cmd = LastCommand(env);
  EXPECT_EQ(cmd.motors[0].mode, hardware::Mode::kNeutral);
  EXPECT_DOUBLE_EQ(cmd.motors[0].demand, 0.0);

  // Also verify disabled gateway neutralizes even with a fresh target
  state.sample_time_us += 5000;
  state.flags &= ~hardware::kEnabled;
  loop.inject(kHwStateTopic, StatePacket(state));
  loop.inject(kShooterTargetTopic,
              ShooterTarget{80.0, static_cast<int64_t>(loop.monotonic_now().nanos()), true});
  loop.run_for(10ms);

  EXPECT_EQ(LastCommand(env).motors[0].mode, hardware::Mode::kNeutral);
  EXPECT_DOUBLE_EQ(LastCommand(env).motors[0].demand, 0.0);
}

TEST(Shooter, BeamBreakTelemetryAndAtSpeed) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  const auto robot_cfg = TestRobotConfig();
  const auto* devs = robot_cfg.GetDevices("shooter");
  ASSERT_NE(devs, nullptr);

  ShooterNode node{loop, robot_cfg.hardware, *devs};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  hardware::SimBackend backend;
  backend.SetDigitalInput(node.beam_break_id(), true);
  auto state = InitialState(backend, robot_cfg, 1000, true);

  loop.inject(kHwStateTopic, StatePacket(state));
  loop.inject(kShooterTargetTopic,
              ShooterTarget{80.0, static_cast<int64_t>(loop.monotonic_now().nanos()), true});
  loop.run_for(10ms);

  auto st1 = LastShooterState(env);
  EXPECT_TRUE(st1.beam_broken());

  // Beam un-broken
  backend.SetDigitalInput(node.beam_break_id(), false);
  state = InitialState(backend, robot_cfg, 15000, true);
  loop.inject(kHwStateTopic, StatePacket(state));
  loop.run_for(10ms);

  auto st2 = LastShooterState(env);
  EXPECT_FALSE(st2.beam_broken());

  // Flywheel velocity and at_speed telemetry
  // When flywheel is at 80 rps and target is 80 rps -> at_speed is true
  for (std::size_t i = 0; i < state.motor_count; ++i) {
    if (state.motors[i].id == node.flywheel_id()) {
      state.motors[i].velocity_rps = 80.0;
      break;
    }
  }
  state.sample_time_us += 5000;
  loop.inject(kHwStateTopic, StatePacket(state));
  loop.inject(kShooterTargetTopic,
              ShooterTarget{80.0, static_cast<int64_t>(loop.monotonic_now().nanos()), true});
  loop.run_for(10ms);

  auto st3 = LastShooterState(env);
  EXPECT_DOUBLE_EQ(st3.flywheel_velocity_rps(), 80.0);
  EXPECT_TRUE(st3.at_speed());

  // When flywheel is at 20 rps and target is 80 rps -> at_speed is false
  for (std::size_t i = 0; i < state.motor_count; ++i) {
    if (state.motors[i].id == node.flywheel_id()) {
      state.motors[i].velocity_rps = 20.0;
      break;
    }
  }
  state.sample_time_us += 5000;
  loop.inject(kHwStateTopic, StatePacket(state));
  loop.inject(kShooterTargetTopic,
              ShooterTarget{80.0, static_cast<int64_t>(loop.monotonic_now().nanos()), true});
  loop.run_for(10ms);

  auto st4 = LastShooterState(env);
  EXPECT_DOUBLE_EQ(st4.flywheel_velocity_rps(), 20.0);
  EXPECT_FALSE(st4.at_speed());
}

TEST(Shooter, RecordedNetworkInputsReplayWithoutHardware) {
  const std::string path =
      ::testing::TempDir() + "/shooter_" + std::to_string(::getpid()) + ".tlog";
  const auto robot_cfg = TestRobotConfig();
  const auto* devs = robot_cfg.GetDevices("shooter");
  ASSERT_NE(devs, nullptr);

  hardware::SimBackend backend;
  auto state = InitialState(backend, robot_cfg, 1000, true);

  {
    event::SimulationEnvironment env;
    event::SimulatedEventLoop<event::log::LogWriter> loop{
        env, event::log::LogWriter{path, "shooter"}};
    ShooterNode node{loop, robot_cfg.hardware, *devs};
    node.Start(event::MonotonicTime::from_nanos(1'000'000));
    loop.inject(kHwStateTopic, StatePacket(state));
    loop.inject(kShooterTargetTopic,
                ShooterTarget{80.0, 1'000'000, true});
    loop.run_for(20ms);
    loop.finish();
    ASSERT_FALSE(loop.recorder().failed());
    EXPECT_EQ(LastCommand(env).motors[0].mode, hardware::Mode::kVelocity);
  }

  event::log::LogReader reader{path};
  event::ReplayEventLoop<> replay{reader};
  ShooterNode node{replay, robot_cfg.hardware, *devs};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));
  EXPECT_NO_THROW(replay.run());
  EXPECT_FALSE(replay.diverged());
  EXPECT_TRUE(replay.reached_exit());
  std::remove(path.c_str());
}

}  // namespace
}  // namespace talos::shooter
