#include "talOS/node_api/node_api.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

constexpr const char* kTopic = "/test/state/node_api";
constexpr uint32_t kBytes = 8;
constexpr uint32_t kAlign = 4;

TEST(NodeApi, AbiVersion) { EXPECT_EQ(talos_abi_version(), 6u); }

TEST(NodeApi, GatePredicatesMatchGateHeader) {
  // Same semantics as gate_test.cc, through the C ABI: first sample gated
  // on config only, stale/duplicates dropped, gateway bits, strict timeout.
  // Next sample: config 7, boot 3, time 1000.
  EXPECT_EQ(talos_accept_state(7, 0, 0, 0, 7, 3, 1000), 1);
  EXPECT_EQ(talos_accept_state(0, 0, 0, 0, 7, 3, 1000), 1);
  EXPECT_EQ(talos_accept_state(99, 0, 0, 0, 7, 3, 1000), 0);
  // Newer sample accepted; older-or-equal from the same boot dropped.
  EXPECT_EQ(talos_accept_state(7, 1, 3, 1000, 7, 3, 2000), 1);
  EXPECT_EQ(talos_accept_state(7, 1, 3, 2000, 7, 3, 1000), 0);
  EXPECT_EQ(talos_accept_state(7, 1, 3, 1000, 7, 3, 1000), 0);
  // New boot restarts the clock: older timestamps accepted again.
  EXPECT_EQ(talos_accept_state(7, 1, 3, 2000, 7, 4, 1000), 1);
  // Wrong config still rejected even when newer.
  EXPECT_EQ(talos_accept_state(7, 1, 3, 1000, 8, 3, 2000), 0);

  EXPECT_EQ(talos_gateway_ready(0x3), 1);
  EXPECT_EQ(talos_gateway_ready(0x0), 0);
  EXPECT_EQ(talos_gateway_ready(0x3 | 0x8), 0);

  // now=5s, issued 1ms ago, state just received, 100ms timeout: fresh.
  const int64_t now = 5000000000LL;
  EXPECT_EQ(talos_target_fresh_ns(now, now - 1000000LL, 1, now, 100000), 1);
  EXPECT_EQ(talos_target_fresh_ns(now, now - 1000000LL, 0, now, 100000), 0);
  // Future-issued targets cannot drive.
  EXPECT_EQ(talos_target_fresh_ns(now, now + 1, 1, now, 100000), 0);
  // Exactly one timeout old is already gone (strict on both timeouts).
  EXPECT_EQ(talos_target_fresh_ns(now, now - 100000LL * 1000LL, 1, now, 100000),
            0);
  EXPECT_EQ(talos_target_fresh_ns(now, now - 1000000LL, 1,
                                  now - 100000LL * 1000LL, 100000),
            0);
}

TEST(NodeApi, RtmsLayoutMatchesFrozenGeometry) {
  EXPECT_EQ(talos_rtms_layout(nullptr), 0u);
  TalosRtmsLayout layout{};
  EXPECT_EQ(talos_rtms_layout(&layout), sizeof(TalosRtmsLayout));
  // Frozen contract: sizeof(RTMSHeader)==640, writer@64, readers@128/64.
  EXPECT_EQ(layout.header_size, 640u);
  EXPECT_EQ(layout.off_writer_seq, 64u);
  EXPECT_EQ(layout.off_readers, 128u);
  EXPECT_EQ(layout.reader_stride, 64u);
}

TEST(NodeApi, MonotonicIncreases) {
  const int64_t a = talos_monotonic_ns();
  const int64_t b = talos_monotonic_ns();
  EXPECT_LE(a, b);
  EXPECT_GT(a, 0);
}

TEST(NodeApi, BadArgs) {
  EXPECT_EQ(talos_publish(nullptr, nullptr, 0), TALOS_ERR_ARG);
  EXPECT_EQ(talos_poll_next(nullptr, nullptr, 0, nullptr, nullptr),
            TALOS_ERR_ARG);
  EXPECT_TRUE(talos_topic_open_publisher(nullptr, kBytes, kAlign) == nullptr);
  EXPECT_TRUE(talos_topic_open_publisher("", kBytes, kAlign) == nullptr);
  EXPECT_TRUE(talos_topic_open_publisher(kTopic, 0, kAlign) == nullptr);
  talos_publisher_close(nullptr);
  talos_subscriber_close(nullptr);
}

TEST(NodeApi, PublishPollRoundTrip) {
  TalosPublisher* pub = talos_topic_open_publisher(kTopic, kBytes, kAlign);
  ASSERT_NE(pub, nullptr) << talos_last_error();
  TalosSubscriber* sub = talos_topic_open_subscriber(kTopic, kBytes, kAlign);
  ASSERT_NE(sub, nullptr) << talos_last_error();

  const uint64_t payload = 0x0102030405060708ULL;
  EXPECT_EQ(talos_publish(pub, &payload, kBytes), TALOS_OK);
  // Wrong size rejected.
  EXPECT_EQ(talos_publish(pub, &payload, kBytes - 1), TALOS_ERR_ARG);

  uint64_t got = 0;
  uint64_t seq = 0, dropped = 0;
  EXPECT_EQ(talos_poll_next(sub, &got, sizeof(got), &seq, &dropped), TALOS_OK);
  EXPECT_EQ(got, payload);
  EXPECT_EQ(talos_poll_next(sub, &got, sizeof(got), nullptr, nullptr),
            TALOS_EMPTY);
  // Small buffer rejected.
  EXPECT_EQ(talos_poll_next(sub, &got, 1, nullptr, nullptr), TALOS_ERR_ARG);

  talos_publisher_close(pub);
  talos_subscriber_close(sub);
}

TEST(NodeApi, DescribeEmitMatchesGroundTruth) {
  const TalosSource sources[] = {
      {TALOS_SOURCE_SENDER, "/test/state/x", 8, 0},
      {TALOS_SOURCE_WATCHER, "/test/state/y", 4, 1},
  };
  uint32_t need = 0;
  // Size probe first.
  EXPECT_EQ(talos_describe_emit("mynode", "//pkg:mynode", sources, 2, nullptr,
                                0, &need),
            TALOS_ERR_SMALL);
  ASSERT_GT(need, 0u);
  std::vector<char> buf(need + 1);
  uint32_t written = 0;
  EXPECT_EQ(
      talos_describe_emit("mynode", "//pkg:mynode", sources, 2, buf.data(),
                          static_cast<uint32_t>(buf.size()), &written),
      TALOS_OK);
  EXPECT_EQ(written, need);
  const std::string json(buf.data());
  EXPECT_NE(json.find("\"name\": \"mynode\""), std::string::npos);
  EXPECT_NE(json.find("\"describe_version\": 1"), std::string::npos);
  EXPECT_NE(json.find("/test/state/x"), std::string::npos);
  EXPECT_NE(json.find("\"external\": true"), std::string::npos);

  // Too-small buffer reports need.
  std::vector<char> tiny(4);
  uint32_t need2 = 0;
  EXPECT_EQ(
      talos_describe_emit("mynode", "//pkg:mynode", sources, 2, tiny.data(),
                          static_cast<uint32_t>(tiny.size()), &need2),
      TALOS_ERR_SMALL);
  EXPECT_EQ(need2, need);

  // Bad kind rejected; Python never formats the envelope itself.
  const TalosSource bad[] = {{99, "/test/state/z", 8, 0}};
  EXPECT_EQ(talos_describe_emit("mynode", "//pkg:mynode", bad, 1, buf.data(),
                                static_cast<uint32_t>(buf.size()), nullptr),
            TALOS_ERR_ARG);
}

TEST(NodeApi, RegisterPublishHeartbeatClose) {
  EXPECT_TRUE(talos_node_register(nullptr, "//pkg:mynode", 0, 0) == nullptr);
  EXPECT_TRUE(talos_node_register("", "//pkg:mynode", 0, 0) == nullptr);
  EXPECT_EQ(talos_node_publish(nullptr, nullptr, 0), TALOS_ERR_ARG);
  EXPECT_EQ(talos_node_heartbeat(nullptr, 0), TALOS_ERR_ARG);
  talos_node_close(nullptr);

  TalosNode* node = talos_node_register(
      "node_api_test", "//talOS/node_api:node_api_test", 4242u, 1u);
  ASSERT_NE(node, nullptr) << talos_last_error();
  const TalosSource sources[] = {
      {TALOS_SOURCE_TIMER, "tick", 0, 0},
      {TALOS_SOURCE_SENDER, "/test/state/node_api_reg", 8, 0},
  };
  EXPECT_EQ(talos_node_publish(node, sources, 2), TALOS_OK);
  EXPECT_EQ(talos_node_publish(node, nullptr, 1), TALOS_ERR_ARG);
  EXPECT_EQ(talos_node_heartbeat(node, 1), TALOS_OK);
  EXPECT_EQ(talos_node_heartbeat(node, 2), TALOS_OK);
  talos_node_close(node);
}

TEST(NodeApi, ConfigLifecycleAndQueries) {
  // Bad file path and invalid args
  EXPECT_EQ(talos_config_load(nullptr, 0), nullptr);
  EXPECT_STRNE(talos_last_error(), "");

  EXPECT_EQ(talos_config_load("", 0), nullptr);
  EXPECT_STRNE(talos_last_error(), "");

  EXPECT_EQ(talos_config_load("/nonexistent/path/to/robot.toml", 0), nullptr);
  EXPECT_STRNE(talos_last_error(), "");

  // Create a local test config file
  namespace fs = std::filesystem;
  const fs::path test_config_file =
      fs::temp_directory_path() / "talos_node_api_test_config.toml";
  {
    std::ofstream out(test_config_file);
    ASSERT_TRUE(out.is_open());
    out << R"(
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.drive.motors.front_left]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.drive.sensors.pigeon]
type = "Pigeon2"
bus = "rio"
can_id = 2

[subsystems.intake.digital_inputs.limit_switch]
dio = 3

[subsystems.intake.digital_outputs.solenoid]
dio = 4

[subsystems.intake.analog_inputs.pressure]
analog = 0

[subsystems.intake.encoders.quad]
type = "QuadratureEncoder"
dio_a = 5
dio_b = 6

[subsystems.intake.pwm_outputs.servo]
channel = 1
)";
  }

  // Load with simulation = 0
  TalosConfig* cfg = talos_config_load(test_config_file.c_str(), 0);
  ASSERT_NE(cfg, nullptr) << talos_last_error();

  // Query period and command timeout
  EXPECT_EQ(talos_config_get_period_us(cfg), 5000u);
  EXPECT_EQ(talos_config_get_command_timeout_us(cfg), 100000u);

  // Query config ID
  const uint64_t config_id_live = talos_config_get_id(cfg);
  EXPECT_NE(config_id_live, 0u);

  // Query device IDs across motors, sensors, digital_inputs, digital_outputs,
  // analog_inputs, encoders, pwm_outputs.
  const uint32_t motor_id =
      talos_config_get_device_id(cfg, "drive", "front_left");
  EXPECT_GT(motor_id, 0u);

  const uint32_t sensor_id =
      talos_config_get_device_id(cfg, "drive", "pigeon");
  EXPECT_GT(sensor_id, 0u);

  const uint32_t di_id =
      talos_config_get_device_id(cfg, "intake", "limit_switch");
  EXPECT_GT(di_id, 0u);

  const uint32_t do_id =
      talos_config_get_device_id(cfg, "intake", "solenoid");
  EXPECT_GT(do_id, 0u);

  const uint32_t ai_id =
      talos_config_get_device_id(cfg, "intake", "pressure");
  EXPECT_GT(ai_id, 0u);

  const uint32_t enc_id =
      talos_config_get_device_id(cfg, "intake", "quad");
  EXPECT_GT(enc_id, 0u);

  const uint32_t pwm_id =
      talos_config_get_device_id(cfg, "intake", "servo");
  EXPECT_GT(pwm_id, 0u);

  // Missing device in valid subsystem
  EXPECT_EQ(talos_config_get_device_id(cfg, "drive", "nonexistent"), 0u);
  EXPECT_NE(std::string(talos_last_error()).find("not found"),
            std::string::npos);

  // Missing subsystem
  EXPECT_EQ(talos_config_get_device_id(cfg, "nonexistent", "front_left"), 0u);
  EXPECT_NE(std::string(talos_last_error()).find("subsystem not found"),
            std::string::npos);

  // Device queried under wrong subsystem
  EXPECT_EQ(talos_config_get_device_id(cfg, "drive", "limit_switch"), 0u);

  // Null / bad arguments to getters
  EXPECT_EQ(talos_config_get_id(nullptr), 0u);
  EXPECT_EQ(talos_config_get_period_us(nullptr), 0u);
  EXPECT_EQ(talos_config_get_command_timeout_us(nullptr), 0u);
  EXPECT_EQ(talos_config_get_device_id(nullptr, "drive", "front_left"), 0u);
  EXPECT_EQ(talos_config_get_device_id(cfg, nullptr, "front_left"), 0u);
  EXPECT_EQ(talos_config_get_device_id(cfg, "", "front_left"), 0u);
  EXPECT_EQ(talos_config_get_device_id(cfg, "drive", nullptr), 0u);
  EXPECT_EQ(talos_config_get_device_id(cfg, "drive", ""), 0u);

  talos_config_close(cfg);
  talos_config_close(nullptr);

  // Load with simulation = 1: sets hardware.commissioned = true, changing config ID.
  TalosConfig* sim_cfg = talos_config_load(test_config_file.c_str(), 1);
  ASSERT_NE(sim_cfg, nullptr) << talos_last_error();
  const uint64_t config_id_sim = talos_config_get_id(sim_cfg);
  EXPECT_NE(config_id_sim, 0u);
  EXPECT_NE(config_id_sim, config_id_live);
  talos_config_close(sim_cfg);

  fs::remove(test_config_file);

  // Standalone subsystem mode test:
  const std::string standalone_file =
      (fs::temp_directory_path() / "talos_node_api_subsystem_test.toml").string();
  {
    std::ofstream out(standalone_file);
    out << "[subsystem]\n"
        << "name = \"intake\"\n"
        << "period_us = 20000\n\n"
        << "[motors.roller]\n"
        << "type = \"TalonFX\"\n"
        << "bus = \"rio\"\n"
        << "can_id = 10\n\n"
        << "[digital_inputs.intake_beam]\n"
        << "dio = 1\n";
  }

  TalosConfig* sub_cfg = talos_config_load(standalone_file.c_str(), 0);
  ASSERT_NE(sub_cfg, nullptr) << talos_last_error();
  EXPECT_EQ(talos_config_get_period_us(sub_cfg), 20000u);
  // intake_beam < roller sorted: 1, 2
  EXPECT_EQ(talos_config_get_device_id(sub_cfg, "intake", "intake_beam"), 1u);
  EXPECT_EQ(talos_config_get_device_id(sub_cfg, "intake", "roller"), 2u);
  talos_config_close(sub_cfg);
  fs::remove(standalone_file);
}

TEST(NodeApi, RealtimeLoopLifecycleAndDispatch) {
  // Invalid arguments
  EXPECT_EQ(talos_loop_run(nullptr), TALOS_ERR_ARG);
  EXPECT_EQ(talos_loop_arm_timer(nullptr, 0, 0, 0), TALOS_ERR_ARG);
  EXPECT_EQ(talos_loop_disarm_timer(nullptr, 0), TALOS_ERR_ARG);
  EXPECT_EQ(talos_loop_send(nullptr, 0, nullptr, 0), TALOS_ERR_ARG);
  EXPECT_EQ(talos_loop_now_ns(nullptr), 0);
  EXPECT_EQ(talos_loop_dispatch_count(nullptr), 0u);
  EXPECT_EQ(talos_loop_diverged(nullptr), 0);
  EXPECT_EQ(talos_loop_reached_exit(nullptr), 0);

  // Create realtime loop with NullRecorder (no log path)
  TalosLoop* loop = talos_loop_create_realtime(nullptr, nullptr, 0);
  ASSERT_NE(loop, nullptr) << talos_last_error();

  // Attach reporter
  EXPECT_EQ(talos_loop_attach_reporter(loop, "rt_node", "//talOS/node_api:node_api_test", 100u, 0), TALOS_OK);

  // Register sender
  constexpr const char* kTxTopic = "/test/telemetry/node_api";
  const uint16_t sender_id =
      talos_loop_register_sender(loop, kTxTopic, sizeof(uint64_t), alignof(uint64_t));
  EXPECT_NE(sender_id, 0xFFFF);

  // Open subscriber to observe messages sent via the loop
  TalosSubscriber* sub =
      talos_topic_open_subscriber(kTxTopic, sizeof(uint64_t), alignof(uint64_t));
  ASSERT_NE(sub, nullptr) << talos_last_error();

  struct RtContext {
    TalosLoop* loop{nullptr};
    uint16_t sender_id{0};
    int timer_ticks{0};
  };
  RtContext ctx{loop, sender_id, 0};

  auto timer_fn = [](const TalosDispatchContext* context, const void* data,
                     uint32_t size, void* user_data) {
    auto* c = static_cast<RtContext*>(user_data);
    c->timer_ticks++;
    uint64_t msg = 0xAA000000ULL + c->timer_ticks;
    talos_loop_send(c->loop, c->sender_id, &msg, sizeof(msg));
    if (c->timer_ticks >= 3) {
      talos_loop_exit(c->loop);
    }
  };

  // Register timer armed at 1ms offset with 1ms period
  const uint16_t timer_id = talos_loop_register_timer(
      loop, "rt_timer", 1'000'000, 1'000'000, timer_fn, &ctx);
  EXPECT_NE(timer_id, 0xFFFF);

  // Run the loop until clean exit requested by handler
  EXPECT_EQ(talos_loop_run(loop), TALOS_OK);

  EXPECT_GE(ctx.timer_ticks, 3);
  EXPECT_GE(talos_loop_dispatch_count(loop), 3u);
  EXPECT_GT(talos_loop_now_ns(loop), 0);
  EXPECT_EQ(talos_loop_diverged(loop), 0);

  // Verify subscriber received messages
  uint64_t received = 0;
  EXPECT_EQ(talos_poll_next(sub, &received, sizeof(received), nullptr, nullptr),
            TALOS_OK);
  EXPECT_GE(received, 0xAA000001ULL);

  talos_subscriber_close(sub);
  talos_loop_destroy(loop);
}

TEST(NodeApi, SimulatedLoopAndReplayDeterminism) {
  namespace fs = std::filesystem;
  const std::string sim_log_path =
      (fs::temp_directory_path() /
       ("talos_node_api_sim_" + std::to_string(::getpid()) + ".tlog"))
          .string();

  constexpr const char* kSensorTopic = "/test/sim/sensor";
  constexpr const char* kCmdTopic = "/test/sim/cmd";

  // 1. Simulated loop creation and execution with log recording
  {
    TalosLoop* sim_loop =
        talos_loop_create_simulated(sim_log_path.c_str(), "sim_robot");
    ASSERT_NE(sim_loop, nullptr) << talos_last_error();

    // sim_inject on invalid args
    EXPECT_EQ(talos_loop_sim_inject(nullptr, kSensorTopic, nullptr, 0),
              TALOS_ERR_ARG);
    EXPECT_EQ(talos_loop_sim_inject(sim_loop, "", nullptr, 0), TALOS_ERR_ARG);

    // Register sender and watcher
    const uint16_t sender_id = talos_loop_register_sender(
        sim_loop, kCmdTopic, sizeof(uint32_t), alignof(uint32_t));
    EXPECT_NE(sender_id, 0xFFFF);

    struct SimContext {
      TalosLoop* loop{nullptr};
      uint16_t sender_id{0};
      int watcher_dispatches{0};
      uint32_t last_sensor{0};
      int timer_ticks{0};
    };
    SimContext ctx{sim_loop, sender_id, 0, 0, 0};

    auto watcher_fn = [](const TalosDispatchContext* context, const void* data,
                         uint32_t size, void* user_data) {
      auto* c = static_cast<SimContext*>(user_data);
      c->watcher_dispatches++;
      if (size >= sizeof(uint32_t)) {
        std::memcpy(&c->last_sensor, data, sizeof(uint32_t));
      }
      uint32_t cmd = c->last_sensor * 2;
      talos_loop_send(c->loop, c->sender_id, &cmd, sizeof(cmd));
    };

    const uint16_t watcher_id = talos_loop_register_watcher(
        sim_loop, kSensorTopic, sizeof(uint32_t), alignof(uint32_t),
        watcher_fn, &ctx);
    EXPECT_NE(watcher_id, 0xFFFF);

    auto timer_fn = [](const TalosDispatchContext* context, const void* data,
                       uint32_t size, void* user_data) {
      auto* c = static_cast<SimContext*>(user_data);
      c->timer_ticks++;
    };

    const uint16_t timer_id = talos_loop_register_timer(
        sim_loop, "sim_periodic", 10'000'000, 10'000'000, timer_fn, &ctx);
    EXPECT_NE(timer_id, 0xFFFF);

    // Inject sensor message 1 and run_for 25ms
    uint32_t sensor1 = 42;
    EXPECT_EQ(talos_loop_sim_inject(sim_loop, kSensorTopic, &sensor1,
                                   sizeof(sensor1)),
              TALOS_OK);

    EXPECT_EQ(talos_loop_run_for(sim_loop, 25'000'000), TALOS_OK);
    EXPECT_EQ(ctx.watcher_dispatches, 1);
    EXPECT_EQ(ctx.last_sensor, 42u);
    EXPECT_EQ(ctx.timer_ticks, 2);

    // Inject sensor message 2 and run_for 10ms
    uint32_t sensor2 = 100;
    EXPECT_EQ(talos_loop_sim_inject(sim_loop, kSensorTopic, &sensor2,
                                   sizeof(sensor2)),
              TALOS_OK);

    EXPECT_EQ(talos_loop_run_for(sim_loop, 10'000'000), TALOS_OK);
    EXPECT_EQ(ctx.watcher_dispatches, 2);
    EXPECT_EQ(ctx.last_sensor, 100u);
    EXPECT_EQ(ctx.timer_ticks, 3);

    // Destroy simulated loop (flushes LogWriter)
    talos_loop_destroy(sim_loop);
  }

  // Verify log file was written
  ASSERT_TRUE(fs::exists(sim_log_path));
  EXPECT_GT(fs::file_size(sim_log_path), 0u);

  // 2. Replay execution and clean exit from the simulated log
  {
    TalosLoop* replay_loop = talos_loop_create_replay(sim_log_path.c_str());
    ASSERT_NE(replay_loop, nullptr) << talos_last_error();

    // sim_inject not supported on replay loop
    uint32_t dummy = 0;
    EXPECT_EQ(talos_loop_sim_inject(replay_loop, kSensorTopic, &dummy, sizeof(dummy)),
              TALOS_ERR);

    const uint16_t sender_id = talos_loop_register_sender(
        replay_loop, kCmdTopic, sizeof(uint32_t), alignof(uint32_t));
    EXPECT_NE(sender_id, 0xFFFF);

    struct ReplayContext {
      TalosLoop* loop{nullptr};
      uint16_t sender_id{0};
      int watcher_dispatches{0};
      uint32_t last_sensor{0};
      int timer_ticks{0};
    };
    ReplayContext ctx{replay_loop, sender_id, 0, 0, 0};

    auto watcher_fn = [](const TalosDispatchContext* context, const void* data,
                         uint32_t size, void* user_data) {
      auto* c = static_cast<ReplayContext*>(user_data);
      c->watcher_dispatches++;
      if (size >= sizeof(uint32_t)) {
        std::memcpy(&c->last_sensor, data, sizeof(uint32_t));
      }
      // Byte-for-byte identical output as recorded
      uint32_t cmd = c->last_sensor * 2;
      talos_loop_send(c->loop, c->sender_id, &cmd, sizeof(cmd));
    };

    talos_loop_register_watcher(replay_loop, kSensorTopic, sizeof(uint32_t),
                                alignof(uint32_t), watcher_fn, &ctx);

    auto timer_fn = [](const TalosDispatchContext* context, const void* data,
                       uint32_t size, void* user_data) {
      auto* c = static_cast<ReplayContext*>(user_data);
      c->timer_ticks++;
    };

    talos_loop_register_timer(replay_loop, "sim_periodic", 10'000'000,
                              10'000'000, timer_fn, &ctx);

    EXPECT_EQ(talos_loop_run(replay_loop), TALOS_OK);

    EXPECT_EQ(talos_loop_diverged(replay_loop), 0);
    EXPECT_EQ(talos_loop_reached_exit(replay_loop), 1);
    EXPECT_EQ(ctx.watcher_dispatches, 2);
    EXPECT_EQ(ctx.last_sensor, 100u);
    EXPECT_EQ(ctx.timer_ticks, 3);

    talos_loop_destroy(replay_loop);
  }

  // 3. Divergence detection when sender payload is changed during replay
  {
    TalosLoop* div_loop = talos_loop_create_replay(sim_log_path.c_str());
    ASSERT_NE(div_loop, nullptr) << talos_last_error();

    const uint16_t sender_id = talos_loop_register_sender(
        div_loop, kCmdTopic, sizeof(uint32_t), alignof(uint32_t));
    EXPECT_NE(sender_id, 0xFFFF);

    struct DivContext {
      TalosLoop* loop{nullptr};
      uint16_t sender_id{0};
    };
    DivContext ctx{div_loop, sender_id};

    auto watcher_fn = [](const TalosDispatchContext* context, const void* data,
                         uint32_t size, void* user_data) {
      auto* c = static_cast<DivContext*>(user_data);
      uint32_t sensor = 0;
      if (size >= sizeof(uint32_t)) {
        std::memcpy(&sensor, data, sizeof(uint32_t));
      }
      // Intentionally divergent payload!
      uint32_t wrong_cmd = sensor * 999;
      talos_loop_send(c->loop, c->sender_id, &wrong_cmd, sizeof(wrong_cmd));
    };

    talos_loop_register_watcher(div_loop, kSensorTopic, sizeof(uint32_t),
                                alignof(uint32_t), watcher_fn, &ctx);

    auto timer_fn = [](const TalosDispatchContext* context, const void* data,
                       uint32_t size, void* user_data) {};
    talos_loop_register_timer(div_loop, "sim_periodic", 10'000'000,
                              10'000'000, timer_fn, nullptr);

    // Replay runs to completion with throw_on_divergence=false
    talos_loop_run(div_loop);

    EXPECT_EQ(talos_loop_diverged(div_loop), 1);
    EXPECT_NE(std::string(talos_last_error()).find("sent different bytes"),
              std::string::npos);
    EXPECT_EQ(talos_loop_reached_exit(div_loop), 1);

    talos_loop_destroy(div_loop);
  }

  // Clean up temporary file
  fs::remove(sim_log_path);
}

TEST(NodeApi, LoopTimerArmDisarm) {
  TalosLoop* sim_loop = talos_loop_create_simulated(nullptr, nullptr);
  ASSERT_NE(sim_loop, nullptr);

  int ticks = 0;
  auto timer_fn = [](const TalosDispatchContext* context, const void* data,
                     uint32_t size, void* user_data) {
    auto* t = static_cast<int*>(user_data);
    (*t)++;
  };

  // Register disarmed (period 0, offset 0)
  uint16_t timer_id = talos_loop_register_timer(sim_loop, "disarmed_timer", 0, 0,
                                               timer_fn, &ticks);
  EXPECT_NE(timer_id, 0xFFFF);

  // Run for 10ms - should not fire
  EXPECT_EQ(talos_loop_run_for(sim_loop, 10'000'000), TALOS_OK);
  EXPECT_EQ(ticks, 0);

  // Arm timer: 5ms offset, 5ms period
  EXPECT_EQ(talos_loop_arm_timer(sim_loop, timer_id, 5'000'000, 5'000'000),
            TALOS_OK);

  // Run for 12ms - should fire at 5ms and 10ms (2 ticks)
  EXPECT_EQ(talos_loop_run_for(sim_loop, 12'000'000), TALOS_OK);
  EXPECT_EQ(ticks, 2);

  // Disarm timer
  EXPECT_EQ(talos_loop_disarm_timer(sim_loop, timer_id), TALOS_OK);

  // Run for another 20ms - no additional ticks
  EXPECT_EQ(talos_loop_run_for(sim_loop, 20'000'000), TALOS_OK);
  EXPECT_EQ(ticks, 2);

  talos_loop_destroy(sim_loop);
}

}  // namespace
