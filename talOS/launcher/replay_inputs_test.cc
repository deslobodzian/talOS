// launcher --replay-inputs: a session recorded by the launcher with real
// node processes on real shared memory, then replayed into a node that was
// not in it (testdata/replay_consumer.cc), twice, into one that was
// (testdata/replay_sampler.cc), into one that reads the recorded topic as
// another type (testdata/replay_impostor.cc), and into one that reads the
// declared graph and keeps a record under its session directory
// (testdata/replay_graph_reader.cc).

#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "replay_inputs_test/replay_message_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/context.h"
#include "talOS/events/handles.h"
#include "talOS/events/log/gap_testing.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/manifest.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/events/time.h"
#include "talOS/hardware/config.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/launcher/launcher.h"
#include "talOS/launcher/spawn.h"
#include "talOS/launcher/testdata/first_exec.h"
#include "talOS/protocol/types.h"

namespace talos::launcher {
namespace {

using replay_inputs_test::MotorReading;
using replay_inputs_test::Reading;
using replay_inputs_test::Summary;

constexpr std::uint32_t kReadings = 100;  // ReplaySource::kReadings

std::string Runfile(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"runfile unavailable: "} + env + " " +
                             error);
  }
  return runfiles->Rlocation(location);
}

std::string TempPath(std::string_view name) {
  return ::testing::TempDir() + "/talos_replay_inputs_" +
         std::to_string(::getpid()) + "_" + std::string{name};
}

// Calls `visit(header, payload)` for each `kind` record of `log_path`'s
// source on `topic` (a watcher by MESSAGE, a sender by SEND), in log order.
template <class Visit>
void ForEachRecord(const std::string& log_path, std::string_view topic,
                   event::EventKind kind, Visit visit) {
  event::log::LogReader reader{log_path};
  std::uint16_t id = 0xFFFF;
  for (const auto& registration : reader.manifest()) {
    const bool sender = registration.kind == event::SourceKind::SENDER;
    if (registration.name == topic &&
        sender == (kind == event::EventKind::SEND)) {
      id = registration.id;
    }
  }
  EXPECT_NE(id, 0xFFFF) << log_path << " has no " << topic;
  event::log::LogReader::Record record;
  while (reader.next(record)) {
    if (record.header.kind == static_cast<std::uint16_t>(kind) &&
        record.header.source_id == id) {
      visit(record.header, std::vector<std::byte>{record.payload.begin(),
                                                  record.payload.end()});
    }
  }
}

struct Sent {
  std::int64_t now_ns;
  std::vector<std::byte> payload;
  bool operator==(const Sent&) const = default;
};

// What `log_path`'s node sent on `topic`, with when.
std::vector<Sent> Sends(const std::string& log_path, std::string_view topic) {
  std::vector<Sent> sent;
  ForEachRecord(log_path, topic, event::EventKind::SEND,
                [&](const auto& header, std::vector<std::byte> payload) {
                  sent.push_back({header.now_ns, std::move(payload)});
                });
  return sent;
}

// The same, by the deadline of the dispatch that sent it.
std::map<std::int64_t, std::vector<std::byte>> SendsByDeadline(
    const std::string& log_path, std::string_view topic) {
  std::map<std::int64_t, std::vector<std::byte>> sent;
  ForEachRecord(log_path, topic, event::EventKind::SEND,
                [&](const auto& header, std::vector<std::byte> payload) {
                  sent[header.event_time_ns] = std::move(payload);
                });
  return sent;
}

struct Delivery {
  std::int64_t event_ns;    // the drain that delivered it
  std::int64_t publish_ns;  // when it was published
  std::vector<std::byte> payload;
  bool operator==(const Delivery&) const = default;
};

// What `log_path`'s node received on `topic`, in the order it received it.
std::vector<Delivery> Deliveries(const std::string& log_path,
                                 std::string_view topic) {
  std::vector<Delivery> received;
  ForEachRecord(
      log_path, topic, event::EventKind::MESSAGE,
      [&](const auto& header, std::vector<std::byte> payload) {
        received.push_back(
            {header.event_time_ns, header.publish_time_ns, std::move(payload)});
      });
  return received;
}

template <class M>
M As(const std::vector<std::byte>& payload) {
  M message{};
  EXPECT_EQ(payload.size(), sizeof message);
  std::memcpy(&message, payload.data(), sizeof message);
  return message;
}

template <class M>
M As(const Sent& sent) {
  return As<M>(sent.payload);
}

// `binary` with `args`, to its exit; its exit code, or -1.
int RunNode(const std::string& binary, std::vector<std::string> args) {
  args.insert(args.begin(), binary);
  std::string error;
  const pid_t pid = SpawnProcess(std::move(args), error);
  EXPECT_GT(pid, 0) << error;
  if (pid <= 0) return -1;
  int status = 0;
  while (::waitpid(pid, &status, 0) < 0) {
    if (errno != EINTR) return -1;
  }
  FinishProcessGroup(pid);
  return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

using GatewayLoop = event::SimulatedEventLoop<event::log::LogWriter>;

// Writes, as hardware_node's log, what a real robot's bridge publishes:
// a State every 5 ms for the config's motor m2 of replay_motor, its
// position moving, stamped with this config's id, a gateway boot id and
// the gateway's clock, 195 s behind this host's, and on every fourth State
// the bridge's /hw/status with the clock mapping for that boot and the
// bridge's hardware protocol (this build's unless given). No Driver Station.
class RealGateway {
 public:
  static constexpr std::uint64_t kBootId = 1989997566370484863ULL;
  static constexpr std::int64_t kGatewayBehindUs = 195'000'000;

  RealGateway(GatewayLoop& loop, const config::RobotConfig& config,
              std::uint16_t protocol_version = protocol::kProtocolVersion)
      : config_id_{hardware::ConfigurationId(config.hardware)},
        protocol_version_{protocol_version},
        motor_id_{config.GetDevices("replay_motor")->motors.front()},
        state_{
            event::make_sender<hardware::Packet>(loop, hardware::kStateTopic)},
        status_{event::make_sender<::bridge::BridgeStatus>(
            loop, ::bridge::topics::kStatus)},
        tick_{event::make_timer<&RealGateway::OnTick>(loop, "gateway", this)} {
    tick_.setup_periodic(loop.monotonic_now() + std::chrono::microseconds{1700},
                         std::chrono::milliseconds{5});
  }

 private:
  void OnTick(const event::Context& context) {
    const std::int64_t now_us = context.event_time.nanos() / 1000;
    hardware::State state{};
    state.config_id = config_id_;
    state.boot_id = kBootId;
    state.epoch = 3;
    state.sample_time_us =
        static_cast<std::uint64_t>(now_us - kGatewayBehindUs);
    state.sequence = 33558 + ticks_;
    state.motor_count = 1;
    state.motors[0] = {
        .id = motor_id_,
        .valid = true,
        .position_rot = -8.0 + 0.25 * static_cast<double>(ticks_)};
    hardware::Packet packet{};
    packet.size =
        static_cast<std::uint32_t>(hardware::Encode(state, packet.data));
    state_.send(packet);
    if (ticks_ % 4 == 0) {
      ::bridge::BridgeStatus status{};
      status.mutate_configured(true);
      status.mutate_clock_boot_id(kBootId);
      status.mutate_clock_offset_us(kGatewayBehindUs);
      status.mutate_clock_offset_valid(true);
      status.mutate_protocol_version_id(protocol_version_);
      status_.send(status);
    }
    ++ticks_;
  }

  std::uint64_t config_id_;
  std::uint16_t protocol_version_;
  std::uint16_t motor_id_;
  event::Sender<GatewayLoop, hardware::Packet> state_;
  event::Sender<GatewayLoop, ::bridge::BridgeStatus> status_;
  event::Timer<GatewayLoop> tick_;
  std::uint64_t ticks_ = 0;
};

class ReplayInputsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::ofstream{config_} << R"([robot]
name = "replay_inputs"
period_us = 5000

[subsystems.replay_source]
node = "//talOS/launcher:replay_source"

[subsystems.replay_source.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.replay_consumer]
node = "//talOS/launcher:replay_consumer"

[subsystems.replay_sampler]
node = "//talOS/launcher:replay_sampler"

[subsystems.replay_motor]
node = "//talOS/launcher:replay_motor"

[subsystems.replay_motor.motors.m2]
type = "TalonFX"
bus = "rio"
can_id = 2

[subsystems.replay_impostor]
node = "//talOS/launcher:replay_impostor"

[subsystems.replay_graph_reader]
node = "//talOS/launcher:replay_graph_reader"
)";
    // The recorded session runs the source, the sampler and the graph
    // reader.
    std::ofstream{record_config_} << R"([robot]
name = "replay_inputs"
period_us = 5000

[subsystems.replay_source]
node = "//talOS/launcher:replay_source"

[subsystems.replay_source.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.replay_sampler]
node = "//talOS/launcher:replay_sampler"

[subsystems.replay_graph_reader]
node = "//talOS/launcher:replay_graph_reader"
)";
    introspect::Description hardware;
    hardware.name = "hardware_node";
    hardware.target = "//talOS/bridge:hardware_node";
    const std::string stub = record_config_ + ".hardware.sh";
    std::ostringstream script;
    script << "#!/bin/sh\n"
           << "for arg in \"$@\"; do\n"
           << "  if [ \"$arg\" = \"--describe\" ]; then\n"
           << "    cat <<'TALOS_DESCRIBE'\n"
           << introspect::DescribeToJson(hardware) << "TALOS_DESCRIBE\n"
           << "    exit 0\n"
           << "  fi\n"
           << "done\n"
           // Alive until the session ends: an exited stub restarts, and the
           // end's SIGINT landing before the restart's trap ends it by the
           // signal rather than the exit 0 its handler gives.
           << "trap 'exit 0' INT TERM\n"
           << "while :; do sleep 0.05; done\n";
    talos::launcher::testing::WriteWarmStub(stub, script.str(), 0700);
  }

  void TearDown() override {
    std::filesystem::remove(config_);
    std::filesystem::remove(record_config_);
    std::filesystem::remove(record_config_ + ".hardware.sh");
  }

  LauncherOptions Options() const {
    LauncherOptions options;
    options.session_id = 1;
    options.simulation = true;
    options.describe_timeout_ms = 4000;
    options.binary_overrides["//talOS/bridge:hardware_node"] =
        record_config_ + ".hardware.sh";
    options.binary_overrides["//talOS/launcher:replay_source"] =
        Runfile("TALOS_SOURCE_NODE");
    options.binary_overrides["//talOS/launcher:replay_consumer"] =
        Runfile("TALOS_CONSUMER_NODE");
    options.binary_overrides["//talOS/launcher:replay_impostor"] =
        Runfile("TALOS_IMPOSTOR_NODE");
    options.binary_overrides["//talOS/launcher:replay_graph_reader"] =
        Runfile("TALOS_GRAPH_READER_NODE");
    options.binary_overrides["//talOS/launcher:replay_sampler"] =
        Runfile("TALOS_SAMPLER_NODE");
    options.binary_overrides["//talOS/launcher:replay_motor"] =
        Runfile("TALOS_MOTOR_NODE");
    return options;
  }

  // The recorded session, shared by the suite: recorded by the first test
  // that needs it and read by each. A test that changes a session changes a
  // copy of it (CopyOfSession). Recording it once per test only repeated
  // its 2 s.
  void Record() {
    if (!recorded_) {
      std::filesystem::remove_all(session_dir_);
      LauncherOptions options = Options();
      options.config_path = record_config_;
      options.output_dir =
          std::filesystem::path{session_dir_}.parent_path().string();
      options.duration_s = 2;
      recorded_ = Launcher{options}.Run();
    }
    ASSERT_EQ(*recorded_, 0);
  }

  static void TearDownTestSuite() {
    std::filesystem::remove_all(TempPath("session"));
    recorded_.reset();
  }

  // A private copy of the recorded session, as `<name>/1`.
  std::string CopyOfSession(std::string_view name) const {
    const std::string copy = TempPath(name) + "/1";
    std::filesystem::remove_all(TempPath(name));
    std::filesystem::create_directories(copy);
    std::filesystem::copy(session_dir_, copy,
                          std::filesystem::copy_options::recursive);
    return copy;
  }

  // `node` on the recorded session's topics, its log in `out_dir`.
  int Replay(const std::string& node, const std::string& out_dir) {
    return Replay(node, out_dir, session_dir_);
  }
  int Replay(const std::string& node, const std::string& out_dir,
             const std::string& session, bool allow_capped_replay = false) {
    LauncherOptions options = Options();
    options.config_path = config_;
    options.output_dir = out_dir;
    options.replay_inputs = session;
    options.only = node;
    options.allow_capped_replay = allow_capped_replay;
    return Launcher{options}.Run();
  }

  // The launcher's exit status for the shared recording, once it ran.
  static inline std::optional<int> recorded_;
  const std::string session_dir_ = TempPath("session") + "/1";
  const std::string config_ = TempPath("robot.toml");
  const std::string record_config_ = TempPath("record.toml");
};

TEST_F(ReplayInputsTest, ANodeThatWasNotInTheSessionRunsOnItsTopics) {
  Record();
  const std::vector<Sent> readings =
      Sends(session_dir_ + "/replay_source.tlog", "/replay_source/state");
  ASSERT_EQ(readings.size(), kReadings);

  const std::string first = TempPath("first");
  const std::string second = TempPath("second");
  std::filesystem::remove_all(first);
  std::filesystem::remove_all(second);
  ASSERT_EQ(Replay("replay_consumer", first), 0);
  ASSERT_EQ(Replay("replay_consumer", second), 0);

  const std::vector<Sent> summaries =
      Sends(first + "/1/replay_consumer.tlog", "/replay_consumer/state");
  ASSERT_FALSE(summaries.empty());
  // On the recording's clock: nothing before the first reading arrived
  // counted it, and every reading was counted by the end.
  float expected_total_m = 0.0F;
  for (const Sent& reading : readings) {
    expected_total_m += As<Reading>(reading).position_m();
  }
  const Summary last = As<Summary>(summaries.back());
  EXPECT_EQ(last.reading_count(), kReadings);
  EXPECT_EQ(last.total_m(), expected_total_m);
  for (const Sent& sent : summaries) {
    if (sent.now_ns < readings.front().now_ns) {
      EXPECT_EQ(As<Summary>(sent).reading_count(), 0u);
    }
    std::uint32_t published_by_then = 0;
    for (const Sent& reading : readings) {
      published_by_then += reading.now_ns <= sent.now_ns ? 1 : 0;
    }
    EXPECT_LE(As<Summary>(sent).reading_count(), published_by_then);
  }
  // Reproducible: the second replay sent the same bytes at the same times.
  EXPECT_EQ(Sends(second + "/1/replay_consumer.tlog", "/replay_consumer/state"),
            summaries);

  std::filesystem::remove_all(first);
  std::filesystem::remove_all(second);
}

// REQ-OPS-14: a node the session ran replays on its own recorded timeline.
// Each process's log has its own origin (the sampler was spawned after the
// source, and its loop began running after its construction), and the
// sampler saw each reading at the tick that drained it live. Replayed, it
// sees the same readings at the same ticks, in the same order, with their
// recorded publication times; a replay counted from the session's earliest
// origin lands each reading published between the two timelines' ticks on a
// neighbouring tick. The replayed loop drains at each deadline, without the
// live loop's wake-up latency, so a reading the live drain read although it
// was published during that latency is stamped with the drain's deadline.
TEST_F(ReplayInputsTest, ANodeThatWasInTheSessionSeesItsInputsWhenItDid) {
  Record();
  const std::string recorded_log = session_dir_ + "/replay_sampler.tlog";
  ASSERT_NE(
      event::log::LogReader{recorded_log}.start_time(),
      event::log::LogReader{session_dir_ + "/replay_source.tlog"}.start_time());
  std::vector<Delivery> expected =
      Deliveries(recorded_log, "/replay_source/state");
  ASSERT_FALSE(expected.empty());
  for (Delivery& delivery : expected) {
    delivery.publish_ns = std::min(delivery.publish_ns, delivery.event_ns);
  }

  const std::string out = TempPath("sampler");
  std::filesystem::remove_all(out);
  ASSERT_EQ(Replay("replay_sampler", out), 0);
  const std::vector<Delivery> replayed =
      Deliveries(out + "/1/replay_sampler.tlog", "/replay_source/state");
  EXPECT_EQ(replayed.size(), expected.size());
  const auto [want, got] = std::mismatch(expected.begin(), expected.end(),
                                         replayed.begin(), replayed.end());
  if (want != expected.end() && got != replayed.end()) {
    ADD_FAILURE() << "delivery " << (want - expected.begin())
                  << ": recorded at " << want->event_ns << " ns, published "
                  << want->publish_ns << " ns; replayed at " << got->event_ns
                  << " ns, published " << got->publish_ns << " ns"
                  << (want->payload == got->payload ? "" : ", another reading");
  }
  // So it sends what it sent, at each deadline it sent at live (the replay
  // also ticks at any deadline a late live tick coalesced).
  const auto replayed_sends =
      SendsByDeadline(out + "/1/replay_sampler.tlog", "/replay_sampler/state");
  const auto recorded_sends =
      SendsByDeadline(recorded_log, "/replay_sampler/state");
  ASSERT_FALSE(recorded_sends.empty());
  for (const auto& [deadline_ns, payload] : recorded_sends) {
    const auto replayed_send = replayed_sends.find(deadline_ns);
    if (replayed_send == replayed_sends.end() ||
        replayed_send->second != payload) {
      ADD_FAILURE() << "the tick at " << deadline_ns << " ns sent "
                    << (replayed_send == replayed_sends.end()
                            ? "nothing"
                            : "another summary")
                    << " in the replay";
      break;
    }
  }

  std::filesystem::remove_all(out);
}

// REQ-OPS-14: a session launched without --sim replays without it. The
// devices layer takes only a /hw/state whose config id is its own config's,
// and under --sim it treats the config as commissioned, which is part of
// that id: replayed under --sim, a node from a real robot's session (an
// uncommissioned bench) accepts no State and reads its motor disconnected
// and still. The session is a real one's shape: the gateway's clock 195 s
// behind the host's, a gateway boot id, no Driver Station, the motor node's
// log recorded without --sim (here the node binary itself on the gateway's
// log), and a manifest that says it was no simulation.
TEST_F(ReplayInputsTest, ASessionRecordedWithoutSimReplaysWithoutIt) {
  const std::string root = TempPath("hardware");
  const std::string dir = root + "/1";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(dir);
  {
    const config::RobotConfig config = config::ParseRobotConfig(config_);
    const event::MonotonicTime origin =
        event::MonotonicTime::from_nanos(2'038'406'047'920);
    event::SimulationEnvironment environment;
    GatewayLoop loop{
        environment,
        event::log::LogWriter{dir + "/hardware_node.tlog", "hardware_node",
                              event::log::LogWriterOptions{}, 1},
        GatewayLoop::Options{.start = origin}};
    RealGateway gateway{loop, config};
    loop.run_until(origin + std::chrono::milliseconds{400});
    loop.finish();
  }
  const std::string recording = root + "/replay_motor.tlog";
  ASSERT_EQ(RunNode(Runfile("TALOS_MOTOR_NODE"),
                    {"--replay-inputs", dir, "--session-id", "1", "--log",
                     recording, "--config", config_}),
            0);
  std::filesystem::rename(recording, dir + "/replay_motor.tlog");
  std::ofstream{dir + "/manifest.json"}
      << "{\n  \"session_id\": 1,\n  \"simulation\": false\n}\n";
  const auto recorded =
      SendsByDeadline(dir + "/replay_motor.tlog", "/replay_motor/state");
  ASSERT_FALSE(recorded.empty());
  const auto last = As<MotorReading>(recorded.rbegin()->second);
  ASSERT_TRUE(last.connected());
  ASSERT_GE(last.sample_time_ns(), 0);

  const std::string out = TempPath("motor");
  std::filesystem::remove_all(out);
  ASSERT_EQ(Replay("replay_motor", out, dir), 0);
  const auto replayed =
      SendsByDeadline(out + "/1/replay_motor.tlog", "/replay_motor/state");
  EXPECT_EQ(replayed.size(), recorded.size());
  const auto describe = [](const std::vector<std::byte>& payload) {
    const auto reading = As<MotorReading>(payload);
    return std::string{reading.connected() ? "connected" : "disconnected"} +
           " at " + std::to_string(reading.position_rot()) +
           " rot, sampled at " + std::to_string(reading.sample_time_ns()) +
           " ns";
  };
  for (const auto& [deadline_ns, payload] : recorded) {
    const auto found = replayed.find(deadline_ns);
    if (found == replayed.end() || found->second != payload) {
      ADD_FAILURE() << "the tick at " << deadline_ns << " ns read "
                    << describe(payload) << "; replayed: "
                    << (found == replayed.end() ? "no tick"
                                                : describe(found->second));
      break;
    }
  }

  std::filesystem::remove_all(out);
  std::filesystem::remove_all(root);
}

// REQ-OPS-14: a session launched with --sim replays with --sim even when the
// replay invocation omits --sim (options.simulation = false). Under --sim, the
// devices layer commissions the config in memory, which changes config_id:
// replayed without --sim, the motor node would reject every State and read
// disconnected.
TEST_F(ReplayInputsTest, ASessionRecordedWithSimReplaysWithIt) {
  const std::string root = TempPath("sim_hardware");
  const std::string dir = root + "/1";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(dir);
  {
    config::RobotConfig config = config::ParseRobotConfig(config_);
    config.hardware.commissioned = true;
    const event::MonotonicTime origin =
        event::MonotonicTime::from_nanos(2'038'406'047'920);
    event::SimulationEnvironment environment;
    GatewayLoop loop{
        environment,
        event::log::LogWriter{dir + "/hardware_node.tlog", "hardware_node",
                              event::log::LogWriterOptions{}, 1},
        GatewayLoop::Options{.start = origin}};
    RealGateway gateway{loop, config};
    loop.run_until(origin + std::chrono::milliseconds{400});
    loop.finish();
  }
  const std::string recording = root + "/replay_motor.tlog";
  ASSERT_EQ(RunNode(Runfile("TALOS_MOTOR_NODE"),
                    {"--replay-inputs", dir, "--session-id", "1", "--log",
                     recording, "--config", config_, "--sim"}),
            0);
  std::filesystem::rename(recording, dir + "/replay_motor.tlog");
  std::ofstream{dir + "/manifest.json"}
      << "{\n  \"session_id\": 1,\n  \"simulation\": true\n}\n";
  const auto recorded =
      SendsByDeadline(dir + "/replay_motor.tlog", "/replay_motor/state");
  ASSERT_FALSE(recorded.empty());
  const auto last = As<MotorReading>(recorded.rbegin()->second);
  ASSERT_TRUE(last.connected());
  ASSERT_GE(last.sample_time_ns(), 0);

  const std::string out = TempPath("sim_motor_out");
  std::filesystem::remove_all(out);
  LauncherOptions options = Options();
  options.simulation = false;
  options.config_path = config_;
  options.output_dir = out;
  options.replay_inputs = dir;
  options.only = "replay_motor";
  ASSERT_EQ(Launcher{options}.Run(), 0);

  const auto replayed =
      SendsByDeadline(out + "/1/replay_motor.tlog", "/replay_motor/state");
  EXPECT_EQ(replayed.size(), recorded.size());
  EXPECT_EQ(replayed, recorded);

  std::filesystem::remove_all(out);
  std::filesystem::remove_all(root);
}

// REQ-OPS-14: a node fed /hw/state from the bridge's log is never fed across
// a gap in it. The session is a real one's shape (as above), its bridge log
// holding States, then a closed GAP record where its writer's pool filled
// (log/gap_testing.h; the bridge's own writer reaches the same record through
// node.cc's log_options_for_test, hardware_node_test), then more States. The
// replay stops before the gap, says so naming the bridge's log, and exits 1;
// the node received no State published at or after the gap's start.
TEST_F(ReplayInputsTest, AReplayFedFromTheBridgesLogStopsAtItsGap) {
  const std::string root = TempPath("bridge_gap");
  const std::string dir = root + "/1";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(dir);
  const std::string whole = root + "/whole.tlog";
  {
    const event::MonotonicTime origin =
        event::MonotonicTime::from_nanos(2'038'406'047'920);
    event::SimulationEnvironment environment;
    GatewayLoop loop{environment,
                     event::log::LogWriter{whole, "hardware_node",
                                           event::log::LogWriterOptions{}, 1},
                     GatewayLoop::Options{.start = origin}};
    RealGateway gateway{loop, config::ParseRobotConfig(config_)};
    loop.run_until(origin + std::chrono::milliseconds{400});
    loop.finish();
  }
  // Drop four gateway ticks from the middle of the run.
  std::vector<event::log::RecordHeader> headers;
  {
    event::log::LogReader reader{whole};
    event::log::LogReader::Record record;
    while (reader.next(record)) headers.push_back(record.header);
  }
  const auto tick_at = [&](std::size_t from) {
    while (from < headers.size() &&
           headers[from].kind !=
               static_cast<std::uint16_t>(event::EventKind::TIMER)) {
      ++from;
    }
    return from;
  };
  const std::size_t first = tick_at(headers.size() / 2);
  std::size_t end = first;
  for (int ticks = 0; ticks < 4; ++ticks) end = tick_at(end + 1);
  ASSERT_LT(end, headers.size()) << "too few records: " << headers.size();
  const std::string bridge_log = dir + "/hardware_node.tlog";
  const event::log::LogGap gap =
      event::log::CopyWithGap(whole, bridge_log, first, end);
  std::ofstream{dir + "/manifest.json"}
      << "{\n  \"session_id\": 1,\n  \"simulation\": false\n}\n";
  const std::vector<Sent> states = Sends(bridge_log, "/hw/state");
  ASSERT_TRUE(
      std::any_of(states.begin(), states.end(),
                  [&](const Sent& s) { return s.now_ns > gap.resumed_ns; }))
      << "no State after the gap in " << bridge_log;

  const std::string out = TempPath("bridge_gap_out");
  std::filesystem::remove_all(out);
  ::testing::internal::CaptureStdout();
  const int status = Replay("replay_motor", out, dir);
  const std::string output = ::testing::internal::GetCapturedStdout();
  EXPECT_EQ(status, 1) << output;
  const std::size_t stop = output.find("stopped before a log gap in ");
  ASSERT_NE(stop, std::string::npos) << output;
  const std::string line = output.substr(stop, output.find('\n', stop) - stop);
  EXPECT_NE(line.find(bridge_log), std::string::npos) << line;
  EXPECT_TRUE(line.find("(topic /hw/state)") != std::string::npos ||
              line.find("(topic /hw/status)") != std::string::npos)
      << line;
  const std::vector<Delivery> received =
      Deliveries(out + "/1/replay_motor.tlog", "/hw/state");
  ASSERT_FALSE(received.empty()) << output;
  for (const Delivery& delivery : received) {
    EXPECT_LT(delivery.publish_ns, gap.start_ns)
        << "a State published " << delivery.publish_ns - gap.start_ns
        << " ns after the gap's start was fed";
  }

  std::filesystem::remove_all(out);
  std::filesystem::remove_all(root);
}

// REQ-OPS-14: a schema mismatch fails before running and names both types.
TEST_F(ReplayInputsTest, AReaderOfAnotherSchemaIsRefused) {
  Record();
  const std::string out = TempPath("impostor");
  std::filesystem::remove_all(out);
  ::testing::internal::CaptureStderr();
  const int status = Replay("replay_impostor", out);
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(status, 0) << err;
  EXPECT_NE(err.find("replay_inputs_test.Reading"), std::string::npos) << err;
  EXPECT_NE(err.find("replay_inputs_test.Impostor"), std::string::npos) << err;

  std::filesystem::remove_all(out);
}

// REQ-OPS-14: a node whose endpoints come from the declared graph (teleop,
// telemetry) replays on the graph its session was launched with, read from
// the replayed session's declared_graph.json: here two state topics besides
// its own, the source's and the sampler's. A session without that file is
// refused, not replayed on an empty graph.
TEST_F(ReplayInputsTest, AGraphReaderReplaysOnTheSessionsDeclaredGraph) {
  Record();
  const std::string session = CopyOfSession("graph_session");
  const std::string out = TempPath("graph_reader");
  std::filesystem::remove_all(out);
  ASSERT_EQ(Replay("replay_graph_reader", out, session), 0);
  const std::vector<Sent> summaries =
      Sends(out + "/1/replay_graph_reader.tlog", "/replay_graph_reader/state");
  ASSERT_FALSE(summaries.empty());
  EXPECT_EQ(As<Summary>(summaries.back()).reading_count(), 2u);

  std::filesystem::remove_all(out);
  ASSERT_TRUE(std::filesystem::remove(session + "/declared_graph.json"));
  EXPECT_NE(Replay("replay_graph_reader", out, session), 0);

  std::filesystem::remove_all(out);
  std::filesystem::remove_all(TempPath("graph_session"));
}

// REQ-OPS-14: a replayed node has no session directory, although the launcher
// passes it --log, so state a node keeps per session (the 2026 autonomous
// node's routine selection, and its progress object, which outlives the
// session in shared memory) is neither read nor written by a replay. The
// recording shows the same node keeps its record under a live session.
TEST_F(ReplayInputsTest, AReplayedNodeKeepsNoSessionState) {
  Record();
  ASSERT_TRUE(
      std::filesystem::exists(session_dir_ + "/replay_graph_reader.record"));
  const std::string out = TempPath("session_state");
  std::filesystem::remove_all(out);
  ASSERT_EQ(Replay("replay_graph_reader", out), 0);
  EXPECT_FALSE(std::filesystem::exists(out + "/1/replay_graph_reader.record"));

  std::filesystem::remove_all(out);
}

// REQ-OPS-51: a session whose bridge recorded another hardware protocol on
// /hw/status is not replayed, though its manifest records this build's (the
// launcher's): its States would reach the node's devices layer decoded with
// this build's layout (an equal-length change decodes, wrongly), and a
// replay is compared with the recording, which another protocol's run
// cannot match. The node is not started, so it writes no log.
TEST_F(ReplayInputsTest, ASessionOfAnotherHardwareProtocolIsRefused) {
  Record();
  const std::uint16_t kBuild = protocol::kProtocolVersion;
  const std::uint16_t kOther = static_cast<std::uint16_t>(kBuild - 1);
  const std::string session = CopyOfSession("other_protocol");
  const auto write_bridge_log = [&](const std::string& filename,
                                    std::uint16_t version) {
    const event::MonotonicTime origin =
        event::MonotonicTime::from_nanos(2'038'406'047'920);
    event::SimulationEnvironment environment;
    GatewayLoop loop{
        environment,
        event::log::LogWriter{session + "/" + filename, "hardware_node",
                              event::log::LogWriterOptions{}, 1},
        GatewayLoop::Options{.start = origin}};
    RealGateway gateway{loop, config::ParseRobotConfig(config_), version};
    loop.run_until(origin + std::chrono::milliseconds{20});
    loop.finish();
  };
  const auto refusal_line = [](const std::string& err) {
    std::istringstream lines{err};
    for (std::string line; std::getline(lines, line);) {
      if (line.find("not replayed") != std::string::npos) return line;
    }
    return std::string{};
  };

  // Subcase 1: single bridge run recording another protocol version.
  write_bridge_log("hardware_node.tlog", kOther);
  const std::string out = TempPath("other_protocol_out");
  std::filesystem::remove_all(out);
  ::testing::internal::CaptureStderr();
  const int single_rc = Replay("replay_consumer", out, session);
  const std::string single_err = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(single_rc, 0) << single_err;
  EXPECT_FALSE(std::filesystem::exists(out + "/1/replay_consumer.tlog"));
  const std::string single_refusal = refusal_line(single_err);
  ASSERT_FALSE(single_refusal.empty()) << single_err;
  EXPECT_NE(single_refusal.find(std::to_string(kOther)), std::string::npos)
      << single_refusal;
  EXPECT_NE(single_refusal.find(std::to_string(kBuild)), std::string::npos)
      << single_refusal;
  std::filesystem::remove_all(out);

  // Subcase 2: bridge runs of two different protocol versions in one session.
  write_bridge_log("hardware_node.restart1.tlog", kBuild);
  ::testing::internal::CaptureStderr();
  const int two_rc = Replay("replay_consumer", out, session);
  const std::string two_err = ::testing::internal::GetCapturedStderr();
  EXPECT_NE(two_rc, 0) << two_err;
  EXPECT_FALSE(std::filesystem::exists(out + "/1/replay_consumer.tlog"));
  const std::string two_refusal = refusal_line(two_err);
  ASSERT_FALSE(two_refusal.empty()) << two_err;
  EXPECT_NE(two_refusal.find(std::to_string(kOther)), std::string::npos)
      << two_refusal;
  EXPECT_NE(two_refusal.find(std::to_string(kBuild)), std::string::npos)
      << two_refusal;
  EXPECT_NE(two_refusal.find("different builds"), std::string::npos)
      << two_refusal;
  std::filesystem::remove_all(out);
  std::filesystem::remove_all(TempPath("other_protocol"));
}

// REQ-OPS-51: when the bridge's /hw/status protocol_version_id and
// manifest.json's hardware_protocol_version disagree, --replay-inputs prints
// one warning naming both versions and reads the session under the bridge's
// version.
TEST_F(ReplayInputsTest,
       BridgeAndManifestProtocolDisagreementWarnsAndUsesBridgesVersion) {
  Record();
  const std::uint16_t kBuild = protocol::kProtocolVersion;
  const std::uint16_t kOther = static_cast<std::uint16_t>(kBuild - 1);
  const std::string session = CopyOfSession("protocol_disagree");
  {
    const event::MonotonicTime origin =
        event::log::LogReader{session + "/replay_source.tlog"}.start_time();
    event::SimulationEnvironment environment;
    GatewayLoop loop{
        environment,
        event::log::LogWriter{session + "/hardware_node.tlog", "hardware_node",
                              event::log::LogWriterOptions{}, 1},
        GatewayLoop::Options{.start = origin}};
    RealGateway gateway{loop, config::ParseRobotConfig(config_), kBuild};
    loop.run_until(origin + std::chrono::milliseconds{20});
    loop.finish();
  }
  std::ofstream{session + "/manifest.json"}
      << "{\n  \"session_id\": 1,\n  \"simulation\": true,\n  "
         "\"hardware_protocol_version\": "
      << kOther << ",\n  \"nodes\": []\n}\n";

  const std::string out = TempPath("protocol_disagree_out");
  std::filesystem::remove_all(out);
  ::testing::internal::CaptureStderr();
  const int rc = Replay("replay_consumer", out, session);
  const std::string err = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(rc, 0) << err;
  EXPECT_TRUE(std::filesystem::exists(out + "/1/replay_consumer.tlog"));

  std::vector<std::string> warnings;
  std::istringstream lines{err};
  for (std::string line; std::getline(lines, line);) {
    if (line.find("the session's bridge recorded hardware protocol ") !=
        std::string::npos) {
      warnings.push_back(line);
    }
  }
  ASSERT_EQ(warnings.size(), 1u) << err;
  EXPECT_NE(warnings.front().find(std::to_string(kBuild)), std::string::npos)
      << warnings.front();
  EXPECT_NE(warnings.front().find(std::to_string(kOther)), std::string::npos)
      << warnings.front();
  EXPECT_NE(warnings.front().find("read under the bridge's"), std::string::npos)
      << warnings.front();

  std::filesystem::remove_all(out);
  std::filesystem::remove_all(TempPath("protocol_disagree"));
}

TEST_F(ReplayInputsTest, RefusesToWriteIntoTheSessionItReads) {
  Record();
  const std::string session = CopyOfSession("refused_session");
  EXPECT_NE(Replay("replay_consumer", session, session), 0);
  EXPECT_FALSE(std::filesystem::exists(session + "/replay_consumer.tlog"));
  std::filesystem::remove_all(TempPath("refused_session"));
}

// REQ-OPS-14: --allow-capped-replay requires --replay-inputs on both the
// launcher and node binaries; with --replay-inputs, it forwards to the node so
// a byte-capped session replays deterministically from the kept window with a
// warning on stderr, while default replay still refuses it and uncapped
// log_gap sessions without EXIT are still refused.
TEST_F(ReplayInputsTest, AllowCappedReplayFlagEndToEnd) {
  // 1. Launcher refuses --allow-capped-replay without --replay-inputs.
  {
    LauncherOptions options = Options();
    options.config_path = config_;
    options.output_dir = TempPath("allow_capped_no_replay");
    options.allow_capped_replay = true;
    ::testing::internal::CaptureStderr();
    const int status = Launcher{options}.Run();
    const std::string err = ::testing::internal::GetCapturedStderr();
    EXPECT_EQ(status, 1) << err;
    EXPECT_NE(err.find("--allow-capped-replay"), std::string::npos) << err;
    EXPECT_NE(err.find("--replay-inputs"), std::string::npos) << err;
    std::filesystem::remove_all(options.output_dir);
  }

  // 2. Node binary refuses --allow-capped-replay without --replay-inputs.
  EXPECT_NE(RunNode(Runfile("TALOS_CONSUMER_NODE"),
                    {"--allow-capped-replay", "--config", config_}),
            0);

  // 3. Capped session: default refuses, --allow-capped-replay replays
  // deterministically and warns on stderr.
  Record();
  const std::string session = CopyOfSession("capped_session");
  const std::string source_log = session + "/replay_source.tlog";
  std::ofstream{event::log::CappedMarkerPath(source_log)}.close();

  const std::string refused_out = TempPath("capped_refused");
  std::filesystem::remove_all(refused_out);
  EXPECT_NE(Replay("replay_consumer", refused_out, session,
                   /*allow_capped_replay=*/false),
            0);
  std::filesystem::remove_all(refused_out);

  const std::string out1 = TempPath("capped_out1");
  const std::string out2 = TempPath("capped_out2");
  std::filesystem::remove_all(out1);
  std::filesystem::remove_all(out2);
  ASSERT_EQ(
      Replay("replay_consumer", out1, session, /*allow_capped_replay=*/true),
      0);
  ASSERT_EQ(
      Replay("replay_consumer", out2, session, /*allow_capped_replay=*/true),
      0);
  const std::vector<Sent> sends1 =
      Sends(out1 + "/1/replay_consumer.tlog", "/replay_consumer/state");
  ASSERT_FALSE(sends1.empty());
  EXPECT_EQ(Sends(out2 + "/1/replay_consumer.tlog", "/replay_consumer/state"),
            sends1);

  // Verify the node itself prints the capped-replay warning to stderr.
  {
    int pipefd[2];
    ASSERT_EQ(::pipe(pipefd), 0);
    const std::string direct_log = TempPath("capped_direct_out.tlog");
    std::filesystem::remove(direct_log);
    const pid_t pid = ::fork();
    ASSERT_GE(pid, 0);
    if (pid == 0) {
      ::close(pipefd[0]);
      ::dup2(pipefd[1], STDERR_FILENO);
      ::close(pipefd[1]);
      const std::string bin = Runfile("TALOS_CONSUMER_NODE");
      ::execl(bin.c_str(), bin.c_str(), "--replay-inputs", session.c_str(),
              "--allow-capped-replay", "--session-id", "1", "--log",
              direct_log.c_str(), "--config", config_.c_str(), nullptr);
      ::_exit(127);
    }
    ::close(pipefd[1]);
    std::string node_stderr;
    char buf[256];
    for (ssize_t n = 0; (n = ::read(pipefd[0], buf, sizeof(buf))) > 0;) {
      node_stderr.append(buf, static_cast<std::size_t>(n));
    }
    ::close(pipefd[0]);
    int status = 0;
    ASSERT_EQ(::waitpid(pid, &status, 0), pid);
    ASSERT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0) << node_stderr;
    EXPECT_NE(node_stderr.find(source_log), std::string::npos) << node_stderr;
    EXPECT_NE(node_stderr.find("kept window"), std::string::npos)
        << node_stderr;
    std::filesystem::remove(direct_log);
  }

  // 4. Uncapped log_gap without EXIT is still refused under
  // --allow-capped-replay.
  {
    const std::string gap_session = CopyOfSession("capped_with_log_gap");
    const std::string gap_source = gap_session + "/replay_source.tlog";
    std::uint64_t exit_offset = 0;
    {
      event::log::LogReader reader{gap_source};
      event::log::LogReader::Record rec;
      while (reader.next(rec)) {
        if (rec.header.kind ==
            static_cast<std::uint16_t>(event::EventKind::EXIT)) {
          exit_offset = rec.offset;
        }
      }
    }
    ASSERT_GT(exit_offset, 0u);
    std::filesystem::resize_file(gap_source, exit_offset);
    std::ofstream{event::log::CappedMarkerPath(gap_source)}.close();
    std::ofstream{gap_session + "/manifest.json"}
        << "{\n  \"session_id\": 1,\n  \"simulation\": true,\n  \"nodes\": "
           "[{\"name\": \"replay_source\", \"log_paths\": [\""
        << gap_source
        << "\"], \"log_gap\": true, \"log_capped\": true, "
           "\"recorder_failed\": false}]\n}\n";
    const std::string gap_out = TempPath("capped_gap_out");
    std::filesystem::remove_all(gap_out);
    EXPECT_NE(Replay("replay_consumer", gap_out, gap_session,
                     /*allow_capped_replay=*/true),
              0);
    std::filesystem::remove_all(gap_out);
    std::filesystem::remove_all(TempPath("capped_with_log_gap"));
  }

  std::filesystem::remove_all(out1);
  std::filesystem::remove_all(out2);
  std::filesystem::remove_all(TempPath("capped_session"));
}

}  // namespace
}  // namespace talos::launcher
