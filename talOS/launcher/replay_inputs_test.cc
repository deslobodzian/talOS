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
// the bridge's /hw/status with the clock mapping for that boot. No Driver
// Station.
class RealGateway {
 public:
  static constexpr std::uint64_t kBootId = 1989997566370484863ULL;
  static constexpr std::int64_t kGatewayBehindUs = 195'000'000;

  RealGateway(GatewayLoop& loop, const config::RobotConfig& config)
      : config_id_{hardware::ConfigurationId(config.hardware)},
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
      status_.send(status);
    }
    ++ticks_;
  }

  std::uint64_t config_id_;
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
    std::ofstream{stub} << "#!/bin/sh\n"
                        << "for arg in \"$@\"; do\n"
                        << "  if [ \"$arg\" = \"--describe\" ]; then\n"
                        << "    cat <<'TALOS_DESCRIBE'\n"
                        << introspect::DescribeToJson(hardware)
                        << "TALOS_DESCRIBE\n"
                        << "    exit 0\n"
                        << "  fi\n"
                        << "done\n"
                        // Alive until the session ends: an exited stub
                        // restarts, and the end's SIGINT landing before the
                        // restart's exec makes the session status nonzero.
                        << "trap 'exit 0' INT TERM\n"
                        << "while :; do sleep 0.05; done\n";
    std::filesystem::permissions(stub, std::filesystem::perms::owner_all);
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
             const std::string& session) {
    LauncherOptions options = Options();
    options.config_path = config_;
    options.output_dir = out_dir;
    options.replay_inputs = session;
    options.only = node;
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

TEST_F(ReplayInputsTest, AReaderOfAnotherSchemaIsRefused) {
  Record();
  const std::string out = TempPath("impostor");
  std::filesystem::remove_all(out);
  EXPECT_NE(Replay("replay_impostor", out), 0);

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

TEST_F(ReplayInputsTest, RefusesToWriteIntoTheSessionItReads) {
  Record();
  const std::string session = CopyOfSession("refused_session");
  EXPECT_NE(Replay("replay_consumer", session, session), 0);
  EXPECT_FALSE(std::filesystem::exists(session + "/replay_consumer.tlog"));
  std::filesystem::remove_all(TempPath("refused_session"));
}

}  // namespace
}  // namespace talos::launcher
