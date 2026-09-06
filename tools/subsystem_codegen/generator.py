"""Emit an ARCHITECTURE.md "A new subsystem" package from a Subsystem.

Step -> file mapping (nothing outside the package is touched):

  1. robot.toml stanza  -> returned by robot_stanza(), printed, never written
     into the robot config by this tool.
  2. packet.h           -> topic constants owned by the package (Rule 5: never
     node.h).
  3. <name>_message.fbs -> FlatBuffers structs (fixed size, never tables).
  4. node.h             -> template <typename Loop> class <Name>Node.
  5. main.cc            -> kNodeName/kNodeTarget once; --describe on a
     SimulatedEventLoop, --replay on a ReplayEventLoop, realtime otherwise;
     InstallStopHandlers(); Reporter built after the loop.
  6. node_test.cc       -> the node on a SimulatedEventLoop.
  7. BUILD              -> flatbuffer_cc_library + cc_library(<name>) +
     cc_binary(node) + cc_test. Rule named `node`, package leaf <name>.
"""
from pathlib import Path

from model import Subsystem, class_name

PACKAGE_ROOT = "2026-robot/main_processor"


def package_dir(subsystem: Subsystem) -> str:
    return f"{PACKAGE_ROOT}/{subsystem.name}"


def _toml_value(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return f'"{value}"'
    if isinstance(value, dict):
        inner = ", ".join(f"{k} = {_toml_value(v)}" for k, v in value.items())
        return "{ " + inner + " }"
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_toml_value(v) for v in value) + "]"
    return str(value)


def render_subsystem_toml(subsystem: Subsystem) -> str:
    lines = [
        "[subsystem]",
        f'name = "{subsystem.name}"',
        f'node = "{subsystem.node_target}"',
        f"period_us = {subsystem.period_us}",
    ]
    for key, value in subsystem.header_extra.items():
        lines.append(f"{key} = {_toml_value(value)}")
    kinds: dict[str, list] = {}
    for dev in subsystem.devices:
        kinds.setdefault(dev.kind, []).append(dev)
    for kind, devs in kinds.items():
        for dev in devs:
            lines.append("")
            lines.append(f"[{kind}.{dev.name}]")
            for key, value in dev.attrs.items():
                lines.append(f"{key} = {_toml_value(value)}")
    for section, table in subsystem.extra.items():
        lines.append("")
        lines.append(f"[{section}]")
        for key, value in table.items():
            lines.append(f"{key} = {_toml_value(value)}")
    return "\n".join(lines) + "\n"


def render_packet_h(subsystem: Subsystem) -> str:
    name = subsystem.name
    return (
        "#pragma once\n"
        "\n"
        '#include "talOS/hardware/packet.h"\n'
        "\n"
        f"namespace talos::{name} {{\n"
        "using Packet = hardware::Packet;\n"
        "\n"
        "inline constexpr const char* kHwStateTopic = hardware::kStateTopic;\n"
        f'inline constexpr const char* kStateTopic = "/{name}/state";\n'
        "// Written only by the arbiter; producers use the per-source topics below.\n"
        f'inline constexpr const char* kTargetTopic = "/{name}/target";\n'
        f'inline constexpr const char* kTeleopTargetTopic = "/{name}/target/teleop";\n'
        f'inline constexpr const char* kAutoTargetTopic = "/{name}/target/auto";\n'
        f"// hardware::kRequestTopicPrefix followed by the `[subsystems.{name}]` key,\n"
        "// which is how the bridge spells its end of the same topic.\n"
        f'inline constexpr const char* kRequestTopic = "/hw/request/{name}";\n'
        f"}}  // namespace talos::{name}\n"
    )


def render_fbs(subsystem: Subsystem) -> str:
    name = subsystem.name
    cls = class_name(name)
    return (
        f"namespace talos.{name};\n"
        "\n"
        f"struct {cls}Target {{\n"
        "  target_velocity_rps:double;\n"
        "  issued_ns:long;\n"
        "  enabled:bool;\n"
        "}\n"
        "\n"
        f"struct {cls}State {{\n"
        "  timestamp_ns:ulong;\n"
        "  velocity_rps:double;\n"
        "  target_velocity_rps:double;\n"
        "  at_target:bool;\n"
        "  enabled:bool;\n"
        "}\n"
    )


def render_node_h(subsystem: Subsystem) -> str:
    name = subsystem.name
    cls = class_name(name)
    pkg = package_dir(subsystem)
    L = "{"  # noqa: literal braces inside the template below are doubled
    return (
        "#pragma once\n"
        "\n"
        "#include <algorithm>\n"
        "#include <chrono>\n"
        "#include <cmath>\n"
        "#include <cstdint>\n"
        "#include <utility>\n"
        "\n"
        '#include "talOS/hardware/messages.h"\n'
        '#include "talOS/events/handles.h"\n'
        f'#include "{pkg}/packet.h"\n'
        f'#include "{pkg}/{name}_message_generated.h"\n'
        "\n"
        f"namespace talos::{name} {{\n"
        "\n"
        "template <typename Loop>\n"
        f"class {cls}Node {{\n"
        " public:\n"
        f"  explicit {cls}Node(Loop& loop, hardware::Config config,\n"
        "                     hardware::Devices devices = {})\n"
        "      : config_(std::move(config)),\n"
        "        devices_(std::move(devices)),\n"
        "        config_id_(hardware::ConfigurationId(config_)) {\n"
        "    // The first declared motor is the controlled joint. A subsystem with\n"
        "    // several joints replaces this lookup with one id per joint.\n"
        "    for (const auto& m : config_.motors) {\n"
        "      if (!devices_.motors.empty() &&\n"
        "          std::find(devices_.motors.begin(), devices_.motors.end(), m.id) ==\n"
        "              devices_.motors.end())\n"
        "        continue;\n"
        "      leader_id_ = m.id;\n"
        "      break;\n"
        "    }\n"
        "    if (leader_id_ == 0 && !devices_.motors.empty()) {\n"
        "      leader_id_ = devices_.motors[0];\n"
        "    }\n"
        "\n"
        f"    event::watch<Packet, &{cls}Node::OnState>(loop, kHwStateTopic, this);\n"
        f"    event::watch<{cls}Target, &{cls}Node::OnTarget>(loop, kTargetTopic, this);\n"
        "    request_ = event::make_sender<Packet>(loop, kRequestTopic);\n"
        f"    state_sender_ = event::make_sender<{cls}State>(loop, kStateTopic);\n"
        f'    timer_ = event::make_timer<&{cls}Node::Tick>(loop, "{name}", this);\n'
        "  }\n"
        "\n"
        "  void Start(event::MonotonicTime first) {\n"
        "    timer_.setup_periodic(first, std::chrono::microseconds{config_.period_us});\n"
        "  }\n"
        "\n"
        "  uint16_t leader_id() const { return leader_id_; }\n"
        "\n"
        " private:\n"
        "  void OnState(const event::Context& context, const Packet& packet) {\n"
        "    hardware::State state;\n"
        "    if (!hardware::Decode(packet.bytes(), state)) return;\n"
        "    if (config_id_ != 0 && state.config_id != config_id_) return;\n"
        "    if (have_state_ && state.boot_id == state_.boot_id &&\n"
        "        state.sample_time_us <= state_.sample_time_us) {\n"
        "      return;\n"
        "    }\n"
        "    state_ = state;\n"
        "    state_received_ = context.now;\n"
        "    have_state_ = true;\n"
        "  }\n"
        "\n"
        f"  void OnTarget(const event::Context&, const {cls}Target& target) {{\n"
        "    target_ = target;\n"
        "  }\n"
        "\n"
        "  void Tick(const event::Context& context) {\n"
        "    if (!have_state_) return;\n"
        "\n"
        "    const auto timeout = std::chrono::microseconds{config_.command_timeout_us};\n"
        "    const auto issued = event::MonotonicTime::from_nanos(target_.issued_ns());\n"
        "\n"
        "    const bool gateway_ok =\n"
        "        (state_.flags & (hardware::kConfigured | hardware::kEnabled)) ==\n"
        "            (hardware::kConfigured | hardware::kEnabled) &&\n"
        "        !(state_.flags & hardware::kHardwareFault);\n"
        "\n"
        "    const bool target_fresh = target_.enabled() && issued <= context.now &&\n"
        "                              context.now - issued < timeout &&\n"
        "                              context.now - state_received_ < timeout;\n"
        "\n"
        "    double leader_velocity_rps = 0.0;\n"
        "    bool leader_sample_valid = false;\n"
        "    for (std::size_t i = 0; i < state_.motor_count; ++i) {\n"
        "      if (state_.motors[i].id == leader_id_) {\n"
        "        leader_sample_valid = state_.motors[i].valid;\n"
        "        leader_velocity_rps = state_.motors[i].velocity_rps;\n"
        "        break;\n"
        "      }\n"
        "    }\n"
        "\n"
        "    const bool valid = gateway_ok && target_fresh && leader_sample_valid &&\n"
        "                       std::isfinite(target_.target_velocity_rps());\n"
        "\n"
        "    hardware::Command command;\n"
        "    command.config_id = state_.config_id;\n"
        "    command.boot_id = state_.boot_id;\n"
        "    command.epoch = state_.epoch;\n"
        "    command.observed_time_us = state_.sample_time_us;\n"
        "    command.count = 1;\n"
        "    command.motors[0].id = leader_id_;\n"
        "    command.motors[0].slot = 0;\n"
        "    if (valid) {\n"
        "      command.motors[0].mode = hardware::Mode::kVelocity;\n"
        "      command.motors[0].demand = target_.target_velocity_rps();\n"
        "      command.motors[0].feedforward_v = 0.0;\n"
        "    } else {\n"
        "      command.motors[0].mode = hardware::Mode::kNeutral;\n"
        "      command.motors[0].demand = 0.0;\n"
        "      command.motors[0].feedforward_v = 0.0;\n"
        "    }\n"
        "\n"
        "    Packet packet{};\n"
        "    packet.size = static_cast<uint32_t>(hardware::Encode(command, packet.data));\n"
        "    if (packet.size > 0) {\n"
        "      request_.send(packet);\n"
        "    }\n"
        "\n"
        "    const double target_vel = valid ? target_.target_velocity_rps() : 0.0;\n"
        "    constexpr double kAtTargetToleranceRps = 2.0;\n"
        "    const bool at_target =\n"
        "        valid && std::abs(target_.target_velocity_rps()) > 1e-3 &&\n"
        "        std::abs(leader_velocity_rps - target_.target_velocity_rps()) <=\n"
        "            kAtTargetToleranceRps;\n"
        "    const bool enabled =\n"
        "        (state_.flags & (hardware::kConfigured | hardware::kEnabled)) ==\n"
        "        (hardware::kConfigured | hardware::kEnabled);\n"
        "\n"
        f"    {cls}State out{{state_.sample_time_us * 1000, leader_velocity_rps,\n"
        "                    target_vel, at_target, enabled};\n"
        "    state_sender_.send(out);\n"
        "  }\n"
        "\n"
        "  hardware::Config config_;\n"
        "  hardware::Devices devices_;\n"
        "  uint64_t config_id_{0};\n"
        "  uint16_t leader_id_{0};\n"
        "  hardware::State state_{};\n"
        "  bool have_state_{false};\n"
        "  event::MonotonicTime state_received_{};\n"
        f"  {cls}Target target_{{}};\n"
        "  event::Sender<Loop, Packet> request_;\n"
        f"  event::Sender<Loop, {cls}State> state_sender_;\n"
        "  event::Timer<Loop> timer_;\n"
        "};\n"
        "\n"
        f"}}  // namespace talos::{name}\n"
    )


def render_main_cc(subsystem: Subsystem) -> str:
    name = subsystem.name
    cls = class_name(name)
    pkg = package_dir(subsystem)
    return (
        "#include <cstdio>\n"
        "#include <stdexcept>\n"
        "#include <string>\n"
        "#include <thread>\n"
        "\n"
        f'#include "{pkg}/node.h"\n'
        '#include "talOS/configuration/config_parser.h"\n'
        '#include "talOS/events/log/log_reader.h"\n'
        '#include "talOS/events/log/log_writer.h"\n'
        '#include "talOS/events/realtime_event_loop.h"\n'
        '#include "talOS/events/replay_event_loop.h"\n'
        '#include "talOS/events/simulated_event_loop.h"\n'
        '#include "talOS/introspection/describe.h"\n'
        '#include "talOS/introspection/reporter.h"\n'
        '#include "talOS/process/process.h"\n'
        "\n"
        "namespace {\n"
        "// This node's identity, written once. `--describe` and the live registry row\n"
        "// have to name the same node and the same build target, or a viewer cannot tell\n"
        "// a graph that changed from a graph it is reading two different names for.\n"
        f'constexpr const char* kNodeName = "{name}";\n'
        f'constexpr const char* kNodeTarget = "{subsystem.node_target}";\n'
        "}  // namespace\n"
        "\n"
        "int main(int argc, char** argv) {\n"
        "  try {\n"
        "    bool simulation = false;\n"
        "    bool describe = false;\n"
        f'    std::string log_path = "/tmp/{name}.tlog", replay_path;\n'
        "    std::string config_path =\n"
        '        "2026-robot/main_processor/configuration/robot.toml";\n'
        "    int duration_s = 0;\n"
        "    uint64_t session_id = 0;\n"
        "    for (int i = 1; i < argc; ++i) {\n"
        "      const std::string arg = argv[i];\n"
        '      if (arg == "--sim")\n'
        "        simulation = true;\n"
        '      else if (arg == "--describe")\n'
        "        describe = true;\n"
        '      else if (arg == "--log" && i + 1 < argc)\n'
        "        log_path = argv[++i];\n"
        '      else if (arg == "--session-id" && i + 1 < argc)\n'
        "        session_id = std::stoull(argv[++i]);\n"
        '      else if (arg == "--config" && i + 1 < argc)\n'
        "        config_path = argv[++i];\n"
        '      else if (arg == "--replay" && i + 1 < argc)\n'
        "        replay_path = argv[++i];\n"
        '      else if (arg == "--duration-s" && i + 1 < argc)\n'
        "        duration_s = std::stoi(argv[++i]);\n"
        "      else\n"
        "        throw std::invalid_argument(\n"
        '            "usage: node [--sim] [--describe] [--duration-s N] [--log PATH] "\n'
        '            "[--session-id ID] [--config PATH] [--replay PATH]");\n'
        "    }\n"
        "    if (duration_s < 0)\n"
        '      throw std::invalid_argument("duration must be nonnegative");\n'
        "    auto robot_config = talos::config::ParseRobotConfig(config_path);\n"
        "    if (simulation) {\n"
        "      robot_config.hardware.commissioned = true;\n"
        "    }\n"
        f'    const auto* dev_ptr = robot_config.GetDevices("{name}");\n'
        "    talos::hardware::Devices devices =\n"
        "        dev_ptr ? *dev_ptr : talos::hardware::Devices{};\n"
        "    auto config = robot_config.hardware;\n"
        "    // --describe builds the node for real and prints what its constructor\n"
        "    // registered. The loop is the only difference: a simulated loop's channels\n"
        "    // live in this process, so no shared memory, no hardware and no network is\n"
        "    // touched, and the launcher can ask what a node is without starting a\n"
        "    // robot. Describing from the same constructor is the point -- a topology\n"
        "    // written down a second time is a topology to forget to update.\n"
        "    if (describe) {\n"
        "      talos::event::SimulationEnvironment environment;\n"
        "      talos::event::SimulatedEventLoop<> loop{environment};\n"
        f"      talos::{name}::{cls}Node node{{loop, config, devices}};\n"
        '      std::printf("%s", talos::introspect::DescribeToJson(\n'
        "                            talos::introspect::DescribeManifest(\n"
        "                                kNodeName, kNodeTarget, loop.manifest()))\n"
        "                            .c_str());\n"
        "      return 0;\n"
        "    }\n"
        "    if (!replay_path.empty()) {\n"
        "      talos::event::log::LogReader reader{replay_path};\n"
        "      talos::event::ReplayEventLoop<> loop{reader};\n"
        f"      talos::{name}::{cls}Node node{{loop, config, devices}};\n"
        "      node.Start(loop.monotonic_now() +\n"
        "                 std::chrono::microseconds{config.period_us});\n"
        "      loop.run();\n"
        '      std::printf("replay: diverged=%d clean_exit=%d\\n", loop.diverged(),\n'
        "                  loop.reached_exit());\n"
        "      return loop.diverged() || !loop.reached_exit() ? 1 : 0;\n"
        "    }\n"
        "    talos::event::RealtimeEventLoop<talos::event::log::LogWriter> loop{\n"
        f'        talos::event::log::LogWriter{{log_path, "{name}",\n'
        "                                     talos::event::log::LogWriterOptions{},\n"
        "                                     session_id}}};\n"
        f"    talos::{name}::{cls}Node node{{loop, config, devices}};\n"
        "    node.Start(loop.monotonic_now() +\n"
        "               std::chrono::microseconds{config.period_us});\n"
        "    talos::process::InstallStopHandlers();\n"
        "    // Publishes this node's topics and activity into the live registry, so\n"
        "    // Studio and any agent can see what is running without being told.\n"
        "    // Declared after the loop: reverse destruction order joins its thread\n"
        "    // before the counters it samples are destroyed.\n"
        "    talos::introspect::Reporter reporter{loop,\n"
        "                                         {.name = kNodeName,\n"
        "                                          .target = kNodeTarget,\n"
        "                                          .session_id = session_id,\n"
        "                                          .simulation = simulation}};\n"
        "    std::jthread stopper{[&](std::stop_token stop) {\n"
        "      while (!stop.stop_requested()) {\n"
        "        if (talos::process::stop_requested.load() && loop.running()) {\n"
        "          loop.exit();\n"
        "          return;\n"
        "        }\n"
        "        std::this_thread::sleep_for(std::chrono::milliseconds{20});\n"
        "      }\n"
        "    }};\n"
        "    if (duration_s)\n"
        "      loop.run_for(std::chrono::seconds{duration_s});\n"
        "    else\n"
        "      loop.run();\n"
        "    stopper.request_stop();\n"
        f'    std::printf("{name}: dispatches=%llu recording_failed=%d\\n",\n'
        "                static_cast<unsigned long long>(loop.dispatch_count()),\n"
        "                loop.recorder().failed());\n"
        "    if (loop.recorder().failed()) {\n"
        '      std::fprintf(stderr, "%s\\n", loop.recorder().error().c_str());\n'
        "      return 1;\n"
        "    }\n"
        "    return 0;\n"
        "  } catch (const std::exception& e) {\n"
        '    std::fprintf(stderr, "%s\\n", e.what());\n'
        "    return 1;\n"
        "  }\n"
        "}\n"
    )


def render_node_test_cc(subsystem: Subsystem) -> str:
    name = subsystem.name
    cls = class_name(name)
    pkg = package_dir(subsystem)
    return (
        f'#include "{pkg}/node.h"\n'
        "\n"
        "#include <gtest/gtest.h>\n"
        "\n"
        '#include "talOS/configuration/config_parser.h"\n'
        '#include "talOS/events/simulated_event_loop.h"\n'
        '#include "talOS/hardware/sim_backend.h"\n'
        "\n"
        f"namespace talos::{name} {{\n"
        "namespace {\n"
        "using namespace std::chrono_literals;\n"
        "\n"
        "config::RobotConfig TestRobotConfig() {\n"
        "  auto cfg = config::ParseRobotConfig(\n"
        '      "2026-robot/main_processor/configuration/robot.toml");\n'
        "  cfg.hardware.commissioned = true;\n"
        "  return cfg;\n"
        "}\n"
        "\n"
        f"TEST({cls}Node, RegistersTwoWatchersTwoSendersAndTimer) {{\n"
        "  const auto robot_cfg = TestRobotConfig();\n"
        "  event::SimulationEnvironment environment;\n"
        "  event::SimulatedEventLoop<> loop{environment};\n"
        f"  {cls}Node node{{loop, robot_cfg.hardware}};\n"
        "  node.Start(loop.monotonic_now());\n"
        "  // Two watchers (hw state + target), two senders (request + state),\n"
        "  // one periodic timer. Registration order defines source ids, so any\n"
        "  // reordering invalidates existing logs.\n"
        "  EXPECT_EQ(loop.manifest().size(), 5u);\n"
        "}\n"
        "\n"
        "}  // namespace\n"
        f"}}  // namespace talos::{name}\n"
    )


def render_build(subsystem: Subsystem) -> str:
    name = subsystem.name
    return (
        'load("@rules_cc//cc:cc_library.bzl", "cc_library")\n'
        'load("@rules_cc//cc:cc_binary.bzl", "cc_binary")\n'
        'load("@rules_cc//cc:cc_test.bzl", "cc_test")\n'
        'load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")\n'
        "\n"
        "flatbuffer_cc_library(\n"
        f'    name = "{name}_message",\n'
        f'    srcs = ["{name}_message.fbs"],\n'
        '    visibility = ["//visibility:public"],\n'
        ")\n"
        "\n"
        "cc_library(\n"
        f'    name = "{name}",\n'
        '    hdrs = ["packet.h", "node.h"],\n'
        "    deps = [\n"
        f'        ":{name}_message",\n'
        '        "//talOS/hardware:hardware",\n'
        '        "//talOS/configuration:configuration",\n'
        '        "//talOS/events:events",\n'
        "    ],\n"
        '    visibility = ["//visibility:public"],\n'
        ")\n"
        "\n"
        "cc_binary(\n"
        '    name = "node",\n'
        '    srcs = ["main.cc"],\n'
        "    deps = [\n"
        f'        ":{name}",\n'
        '        "//talOS/configuration:configuration",\n'
        '        "//talOS/events:events",\n'
        '        "//talOS/introspection:describe",\n'
        '        "//talOS/introspection:names",\n'
        '        "//talOS/introspection:reporter",\n'
        '        "//talOS/process:process",\n'
        "    ],\n"
        '    visibility = ["//visibility:public"],\n'
        ")\n"
        "\n"
        "cc_test(\n"
        '    name = "node_test",\n'
        '    size = "small",\n'
        '    srcs = ["node_test.cc"],\n'
        '    data = ["//2026-robot/main_processor/configuration:robot.toml"],\n'
        "    deps = [\n"
        f'        ":{name}",\n'
        '        "@googletest//:gtest_main",\n'
        "    ],\n"
        ")\n"
    )


def robot_stanza(subsystem: Subsystem, toml_relpath: str | None = None) -> str:
    """Step 1: the manifest block to merge into robot.toml. Printed, never
    auto-applied: adding a device renumbers logical ids, so the config golden
    test must be updated alongside by a human."""
    rel = toml_relpath or f"../{subsystem.name}/subsystem.toml"
    return (
        "[[subsystems]]\n"
        f'name = "{subsystem.name}"\n'
        f'path = "{rel}"\n'
    )


def generate(subsystem: Subsystem, out_dir: Path | str) -> list[Path]:
    """Write the package; return every file written, in ARCHITECTURE.md order."""
    dest = Path(out_dir) / subsystem.name
    dest.mkdir(parents=True, exist_ok=True)
    files = {
        "subsystem.toml": render_subsystem_toml(subsystem),
        "packet.h": render_packet_h(subsystem),
        f"{subsystem.name}_message.fbs": render_fbs(subsystem),
        "node.h": render_node_h(subsystem),
        "main.cc": render_main_cc(subsystem),
        "node_test.cc": render_node_test_cc(subsystem),
        "BUILD": render_build(subsystem),
    }
    written = []
    for filename, content in files.items():
        path = dest / filename
        path.write_text(content)
        written.append(path)
    return written
