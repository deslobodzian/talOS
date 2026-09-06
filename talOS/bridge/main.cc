#include <cstdio>
#include <stdexcept>
#include <string>

#include "node.h"
#include "talOS/introspection/describe.h"

int main(int argc, char** argv) {
  try {
    // No default: the framework names no robot (ARCHITECTURE.md Rule 1), so
    // --config is required. Empty means missing.
    std::string config_path;
    std::string remote_ip = "127.0.0.1";
    uint16_t remote_port = 5802;
    uint16_t local_port = 5803;
    int duration_s = 0;
    bool simulation = false;
    uint64_t session_id = 0;
    bool describe = false;

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--config" && i + 1 < argc) {
        config_path = argv[++i];
      } else if (arg == "--remote" && i + 1 < argc) {
        remote_ip = argv[++i];
      } else if (arg == "--remote-port" && i + 1 < argc) {
        remote_port = static_cast<uint16_t>(std::stoi(argv[++i]));
      } else if (arg == "--local-port" && i + 1 < argc) {
        local_port = static_cast<uint16_t>(std::stoi(argv[++i]));
      } else if (arg == "--duration-s" && i + 1 < argc) {
        duration_s = std::stoi(argv[++i]);
      } else if (arg == "--sim") {
        simulation = true;
      } else if (arg == "--describe") {
        describe = true;
      } else if (arg == "--log" && i + 1 < argc) {
        ++i;  // Optional log path
      } else if (arg == "--session-id" && i + 1 < argc) {
        session_id = std::stoull(argv[++i]);
      } else {
        throw std::invalid_argument(
            "usage: hardware_node --config PATH [--remote IP] "
            "[--duration-s N] [--sim] [--log PATH] [--session-id ID] "
            "[--describe]");
      }
    }

    if (config_path.empty()) {
      throw std::invalid_argument(
          "usage: hardware_node --config PATH [--remote IP] "
          "[--duration-s N] [--sim] [--log PATH] [--session-id ID] "
          "[--describe]");
    }

    auto robot_config = talos::config::ParseRobotConfig(config_path);
    talos::hardware::HardwareNode node{std::move(robot_config),
                                       remote_ip,
                                       remote_port,
                                       local_port,
                                       simulation,
                                       session_id};

    // --describe answers out of the constructor alone: no socket, no shared
    // memory, no registry slot, nothing the real node holds. That is what lets
    // the launcher ask every binary in a config what it would connect to
    // before it has started any of them, and refuse a graph whose ends do not
    // meet rather than discover it from a robot that does not move.
    if (describe) {
      std::fputs(talos::introspect::DescribeToJson(node.Describe()).c_str(),
                 stdout);
      return 0;
    }

    if (!node.Open()) {
      throw std::runtime_error(
          "failed to open hardware node network/ipc resources");
    }

    return node.Run(duration_s);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "hardware_node error: %s\n", e.what());
    return 1;
  }
}
