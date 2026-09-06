#include <cstdio>
#include <stdexcept>
#include <string>

#include "node.h"

int main(int argc, char** argv) {
  try {
    std::string config_path = "talOS/configuration/robot.toml";
    std::string remote_ip = "127.0.0.1";
    uint16_t remote_port = 5802;
    uint16_t local_port = 5803;
    int duration_s = 0;
    bool simulation = false;

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
      } else if (arg == "--log" && i + 1 < argc) {
        ++i;  // Optional log path
      } else if (arg == "--session-id" && i + 1 < argc) {
        ++i;  // Session ID
      } else {
        throw std::invalid_argument(
            "usage: hardware_node [--config PATH] [--remote IP] [--duration-s N] [--sim] [--log PATH] [--session-id ID]");
      }
    }

    auto robot_config = talos::config::ParseRobotConfig(config_path);
    talos::hardware::HardwareNode node{std::move(robot_config), remote_ip,
                                       remote_port, local_port, simulation};
    if (!node.Open()) {
      throw std::runtime_error("failed to open hardware node network/ipc resources");
    }

    return node.Run(duration_s);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "hardware_node error: %s\n", e.what());
    return 1;
  }
}
