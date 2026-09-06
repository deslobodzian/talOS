#include <iostream>
#include <stdexcept>
#include <string>

#include "talOS/launcher/launcher.h"

int main(int argc, char** argv) {
  try {
    talos::launcher::LauncherOptions options;

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--config" && i + 1 < argc) {
        options.config_path = argv[++i];
      } else if (arg == "--output-dir" && i + 1 < argc) {
        options.output_dir = argv[++i];
      } else if (arg == "--duration-s" && i + 1 < argc) {
        options.duration_s = std::stoi(argv[++i]);
      } else if (arg == "--session-id" && i + 1 < argc) {
        options.session_id = std::stoull(argv[++i]);
      } else if (arg == "--sim") {
        options.simulation = true;
      } else if (arg == "--start-sim-gateway") {
        options.start_sim_gateway = true;
      } else if (arg == "--help" || arg == "-h") {
        std::cout << "usage: launcher [--config PATH] [--output-dir PATH] "
                     "[--duration-s N] [--session-id ID] [--sim] "
                     "[--start-sim-gateway]\n";
        return 0;
      } else {
        throw std::invalid_argument("unknown argument: " + arg);
      }
    }

    talos::launcher::Launcher launcher{std::move(options)};
    return launcher.Run();
  } catch (const std::exception& e) {
    std::cerr << "launcher error: " << e.what() << "\n";
    return 1;
  }
}
