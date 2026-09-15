#include <unistd.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "talOS/launcher/launcher.h"

namespace {

// Node targets resolve to bazel-bin/... paths, and a relative --config path
// is workspace-relative, so the launcher only makes sense with the
// workspace as its working directory. `bazel run` starts a binary in its
// runfiles tree instead, where neither resolves -- but it exports the
// workspace it was invoked from, so honour that and behave the same either way.
void EnterWorkspaceDirectory() {
  const char* workspace = std::getenv("BUILD_WORKSPACE_DIRECTORY");
  if (workspace == nullptr || *workspace == '\0') return;
  if (chdir(workspace) != 0) {
    std::cerr << "launcher: cannot enter workspace directory " << workspace
              << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    EnterWorkspaceDirectory();
    talos::launcher::LauncherOptions options;

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--config" && i + 1 < argc) {
        options.config_path = argv[++i];
      } else if (arg == "--bundle" && i + 1 < argc) {
        options.bundle_dir = argv[++i];
      } else if (arg == "--output-dir" && i + 1 < argc) {
        options.output_dir = argv[++i];
      } else if (arg == "--remote" && i + 1 < argc) {
        options.remote = argv[++i];
      } else if (arg == "--remote-port" && i + 1 < argc) {
        options.remote_port = static_cast<uint16_t>(std::stoi(argv[++i]));
      } else if (arg == "--local-port" && i + 1 < argc) {
        options.local_port = static_cast<uint16_t>(std::stoi(argv[++i]));
      } else if (arg == "--duration-s" && i + 1 < argc) {
        options.duration_s = std::stoi(argv[++i]);
      } else if (arg == "--session-id" && i + 1 < argc) {
        options.session_id = std::stoull(argv[++i]);
      } else if (arg == "--sim") {
        options.simulation = true;
      } else if (arg == "--start-sim-gateway") {
        options.start_sim_gateway = true;
      } else if (arg == "--start-studio-bridge") {
        options.start_studio_bridge = true;
      } else if (arg == "--studio-http-port" && i + 1 < argc) {
        options.studio_http_port = static_cast<uint16_t>(std::stoi(argv[++i]));
      } else if (arg == "--allow-graph-errors") {
        options.allow_graph_errors = true;
      } else if (arg == "--describe-only") {
        options.describe_only = true;
      } else if (arg == "--describe-timeout-ms" && i + 1 < argc) {
        options.describe_timeout_ms = std::stoi(argv[++i]);
      } else if (arg == "--help" || arg == "-h") {
        std::cout << "usage: launcher (--config PATH | --bundle PATH) [--output-dir PATH] "
                     "[--duration-s N] [--session-id ID] [--sim] "
                     "[--start-sim-gateway] [--start-studio-bridge] "
                     "[--studio-http-port PORT] [--describe-only] "
                     "[--allow-graph-errors] [--describe-timeout-ms MS] "
                     "[--remote IP] [--remote-port PORT] [--local-port PORT]\n"
                     "\n"
                     "  --bundle PATH          launch from a bundle directory (assembled by talos_bundle)\n"
                     "  --remote IP            controller address; overrides [robot] controller_address\n"
                     "  --remote-port PORT     controller port; overrides [robot] controller_port (default 5802)\n"
                     "  --local-port PORT      local bind port; overrides [robot] local_port (default 5803)\n"
                     "  --describe-only        ask every node in the config "
                     "what it is, lint the\n"
                     "                         graph they describe, and exit "
                     "without starting a robot\n"
                     "  --allow-graph-errors   start anyway when the declared "
                     "graph has errors\n";
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
// rebuild
