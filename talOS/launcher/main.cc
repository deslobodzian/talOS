#include <unistd.h>

#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "talOS/launcher/launcher.h"

namespace {

int ParseIntFlag(std::string_view flag, std::string_view value, int min_value) {
  int parsed = 0;
  const char* const end = value.data() + value.size();
  const auto [ptr, ec] = std::from_chars(value.data(), end, parsed);
  if (value.empty() || ec != std::errc{} || ptr != end || parsed < min_value) {
    throw std::invalid_argument("invalid value for " + std::string{flag} +
                                ": " + std::string{value});
  }
  return parsed;
}

uint64_t ParseUint64Flag(std::string_view flag, std::string_view value) {
  uint64_t parsed = 0;
  const char* const end = value.data() + value.size();
  const auto [ptr, ec] = std::from_chars(value.data(), end, parsed);
  if (value.empty() || ec != std::errc{} || ptr != end) {
    throw std::invalid_argument("invalid value for " + std::string{flag} +
                                ": " + std::string{value});
  }
  return parsed;
}

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
      } else if (arg == "--local-address" && i + 1 < argc) {
        options.local_address = argv[++i];
      } else if (arg == "--remote-port" && i + 1 < argc) {
        options.remote_port = talos::config::ParsePortFlag(arg, argv[++i]);
      } else if (arg == "--local-port" && i + 1 < argc) {
        options.local_port = talos::config::ParsePortFlag(arg, argv[++i]);
      } else if (arg == "--duration-s" && i + 1 < argc) {
        options.duration_s = ParseIntFlag(arg, argv[++i], 0);
      } else if (arg == "--session-id" && i + 1 < argc) {
        options.session_id = ParseUint64Flag(arg, argv[++i]);
      } else if (arg == "--sim") {
        options.simulation = true;
      } else if (arg == "--start-sim-gateway") {
        options.start_sim_gateway = true;
      } else if (arg == "--sim-gateway-arg" && i + 1 < argc) {
        options.sim_gateway_args.emplace_back(argv[++i]);
      } else if (arg == "--start-studio-bridge") {
        options.start_studio_bridge = true;
      } else if (arg == "--serve-refusal") {
        options.serve_refusal = true;
      } else if (arg == "--studio-http-port" && i + 1 < argc) {
        options.studio_http_port =
            talos::config::ParsePortFlag(arg, argv[++i], /*allow_zero=*/true);
      } else if (arg == "--allow-graph-errors") {
        options.allow_graph_errors = true;
      } else if (arg == "--describe-only") {
        options.describe_only = true;
      } else if (arg == "--check-session-lock") {
        options.check_session_lock = true;
      } else if (arg == "--describe-as-launch") {
        options.describe_as_launch = true;
      } else if (arg == "--describe-timeout-ms" && i + 1 < argc) {
        options.describe_timeout_ms = ParseIntFlag(arg, argv[++i], 1);
      } else if (arg == "--replay-inputs" && i + 1 < argc) {
        options.replay_inputs = argv[++i];
      } else if (arg == "--only" && i + 1 < argc) {
        options.only = argv[++i];
      } else if (arg == "--allow-capped-replay") {
        options.allow_capped_replay = true;
      } else if (arg == "--max-restarts" && i + 1 < argc) {
        options.max_restarts = ParseIntFlag(arg, argv[++i], 0);
      } else if (arg == "--restart-window-ms" && i + 1 < argc) {
        options.restart_window_ms = ParseIntFlag(arg, argv[++i], 1);
      } else if (arg == "--heartbeat-periods" && i + 1 < argc) {
        options.heartbeat_periods = ParseIntFlag(arg, argv[++i], 1);
      } else if (arg == "--heartbeat-startup-grace-ms" && i + 1 < argc) {
        options.heartbeat_startup_grace_ms = ParseIntFlag(arg, argv[++i], 0);
      } else if (arg == "--help" || arg == "-h") {
        std::cout
            << "usage: launcher (--config PATH | --bundle PATH) "
               "[--output-dir PATH] "
               "[--duration-s N] [--session-id ID] [--sim] "
               "[--start-sim-gateway [--sim-gateway-arg ARG]...] "
               "[--start-studio-bridge] [--serve-refusal] "
               "[--studio-http-port PORT] "
               "[--describe-only [--describe-as-launch]] "
               "[--check-session-lock] "
               "[--allow-graph-errors] [--describe-timeout-ms MS] "
               "[--max-restarts N] "
               "[--remote IP] [--local-address IP] [--remote-port PORT] "
               "[--local-port PORT] "
               "[--replay-inputs SESSION_DIR --only NODE "
               "[--allow-capped-replay]]\n"
               "\n"
               "  --bundle PATH          launch from a bundle directory "
               "(assembled by talos_bundle)\n"
               "  --remote IP            controller address; overrides "
               "[robot] controller_address\n"
               "  --local-address IP     local bind IPv4 address; overrides "
               "[robot] local_address (default: wildcard 0.0.0.0)\n"
               "  --remote-port PORT     controller port; overrides "
               "[robot] controller_port (default 5802)\n"
               "  --local-port PORT      local bind port; overrides "
               "[robot] local_port (default 5803)\n"
               "  --sim-gateway-arg ARG  pass one argument verbatim to the "
               "gateway's probe and runtime;\n"
               "                         repeat for each argument, requires "
               "--start-sim-gateway\n"
               "  --studio-http-port PORT  Studio HTTP port; overrides "
               "[studio] http_port (0 lets\n"
               "                         the OS pick a free port, recorded in "
               "manifest.json)\n"
               "  --serve-refusal        with --start-studio-bridge, keep the "
               "Studio bridge running\n"
               "                         on a phase-1 refusal so /drive shows "
               "LAUNCH REFUSED instead\n"
               "                         of BRIDGE LINK until stopped (still "
               "exits 1)\n"
               "  --describe-only        ask every node in the config "
               "what it is, lint the\n"
               "                         graph they describe, and exit "
               "without starting a robot\n"
               "  --check-session-lock   test whether this namespace's "
               "session lock is free\n"
               "                         (exits 0 if free, 1 naming the "
               "holder if held)\n"
               "  --describe-as-launch   with --describe-only, also refuse "
               "what the same\n"
               "                         launch would refuse before its "
               "probes ([robot]\n"
               "                         simulation_only without --sim)\n"
               "  --allow-graph-errors   start anyway when the declared "
               "graph has errors;\n"
               "                         requires --sim\n"
               "  --max-restarts N       restarts per node per rolling window; "
               "overrides [robot] max_restarts (default 5, 0 disables)\n"
               "  --restart-window-ms MS rolling restart window (default "
               "60000)\n"
               "  --heartbeat-periods N  stall (no progress) timeout in "
               "node "
               "periods (default 10, minimum timeout 500ms)\n"
               "  --heartbeat-startup-grace-ms MS startup grace (default "
               "5000)\n"
               "  --replay-inputs DIR    run only the --only node, fed "
               "the topics the session\n"
               "                         in DIR logged at their "
               "recorded times; its log goes\n"
               "                         to --output-dir\n"
               "  --allow-capped-replay  with --replay-inputs, replay "
               "byte-capped logs from\n"
               "                         the start of their kept window "
               "(warns of pre-window\n"
               "                         loss; still refuses uncapped gaps)\n";
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
