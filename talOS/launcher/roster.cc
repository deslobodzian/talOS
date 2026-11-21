#include "talOS/launcher/roster.h"

#include <unistd.h>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <utility>

#include "toml.hpp"

namespace talos::launcher {

namespace {

std::optional<std::string> ReadFileContents(const std::filesystem::path& path) {
  std::error_code ec;
  if (!std::filesystem::is_regular_file(path, ec)) return std::nullopt;
  std::ifstream input(path);
  if (!input.is_open()) return std::nullopt;
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::optional<std::string> ReadMappingBesideExecutable(
    const std::filesystem::path& exe_path) {
  if (exe_path.empty()) return std::nullopt;
  const std::string base = exe_path.string();
  if (auto content = ReadFileContents(
          std::filesystem::path{base + ".runfiles"} / "_repo_mapping")) {
    return content;
  }
  return ReadFileContents(std::filesystem::path{base + ".repo_mapping"});
}

std::filesystem::path CurrentExecutablePath() {
#ifdef __APPLE__
  uint32_t size = 0;
  _NSGetExecutablePath(nullptr, &size);
  if (size == 0) return {};
  std::string buffer(size, '\0');
  if (_NSGetExecutablePath(buffer.data(), &size) != 0) return {};
  return std::filesystem::path{buffer.c_str()};
#elif defined(__linux__)
  std::error_code ec;
  auto path = std::filesystem::read_symlink("/proc/self/exe", ec);
  if (ec) return {};
  return path;
#else
  return {};
#endif
}

}  // namespace

RepoMapping ParseRepoMapping(std::string_view content) {
  RepoMapping mapping;
  std::size_t pos = 0;
  while (pos < content.size()) {
    const std::size_t nl = content.find('\n', pos);
    std::string_view line = nl == std::string_view::npos
                                ? content.substr(pos)
                                : content.substr(pos, nl - pos);
    pos = nl == std::string_view::npos ? content.size() : nl + 1;
    if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
    const std::size_t first_comma = line.find(',');
    if (first_comma == std::string_view::npos) continue;
    const std::size_t second_comma = line.find(',', first_comma + 1);
    if (second_comma == std::string_view::npos) continue;
    // Only 3-column rows for the main repository (empty source_canonical).
    if (line.find(',', second_comma + 1) != std::string_view::npos) continue;
    const std::string_view source = line.substr(0, first_comma);
    const std::string_view apparent =
        line.substr(first_comma + 1, second_comma - first_comma - 1);
    const std::string_view canonical = line.substr(second_comma + 1);
    if (source.empty() && !apparent.empty() && !canonical.empty()) {
      mapping.emplace(std::string{apparent}, std::string{canonical});
    }
  }
  return mapping;
}

RepoMapping LoadRepoMapping(std::string_view argv0) {
  if (!argv0.empty()) {
    if (auto content =
            ReadMappingBesideExecutable(std::filesystem::path{argv0})) {
      return ParseRepoMapping(*content);
    }
    return {};
  }
  if (const char* runfiles_dir = std::getenv("RUNFILES_DIR");
      runfiles_dir != nullptr && *runfiles_dir != '\0') {
    if (auto content = ReadFileContents(std::filesystem::path{runfiles_dir} /
                                        "_repo_mapping")) {
      return ParseRepoMapping(*content);
    }
    return {};
  }
  if (auto content = ReadMappingBesideExecutable(CurrentExecutablePath())) {
    return ParseRepoMapping(*content);
  }
  return {};
}

bool LoadBundle(LauncherOptions& options, std::string& error) {
  if (options.bundle_dir.empty()) {
    if (!std::filesystem::exists(kBundleManifest)) return true;
    options.bundle_dir = ".";
  }

  const std::filesystem::path root{options.bundle_dir};
  if (!std::filesystem::exists(root)) {
    error = "bundle directory does not exist: " + options.bundle_dir;
    return false;
  }

  const std::filesystem::path manifest_path = root / kBundleManifest;
  if (std::filesystem::exists(manifest_path)) {
    toml::table manifest;
    try {
      manifest = toml::parse_file(manifest_path.string());
    } catch (const toml::parse_error& e) {
      error = "cannot read " + manifest_path.string() + ": " +
              std::string{e.description()};
      return false;
    }

    if (options.config_path.empty()) {
      if (auto config = manifest["config_path"].value<std::string>()) {
        options.config_path = (root / *config).string();
      }
    }
    options.bundle_config_sha256 =
        manifest["config_sha256"].value_or(std::string{});
    options.bundle_git_revision =
        manifest["git_revision"].value_or(std::string{});
    options.bundle_git_dirty = manifest["git_dirty"].value_or(false);
    options.bundle_talos_git_revision =
        manifest["talos_git_revision"].value_or(std::string{});
    options.bundle_talos_git_dirty =
        manifest["talos_git_dirty"].value_or(false);

    if (const auto* nodes = manifest["nodes"].as_array()) {
      for (const auto& entry : *nodes) {
        const auto* node = entry.as_table();
        if (node == nullptr) continue;
        const auto name = (*node)["name"].value<std::string>();
        if (!name) continue;
        std::vector<std::string> keys{*name};
        if (const auto* targets = (*node)["targets"].as_array()) {
          for (const auto& target : *targets) {
            if (auto label = target.value<std::string>())
              keys.push_back(*label);
          }
        }
        if (const auto binary = (*node)["binary"].value<std::string>()) {
          const std::string path = (root / *binary).string();
          for (const auto& key : keys) {
            options.binary_overrides[key] = path;
          }
        }
      }
    }

    if (const auto* studio = manifest["studio"].as_table()) {
      const auto bridge = (*studio)["bridge"].value<std::string>();
      const auto webroot = (*studio)["webroot"].value<std::string>();
      if (!bridge || !webroot) {
        error = manifest_path.string() +
                ": [studio] needs both `bridge` and `webroot`";
        return false;
      }
      const std::string path = (root / *bridge).string();
      options.binary_overrides[std::string{kStudioBridgeName}] = path;
      if (const auto* targets = (*studio)["targets"].as_array()) {
        for (const auto& target : *targets) {
          if (auto label = target.value<std::string>()) {
            options.binary_overrides[*label] = path;
          }
        }
      }
      options.bundle_has_studio = true;
      options.bundle_studio_webroot = std::filesystem::absolute(root / *webroot)
                                          .lexically_normal()
                                          .string();
    }
  }

  if (options.config_path.empty()) {
    const std::string default_config =
        (root / "config" / "robot.toml").string();
    if (std::filesystem::exists(default_config)) {
      options.config_path = default_config;
    }
  }
  return true;
}

std::string NormalizeTargetKey(std::string_view target,
                               const RepoMapping& repo_mapping) {
  std::string_view rest = target;
  bool canonical = false;
  if (rest.starts_with("@@")) {
    canonical = true;
    rest.remove_prefix(2);
  } else if (rest.starts_with("@")) {
    rest.remove_prefix(1);
  } else {
    return std::string{target};
  }
  const std::size_t slashes = rest.find("//");
  if (slashes == std::string_view::npos) return std::string{target};
  const std::string_view repo = rest.substr(0, slashes);
  const std::string_view path = rest.substr(slashes + 2);
  if (repo.empty() || repo.find_first_of("@/:") != std::string_view::npos) {
    return std::string{target};
  }
  if (!canonical) {
    if (const auto it = repo_mapping.find(std::string{repo});
        it != repo_mapping.end()) {
      if (it->second == "_main") return "//" + std::string{path};
      return it->second + "//" + std::string{path};
    }
  } else {
    if (repo == "_main") return "//" + std::string{path};
    for (const auto& [apparent, mapped] : repo_mapping) {
      if (mapped == repo) {
        return std::string{repo} + "//" + std::string{path};
      }
    }
  }
  // Fallback when `_repo_mapping` does not cover this repository: strip
  // Bazel 8 (`+`) or Bazel 7 (`~`) canonical suffixes so `@wrist` and
  // `@@wrist+` / `@@wrist~` compare equal.
  std::string_view base = repo;
  if (const std::size_t sep = base.find_first_of("+~");
      sep != std::string_view::npos) {
    base = base.substr(0, sep);
  }
  if (base.empty()) return std::string{target};
  return std::string{base} + "//" + std::string{path};
}

std::string NormalizeTargetKey(std::string_view target) {
  return NormalizeTargetKey(target, LoadRepoMapping());
}

std::string ResolveBinary(const std::string& target_or_path,
                          const std::map<std::string, std::string>& overrides,
                          const RepoMapping& repo_mapping) {
  const std::string want = NormalizeTargetKey(target_or_path, repo_mapping);
  for (const auto& [key, path] : overrides) {
    if (NormalizeTargetKey(key, repo_mapping) == want) return path;
  }
  if (target_or_path.starts_with("@")) {
    const bool canonical = target_or_path.starts_with("@@");
    const std::size_t repo_start = canonical ? 2 : 1;
    const std::size_t slashes = target_or_path.find("//", repo_start);
    const std::size_t colon = slashes == std::string::npos
                                  ? std::string::npos
                                  : target_or_path.find(':', slashes + 2);
    if (slashes == std::string::npos || colon == std::string::npos ||
        slashes == repo_start) {
      return target_or_path;
    }
    std::string repo = target_or_path.substr(repo_start, slashes - repo_start);
    if (!canonical) {
      if (const auto it = repo_mapping.find(repo); it != repo_mapping.end()) {
        repo = it->second;
      } else if (!repo.ends_with('+') && !repo.ends_with('~')) {
        // Fallback when no `_repo_mapping` covers this repo: Bazel 8 Bzlmod
        // places apparent `@repo` under `bazel-bin/external/<repo>+/`.
        repo += '+';
      }
    }
    std::string subpath = target_or_path.substr(slashes + 2);
    if (subpath.starts_with(':')) {
      subpath.erase(0, 1);
    } else {
      std::replace(subpath.begin(), subpath.end(), ':', '/');
    }
    if (repo == "_main") return "bazel-bin/" + subpath;
    return "bazel-bin/external/" + repo + "/" + subpath;
  }
  if (!target_or_path.starts_with("//")) return target_or_path;
  std::string subpath = target_or_path.substr(2);
  if (subpath.starts_with(':')) {
    subpath.erase(0, 1);
  } else {
    std::replace(subpath.begin(), subpath.end(), ':', '/');
  }
  return "bazel-bin/" + subpath;
}

std::string ResolveBinary(const std::string& target_or_path,
                          const std::map<std::string, std::string>& overrides) {
  return ResolveBinary(target_or_path, overrides, LoadRepoMapping());
}

NodeSpec MakeNodeSpec(std::string_view name, const std::string& target,
                      const LauncherOptions& options, bool studio_bridge) {
  const RepoMapping repo_mapping =
      options.repo_mapping ? *options.repo_mapping : LoadRepoMapping();
  NodeSpec spec{std::string{name}, target,
                ResolveBinary(target, options.binary_overrides, repo_mapping)};
  spec.studio_bridge = studio_bridge;
  if (::access(spec.binary_path.c_str(), X_OK) != 0 &&
      (studio_bridge || name != kStudioBridgeName)) {
    auto it = options.binary_overrides.find(spec.name);
    if (it != options.binary_overrides.end()) spec.binary_path = it->second;
  }
  return spec;
}

std::vector<NodeSpec> DiscoverNodes(const config::RobotConfig& robot_config,
                                    const LauncherOptions& options) {
  std::vector<NodeSpec> specs;
  const auto robot_string = [&](std::string_view key) {
    return robot_config.toml_data["robot"][key].value<std::string>();
  };

  // 1. The hardware bridge.
  specs.push_back(MakeNodeSpec(
      kHardwareNodeName,
      robot_string("hardware_node").value_or("//talOS/bridge:hardware_node"),
      options));
  specs.back().extra_args = ControllerArgs(robot_config, options);

  // 2. Subsystems.
  if (const auto* subsystems =
          robot_config.toml_data["subsystems"].as_table()) {
    for (auto&& [key, value] : *subsystems) {
      if (const auto* table = value.as_table()) {
        if (auto node = (*table)["node"].value<std::string>()) {
          auto spec = MakeNodeSpec(key.str(), *node, options);
          spec.critical = (*table)["critical"].value_or(true);
          if (const auto* devices =
                  robot_config.GetDevices(std::string{key.str()})) {
            spec.owns_actuators = devices->owns_actuators();
          }
          specs.push_back(std::move(spec));
        }
      }
    }
  }

  // 3. The sim gateway stands in for a controller processor, so which binary
  // that is belongs to the robot config, not to the framework.
  if (options.start_sim_gateway && options.simulation) {
    if (auto gateway = robot_string("sim_gateway")) {
      specs.push_back(MakeNodeSpec(kSimGatewayName, *gateway, options));
      specs.back().sim_gateway = true;
      specs.back().extra_args = SimulatorArgs(robot_config, options);
    }
  }

  // 4. The Studio bridge: a viewer named by [studio], in any session, so a
  // deployed robot is viewable too (an observer: see StudioBridge). A bundle
  // without Studio has none; probing a binary it lacks would fail the graph.
  if (options.start_studio_bridge && !BundleWithoutStudio(options)) {
    specs.push_back(MakeNodeSpec(kStudioBridgeName, robot_config.studio.bridge,
                                 options, /*studio_bridge=*/true));
  }
  return specs;
}

bool BundleWithoutStudio(const LauncherOptions& options) {
  return !options.bundle_dir.empty() && !options.bundle_has_studio;
}

std::string EffectiveRemoteAddress(const config::RobotConfig& config,
                                   const LauncherOptions& options) {
  return config::ResolveControllerAddress(config, options.remote);
}

std::string EffectiveLocalAddress(const config::RobotConfig& config,
                                  const LauncherOptions& options) {
  return config::ResolveLocalAddress(config, options.local_address);
}

uint16_t EffectiveRemotePort(const config::RobotConfig& config,
                             const LauncherOptions& options) {
  return config::ResolveControllerPort(config, options.remote_port);
}

uint16_t EffectiveLocalPort(const config::RobotConfig& config,
                            const LauncherOptions& options) {
  return config::ResolveLocalPort(config, options.local_port);
}

std::vector<std::string> ControllerArgs(const config::RobotConfig& config,
                                        const LauncherOptions& options) {
  std::vector<std::string> args{
      "--remote",      EffectiveRemoteAddress(config, options),
      "--remote-port", std::to_string(EffectiveRemotePort(config, options)),
      "--local-port",  std::to_string(EffectiveLocalPort(config, options))};
  if (const std::string local_address = EffectiveLocalAddress(config, options);
      !local_address.empty()) {
    args.emplace_back("--local-address");
    args.push_back(local_address);
  }
  return args;
}

std::vector<std::string> SimulatorArgs(const config::RobotConfig& config,
                                       const LauncherOptions& options) {
  std::vector<std::string> args{
      "--port", std::to_string(EffectiveRemotePort(config, options)),
      "--remote-port", std::to_string(EffectiveLocalPort(config, options))};
  args.insert(args.end(), options.sim_gateway_args.begin(),
              options.sim_gateway_args.end());
  return args;
}

bool IsSimulatorPortArg(std::string_view arg) {
  return arg == "--port" || arg == "--remote-port";
}

bool HasSimulatorScheduleStartArg(const std::vector<std::string>& args) {
  return std::any_of(args.begin(), args.end(), [](const std::string& arg) {
    return arg == "--schedule-start-file" ||
           arg.starts_with("--schedule-start-file=");
  });
}

uint16_t EffectiveStudioHttpPort(const config::RobotConfig& config,
                                 const LauncherOptions& options) {
  return options.studio_http_port.value_or(config.studio.http_port);
}

std::string EffectiveStudioWebroot(const config::RobotConfig& config,
                                   const LauncherOptions& options) {
  const std::string& configured = config.studio.webroot;
  if (!options.bundle_has_studio ||
      std::filesystem::path{configured}.is_absolute()) {
    return configured;
  }
  return options.bundle_studio_webroot;
}

}  // namespace talos::launcher
