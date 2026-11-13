#include "talOS/launcher/roster.h"

#include <unistd.h>

#include <algorithm>
#include <utility>

#include "toml.hpp"

namespace talos::launcher {

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

std::string NormalizeTargetKey(std::string_view target) {
  std::string_view rest = target;
  if (rest.starts_with("@@")) {
    rest.remove_prefix(2);
  } else if (rest.starts_with("@")) {
    rest.remove_prefix(1);
  } else {
    return std::string{target};
  }
  const std::size_t slashes = rest.find("//");
  if (slashes == std::string_view::npos) return std::string{target};
  std::string_view repo = rest.substr(0, slashes);
  if (const std::size_t plus = repo.find('+'); plus != std::string_view::npos)
    repo = repo.substr(0, plus);
  if (repo.empty() || repo.find_first_of("@/:") != std::string_view::npos)
    return std::string{target};
  return std::string{repo} + "//" + std::string{rest.substr(slashes + 2)};
}

std::string ResolveBinary(const std::string& target_or_path,
                          const std::map<std::string, std::string>& overrides) {
  const std::string want = NormalizeTargetKey(target_or_path);
  for (const auto& [key, path] : overrides) {
    if (NormalizeTargetKey(key) == want) return path;
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
    // Bazel 8 Bzlmod places apparent @repo under bazel-bin/external/<repo>+/,
    // while canonical @@wrist+ already carries the trailing '+'.
    if (!canonical && !repo.ends_with('+')) repo += '+';
    std::string subpath = target_or_path.substr(slashes + 2);
    if (subpath.starts_with(':')) {
      subpath.erase(0, 1);
    } else {
      std::replace(subpath.begin(), subpath.end(), ':', '/');
    }
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

NodeSpec MakeNodeSpec(std::string_view name, const std::string& target,
                      const LauncherOptions& options, bool studio_bridge) {
  NodeSpec spec{std::string{name}, target,
                ResolveBinary(target, options.binary_overrides)};
  spec.studio_bridge = studio_bridge;
  if (::access(spec.binary_path.c_str(), X_OK) != 0 &&
      (studio_bridge || name != kStudioBridgeName)) {
    auto it = options.binary_overrides.find(spec.name);
    if (it != options.binary_overrides.end()) spec.binary_path = it->second;
  }
  if (name == kSimGatewayName) spec.extra_args = options.sim_gateway_args;
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

uint16_t EffectiveRemotePort(const config::RobotConfig& config,
                             const LauncherOptions& options) {
  return config::ResolveControllerPort(config, options.remote_port);
}

uint16_t EffectiveLocalPort(const config::RobotConfig& config,
                            const LauncherOptions& options) {
  return config::ResolveLocalPort(config, options.local_port);
}

uint16_t EffectiveStudioHttpPort(const config::RobotConfig& config,
                                 const LauncherOptions& options) {
  return options.studio_http_port != 0 ? options.studio_http_port
                                       : config.studio.http_port;
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
