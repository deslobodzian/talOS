// Build-time view of the runtime configuration resolver.
#include <filesystem>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "talOS/configuration/config_parser.h"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  try {
    if (argc < 2 || argc > 4) {
      throw std::invalid_argument(
          "usage: resolve CONFIG [OVERLAY_DIRECTORY [AVAILABLE_FILES]]");
    }
    const std::string config =
        fs::path(argv[1]).lexically_normal().generic_string();
    std::set<std::string> files;
    std::set<std::string> available;
    std::map<std::string, std::string> source_paths;
    if (argc == 4) {
      std::ifstream input(argv[3]);
      if (!input) throw std::invalid_argument("cannot read available files");
      for (std::string line; std::getline(input, line);) available.insert(line);
    }
    // Lists a file the config reads; under --check (AVAILABLE_FILES) a file
    // the manifest does not list is reported before anything opens it.
    auto track = [&](const std::string& path) {
      const fs::path normalized = fs::path(path).lexically_normal();
      const std::string name = normalized.generic_string();
      if (normalized.is_absolute() || name == ".." || name.starts_with("../")) {
        throw std::invalid_argument(
            path + ": absolute or workspace-leaving path has no Bazel label");
      }
      if (name != config) files.insert(name);
      const fs::path overlay =
          argc >= 3 ? fs::path(argv[2]) / normalized : fs::path{};
      if (argc == 4 && name != config && !available.contains(name) &&
          !fs::is_regular_file(overlay)) {
        throw std::invalid_argument("UNLISTED:" + name);
      }
      return std::pair{normalized, overlay};
    };
    auto read = [&](const std::string& path) {
      const auto [normalized, overlay] = track(path);
      const std::string name = normalized.generic_string();
      auto document = talos::config::detail::ParseTomlFile(
          !overlay.empty() && fs::is_regular_file(overlay) ? overlay.string()
                                                           : name);
      if (auto subsystem = document["subsystem"]["name"].value<std::string>()) {
        source_paths.try_emplace(*subsystem, name);
      }
      return document;
    };
    // A [trajectories] file is a dependency too, so the bundle ships it. A
    // path that has no Bazel label names the entry that set it.
    auto data = [&](const std::string& path, const std::string& label) {
      try {
        track(path);
      } catch (const std::invalid_argument& error) {
        const std::string message = error.what();
        if (message.starts_with("UNLISTED:")) throw;
        throw std::invalid_argument(label + ": " + message);
      }
    };
    auto effective = talos::config::ResolveRobotConfig(config, read, data);
    // Manifest/scaffold planning accepts incomplete hardware inventories while
    // sharing all parsing and schema validation with the launcher.
    talos::config::ParseRobotConfigTable(effective,
                                         /*validate_hardware=*/false);
    std::set<std::string> labels;
    toml::table subsystems;
    toml::table subsystem_paths;
    if (const auto* subs = effective["subsystems"].as_table()) {
      for (const auto& [name, value] : *subs) {
        const auto* table = value.as_table();
        if (auto node = (*table)["node"].value<std::string>())
          labels.insert(*node);
        subsystems.insert(std::string(name.str()), *table);
        const auto source = source_paths.find(std::string(name.str()));
        subsystem_paths.insert(
            std::string(name.str()),
            source == source_paths.end() ? config : source->second);
      }
    }
    toml::array nodes;
    for (const auto& label : labels) nodes.push_back(label);
    toml::array dependencies;
    for (const auto& file : files) dependencies.push_back(file);
    toml::table result;
    result.insert("effective", std::move(effective));
    result.insert("files", std::move(dependencies));
    result.insert("nodes", std::move(nodes));
    result.insert("subsystems", std::move(subsystems));
    result.insert("subsystem_paths", std::move(subsystem_paths));
    std::cout << toml::json_formatter{result} << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << (argc > 1 ? argv[1] : "resolve") << ": " << error.what()
              << '\n';
    return 1;
  }
}
