#include "talOS/launcher/log_inputs.h"

#include <filesystem>
#include <set>
#include <string>
#include <system_error>
#include <vector>

#include "talOS/events/log/log_reader.h"

namespace talos::launcher {

std::vector<std::string> LogInputFiles(const std::vector<std::string>& inputs) {
  std::vector<std::string> files;
  std::set<std::string> seen;
  const auto add = [&](const std::string& file) {
    std::error_code ec;
    const std::filesystem::path canonical =
        std::filesystem::weakly_canonical(file, ec);
    if (seen.insert(ec ? file : canonical.string()).second) {
      files.push_back(file);
    }
  };
  for (const auto& input : inputs) {
    std::error_code ec;
    if (std::filesystem::is_directory(input, ec)) {
      for (const auto& file : event::log::FindLogFiles(input)) add(file);
      continue;
    }
    const auto found = event::log::LogSegmentFiles(input);
    if (found.empty()) add(input);  // Reported when opened.
    for (const auto& file : found) add(file);
  }
  return files;
}

}  // namespace talos::launcher
