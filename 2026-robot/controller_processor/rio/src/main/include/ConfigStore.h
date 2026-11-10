#pragma once

#include <optional>
#include <string>

#include "hardware/config.h"
#include "hardware/gateway.h"

// The RoboRIO's copy of the last acknowledged hardware configuration, so a
// reboot (a brownout mid-match) drives again without waiting for the companion
// to push it.
namespace talos::rio {

// The persisted configuration, or nullopt when the file is missing or fails
// its CRC, decoding, ceilings or validation.
std::optional<hardware::Config> LoadConfigFile(const std::string& path);

// Writes the configuration durably and atomically: a synced temporary file is
// renamed over `path` and the directory is synced, so a power loss leaves the
// previous file or the new one, never a torn one. Blocks on storage: call it
// only while disabled.
bool SaveConfigFile(const std::string& path, const hardware::Config& config);

// Starts restoring the persisted configuration on the backend's configure
// worker and returns it, or nullopt when none was started. It never waits for
// device traffic, so robot code reports ready at once; until the worker
// finishes the gateway is unconfigured, refuses every command and drives
// nothing.
std::optional<hardware::Config> BeginRestore(hardware::Gateway& gateway,
                                             const std::string& path);

}  // namespace talos::rio
