#pragma once

// Shared telemetry configuration and dictionary validation used by both
// `talos::telemetry::Telemetry` (at node construction) and the launcher's
// phase-1 graph lint (`LintTelemetryConfig`), so `--describe-only` and
// construction enforce the exact same rules for `[[subsystem.labels]]`
// (REQ-OPS-38) and `[studio] dashboard` (REQ-OPS-35, REQ-OPS-39).

#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/describe.h"
#include "talOS/telemetry/dashboard_layout.h"
#include "talOS/telemetry/frames.h"

namespace talos::telemetry {

// Matches `name` (with or without a leading `/`) against any shell-style glob
// in `patterns` (`*` also matches `/`).
bool MatchesAnyGlob(std::string_view name,
                    const std::vector<std::string>& patterns);

// Whether `topic` is selected by `includes` and not excluded by `excludes`.
bool MatchesGlobs(std::string_view topic,
                  const std::vector<std::string>& includes,
                  const std::vector<std::string>& excludes);

// One validated `[[subsystem.labels]]` entry (keys `labels[<i>].channel` and
// `labels[<i>].from`): `channel` is named by `from`, another subsystem's
// string-array parameter.
struct LabelSpec {
  std::string key;  // "labels[<i>]", what an error names
  std::string channel;
  std::vector<std::string> labels;
};

// Optional hooks called when `ParseLabelSpecs` reads keys from the config, so
// `Telemetry` can register the exact same `Node::param` / `Node::param_of`
// reads for `--describe` (`LintParameterKeys`).
struct LabelParamHooks {
  std::function<void(std::string_view key)> on_own_string;
  std::function<void(std::string_view subsystem, std::string_view key)>
      on_from_string_list;
};

// Parses and validates every `[[subsystem.labels]]` entry under
// `[subsystems.<subsystem>]`, resolving each `from` (`<subsystem>.<key>`)
// against `config`. Throws `std::invalid_argument` naming the key
// (`labels`, `labels[<i>]`, `labels[<i>].channel`, `labels[<i>].from`, or an
// unknown key) for malformed entries, unknown keys, missing/non-string fields,
// a `from` that is not `<subsystem>.<key>`, a missing subsystem or parameter,
// a non-string-array value, an empty array, more than `kMaxEnumLabels` labels,
// or a label longer than `kMaxDictionaryStringBytes` bytes.
std::vector<LabelSpec> ParseLabelSpecs(const config::RobotConfig& config,
                                       std::string_view subsystem,
                                       const LabelParamHooks& hooks = {});

// Attaches each `LabelSpec`'s labels to its channel in `dictionary` and clears
// `specs`. Throws `std::invalid_argument` naming `labels[<i>].channel` for a
// channel absent from `dictionary` (excluded, or no state or status topic
// holds it), not an integer (`Int` or `UInt`), or already labelled (an enum or
// an earlier entry).
void ApplyLabelSpecs(std::vector<LabelSpec>& specs,
                     std::vector<ChannelDef>& dictionary);

// Validates `layout` against `dictionary` (talOS/telemetry/README.md
// "Dashboard layout"): throws `DashboardLayoutError` naming the file and key
// for any channel absent from `dictionary`, then runs `CheckDashboardBudget`
// for the whole drive feed (every dictionary channel `DriveFeedChannels`
// carries).
void CheckDashboardAgainstDictionary(const DashboardLayout& layout,
                                     const std::vector<ChannelDef>& dictionary);

// Reads an optional string-array parameter (`include` or `exclude`) from
// `[subsystems.<subsystem>]`, returning `fallback` when absent. Throws
// `std::invalid_argument` when present with a non-string-array value.
std::vector<std::string> ReadTelemetryGlobParam(
    const config::RobotConfig& config, std::string_view subsystem,
    std::string_view key, std::vector<std::string> fallback);

// Builds the telemetry dictionary that `Telemetry::BuildFromGraph` constructs
// from `graph` under `include_globs` and `exclude_globs` (excluding
// subscriptions and pose extraction).
std::vector<ChannelDef> BuildTelemetryDictionary(
    const introspect::DeclaredGraph& graph,
    const std::vector<std::string>& include_globs,
    const std::vector<std::string>& exclude_globs);

}  // namespace talos::telemetry
