#include "talOS/telemetry/dashboard_layout.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <set>
#include <sstream>
#include <utility>

#include "talOS/introspection/names.h"
#include "talOS/telemetry/wire.h"
#include "toml.hpp"

namespace talos::telemetry {
namespace {

using Kind = DashboardWidgetKind;

constexpr std::array<std::pair<Kind, std::string_view>, 11> kKinds{{
    {Kind::kMatch, "match"},
    {Kind::kPower, "power"},
    {Kind::kRoutine, "routine"},
    {Kind::kAlerts, "alerts"},
    {Kind::kSubsystems, "subsystems"},
    {Kind::kField, "field"},
    {Kind::kNumber, "number"},
    {Kind::kBoolean, "boolean"},
    {Kind::kText, "text"},
    {Kind::kGraph, "graph"},
    {Kind::kChecks, "checks"},
}};

// Kinds whose slice has not landed: refused by name, not as unknown.
constexpr std::array<std::string_view, 4> kReservedKinds{"camera", "mechanism",
                                                         "chooser", "tunable"};

// The Driver Station channels a `match` widget reads (channels.h
// DriverStationChannels names them), and the /hw/status fields a `power`
// widget reads. A rename there leaves the telemetry node's dictionary check
// refusing the layout, never a widget silently empty.
constexpr std::array<std::string_view, 12> kMatchRoles{
    "connected",    "enabled",      "autonomous",   "teleop",
    "test",         "estop",        "fms_attached", "alliance",
    "match_time_s", "match_number", "match_type",   "station"};
constexpr std::string_view kDriverStationPrefix = "hw/state/driver_station/";
constexpr std::array<std::string_view, 3> kPowerRoles{"battery_v", "brownout",
                                                      "health_valid"};
constexpr std::string_view kStatusPrefix = "hw/status/";
// The Pose2d fields under a field widget's `pose` prefix, as (role, field).
constexpr std::array<std::pair<std::string_view, std::string_view>, 3>
    kPoseFields{{{"x", "x_m"}, {"y", "y_m"}, {"yaw", "yaw_rad"}}};

// Where a key sits, for errors: "[dashboard]" or "widgets[3]".
class Scope {
 public:
  Scope(std::string file, std::string where)
      : file_(std::move(file)), where_(std::move(where)) {}
  Scope Nested(const std::string& key) const {
    return Scope(file_, where_ + "." + key);
  }
  [[noreturn]] void Fail(std::string_view key,
                         const std::string& problem) const {
    throw DashboardLayoutError(file_ + ": " + where_ + "." + std::string(key) +
                               ": " + problem);
  }
  [[noreturn]] void FailHere(const std::string& problem) const {
    throw DashboardLayoutError(file_ + ": " + where_ + ": " + problem);
  }

 private:
  std::string file_;
  std::string where_;
};

std::string List(std::initializer_list<std::string_view> keys) {
  std::string out;
  for (const auto key : keys)
    out += (out.empty() ? "" : ", ") + std::string(key);
  return out;
}

void RefuseUnknownKeys(const toml::table& table, const Scope& scope,
                       std::initializer_list<std::string_view> known) {
  for (auto&& [key, value] : table) {
    if (std::ranges::find(known, key.str()) == known.end()) {
      scope.Fail(key.str(), "unknown key (known: " + List(known) + ")");
    }
  }
}

std::optional<std::int64_t> Integer(const toml::table& table,
                                    std::string_view key, const Scope& scope) {
  const toml::node* node = table.get(key);
  if (node == nullptr) return std::nullopt;
  const auto* integer = node->as_integer();
  if (integer == nullptr) scope.Fail(key, "must be an integer");
  return integer->get();
}

std::uint32_t Bounded(const toml::table& table, std::string_view key,
                      const Scope& scope, std::uint32_t fallback,
                      std::int64_t low, std::int64_t high) {
  const auto value = Integer(table, key, scope);
  if (!value) return fallback;
  if (*value < low || *value > high) {
    scope.Fail(key, "must be an integer in [" + std::to_string(low) + ", " +
                        std::to_string(high) + "], not " +
                        std::to_string(*value));
  }
  return static_cast<std::uint32_t>(*value);
}

double AsNumber(const toml::node& node, std::string_view key,
                const Scope& scope) {
  double value = 0;
  if (const auto* integer = node.as_integer()) {
    value = static_cast<double>(integer->get());
  } else if (const auto* floating = node.as_floating_point()) {
    value = floating->get();
  } else {
    scope.Fail(key, "must be a number");
  }
  if (!std::isfinite(value)) scope.Fail(key, "must be finite");
  return value;
}

std::optional<double> Number(const toml::table& table, std::string_view key,
                             const Scope& scope) {
  const toml::node* node = table.get(key);
  if (node == nullptr) return std::nullopt;
  return AsNumber(*node, key, scope);
}

std::optional<std::string> String(const toml::table& table,
                                  std::string_view key, const Scope& scope) {
  const toml::node* node = table.get(key);
  if (node == nullptr) return std::nullopt;
  const auto* text = node->as_string();
  if (text == nullptr) scope.Fail(key, "must be a string");
  return text->get();
}

std::string RequiredString(const toml::table& table, std::string_view key,
                           const Scope& scope, const std::string& problem) {
  if (auto value = String(table, key, scope)) return std::move(*value);
  scope.Fail(key, problem);
}

// A title or label: shown as text, never parsed.
std::string Text(const toml::table& table, std::string_view key,
                 const Scope& scope) {
  auto value = String(table, key, scope).value_or("");
  if (value.size() > kDashboardMaxTextBytes) {
    scope.Fail(key, "is " + std::to_string(value.size()) + " bytes; at most " +
                        std::to_string(kDashboardMaxTextBytes));
  }
  return value;
}

// A dictionary channel name (README "Automatic channels and units"):
// segments of letters, digits and `_` joined by `/`, within the dictionary
// string limit once `suffix` is appended.
void CheckChannelName(const std::string& name, std::string_view key,
                      const Scope& scope, std::size_t suffix = 0) {
  if (name.empty()) scope.Fail(key, "must name a telemetry channel");
  if (name.size() + suffix > telemetry_wire::kMaxDictionaryStringBytes) {
    scope.Fail(key,
               "'" + name + "' is longer than a channel name can be (" +
                   std::to_string(telemetry_wire::kMaxDictionaryStringBytes) +
                   " bytes)");
  }
  const bool characters = std::ranges::all_of(name, [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || c == '_' || c == '/';
  });
  if (!characters || name.front() == '/' || name.back() == '/' ||
      name.find("//") != std::string::npos) {
    scope.Fail(key, "'" + name +
                        "' is not a channel name (<owner>/<role>/<field>, "
                        "no leading '/')");
  }
}

std::optional<std::string> ChannelKey(const toml::table& table,
                                      std::string_view key,
                                      const Scope& scope) {
  auto name = String(table, key, scope);
  if (name) CheckChannelName(*name, key, scope);
  return name;
}

std::string RequiredChannel(const toml::table& table, std::string_view key,
                            const Scope& scope) {
  if (auto name = ChannelKey(table, key, scope)) return std::move(*name);
  scope.Fail(key, "is required");
}

const toml::array& RequiredArray(const toml::table& table, std::string_view key,
                                 const Scope& scope, std::size_t most) {
  const toml::node* node = table.get(key);
  if (node == nullptr) scope.Fail(key, "is required");
  const auto* array = node->as_array();
  if (array == nullptr) scope.Fail(key, "must be an array");
  if (array->empty() || array->size() > most) {
    scope.Fail(key, "must hold 1 to " + std::to_string(most) +
                        " entries, not " + std::to_string(array->size()));
  }
  return *array;
}

DashboardCell ReadCell(const toml::table& table, const Scope& scope,
                       const DashboardLayout& layout) {
  const toml::node* node = table.get("at");
  if (node == nullptr)
    scope.Fail("at", "is required: [column, row, width, height]");
  const auto* array = node->as_array();
  std::array<std::int64_t, 4> v{};
  if (array == nullptr || array->size() != 4) {
    scope.Fail("at", "must be [column, row, width, height]");
  }
  for (std::size_t i = 0; i < 4; ++i) {
    const auto* integer = (*array)[i].as_integer();
    if (integer == nullptr) {
      scope.Fail("at", "must be [column, row, width, height] of integers");
    }
    v[i] = integer->get();
  }
  const std::string text = "[" + std::to_string(v[0]) + ", " +
                           std::to_string(v[1]) + ", " + std::to_string(v[2]) +
                           ", " + std::to_string(v[3]) + "]";
  if (v[0] < 0 || v[1] < 0 || v[2] < 1 || v[3] < 1) {
    scope.Fail("at", text +
                         ": column and row must be at least 0, width and "
                         "height at least 1");
  }
  if (v[0] + v[2] > layout.columns || v[1] + v[3] > layout.rows) {
    scope.Fail("at", text + " is outside the " +
                         std::to_string(layout.columns) + " x " +
                         std::to_string(layout.rows) + " grid");
  }
  return {static_cast<std::uint32_t>(v[0]), static_cast<std::uint32_t>(v[1]),
          static_cast<std::uint32_t>(v[2]), static_cast<std::uint32_t>(v[3])};
}

bool Overlap(const DashboardCell& a, const DashboardCell& b) {
  return a.column < b.column + b.width && b.column < a.column + a.width &&
         a.row < b.row + b.height && b.row < a.row + a.height;
}

double Window(const toml::table& table, std::string_view key,
              const Scope& scope, double fallback, bool allow_zero) {
  const auto value = Number(table, key, scope);
  if (!value) return fallback;
  if ((allow_zero ? *value < 0 : *value <= 0) ||
      *value > kDashboardMaxWindowS) {
    scope.Fail(key, std::string("must be in ") + (allow_zero ? "[0" : "(0") +
                        ", 30] s");
  }
  return *value;
}

void CheckRange(const std::optional<double>& low,
                const std::optional<double>& high, std::string_view high_key,
                std::string_view low_key, const Scope& scope) {
  if (low && high && !(*low < *high)) {
    scope.Fail(high_key, "must be greater than " + std::string(low_key));
  }
}

DashboardCheck ReadCheck(const toml::node& node, const Scope& scope) {
  const auto* table = node.as_table();
  if (table == nullptr) {
    scope.FailHere(
        "must be a table { label, channel, and equals, or "
        "at_least and/or at_most }");
  }
  RefuseUnknownKeys(*table, scope,
                    {"label", "channel", "equals", "at_least", "at_most"});
  DashboardCheck check;
  check.label = Text(*table, "label", scope);
  if (check.label.empty()) scope.Fail("label", "is required");
  check.channel = RequiredChannel(*table, "channel", scope);
  if (const toml::node* equals = table->get("equals")) {
    if (const auto* flag = equals->as_boolean()) {
      check.equals = flag->get();
    } else if (const auto* label = equals->as_string()) {
      if (label->get().empty()) scope.Fail("equals", "must not be empty");
      check.equals = label->get();
    } else {
      check.equals = AsNumber(*equals, "equals", scope);
    }
  }
  check.at_least = Number(*table, "at_least", scope);
  check.at_most = Number(*table, "at_most", scope);
  const bool bounded = check.at_least || check.at_most;
  if (check.equals && bounded) {
    scope.Fail("equals", "cannot be combined with at_least or at_most");
  }
  if (!check.equals && !bounded) {
    scope.FailHere("needs equals, or at_least and/or at_most");
  }
  if (check.at_least && check.at_most && *check.at_least > *check.at_most) {
    scope.Fail("at_most", "must be at least at_least");
  }
  return check;
}

void ReadKindKeys(const toml::table& table, const Scope& scope,
                  DashboardWidget& widget) {
  const auto known = [&](std::initializer_list<std::string_view> keys) {
    std::vector<std::string_view> all{"kind", "at", "title"};
    all.insert(all.end(), keys.begin(), keys.end());
    for (auto&& [key, value] : table) {
      if (widget.kind == Kind::kField && key.str() == "trajectory") {
        scope.Fail("trajectory",
                   "the field's trajectory layer is reserved for a later "
                   "slice and not available yet");
      }
      if (std::ranges::find(all, key.str()) == all.end()) {
        std::string list;
        for (const auto k : all)
          list += (list.empty() ? "" : ", ") + std::string(k);
        scope.Fail(key.str(),
                   "unknown key for a " +
                       std::string(DashboardWidgetKindName(widget.kind)) +
                       " widget (known: " + list + ")");
      }
    }
  };
  auto& channels = widget.channels;
  const auto optional_channel = [&](std::string_view key) {
    if (auto name = ChannelKey(table, key, scope)) {
      channels.push_back({std::string(key), std::move(*name)});
    }
  };
  switch (widget.kind) {
    case Kind::kMatch:
      known({});
      for (const auto role : kMatchRoles) {
        channels.push_back(
            {std::string(role),
             std::string(kDriverStationPrefix) + std::string(role)});
      }
      break;
    case Kind::kPower: {
      known({"warn_below_v"});
      for (const auto role : kPowerRoles) {
        channels.push_back({std::string(role),
                            std::string(kStatusPrefix) + std::string(role)});
      }
      if (const auto warn = Number(table, "warn_below_v", scope)) {
        if (*warn <= 0 || *warn > 20) {
          scope.Fail("warn_below_v", "must be in (0, 20] V");
        }
        widget.warn_below_v = *warn;
      }
      break;
    }
    case Kind::kRoutine:
      known({"channel", "phase"});
      channels.push_back({"channel", RequiredChannel(table, "channel", scope)});
      optional_channel("phase");
      break;
    case Kind::kAlerts:
      known({});
      break;
    case Kind::kSubsystems:
      known({"nodes"});
      if (table.contains("nodes")) {
        const auto& nodes =
            RequiredArray(table, "nodes", scope, kDashboardMaxWidgets);
        for (std::size_t i = 0; i < nodes.size(); ++i) {
          const std::string key = "nodes[" + std::to_string(i) + "]";
          const auto* name = nodes[i].as_string();
          if (name == nullptr) scope.Fail(key, "must be a node name");
          if (auto problem = introspect::naming::CheckSegmentName(
                  name->get(), "node name")) {
            scope.Fail(key, *problem);
          }
          if (std::ranges::find(widget.nodes, name->get()) !=
              widget.nodes.end()) {
            scope.Fail(key, "'" + name->get() + "' is listed twice");
          }
          widget.nodes.push_back(name->get());
        }
      }
      break;
    case Kind::kField: {
      known({"pose", "valid", "localized", "trail_s"});
      const std::string pose = RequiredString(
          table, "pose", scope, "is required: the prefix of x_m, y_m, yaw_rad");
      CheckChannelName(pose, "pose", scope,
                       std::string_view("/yaw_rad").size());
      for (const auto& [role, field] : kPoseFields) {
        channels.push_back(
            {std::string(role), pose + "/" + std::string(field)});
      }
      optional_channel("valid");
      optional_channel("localized");
      widget.trail_s = Window(table, "trail_s", scope, widget.trail_s, true);
      break;
    }
    case Kind::kNumber:
      known({"channel", "label", "precision", "warn_below", "warn_above"});
      channels.push_back({"channel", RequiredChannel(table, "channel", scope)});
      widget.label = Text(table, "label", scope);
      widget.precision =
          static_cast<int>(Bounded(table, "precision", scope, widget.precision,
                                   0, kDashboardMaxPrecision));
      widget.warn_below = Number(table, "warn_below", scope);
      widget.warn_above = Number(table, "warn_above", scope);
      CheckRange(widget.warn_below, widget.warn_above, "warn_above",
                 "warn_below", scope);
      break;
    case Kind::kBoolean:
      known({"channel", "label", "good"});
      channels.push_back({"channel", RequiredChannel(table, "channel", scope)});
      widget.label = Text(table, "label", scope);
      if (const toml::node* good = table.get("good")) {
        const auto* flag = good->as_boolean();
        if (flag == nullptr) scope.Fail("good", "must be true or false");
        widget.good = flag->get();
      }
      break;
    case Kind::kText:
      known({"channel", "label"});
      channels.push_back({"channel", RequiredChannel(table, "channel", scope)});
      widget.label = Text(table, "label", scope);
      break;
    case Kind::kGraph: {
      known({"channels", "window_s", "min", "max"});
      const auto& series =
          RequiredArray(table, "channels", scope, kDashboardMaxGraphChannels);
      for (std::size_t i = 0; i < series.size(); ++i) {
        const std::string key = "channels[" + std::to_string(i) + "]";
        const auto* name = series[i].as_string();
        if (name == nullptr) scope.Fail(key, "must be a channel name");
        CheckChannelName(name->get(), key, scope);
        if (std::ranges::find(widget.series, name->get()) !=
            widget.series.end()) {
          scope.Fail(key, "'" + name->get() + "' is listed twice");
        }
        widget.series.push_back(name->get());
      }
      widget.window_s =
          Window(table, "window_s", scope, widget.window_s, false);
      widget.min = Number(table, "min", scope);
      widget.max = Number(table, "max", scope);
      CheckRange(widget.min, widget.max, "max", "min", scope);
      break;
    }
    case Kind::kChecks: {
      known({"items"});
      const auto& items =
          RequiredArray(table, "items", scope, kDashboardMaxCheckItems);
      for (std::size_t i = 0; i < items.size(); ++i) {
        widget.items.push_back(ReadCheck(
            items[i], scope.Nested("items[" + std::to_string(i) + "]")));
      }
      break;
    }
  }
}

DashboardWidget ReadWidget(const toml::node& node, const Scope& scope,
                           const DashboardLayout& layout) {
  const auto* table = node.as_table();
  if (table == nullptr) scope.FailHere("must be a [[widgets]] table");
  const std::string kind = RequiredString(*table, "kind", scope, "is required");
  DashboardWidget widget;
  const auto known = std::ranges::find_if(
      kKinds, [&](const auto& entry) { return entry.second == kind; });
  if (known == kKinds.end()) {
    if (std::ranges::find(kReservedKinds, kind) != kReservedKinds.end()) {
      scope.Fail("kind", "'" + kind +
                             "' is reserved for a later slice and not "
                             "available yet");
    }
    std::string list;
    for (const auto& [value, name] : kKinds) {
      list += (list.empty() ? "" : ", ") + std::string(name);
    }
    scope.Fail("kind", "unknown kind '" + kind + "' (known: " + list + ")");
  }
  widget.kind = known->first;
  ReadKindKeys(*table, scope, widget);
  widget.at = ReadCell(*table, scope, layout);
  widget.title = Text(*table, "title", scope);
  return widget;
}

DashboardLayout Parse(const toml::table& document, const std::string& file) {
  DashboardLayout layout;
  layout.file = file;
  for (auto&& [key, value] : document) {
    if (key.str() != "dashboard" && key.str() != "widgets") {
      throw DashboardLayoutError(file + ": " + std::string(key.str()) +
                                 ": unknown top-level key (known: [dashboard], "
                                 "[[widgets]])");
    }
  }
  if (const toml::node* node = document.get("dashboard")) {
    const Scope scope(file, "[dashboard]");
    const auto* table = node->as_table();
    if (table == nullptr) {
      throw DashboardLayoutError(file + ": [dashboard] must be a table");
    }
    RefuseUnknownKeys(*table, scope,
                      {"title", "columns", "rows", "update_hz",
                       "budget_bytes_per_s", "drive_clients"});
    if (table->contains("title")) layout.title = Text(*table, "title", scope);
    layout.columns = Bounded(*table, "columns", scope, layout.columns, 1,
                             kDashboardMaxGridCells);
    layout.rows =
        Bounded(*table, "rows", scope, layout.rows, 1, kDashboardMaxGridCells);
    layout.update_hz = Bounded(*table, "update_hz", scope, layout.update_hz, 1,
                               kDashboardMaxUpdateHz);
    layout.drive_clients =
        Bounded(*table, "drive_clients", scope, layout.drive_clients, 1,
                kDashboardMaxDriveClients);
    layout.budget_bytes_per_s =
        Bounded(*table, "budget_bytes_per_s", scope, layout.budget_bytes_per_s,
                1, kDashboardMaxReservedBytesPerS);
  }
  if (std::uint64_t{layout.drive_clients} * layout.budget_bytes_per_s >
      kDashboardMaxReservedBytesPerS) {
    Scope(file, "[dashboard]")
        .Fail("budget_bytes_per_s",
              std::to_string(layout.drive_clients) + " drive_clients x " +
                  std::to_string(layout.budget_bytes_per_s) +
                  " B/s exceeds the " +
                  std::to_string(kDashboardMaxReservedBytesPerS) +
                  " B/s the drive feeds may reserve");
  }
  const toml::node* widgets = document.get("widgets");
  const auto* array = widgets != nullptr ? widgets->as_array() : nullptr;
  if (widgets != nullptr && array == nullptr) {
    throw DashboardLayoutError(file + ": widgets must be [[widgets]] tables");
  }
  if (array == nullptr || array->empty()) {
    throw DashboardLayoutError(file +
                               ": widgets: at least one [[widgets]] is "
                               "required");
  }
  if (array->size() > kDashboardMaxWidgets) {
    throw DashboardLayoutError(
        file + ": widgets[" + std::to_string(kDashboardMaxWidgets) +
        "]: more than " + std::to_string(kDashboardMaxWidgets) + " widgets");
  }
  std::set<std::string> distinct;
  for (std::size_t i = 0; i < array->size(); ++i) {
    const Scope scope(file, "widgets[" + std::to_string(i) + "]");
    DashboardWidget widget = ReadWidget((*array)[i], scope, layout);
    for (std::size_t j = 0; j < layout.widgets.size(); ++j) {
      if (Overlap(widget.at, layout.widgets[j].at)) {
        scope.Fail("at", "overlaps widgets[" + std::to_string(j) + "]");
      }
    }
    layout.widgets.push_back(std::move(widget));
    for (const auto& name : RequiredChannels(layout)) distinct.insert(name);
    if (distinct.size() > kDashboardMaxChannels) {
      scope.FailHere("the layout reads more than " +
                     std::to_string(kDashboardMaxChannels) +
                     " distinct channels");
    }
  }
  CheckDashboardBudget(layout, distinct.size());
  return layout;
}

void AppendString(std::string& out, std::string_view text) {
  out += '"';
  for (const char c : text) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          static constexpr char kHex[] = "0123456789abcdef";
          out += "\\u00";
          const auto byte = static_cast<unsigned char>(c);
          out += kHex[byte >> 4];
          out += kHex[byte & 0xf];
        } else {
          out += c;
        }
    }
  }
  out += '"';
}

void AppendNumber(std::string& out, double value) {
  std::array<char, 32> buffer{};
  const auto [end, error] =
      std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
  out.append(buffer.data(), error == std::errc{} ? end : buffer.data());
}

void AppendOptional(std::string& out, const std::optional<double>& value) {
  if (value) {
    AppendNumber(out, *value);
  } else {
    out += "null";
  }
}

void AppendKey(std::string& out, std::string_view key) {
  out += ',';
  AppendString(out, key);
  out += ':';
}

void AppendStrings(std::string& out, const std::vector<std::string>& values) {
  out += '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) out += ',';
    AppendString(out, values[i]);
  }
  out += ']';
}

void AppendWidget(std::string& out, const DashboardWidget& widget) {
  out += "{\"kind\":";
  AppendString(out, DashboardWidgetKindName(widget.kind));
  out += ",\"at\":[" + std::to_string(widget.at.column) + "," +
         std::to_string(widget.at.row) + "," + std::to_string(widget.at.width) +
         "," + std::to_string(widget.at.height) + "]";
  AppendKey(out, "title");
  AppendString(out, widget.title);
  AppendKey(out, "channels");
  out += '{';
  for (std::size_t i = 0; i < widget.channels.size(); ++i) {
    if (i > 0) out += ',';
    AppendString(out, widget.channels[i].role);
    out += ':';
    AppendString(out, widget.channels[i].name);
  }
  out += '}';
  switch (widget.kind) {
    case Kind::kMatch:
    case Kind::kRoutine:
    case Kind::kAlerts:
      break;
    case Kind::kPower:
      AppendKey(out, "warn_below_v");
      AppendNumber(out, widget.warn_below_v);
      break;
    case Kind::kSubsystems:
      AppendKey(out, "nodes");
      AppendStrings(out, widget.nodes);
      break;
    case Kind::kField:
      AppendKey(out, "trail_s");
      AppendNumber(out, widget.trail_s);
      break;
    case Kind::kNumber:
      AppendKey(out, "label");
      AppendString(out, widget.label);
      AppendKey(out, "precision");
      out += std::to_string(widget.precision);
      AppendKey(out, "warn_below");
      AppendOptional(out, widget.warn_below);
      AppendKey(out, "warn_above");
      AppendOptional(out, widget.warn_above);
      break;
    case Kind::kBoolean:
      AppendKey(out, "label");
      AppendString(out, widget.label);
      AppendKey(out, "good");
      out += widget.good ? "true" : "false";
      break;
    case Kind::kText:
      AppendKey(out, "label");
      AppendString(out, widget.label);
      break;
    case Kind::kGraph:
      AppendKey(out, "series");
      AppendStrings(out, widget.series);
      AppendKey(out, "window_s");
      AppendNumber(out, widget.window_s);
      AppendKey(out, "min");
      AppendOptional(out, widget.min);
      AppendKey(out, "max");
      AppendOptional(out, widget.max);
      break;
    case Kind::kChecks:
      AppendKey(out, "items");
      out += '[';
      for (std::size_t i = 0; i < widget.items.size(); ++i) {
        const auto& item = widget.items[i];
        if (i > 0) out += ',';
        out += "{\"label\":";
        AppendString(out, item.label);
        AppendKey(out, "channel");
        AppendString(out, item.channel);
        AppendKey(out, "equals");
        if (!item.equals) {
          out += "null";
        } else if (const auto* flag = std::get_if<bool>(&*item.equals)) {
          out += *flag ? "true" : "false";
        } else if (const auto* number = std::get_if<double>(&*item.equals)) {
          AppendNumber(out, *number);
        } else {
          AppendString(out, std::get<std::string>(*item.equals));
        }
        AppendKey(out, "at_least");
        AppendOptional(out, item.at_least);
        AppendKey(out, "at_most");
        AppendOptional(out, item.at_most);
        out += '}';
      }
      out += ']';
      break;
  }
  out += '}';
}

}  // namespace

std::string_view DashboardWidgetKindName(DashboardWidgetKind kind) {
  for (const auto& [value, name] : kKinds) {
    if (value == kind) return name;
  }
  return "unknown";
}

DashboardLayout ParseDashboardLayoutString(std::string_view toml,
                                           const std::filesystem::path& file) {
  const std::string name = file.string();
  toml::table document;
  try {
    document = toml::parse(toml, name);
  } catch (const toml::parse_error& error) {
    std::ostringstream message;
    message << name << ": TOML parse error at line "
            << error.source().begin.line << ", column "
            << error.source().begin.column << ": " << error.description();
    throw DashboardLayoutError(message.str());
  }
  return Parse(document, name);
}

DashboardLayout ParseDashboardLayout(const std::filesystem::path& file) {
  std::ifstream input(file, std::ios::binary);
  if (!input) {
    throw DashboardLayoutError(file.string() +
                               ": cannot open the dashboard "
                               "layout");
  }
  std::ostringstream text;
  text << input.rdbuf();
  return ParseDashboardLayoutString(text.str(), file);
}

std::vector<std::string> RequiredChannels(const DashboardLayout& layout) {
  std::set<std::string> names;
  for (const auto& widget : layout.widgets) {
    for (const auto& channel : widget.channels) names.insert(channel.name);
    names.insert(widget.series.begin(), widget.series.end());
    for (const auto& item : widget.items) names.insert(item.channel);
  }
  return {names.begin(), names.end()};
}

bool CarriesAlertChannels(const DashboardLayout& layout) {
  return std::ranges::any_of(layout.widgets, [](const auto& widget) {
    return widget.kind == Kind::kAlerts;
  });
}

std::uint64_t DashboardWorstCaseBytesPerSecond(std::uint32_t update_hz,
                                               std::size_t channel_count) {
  using namespace telemetry_wire;
  constexpr std::uint64_t kFramedPage =
      kValuePageOverheadBound + kMaxWebSocketHeaderBytes;
  const std::uint64_t pages = std::max<std::uint64_t>(
      1, (channel_count + kMinSamplesPerPage - 1) / kMinSamplesPerPage);
  const std::uint64_t keyframe =
      channel_count * kSampleBytes + pages * kFramedPage;
  constexpr std::uint64_t kSecondNs = 1'000'000'000;
  constexpr std::uint64_t kSummaries = kSecondNs / kDriveSummaryIntervalNs;
  constexpr std::uint64_t kHeartbeats = kSecondNs / kHeartbeatIntervalNs;
  return update_hz * keyframe +
         kSummaries * (kDriveSummaryMaxBytes + kMaxWebSocketHeaderBytes) +
         kHeartbeats * kFramedPage;
}

void CheckDashboardBudget(const DashboardLayout& layout,
                          std::size_t channel_count) {
  const std::uint64_t worst =
      DashboardWorstCaseBytesPerSecond(layout.update_hz, channel_count);
  if (worst > layout.budget_bytes_per_s) {
    Scope(layout.file.string(), "[dashboard]")
        .Fail("budget_bytes_per_s", std::to_string(layout.budget_bytes_per_s) +
                                        " B/s is below the worst case of " +
                                        std::to_string(channel_count) +
                                        " channels at update_hz " +
                                        std::to_string(layout.update_hz) +
                                        ", " + std::to_string(worst) + " B/s");
  }
}

std::string DashboardLayoutJson(const DashboardLayout& layout) {
  std::string out = "{\"version\":1";
  AppendKey(out, "file");
  AppendString(out, layout.file.filename().string());
  AppendKey(out, "title");
  AppendString(out, layout.title);
  out +=
      ",\"columns\":" + std::to_string(layout.columns) +
      ",\"rows\":" + std::to_string(layout.rows) +
      ",\"update_hz\":" + std::to_string(layout.update_hz) +
      ",\"budget_bytes_per_s\":" + std::to_string(layout.budget_bytes_per_s) +
      ",\"drive_clients\":" + std::to_string(layout.drive_clients);
  AppendKey(out, "alerts");
  out += CarriesAlertChannels(layout) ? "true" : "false";
  AppendKey(out, "channels");
  AppendStrings(out, RequiredChannels(layout));
  AppendKey(out, "widgets");
  out += '[';
  for (std::size_t i = 0; i < layout.widgets.size(); ++i) {
    if (i > 0) out += ',';
    AppendWidget(out, layout.widgets[i]);
  }
  out += "]}";
  return out;
}

}  // namespace talos::telemetry
