#pragma once

// The driver dashboard's layout: the file `[studio] dashboard` names, parsed
// and validated once in C++ (talOS/telemetry/README.md "Dashboard layout").
//
// ParseDashboardLayout checks everything the file alone decides: the grid,
// each widget's kind, keys, cell and options, the feed's rate and budget
// limits, and the worst case of the channels the layout names. What needs
// the telemetry dictionary (that each channel exists, and the worst case
// with every alert channel an `alerts` widget adds) takes its inputs from
// RequiredChannels, CarriesAlertChannels and CheckDashboardBudget.
// DashboardLayoutJson is the layout as the dashboard page reads it
// (/drive/layout.json), so no TOML parser is needed in TypeScript.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace talos::telemetry {

// Every error names the file, then `[dashboard].<key>` or
// `widgets[<index>].<key>` (index from 0, in file order).
class DashboardLayoutError : public std::invalid_argument {
 public:
  using std::invalid_argument::invalid_argument;
};

// Limits a layout is held to (README "Dashboard layout").
inline constexpr std::uint32_t kDashboardMaxGridCells = 24;  // columns, rows
inline constexpr std::size_t kDashboardMaxWidgets = 64;
inline constexpr std::size_t kDashboardMaxChannels = 256;  // distinct, named
inline constexpr std::uint32_t kDashboardMaxUpdateHz = 50;
inline constexpr std::uint32_t kDashboardMaxDriveClients = 4;
// The drive feeds' share of the bridge's 160,000 B/s total: drive_clients x
// budget_bytes_per_s may not exceed it.
inline constexpr std::uint64_t kDashboardMaxReservedBytesPerS = 80000;
inline constexpr double kDashboardMaxWindowS = 30;  // graph window, trail
inline constexpr std::size_t kDashboardMaxGraphChannels = 8;
inline constexpr std::size_t kDashboardMaxCheckItems = 16;
inline constexpr std::size_t kDashboardMaxTextBytes = 64;  // title, label
inline constexpr int kDashboardMaxPrecision = 6;

enum class DashboardWidgetKind : std::uint8_t {
  kMatch,
  kPower,
  kRoutine,
  kAlerts,
  kSubsystems,
  kField,
  kNumber,
  kBoolean,
  kText,
  kGraph,
  kChecks,
};

// The TOML and JSON spelling of a kind ("match", "power", ...).
std::string_view DashboardWidgetKindName(DashboardWidgetKind kind);

// A widget's rectangle in grid cells, from the top-left: `at = [column, row,
// width, height]`.
struct DashboardCell {
  std::uint32_t column{0};
  std::uint32_t row{0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  bool operator==(const DashboardCell&) const = default;
};

// One telemetry channel a widget reads, by the role it plays there: the
// TOML key that named it ("channel", "phase", "valid"), or for a fixed
// channel of a kind (match, power) or the field's pose the role the page
// draws it as ("enabled", "battery_v", "x"). JSON writes each in the
// widget's "channels" object as `"<role>": "<name>"`.
struct DashboardChannel {
  std::string role;
  std::string name;
  bool operator==(const DashboardChannel&) const = default;
};

// A `checks` item: passes while the channel's value meets every condition
// set. `equals` compares a Bool channel with a bool, an integer or double
// with a number, and an enum channel's label with a string.
struct DashboardCheck {
  std::string label;
  std::string channel;
  std::optional<std::variant<bool, double, std::string>> equals;
  std::optional<double> at_least;
  std::optional<double> at_most;
  bool operator==(const DashboardCheck&) const = default;
};

struct DashboardWidget {
  DashboardWidgetKind kind{DashboardWidgetKind::kMatch};
  DashboardCell at;
  std::string title;  // "" lets the page use the kind's own heading
  // The widget's single channels, the kind's fixed ones first, then its keys
  // in README order. A graph's channels are `series` and a check's are in
  // `items`.
  std::vector<DashboardChannel> channels;
  std::vector<std::string> series;  // graph
  // Options, set only for the kinds that take them (others keep defaults).
  std::string label;                  // number, boolean, text
  double warn_below_v{11.5};          // power
  double trail_s{5};                  // field
  int precision{2};                   // number
  std::optional<double> warn_below;   // number
  std::optional<double> warn_above;   // number
  bool good{true};                    // boolean: the value drawn green
  double window_s{10};                // graph
  std::optional<double> min;          // graph's fixed y range
  std::optional<double> max;          // graph
  std::vector<std::string> nodes;     // subsystems: empty shows every node
  std::vector<DashboardCheck> items;  // checks
  bool operator==(const DashboardWidget&) const = default;
};

struct DashboardLayout {
  std::filesystem::path file;  // as given to ParseDashboardLayout
  std::string title{"Drive"};
  std::uint32_t columns{12};
  std::uint32_t rows{8};
  std::uint32_t update_hz{20};
  std::uint32_t budget_bytes_per_s{24000};
  std::uint32_t drive_clients{2};
  std::vector<DashboardWidget> widgets;
  bool operator==(const DashboardLayout&) const = default;
};

// Reads and validates `file`; throws DashboardLayoutError naming the file,
// the widget index and the key (a TOML syntax error names the file, line and
// column).
DashboardLayout ParseDashboardLayout(const std::filesystem::path& file);
// The same from TOML text; `file` names it in errors and in the result.
DashboardLayout ParseDashboardLayoutString(std::string_view toml,
                                           const std::filesystem::path& file);

// The distinct channel names the layout reads, sorted: what the telemetry
// dictionary must hold and the named part of the drive feed's channel mask.
std::vector<std::string> RequiredChannels(const DashboardLayout& layout);
// Whether the feed also carries every alert channel of the dictionary (an
// `alerts` widget), which the layout cannot name.
bool CarriesAlertChannels(const DashboardLayout& layout);

// The bytes one drive client can be sent in a second (README "Dashboard
// layout", worst case): `update_hz` keyframes of `channel_count` channels,
// framed, plus the drive summaries and heartbeats, framed, at their fastest.
std::uint64_t DashboardWorstCaseBytesPerSecond(std::uint32_t update_hz,
                                               std::size_t channel_count);
// Throws DashboardLayoutError naming the file and
// `[dashboard].budget_bytes_per_s` when the worst case of a feed carrying
// `channel_count` channels exceeds the layout's budget. Parsing runs it
// with the named channels; a holder of the dictionary runs it again with
// the dictionary's alert channels added.
void CheckDashboardBudget(const DashboardLayout& layout,
                          std::size_t channel_count);

// The JSON of /drive/layout.json (README "Dashboard layout"): every key
// with its default filled in, every channel by its dictionary name.
std::string DashboardLayoutJson(const DashboardLayout& layout);

}  // namespace talos::telemetry
