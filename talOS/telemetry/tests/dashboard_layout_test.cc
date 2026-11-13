#include "talOS/telemetry/dashboard_layout.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace talos::telemetry {
namespace {

constexpr const char* kFile = "layouts/dashboard.toml";

DashboardLayout Parse(const std::string& text) {
  return ParseDashboardLayoutString(text, kFile);
}

// Every widget kind of the first slices, each option set or left to its
// default, is served as the JSON the dashboard page reads: no TOML parser
// in TypeScript, so this is the page's whole view of the file.
// REQ-OPS-39
TEST(DashboardLayoutTest, ValidLayoutSerializesToTheServedJson) {
  const auto layout = Parse(R"(
[dashboard]
title = "Match \"7\""
columns = 12
rows = 8
update_hz = 10
budget_bytes_per_s = 40000
drive_clients = 2

[[widgets]]
kind = "match"
at = [0, 0, 4, 1]

[[widgets]]
kind = "power"
at = [4, 0, 2, 1]
warn_below_v = 12

[[widgets]]
kind = "routine"
at = [6, 0, 3, 1]
title = "Auto"
channel = "autonomous/state/routine_index"
phase = "autonomous/state/phase"

[[widgets]]
kind = "alerts"
at = [9, 0, 3, 2]

[[widgets]]
kind = "subsystems"
at = [0, 1, 4, 2]
nodes = ["drivetrain", "shooter"]

[[widgets]]
kind = "field"
at = [4, 1, 5, 4]
pose = "pose_estimator/state/pose"
valid = "pose_estimator/state/valid"

[[widgets]]
kind = "number"
at = [0, 3, 2, 1]
channel = "shooter/state/flywheel_velocity_rps"
label = "Flywheel"
precision = 1
warn_below = 10

[[widgets]]
kind = "boolean"
at = [2, 3, 2, 1]
channel = "intake/state/beam_broken"
good = false

[[widgets]]
kind = "text"
at = [0, 4, 2, 1]
channel = "intake/state/inhibit"

[[widgets]]
kind = "graph"
at = [0, 5, 6, 3]
channels = ["shooter/state/flywheel_velocity_rps", "shooter/state/target_velocity_rps"]
window_s = 7.5
min = 0

[[widgets]]
kind = "checks"
at = [6, 5, 6, 3]
items = [
  { label = "Localized", channel = "pose_estimator/state/localized", equals = true },
  { label = "Battery", channel = "hw/status/battery_v", at_least = 12.5 },
  { label = "Idle", channel = "autonomous/state/phase", equals = "IDLE" },
]
)");
  const std::string expected =
      R"({"version":1,"file":"dashboard.toml","title":"Match \"7\"",)"
      R"("columns":12,"rows":8,"update_hz":10,"budget_bytes_per_s":40000,)"
      R"("drive_clients":2,"alerts":true,"channels":[)"
      R"("autonomous/state/phase","autonomous/state/routine_index",)"
      R"("hw/state/driver_station/alliance",)"
      R"("hw/state/driver_station/autonomous",)"
      R"("hw/state/driver_station/connected",)"
      R"("hw/state/driver_station/enabled","hw/state/driver_station/estop",)"
      R"("hw/state/driver_station/fms_attached",)"
      R"("hw/state/driver_station/match_time_s",)"
      R"("hw/state/driver_station/teleop","hw/state/driver_station/test",)"
      R"("hw/status/battery_v","hw/status/brownout","hw/status/health_valid",)"
      R"("intake/state/beam_broken","intake/state/inhibit",)"
      R"("pose_estimator/state/localized","pose_estimator/state/pose/x_m",)"
      R"("pose_estimator/state/pose/y_m","pose_estimator/state/pose/yaw_rad",)"
      R"("pose_estimator/state/valid","shooter/state/flywheel_velocity_rps",)"
      R"("shooter/state/target_velocity_rps"],"widgets":[)"
      R"({"kind":"match","at":[0,0,4,1],"title":"","channels":{)"
      R"("connected":"hw/state/driver_station/connected",)"
      R"("enabled":"hw/state/driver_station/enabled",)"
      R"("autonomous":"hw/state/driver_station/autonomous",)"
      R"("teleop":"hw/state/driver_station/teleop",)"
      R"("test":"hw/state/driver_station/test",)"
      R"("estop":"hw/state/driver_station/estop",)"
      R"("fms_attached":"hw/state/driver_station/fms_attached",)"
      R"("alliance":"hw/state/driver_station/alliance",)"
      R"("match_time_s":"hw/state/driver_station/match_time_s"}},)"
      R"({"kind":"power","at":[4,0,2,1],"title":"","channels":{)"
      R"("battery_v":"hw/status/battery_v","brownout":"hw/status/brownout",)"
      R"("health_valid":"hw/status/health_valid"},"warn_below_v":12},)"
      R"({"kind":"routine","at":[6,0,3,1],"title":"Auto","channels":{)"
      R"("channel":"autonomous/state/routine_index",)"
      R"("phase":"autonomous/state/phase"}},)"
      R"({"kind":"alerts","at":[9,0,3,2],"title":"","channels":{}},)"
      R"({"kind":"subsystems","at":[0,1,4,2],"title":"","channels":{},)"
      R"("nodes":["drivetrain","shooter"]},)"
      R"({"kind":"field","at":[4,1,5,4],"title":"","channels":{)"
      R"("x":"pose_estimator/state/pose/x_m",)"
      R"("y":"pose_estimator/state/pose/y_m",)"
      R"("yaw":"pose_estimator/state/pose/yaw_rad",)"
      R"("valid":"pose_estimator/state/valid"},"trail_s":5},)"
      R"({"kind":"number","at":[0,3,2,1],"title":"","channels":{)"
      R"("channel":"shooter/state/flywheel_velocity_rps"},"label":"Flywheel",)"
      R"("precision":1,"warn_below":10,"warn_above":null},)"
      R"({"kind":"boolean","at":[2,3,2,1],"title":"","channels":{)"
      R"("channel":"intake/state/beam_broken"},"label":"","good":false},)"
      R"({"kind":"text","at":[0,4,2,1],"title":"","channels":{)"
      R"("channel":"intake/state/inhibit"},"label":""},)"
      R"({"kind":"graph","at":[0,5,6,3],"title":"","channels":{},"series":[)"
      R"("shooter/state/flywheel_velocity_rps",)"
      R"("shooter/state/target_velocity_rps"],"window_s":7.5,"min":0,)"
      R"("max":null},)"
      R"({"kind":"checks","at":[6,5,6,3],"title":"","channels":{},"items":[)"
      R"({"label":"Localized","channel":"pose_estimator/state/localized",)"
      R"("equals":true,"at_least":null,"at_most":null},)"
      R"({"label":"Battery","channel":"hw/status/battery_v","equals":null,)"
      R"("at_least":12.5,"at_most":null},)"
      R"({"label":"Idle","channel":"autonomous/state/phase","equals":"IDLE",)"
      R"("at_least":null,"at_most":null}]}]})";
  EXPECT_EQ(DashboardLayoutJson(layout), expected);
}

// A layout the page could not draw is refused at parse, naming the file, the
// widget index and the key, so the fix is found without reading the parser.
// REQ-OPS-39
TEST(DashboardLayoutTest, RefusesAnInvalidLayoutNamingFileIndexAndKey) {
  const std::string first =
      "[[widgets]]\nkind = \"alerts\"\nat = [0, 0, 2, 2]\n";
  struct Case {
    const char* name;
    std::string text;
    const char* named;  // the index and key, or the [dashboard] key
  };
  const std::vector<Case> cases = {
      {"unknown kind",
       first + "[[widgets]]\nkind = \"gauge\"\nat = [2, 0, 1, 1]\n",
       "widgets[1].kind"},
      {"reserved kind",
       first + "[[widgets]]\nkind = \"camera\"\nat = [2, 0, 1, 1]\n",
       "widgets[1].kind"},
      {"reserved trajectory layer",
       "[[widgets]]\nkind = \"field\"\nat = [0, 0, 4, 4]\npose = "
       "\"p/state/pose\"\n"
       "trajectory = \"two_piece\"\n",
       "widgets[0].trajectory"},
      {"missing kind", first + "[[widgets]]\nat = [2, 0, 1, 1]\n",
       "widgets[1].kind"},
      {"unknown widget key",
       first + "[[widgets]]\nkind = \"text\"\nat = [2, 0, 1, 1]\nchannel = "
               "\"a/state/b\"\ncolour = \"red\"\n",
       "widgets[1].colour"},
      {"key of another kind",
       first + "[[widgets]]\nkind = \"match\"\nat = [2, 0, 1, 1]\nchannel = "
               "\"a/state/b\"\n",
       "widgets[1].channel"},
      {"cell past the grid's columns",
       first + "[[widgets]]\nkind = \"match\"\nat = [10, 0, 3, 1]\n",
       "widgets[1].at"},
      {"cell past the grid's rows",
       "[dashboard]\nrows = 4\n[[widgets]]\nkind = \"match\"\nat = [0, 3, 1, "
       "2]\n",
       "widgets[0].at"},
      {"empty cell",
       first + "[[widgets]]\nkind = \"match\"\nat = [2, 0, 0, 1]\n",
       "widgets[1].at"},
      {"cell not four integers",
       first + "[[widgets]]\nkind = \"match\"\nat = [2, 0, 1]\n",
       "widgets[1].at"},
      {"overlapping cells",
       first + "[[widgets]]\nkind = \"match\"\nat = [1, 1, 2, 2]\n",
       "widgets[1].at"},
      {"missing channel",
       first + "[[widgets]]\nkind = \"number\"\nat = [2, 0, 1, 1]\n",
       "widgets[1].channel"},
      {"channel with a leading slash",
       first + "[[widgets]]\nkind = \"number\"\nat = [2, 0, 1, 1]\nchannel = "
               "\"/shooter/state/at_speed\"\n",
       "widgets[1].channel"},
      {"graph of too many channels",
       first + "[[widgets]]\nkind = \"graph\"\nat = [2, 0, 4, 2]\nchannels = "
               "[\"a/s/a\", \"a/s/b\", \"a/s/c\", \"a/s/d\", \"a/s/e\", "
               "\"a/s/f\", \"a/s/g\", \"a/s/h\", \"a/s/i\"]\n",
       "widgets[1].channels"},
      {"graph window past 30 s",
       first + "[[widgets]]\nkind = \"graph\"\nat = [2, 0, 4, 2]\nchannels = "
               "[\"a/s/a\"]\nwindow_s = 31\n",
       "widgets[1].window_s"},
      {"graph range reversed",
       first + "[[widgets]]\nkind = \"graph\"\nat = [2, 0, 4, 2]\nchannels = "
               "[\"a/s/a\"]\nmin = 5\nmax = 5\n",
       "widgets[1].max"},
      {"check with two conditions",
       first + "[[widgets]]\nkind = \"checks\"\nat = [2, 0, 4, 2]\nitems = [{ "
               "label = \"L\", channel = \"a/s/a\", equals = 1, at_least = 0 "
               "}]\n",
       "widgets[1].items[0].equals"},
      {"check with no condition",
       first + "[[widgets]]\nkind = \"checks\"\nat = [2, 0, 4, 2]\nitems = [{ "
               "label = \"L\", channel = \"a/s/a\" }]\n",
       "widgets[1].items[0]"},
      {"subsystems naming a bad node",
       first + "[[widgets]]\nkind = \"subsystems\"\nat = [2, 0, 4, 2]\nnodes = "
               "[\"Drivetrain\"]\n",
       "widgets[1].nodes[0]"},
      {"update_hz above 50", "[dashboard]\nupdate_hz = 51\n" + first,
       "[dashboard].update_hz"},
      {"drive_clients above 4", "[dashboard]\ndrive_clients = 5\n" + first,
       "[dashboard].drive_clients"},
      {"reservation above 80000 B/s",
       "[dashboard]\ndrive_clients = 4\nbudget_bytes_per_s = 20001\n" + first,
       "[dashboard].budget_bytes_per_s"},
      {"unknown [dashboard] key", "[dashboard]\ncolumn = 4\n" + first,
       "[dashboard].column"},
      {"unknown top-level key", "[layout]\ncolumns = 4\n" + first, "layout"},
      {"no widgets", "[dashboard]\ntitle = \"Drive\"\n", "widgets"},
      {"TOML syntax error", first + "at = [\n", "dashboard.toml"},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.name);
    try {
      Parse(c.text);
      ADD_FAILURE() << "accepted:\n" << c.text;
    } catch (const DashboardLayoutError& error) {
      const std::string what = error.what();
      EXPECT_NE(what.find(kFile), std::string::npos) << what;
      EXPECT_NE(what.find(c.named), std::string::npos) << what;
    }
  }
}

// The feed's worst case (README "Dashboard layout"): update_hz keyframes,
// two whole summaries and ten empty heartbeats a second, each page with its
// overhead and WebSocket header. A keyframe takes a second page past 83
// channels.
// REQ-OPS-35
TEST(DashboardLayoutTest, WorstCaseBudgetIsHeldAtItsBoundary) {
  // 2 x (4096 + 10) summary bytes + 10 x (200 + 10) heartbeat bytes.
  constexpr std::uint64_t kFixed = 8212 + 2100;
  EXPECT_EQ(DashboardWorstCaseBytesPerSecond(1, 0), 210 + kFixed);
  EXPECT_EQ(DashboardWorstCaseBytesPerSecond(1, 83), 83 * 12 + 210 + kFixed);
  EXPECT_EQ(DashboardWorstCaseBytesPerSecond(1, 84), 84 * 12 + 420 + kFixed);
  EXPECT_EQ(DashboardWorstCaseBytesPerSecond(20, 25),
            20 * (25 * 12 + 210) + kFixed);

  // One number widget at 10 Hz: 10 x (12 + 210) + kFixed = 12532 B/s.
  const auto layout = [](int budget) {
    return "[dashboard]\nupdate_hz = 10\nbudget_bytes_per_s = " +
           std::to_string(budget) +
           "\n[[widgets]]\nkind = \"number\"\nat = [0, 0, 1, 1]\nchannel = "
           "\"shooter/state/at_speed\"\n";
  };
  const auto fits = Parse(layout(12532));
  try {
    Parse(layout(12531));
    ADD_FAILURE() << "a budget 1 B/s below the worst case was accepted";
  } catch (const DashboardLayoutError& error) {
    EXPECT_NE(std::string(error.what()).find("[dashboard].budget_bytes_per_s"),
              std::string::npos)
        << error.what();
  }
  // The telemetry node's second check, with the dictionary's alert channels
  // added to the one the layout names.
  EXPECT_NO_THROW(CheckDashboardBudget(fits, 1));
  EXPECT_THROW(CheckDashboardBudget(fits, 2), DashboardLayoutError);
}

}  // namespace
}  // namespace talos::telemetry
