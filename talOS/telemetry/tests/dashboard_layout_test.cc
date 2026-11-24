#include "talOS/telemetry/dashboard_layout.h"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
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
      R"("hw/state/driver_station/match_number",)"
      R"("hw/state/driver_station/match_time_s",)"
      R"("hw/state/driver_station/match_type",)"
      R"("hw/state/driver_station/station",)"
      R"("hw/state/driver_station/teleop","hw/state/driver_station/test",)"
      R"("hw/status/alert_error_link_lost",)"
      R"("hw/status/alert_error_protocol_mismatch",)"
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
      R"("match_time_s":"hw/state/driver_station/match_time_s",)"
      R"("match_number":"hw/state/driver_station/match_number",)"
      R"("match_type":"hw/state/driver_station/match_type",)"
      R"("station":"hw/state/driver_station/station"}},)"
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
      {"cell column + width overflows int64",
       first +
           "[[widgets]]\nkind = \"match\"\nat = [9223372036854775807, 0, 1, "
           "1]\n",
       "widgets[1].at"},
      {"cell row + height overflows int64",
       first +
           "[[widgets]]\nkind = \"match\"\nat = [0, 9223372036854775807, 1, "
           "1]\n",
       "widgets[1].at"},
      {"cell width overflows column + width",
       first +
           "[[widgets]]\nkind = \"match\"\nat = [1, 0, 9223372036854775807, "
           "1]\n",
       "widgets[1].at"},
      {"cell height overflows row + height",
       first + "[[widgets]]\nkind = \"match\"\nat = [0, 1, 1, "
               "9223372036854775807]\n",
       "widgets[1].at"},
      {"cell column and width sum past INT64_MAX",
       first + "[[widgets]]\nkind = \"match\"\nat = [4611686018427387904, 0, "
               "4611686018427387904, 1]\n",
       "widgets[1].at"},
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
      {"non-table [dashboard]", "dashboard = 1\n" + first, "[dashboard]"},
      {"non-array widgets", "widgets = \"bad\"\n", "widgets"},
      {"TOML syntax error", first + "at = [\n", "dashboard.toml"},
      {"dashboard title over 64 bytes",
       "[dashboard]\ntitle = \"" +
           std::string(kDashboardMaxTextBytes + 1, 't') + "\"\n" + first,
       "[dashboard].title"},
      {"widget title over 64 bytes",
       first + "[[widgets]]\nkind = \"alerts\"\nat = [2, 0, 1, 1]\ntitle = \"" +
           std::string(kDashboardMaxTextBytes + 1, 't') + "\"\n",
       "widgets[1].title"},
      {"widget label over 64 bytes",
       first +
           "[[widgets]]\nkind = \"number\"\nat = [2, 0, 1, 1]\nchannel = "
           "\"a/s/a\"\nlabel = \"" +
           std::string(kDashboardMaxTextBytes + 1, 'l') + "\"\n",
       "widgets[1].label"},
      {"channel over 256 bytes",
       first +
           "[[widgets]]\nkind = \"number\"\nat = [2, 0, 1, 1]\nchannel = "
           "\"a/s/" +
           std::string(253, 'c') + "\"\n",
       "widgets[1].channel"},
      {"pose prefix + /yaw_rad over 256 bytes",
       first +
           "[[widgets]]\nkind = \"field\"\nat = [2, 0, 2, 2]\npose = "
           "\"a/s/" +
           std::string(245, 'p') + "\"\n",
       "widgets[1].pose"},
      {"duplicate graph channels",
       first + "[[widgets]]\nkind = \"graph\"\nat = [2, 0, 4, 2]\nchannels = "
               "[\"a/s/a\", \"a/s/a\"]\n",
       "widgets[1].channels[1]"},
      {"duplicate subsystems nodes",
       first + "[[widgets]]\nkind = \"subsystems\"\nat = [2, 0, 4, 2]\nnodes = "
               "[\"drivetrain\", \"drivetrain\"]\n",
       "widgets[1].nodes[1]"},
      {"check at_least > at_most",
       first + "[[widgets]]\nkind = \"checks\"\nat = [2, 0, 4, 2]\nitems = [{ "
               "label = \"L\", channel = \"a/s/a\", at_least = 5, at_most = 4 "
               "}]\n",
       "widgets[1].items[0].at_most"},
      {"check empty equals",
       first + "[[widgets]]\nkind = \"checks\"\nat = [2, 0, 4, 2]\nitems = [{ "
               "label = \"L\", channel = \"a/s/a\", equals = \"\" }]\n",
       "widgets[1].items[0].equals"},
      {"check empty label",
       first + "[[widgets]]\nkind = \"checks\"\nat = [2, 0, 4, 2]\nitems = [{ "
               "label = \"\", channel = \"a/s/a\", equals = true }]\n",
       "widgets[1].items[0].label"},
      {"check label over 64 bytes",
       first +
           "[[widgets]]\nkind = \"checks\"\nat = [2, 0, 4, 2]\nitems = [{ "
           "label = \"" +
           std::string(kDashboardMaxTextBytes + 1, 'l') +
           "\", channel = \"a/s/a\", equals = true }]\n",
       "widgets[1].items[0].label"},
      {"power warn_below_v at 0",
       first + "[[widgets]]\nkind = \"power\"\nat = [2, 0, 2, 1]\n"
               "warn_below_v = 0\n",
       "widgets[1].warn_below_v"},
      {"power warn_below_v above 20",
       first + "[[widgets]]\nkind = \"power\"\nat = [2, 0, 2, 1]\n"
               "warn_below_v = 20.1\n",
       "widgets[1].warn_below_v"},
      {"power warn_below_v non-finite",
       first + "[[widgets]]\nkind = \"power\"\nat = [2, 0, 2, 1]\n"
               "warn_below_v = nan\n",
       "widgets[1].warn_below_v"},
      {"field trail_s below 0",
       first + "[[widgets]]\nkind = \"field\"\nat = [2, 0, 2, 2]\npose = "
               "\"p/s/pose\"\ntrail_s = -0.1\n",
       "widgets[1].trail_s"},
      {"field trail_s above 30",
       first + "[[widgets]]\nkind = \"field\"\nat = [2, 0, 2, 2]\npose = "
               "\"p/s/pose\"\ntrail_s = 30.1\n",
       "widgets[1].trail_s"},
      {"number precision below 0",
       first + "[[widgets]]\nkind = \"number\"\nat = [2, 0, 1, 1]\nchannel = "
               "\"a/s/a\"\nprecision = -1\n",
       "widgets[1].precision"},
      {"number precision above 6",
       first + "[[widgets]]\nkind = \"number\"\nat = [2, 0, 1, 1]\nchannel = "
               "\"a/s/a\"\nprecision = 7\n",
       "widgets[1].precision"},
      {"number warn_below == warn_above",
       first + "[[widgets]]\nkind = \"number\"\nat = [2, 0, 1, 1]\nchannel = "
               "\"a/s/a\"\nwarn_below = 5\nwarn_above = 5\n",
       "widgets[1].warn_above"},
      {"number warn_below > warn_above",
       first + "[[widgets]]\nkind = \"number\"\nat = [2, 0, 1, 1]\nchannel = "
               "\"a/s/a\"\nwarn_below = 6\nwarn_above = 5\n",
       "widgets[1].warn_above"},
      {"boolean non-bool good",
       first + "[[widgets]]\nkind = \"boolean\"\nat = [2, 0, 1, 1]\nchannel = "
               "\"a/s/a\"\ngood = \"yes\"\n",
       "widgets[1].good"},
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
            std::uint64_t{20} * (25 * 12 + 210) + kFixed);

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

// REQ-OPS-39: Boundary limits for widgets (<=64), distinct channels (<=256),
// text length (<=64), channel/pose length (<=256), trail_s ([0, 30]),
// precision ([0, 6]), power warn_below_v ((0, 20]), and checks at_least ==
// at_most are accepted at their exact limits and refused one past.
TEST(DashboardLayoutTest, ExactBoundsAcceptedAndOnePastRefused) {
  // Exact 64-byte title/label, 256-byte channel, 248-byte pose (+ "/yaw_rad" =
  // 256), trail_s = 0 and 30, precision = 0 and 6, warn_below_v = 20, and
  // checks at_least == at_most all parse cleanly.
  const std::string max_text(kDashboardMaxTextBytes, 'x');
  const std::string max_channel = "a/s/" + std::string(252, 'c');
  const std::string max_pose = "a/s/" + std::string(244, 'p');
  EXPECT_NO_THROW(Parse(
      "[dashboard]\n"
      "title = \"" +
      max_text +
      "\"\n"
      "[[widgets]]\nkind = \"number\"\nat = [0, 0, 1, 1]\n"
      "title = \"" +
      max_text +
      "\"\n"
      "label = \"" +
      max_text +
      "\"\n"
      "channel = \"" +
      max_channel +
      "\"\n"
      "precision = 0\n"
      "[[widgets]]\nkind = \"number\"\nat = [1, 0, 1, 1]\n"
      "channel = \"a/s/b\"\nprecision = 6\n"
      "[[widgets]]\nkind = \"field\"\nat = [2, 0, 2, 2]\n"
      "pose = \"" +
      max_pose +
      "\"\ntrail_s = 0\n"
      "[[widgets]]\nkind = \"field\"\nat = [4, 0, 2, 2]\n"
      "pose = \"p/s/pose\"\ntrail_s = 30\n"
      "[[widgets]]\nkind = \"power\"\nat = [6, 0, 2, 1]\n"
      "warn_below_v = 20\n"
      "[[widgets]]\nkind = \"checks\"\nat = [8, 0, 2, 1]\n"
      "items = [{ label = \"" +
      max_text + "\", channel = \"a/s/c\", at_least = 5, at_most = 5 }]\n"));

  // Exact 64 widgets accepted; 65th refused naming widgets[64].
  auto make_widgets = [](std::size_t count) {
    std::string toml;
    for (std::size_t i = 0; i < count; ++i) {
      toml += "[[widgets]]\nkind = \"alerts\"\nat = [" +
              std::to_string(i % 12) + ", " + std::to_string(i / 12) +
              ", 1, 1]\n";
    }
    return toml;
  };
  EXPECT_EQ(Parse(make_widgets(kDashboardMaxWidgets)).widgets.size(),
            kDashboardMaxWidgets);
  try {
    static_cast<void>(Parse(make_widgets(kDashboardMaxWidgets + 1)));
    ADD_FAILURE() << "accepted 65 widgets";
  } catch (const DashboardLayoutError& error) {
    const std::string what = error.what();
    EXPECT_NE(what.find(kFile), std::string::npos) << what;
    EXPECT_NE(what.find("widgets[64]"), std::string::npos) << what;
  }

  // Exact 256 distinct channels (16 checks widgets x 16 items) accepted; 257th
  // refused naming widgets[16].
  auto make_channels = [](std::size_t extra_channels) {
    std::string toml =
        "[dashboard]\nupdate_hz = 1\ndrive_clients = 1\n"
        "budget_bytes_per_s = 80000\n";
    std::size_t ch = 0;
    for (std::size_t w = 0; w < 16; ++w) {
      toml += "[[widgets]]\nkind = \"checks\"\nat = [" +
              std::to_string(w % 12) + ", " + std::to_string(w / 12) +
              ", 1, 1]\nitems = [\n";
      for (std::size_t k = 0; k < kDashboardMaxCheckItems; ++k) {
        toml += "  { label = \"c" + std::to_string(ch) +
                "\", channel = \"a/s/c" + std::to_string(ch) +
                "\", equals = true },\n";
        ++ch;
      }
      toml += "]\n";
    }
    for (std::size_t e = 0; e < extra_channels; ++e) {
      const std::size_t w = 16 + e;
      toml += "[[widgets]]\nkind = \"number\"\nat = [" +
              std::to_string(w % 12) + ", " + std::to_string(w / 12) +
              ", 1, 1]\nchannel = \"a/s/c" + std::to_string(ch++) + "\"\n";
    }
    return toml;
  };
  EXPECT_EQ(RequiredChannels(Parse(make_channels(0))).size(),
            kDashboardMaxChannels);
  try {
    static_cast<void>(Parse(make_channels(1)));
    ADD_FAILURE() << "accepted 257 distinct channels";
  } catch (const DashboardLayoutError& error) {
    const std::string what = error.what();
    EXPECT_NE(what.find(kFile), std::string::npos) << what;
    EXPECT_NE(what.find("widgets[16]"), std::string::npos) << what;
  }
}

// REQ-OPS-39: ParseDashboardLayout(path) must refuse a directory path with the
// same "cannot open the dashboard layout" error as a missing file rather than
// reading 0 bytes via std::ifstream on Linux.
TEST(DashboardLayoutTest, ParseDashboardLayoutRefusesDirectoryPath) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  ASSERT_NE(tmp, nullptr);
  const std::filesystem::path dir =
      std::filesystem::path{tmp} / "dashboard_layout_dir";
  std::filesystem::create_directories(dir);
  try {
    static_cast<void>(ParseDashboardLayout(dir));
    ADD_FAILURE() << "accepted a directory as a dashboard layout";
  } catch (const DashboardLayoutError& error) {
    const std::string what = error.what();
    EXPECT_NE(what.find("cannot open the dashboard layout"), std::string::npos)
        << what;
  }
  std::filesystem::remove_all(dir);
}

}  // namespace
}  // namespace talos::telemetry
