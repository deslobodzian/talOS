#include "studio/bridge/drive.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/introspection/registry.h"
#include "talOS/telemetry/frames.h"
#include "talOS/telemetry/wire.h"

namespace studio {
namespace {

using talos::telemetry::ChannelDef;
using talos::telemetry::PageHeader;
using talos::telemetry::RawFromDouble;

constexpr std::uint64_t kStream = 77;
constexpr std::uint32_t kDictionaryId = 4242;
constexpr std::int64_t kMs = 1'000'000;

// Channel 0 is named by the layout, 4 is an alert, the rest neither.
std::vector<ChannelDef> Dictionary() {
  return {
      {"drive/velocity", "m/s", {}, Talos::Telemetry::ChannelKind_Double},
      {"intake/inhibit", "", {}, Talos::Telemetry::ChannelKind_UInt},
      {"pose/valid", "", {}, Talos::Telemetry::ChannelKind_Bool},
      {"hw/status/clock_boot_id", "", {}, Talos::Telemetry::ChannelKind_UInt},
      {"hw/status/alert_error_link_lost",
       "",
       {},
       Talos::Telemetry::ChannelKind_Bool,
       Talos::Telemetry::AlertLevel_Error},
  };
}

void Ingest(TelemetryState& state, const std::vector<std::string>& pages) {
  for (const std::string& page : pages) {
    ASSERT_TRUE(IngestTelemetry(state, page, 0));
  }
}

TelemetryState WithDictionary() {
  TelemetryState state;
  const auto dictionary = Dictionary();
  std::vector<std::string> pages;
  flatbuffers::FlatBufferBuilder builder;
  talos::telemetry::EncodeDictionaryPages(
      builder,
      PageHeader{kStream, 0, 1, kDictionaryId,
                 static_cast<std::uint32_t>(dictionary.size())},
      dictionary, [&](std::span<const std::uint8_t> page) {
        pages.emplace_back(reinterpret_cast<const char*>(page.data()),
                           page.size());
      });
  Ingest(state, pages);
  return state;
}

// A keyframe tick: velocity `velocity`, inhibit `inhibit`, the rest fixed.
void Tick(TelemetryState& state, std::uint64_t sequence, double velocity,
          std::uint64_t inhibit = 0) {
  std::vector<std::string> pages;
  flatbuffers::FlatBufferBuilder builder;
  const Talos::Telemetry::Pose chassis{1, 2, 0, 0, 0, 0.5};
  const std::vector<std::uint32_t> channels{0, 1, 2, 3, 4};
  const std::vector<std::uint64_t> values{RawFromDouble(velocity), inhibit, 1,
                                          9, 0};
  ASSERT_TRUE(talos::telemetry::EncodeValuePages(
      builder, PageHeader{kStream, sequence, 1000 + sequence, kDictionaryId, 5},
      chassis, true, channels, values, {},
      [&](std::span<const std::uint8_t> page,
          const talos::telemetry::PageSpan&) {
        pages.emplace_back(reinterpret_cast<const char*>(page.data()),
                           page.size());
      }));
  Ingest(state, pages);
}

// What one drive client received.
struct Received {
  std::size_t dictionary_pages{0};
  std::vector<const Talos::Telemetry::Frame*> value_pages;
  std::deque<std::string> bytes;  // the frames' storage, never moved
  std::vector<std::string> texts;
  std::vector<std::int64_t> value_times;

  // Channels any value page carried or named absent.
  std::set<std::uint32_t> channels() const {
    std::set<std::uint32_t> seen;
    for (const auto* frame : value_pages) {
      if (frame->channels() != nullptr) {
        seen.insert(frame->channels()->begin(), frame->channels()->end());
      }
      if (frame->absent() != nullptr) {
        seen.insert(frame->absent()->begin(), frame->absent()->end());
      }
    }
    return seen;
  }
};

// Pumps `feed` with an unlimited budget, recording what it sends.
void Pump(DriveFeed& feed, const TelemetryState& state,
          const std::vector<std::uint8_t>& mask, std::int64_t now,
          Received& received, std::string_view summary = {}) {
  feed.Pump(
      summary, state, mask, now, [](std::size_t) { return true; },
      [&](std::string_view text) {
        received.texts.emplace_back(text);
        return true;
      },
      [&](std::string_view page) {
        received.bytes.emplace_back(page);
        const auto* frame = Talos::Telemetry::GetSizePrefixedFrame(
            received.bytes.back().data());
        if (frame->dictionary() != nullptr) {
          ++received.dictionary_pages;
        } else {
          received.value_pages.push_back(frame);
          received.value_times.push_back(now);
        }
        return true;
      });
}

// REQ-OPS-35
// A drive client gets the whole dictionary, so indices mean what they mean
// to Studio, but only its layout's channels, plus every alert channel when
// the layout has an `alerts` widget. A change to any other channel sends it
// nothing.
TEST(DriveFeed, SendsTheDictionaryThenOnlyItsMask) {
  struct Case {
    bool alerts;
    std::set<std::uint32_t> expected;
  };
  for (const Case& c : {Case{false, {0}}, Case{true, {0, 4}}}) {
    SCOPED_TRACE(c.alerts ? "with an alerts widget" : "without");
    TelemetryState state = WithDictionary();
    Tick(state, 1, 0.5);
    DriveMask mask({"not/in/dictionary", "drive/velocity"}, c.alerts);
    DriveFeed feed;
    feed.Configure(20);
    Received received;
    Pump(feed, state, mask.For(state), 0, received);
    EXPECT_EQ(received.dictionary_pages, state.dictionary_pages.size());
    ASSERT_EQ(received.value_pages.size(), 1u);
    EXPECT_TRUE(received.value_pages[0]->keyframe());
    EXPECT_EQ(received.channels(), c.expected);

    // Only unmasked channels change: past the update interval and before
    // the heartbeat, nothing goes.
    Tick(state, 2, 0.5, 1);
    Pump(feed, state, mask.For(state), 60 * kMs, received);
    EXPECT_EQ(received.value_pages.size(), 1u);
    // A masked channel changes: one update, carrying only it.
    Tick(state, 3, 0.75, 1);
    Pump(feed, state, mask.For(state), 70 * kMs, received);
    ASSERT_EQ(received.value_pages.size(), 2u);
    const auto* update = received.value_pages[1];
    ASSERT_NE(update->channels(), nullptr);
    ASSERT_EQ(update->channels()->size(), 1u);
    EXPECT_EQ(update->channels()->Get(0), 0u);
    EXPECT_EQ(received.channels(), c.expected);
    EXPECT_TRUE(feed.telemetry().current(state, &mask.For(state)));
  }
}

// REQ-OPS-35
// With a change every 10 ms, a 20 Hz layout's client gets an update every
// 50 ms and no more, and the last change still reaches it, at the instant
// the update interval ends (NextDueNs, which wakes the bridge).
TEST(DriveFeed, AtMostUpdateHzUpdatesASecondAndTheLastChangeArrives) {
  constexpr std::uint32_t kUpdateHz = 20;
  constexpr std::int64_t kIntervalNs = 1'000'000'000 / kUpdateHz;
  TelemetryState state = WithDictionary();
  DriveMask mask({"drive/velocity"}, false);
  DriveFeed feed;
  feed.Configure(kUpdateHz);
  Received received;
  std::uint64_t sequence = 0;
  std::int64_t now = 0;
  for (; now < 1'000'000'000; now += 10 * kMs) {
    ++sequence;
    Tick(state, sequence, static_cast<double>(sequence));
    Pump(feed, state, mask.For(state), now, received);
  }
  ASSERT_EQ(received.value_times.size(), kUpdateHz)
      << "updates in one second at " << kUpdateHz << " Hz";
  for (std::size_t i = 1; i < received.value_times.size(); ++i) {
    EXPECT_GE(received.value_times[i] - received.value_times[i - 1],
              kIntervalNs)
        << "update " << i;
  }
  // The producer stops; the last change waits for the interval only.
  const std::int64_t due = feed.NextDueNs();
  EXPECT_EQ(due, received.value_times.back() + kIntervalNs);
  Pump(feed, state, mask.For(state), due - 1, received);
  EXPECT_FALSE(feed.telemetry().current(state, &mask.For(state)));
  Pump(feed, state, mask.For(state), due, received);
  EXPECT_TRUE(feed.telemetry().current(state, &mask.For(state)));
}

// REQ-OPS-36
// A layout slower than the heartbeat (2 Hz) still hears from the stream
// every 100 ms while a change waits, so the page never takes a held update
// for a dead link; each such heartbeat carries the tick the client holds,
// never the newer one whose values it has not been sent.
TEST(DriveFeed, HeldChangesStillHeartbeatTheTickTheClientHolds) {
  TelemetryState state = WithDictionary();
  DriveMask mask({"drive/velocity"}, false);
  DriveFeed feed;
  feed.Configure(2);
  Received received;
  std::uint64_t sequence = 0;
  for (std::int64_t now = 0; now < 1'000'000'000; now += 10 * kMs) {
    ++sequence;
    Tick(state, sequence, static_cast<double>(sequence));
    Pump(feed, state, mask.For(state), now, received);
    // The bridge also wakes at the feed's own due instants.
    for (std::int64_t due = feed.NextDueNs(); due < now + 10 * kMs;
         due = feed.NextDueNs()) {
      Pump(feed, state, mask.For(state), due, received);
    }
  }
  ASSERT_GE(received.value_times.size(), 2u);
  std::uint64_t held = 0;
  double held_velocity = 0;
  for (std::size_t i = 0; i < received.value_pages.size(); ++i) {
    SCOPED_TRACE("page " + std::to_string(i));
    const auto* page = received.value_pages[i];
    if (i > 0) {
      EXPECT_LE(received.value_times[i] - received.value_times[i - 1],
                ClientFeed::kHeartbeatNs);
    }
    if (page->channels() != nullptr) {
      held = page->sequence_id();
      held_velocity = talos::telemetry::DoubleFromRaw(page->values()->Get(0));
    } else {
      EXPECT_EQ(page->sequence_id(), held);
    }
    // A tick's velocity is its sequence: what the client shows is the tick
    // it is stamped with.
    EXPECT_EQ(held_velocity, static_cast<double>(held));
  }
}

// REQ-OPS-36
// Before any telemetry tick (the telemetry node is down or not started) a
// drive client still hears from the bridge every 100 ms, so the page tells
// "telemetry stale" from "dashboard to bridge"; once it holds a tick, the
// telemetry heartbeat takes over and the text heartbeat stops.
TEST(DriveFeed, WithoutTelemetryTheBridgeStillHeartbeats) {
  TelemetryState state;  // nothing published yet
  DriveMask mask({"drive/velocity"}, false);
  DriveFeed feed;
  feed.Configure(20);
  Received received;
  const auto heartbeats = [&] {
    return std::count(received.texts.begin(), received.texts.end(),
                      DriveFeed::kHeartbeat);
  };
  Pump(feed, state, mask.For(state), 0, received);
  EXPECT_EQ(heartbeats(), 1);
  EXPECT_EQ(feed.NextDueNs(), ClientFeed::kHeartbeatNs);
  Pump(feed, state, mask.For(state), ClientFeed::kHeartbeatNs - 1, received);
  EXPECT_EQ(heartbeats(), 1);
  Pump(feed, state, mask.For(state), ClientFeed::kHeartbeatNs, received);
  EXPECT_EQ(heartbeats(), 2);

  state = WithDictionary();
  Tick(state, 1, 0.5);
  Pump(feed, state, mask.For(state), 150 * kMs, received);
  ASSERT_EQ(received.value_pages.size(), 1u);
  for (std::int64_t now = 160 * kMs; now < 1'000'000'000; now += 10 * kMs) {
    Pump(feed, state, mask.For(state), now, received);
  }
  EXPECT_EQ(heartbeats(), 2);
  EXPECT_GE(received.value_pages.size(), 8u) << "telemetry heartbeats";
}

// REQ-OPS-35
// The summary goes when it changes, at most every 500 ms, and never past
// 4,096 B; each bound at its edge.
TEST(DriveSummaryFeed, OnChangeAtMostEveryIntervalAndNeverPastTheCap) {
  DriveSummaryFeed feed;
  std::vector<std::pair<std::int64_t, std::string>> sent;
  const auto pump = [&](std::string_view summary, std::int64_t now) {
    feed.Pump(
        summary, now, [](std::size_t) { return true; },
        [&](std::string_view text) {
          sent.emplace_back(now, text);
          return true;
        });
  };
  const std::string a(100, 'a');
  const std::string b(DriveSummaryFeed::kMaxBytes, 'b');
  const std::string too_large(DriveSummaryFeed::kMaxBytes + 1, 'c');
  pump(a, 0);
  pump(a, 50 * kMs);  // unchanged: not resent
  ASSERT_EQ(sent.size(), 1u);
  pump(b, 100 * kMs);
  EXPECT_EQ(feed.NextDueNs(), DriveSummaryFeed::kIntervalNs);
  pump(b, DriveSummaryFeed::kIntervalNs - 1);
  ASSERT_EQ(sent.size(), 1u) << "a change inside the interval went";
  pump(b, DriveSummaryFeed::kIntervalNs);
  ASSERT_EQ(sent.size(), 2u) << "the change did not go at the interval";
  EXPECT_EQ(sent[1].second.size(), DriveSummaryFeed::kMaxBytes);
  pump(too_large, 10'000'000'000);
  EXPECT_EQ(sent.size(), 2u) << "a summary past the cap was sent";
  EXPECT_EQ(feed.NextDueNs(), kNeverNs);
}

// REQ-OPS-35
// DriveSummaryJson keeps every node up to exactly 4,096 B; one byte more and
// the last node is left out and counted, and the message still fits.
TEST(DriveSummary, CapKeepsWholeNodesAndCountsTheRest) {
  DriveSummary summary;
  summary.supervised = true;
  summary.nodes.reserve(36);
  for (int i = 0; i < 36; ++i) {
    DriveSummaryNode node;
    node.name = "subsystem_" + std::to_string(i);
    node.alive = i % 2 == 0;
    node.restarts = static_cast<std::uint64_t>(i);
    node.drives = "teleop";
    summary.nodes.push_back(node);
  }
  const std::size_t base = DriveSummaryJson(summary, SIZE_MAX).size();
  ASSERT_LT(base, talos::telemetry_wire::kDriveSummaryMaxBytes);
  // The session pads the message to exactly the cap.
  summary.session.assign(talos::telemetry_wire::kDriveSummaryMaxBytes - base,
                         '7');
  std::string json = DriveSummaryJson(summary);
  EXPECT_EQ(json.size(), talos::telemetry_wire::kDriveSummaryMaxBytes);
  EXPECT_TRUE(json.ends_with("],\"omitted\":0}")) << json;
  EXPECT_NE(json.find("\"subsystem_35\""), std::string::npos);

  summary.session += '7';
  json = DriveSummaryJson(summary);
  EXPECT_LE(json.size(), talos::telemetry_wire::kDriveSummaryMaxBytes);
  EXPECT_TRUE(json.ends_with("],\"omitted\":1}")) << json;
  EXPECT_EQ(json.find("\"subsystem_35\""), std::string::npos);
  EXPECT_NE(json.find("\"subsystem_34\""), std::string::npos);
}

// REQ-OPS-35
// The summary says what the `subsystems` widget shows and channels cannot:
// each member the launcher supervises, in its manifest's order (the sim
// gateway the config does not list included, the Studio bridge itself
// left out), alive from the registry (a dead node raises no alert), its
// lane from its Target's selection, restarts and quarantine from the
// manifest, critical from the config by name. Without a manifest the
// config's roster is listed and it says so instead of reporting zero
// restarts as a fact.
TEST(DriveSummary, BuildsFromManifestRosterRegistryAndConfig) {
  const std::vector<DriveRosterEntry> config{{"hardware_node", true},
                                             {"drivetrain", true},
                                             {"intake", false},
                                             {"shooter", true}};
  std::vector<talos::introspect::NodeSnapshot> registry(3);
  registry[0].name = "hardware_node";
  registry[0].alive = true;
  registry[0].session_id = 41;
  registry[1].name = "drivetrain";
  registry[1].alive = true;
  registry[1].selections.push_back({{"teleop", "auto"}, 2, 1});
  registry[2].name = "intake";  // its row outlived it
  registry[2].alive = false;
  registry[2].selections.push_back({{"teleop"}, 1, 0});
  // As the launcher writes it with the simulation's gateway and Studio.
  const std::string manifest =
      R"({"session_id": 42, "nodes": [)"
      R"({"name": "hardware_node", "restart_count": 0, "quarantined": false},)"
      R"({"name": "drivetrain", "restart_count": 2, "quarantined": false},)"
      R"({"name": "intake", "restart_count": 5, "quarantined": true},)"
      R"({"name": "shooter", "restart_count": 0, "quarantined": false},)"
      R"({"name": "sim_gateway", "restart_count": 2, "quarantined": false},)"
      R"({"name": "studio_bridge", "restart_count": 0, "quarantined": false}]})";

  const DriveSummary summary = BuildDriveSummary(config, &registry, manifest);
  EXPECT_EQ(summary.session, "42");
  EXPECT_TRUE(summary.supervised);
  std::vector<std::string> names;
  names.reserve(summary.nodes.size());
  for (const auto& node : summary.nodes) names.push_back(node.name);
  ASSERT_EQ(names,
            (std::vector<std::string>{"hardware_node", "drivetrain", "intake",
                                      "shooter", "sim_gateway"}));
  const auto& hardware = summary.nodes[0];
  EXPECT_TRUE(hardware.alive);
  EXPECT_FALSE(hardware.drives.has_value());
  const auto& drivetrain = summary.nodes[1];
  EXPECT_TRUE(drivetrain.alive);
  EXPECT_EQ(drivetrain.drives, "auto");
  EXPECT_EQ(drivetrain.restarts, 2u);
  const auto& intake = summary.nodes[2];
  EXPECT_FALSE(intake.alive);
  EXPECT_FALSE(intake.critical);
  EXPECT_FALSE(intake.drives.has_value()) << "a dead node drives nothing";
  EXPECT_EQ(intake.restarts, 5u);
  EXPECT_TRUE(intake.quarantined);
  EXPECT_FALSE(summary.nodes[3].alive) << "never registered";
  const auto& gateway = summary.nodes[4];
  EXPECT_EQ(gateway.restarts, 2u);
  EXPECT_TRUE(gateway.critical) << "a member the config does not list";

  const DriveSummary unsupervised =
      BuildDriveSummary(config, &registry, "{not json");
  EXPECT_FALSE(unsupervised.supervised);
  EXPECT_EQ(unsupervised.session, "41");
  ASSERT_EQ(unsupervised.nodes.size(), config.size());
  EXPECT_EQ(unsupervised.nodes[2].name, "intake");
  EXPECT_FALSE(unsupervised.nodes[2].critical);
  EXPECT_EQ(unsupervised.nodes[2].restarts, 0u);
}

// REQ-OPS-35
// A registry the bridge cannot open is said, `"registry":false` right
// after `supervised`, so the page reads every `alive: false` as unknown
// instead of every node dead; a readable one, even with no row, says true.
TEST(DriveSummary, SaysWhenTheRegistryIsUnreadable) {
  const std::vector<DriveRosterEntry> config{{"hardware_node", true}};
  const DriveSummary unread = BuildDriveSummary(config, nullptr, "");
  EXPECT_FALSE(unread.registry);
  EXPECT_NE(DriveSummaryJson(unread).find(
                R"("supervised":false,"registry":false,"nodes":[)"),
            std::string::npos)
      << DriveSummaryJson(unread);
  const std::vector<talos::introspect::NodeSnapshot> none;
  const DriveSummary read = BuildDriveSummary(config, &none, "");
  EXPECT_TRUE(read.registry);
  EXPECT_NE(DriveSummaryJson(read).find(
                R"("supervised":false,"registry":true,"nodes":[)"),
            std::string::npos)
      << DriveSummaryJson(read);
}

// REQ-OPS-35
// The drive budget is reserved out of the total: four programmer clients
// that spend every byte the shared bucket affords never defer a drive
// client's update, and the rates still add up to 160,000 B/s.
TEST(BridgeEgress, AProgrammerFloodNeverDefersADriveClient) {
  constexpr std::size_t kMaxClients = 4;
  constexpr std::uint32_t kDriveClients = 2;
  constexpr std::uint32_t kDriveBudget = 24000;
  const EgressPlan plan =
      EgressPlan::Of(kMaxClients, kDriveClients, kDriveBudget);
  EXPECT_EQ(plan.programmer_bytes_per_s + static_cast<double>(kDriveClients) *
                                              plan.drive_client_bytes_per_s,
            EgressPlan::kTotalBytesPerSecond);
  EXPECT_EQ(plan.programmer_client_bytes_per_s,
            plan.programmer_bytes_per_s / static_cast<double>(kMaxClients));

  BridgeEgress egress{plan};
  std::vector<EgressBudget> programmers(kMaxClients);
  std::vector<EgressBudget> drives(kDriveClients);
  // A drive update of the layout's worst case at 20 Hz, framed.
  constexpr std::size_t kDriveUpdate = kDriveBudget / 20 - 100;
  constexpr std::size_t kPage = 1210;
  constexpr std::int64_t kEndNs = 10'000'000'000;
  double programmer_bytes = 0;
  std::size_t drive_updates = 0;
  for (std::int64_t now = 0; now < kEndNs; now += kMs) {
    for (auto& own : programmers) {
      std::int64_t wait = 0;
      while (egress.Affordable(own, false, kPage, now, wait)) {
        egress.Spend(own, false, kPage);
        programmer_bytes += static_cast<double>(kPage);
      }
    }
    if (now % (50 * kMs) != 0) continue;
    for (std::size_t d = 0; d < drives.size(); ++d) {
      std::int64_t wait = 0;
      ASSERT_TRUE(egress.Affordable(drives[d], true, kDriveUpdate, now, wait))
          << "drive client " << d << " deferred at " << now << " ns, " << wait
          << " ns to wait";
      egress.Spend(drives[d], true, kDriveUpdate);
      ++drive_updates;
    }
  }
  EXPECT_EQ(drive_updates, 2u * 200u);
  // Programmer clients took no more than their share and its burst.
  EXPECT_LE(programmer_bytes,
            plan.programmer_bytes_per_s * 10 + EgressBudget::kBurstBytes);
}

// REQ-OPS-35
// At most `drive_clients` are held; the newest evicts the oldest, so a
// laptop reconnecting after a Wi-Fi drop always gets a slot.
TEST(DriveSlots, ANewClientPastCapacityEvictsTheOldest) {
  DriveSlots<char> slots(2);
  EXPECT_FALSE(slots.Admit('a').evicted.has_value());
  EXPECT_FALSE(slots.Admit('b').evicted.has_value());
  const auto c = slots.Admit('c');
  EXPECT_EQ(c.evicted, 'a');
  EXPECT_EQ(c.slot, 0u) << "c takes the slot a held";
  EXPECT_EQ(slots.Budget('a'), nullptr) << "an evicted client is sent nothing";
  EXPECT_EQ(slots.clients(), (std::vector<char>{'b', 'c'}));
  slots.Remove('b');
  const auto d = slots.Admit('d');
  EXPECT_FALSE(d.evicted.has_value());
  EXPECT_EQ(d.slot, 1u) << "d takes the slot b freed";
  EXPECT_EQ(slots.Admit('e').evicted, 'c');
  EXPECT_EQ(slots.clients(), (std::vector<char>{'d', 'e'}));
}

// REQ-OPS-35
// Reconnecting never mints bytes: three pages on two slots replace each
// other every 20 ms for 10 s, and each new connection is sent the
// dictionary, a keyframe and a 4,000 B summary from the bucket of the slot
// it fills. Every drive byte sent by any instant t stays within
// drive_clients x budget_bytes_per_s x t + drive_clients x burst, so the
// reservation out of the bridge's 160,000 B/s holds. Once the churn stops,
// the newest page still gets the whole dictionary, in order, then a
// keyframe.
TEST(DriveSlots, ReconnectingNeverMintsBytes) {
  constexpr std::uint32_t kDriveClients = 2;
  constexpr std::uint32_t kBudget = 24000;
  constexpr std::int64_t kChurnEndNs = 10'000'000'000;
  BridgeEgress egress{EgressPlan::Of(4, kDriveClients, kBudget)};
  DriveSlots<int> slots(kDriveClients);
  std::map<int, DriveFeed> feeds;
  std::map<int, std::vector<std::string>> binaries;
  TelemetryState state = WithDictionary();
  Tick(state, 1, 0.5);
  DriveMask mask({"drive/velocity"}, true);
  const std::string summary(4000, 's');
  double sent = 0;
  const auto pump = [&](int client, std::int64_t now) {
    EgressBudget* budget = slots.Budget(client);
    ASSERT_NE(budget, nullptr);
    const auto charge = [&](std::string_view bytes) {
      const std::size_t framed = EgressBudget::framed(bytes.size());
      egress.Spend(*budget, true, framed);
      sent += static_cast<double>(framed);
      return true;
    };
    feeds[client].Pump(
        summary, state, mask.For(state), now,
        [&](std::size_t framed) {
          std::int64_t wait = 0;
          return egress.Affordable(*budget, true, framed, now, wait);
        },
        charge,
        [&](std::string_view page) {
          binaries[client].emplace_back(page);
          return charge(page);
        });
  };
  int next = 0;
  for (std::int64_t now = 0; now <= kChurnEndNs; now += kMs) {
    if (now % (20 * kMs) == 0) {
      const auto admitted = slots.Admit(next);
      if (admitted.evicted) feeds.erase(*admitted.evicted);
      feeds[next].Configure(20);
      ++next;
    }
    for (const int client : slots.clients()) pump(client, now);
    const double bound =
        static_cast<double>(kDriveClients) * kBudget *
            (static_cast<double>(now) * 1e-9) +
        static_cast<double>(kDriveClients) * EgressBudget::kBurstBytes;
    ASSERT_LE(sent, bound) << "drive bytes past the reservation at " << now
                           << " ns, after " << next << " connections";
  }
  const int newest = next - 1;
  for (std::int64_t now = kChurnEndNs; now <= kChurnEndNs + 1'000'000'000;
       now += kMs) {
    pump(newest, now);
  }
  const std::vector<std::string>& got = binaries[newest];
  ASSERT_GT(got.size(), state.dictionary_pages.size());
  for (std::size_t i = 0; i < state.dictionary_pages.size(); ++i) {
    EXPECT_EQ(got[i], state.dictionary_pages[i]) << "dictionary page " << i;
  }
  const auto* first_update = Talos::Telemetry::GetSizePrefixedFrame(
      got[state.dictionary_pages.size()].data());
  EXPECT_TRUE(first_update->keyframe());
}

}  // namespace
}  // namespace studio
