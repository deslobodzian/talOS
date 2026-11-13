#include "studio/bridge/telemetry.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "studio/bridge/system.h"
#include "talOS/introspection/registry.h"
#include "talOS/telemetry/frames.h"

namespace studio {
namespace {

using talos::telemetry::AlertLevel;
using talos::telemetry::ChannelDef;
using talos::telemetry::PageHeader;
using talos::telemetry::RawFromDouble;

constexpr std::uint64_t kStream = 9007199254740993ull;
constexpr std::uint32_t kDictionaryId = 4242;
// 2^53 + 1: no double holds it.
constexpr std::uint64_t kBeyondDouble = (std::uint64_t{1} << 53) + 1;

std::vector<ChannelDef> SmallDictionary(const char* third = "pose/valid") {
  return {
      {"drive/velocity", "m/s", {}, Talos::Telemetry::ChannelKind_Double},
      {"intake/inhibit",
       "",
       {"DRIVE", "TARGET_STALE"},
       Talos::Telemetry::ChannelKind_UInt},
      {third, "", {}, Talos::Telemetry::ChannelKind_Bool},
      {"hw/status/clock_boot_id", "", {}, Talos::Telemetry::ChannelKind_UInt},
  };
}

std::vector<std::string> DictionaryPages(
    const std::vector<ChannelDef>& dictionary, std::uint64_t stream = kStream,
    std::uint32_t id = kDictionaryId) {
  std::vector<std::string> pages;
  flatbuffers::FlatBufferBuilder builder;
  talos::telemetry::EncodeDictionaryPages(
      builder,
      PageHeader{stream, 0, 1, id,
                 static_cast<std::uint32_t>(dictionary.size())},
      dictionary, [&](std::span<const std::uint8_t> page) {
        pages.emplace_back(reinterpret_cast<const char*>(page.data()),
                           page.size());
      });
  return pages;
}

// One producer tick (a keyframe) carrying `channels`/`values`.
std::vector<std::string> TickPages(std::uint32_t channel_count,
                                   std::uint64_t sequence,
                                   const std::vector<std::uint32_t>& channels,
                                   const std::vector<std::uint64_t>& values,
                                   double yaw = 0.5,
                                   std::uint64_t stream = kStream,
                                   std::uint32_t id = kDictionaryId) {
  std::vector<std::string> pages;
  flatbuffers::FlatBufferBuilder builder;
  const Talos::Telemetry::Pose chassis{1.5, 2.5, 0, 0, 0, yaw};
  EXPECT_TRUE(talos::telemetry::EncodeValuePages(
      builder,
      PageHeader{stream, sequence, 123456789 + sequence, id, channel_count},
      chassis, true, channels, values, {},
      [&](std::span<const std::uint8_t> page,
          const talos::telemetry::PageSpan&) {
        pages.emplace_back(reinterpret_cast<const char*>(page.data()),
                           page.size());
      }));
  return pages;
}

std::vector<std::string> SmallTick(double velocity, std::uint64_t inhibit,
                                   double yaw = 0.5) {
  return TickPages(4, 7, {0, 1, 2, 3},
                   {RawFromDouble(velocity), inhibit, 1, kBeyondDouble}, yaw);
}

bool IngestAll(TelemetryState& state, const std::vector<std::string>& pages,
               std::int64_t now = 0) {
  for (const std::string& page : pages) {
    if (!IngestTelemetry(state, page, now)) return false;
  }
  return true;
}

TelemetryState SmallState(double velocity = 0.75, std::uint64_t inhibit = 1) {
  TelemetryState state;
  EXPECT_TRUE(IngestAll(state, DictionaryPages(SmallDictionary())));
  EXPECT_TRUE(IngestAll(state, SmallTick(velocity, inhibit), 1'000'000'000));
  return state;
}

// A client as Studio decodes it: dictionary pages, then value pages applied
// in order; a keyframe's first page starts from every channel absent, and an
// update is complete (a Studio snapshot) at its last page.
struct Mirror {
  std::size_t dictionary_pages{0};
  bool has_values{false};
  std::uint64_t sequence_id{0};
  std::vector<std::uint64_t> values;
  std::vector<std::uint8_t> present;
  std::size_t updates{0};
  std::size_t pages{0};

  void Apply(std::string_view bytes) {
    ++pages;
    const auto* frame = Talos::Telemetry::GetSizePrefixedFrame(bytes.data());
    if (frame->dictionary() != nullptr) {
      ++dictionary_pages;
      return;
    }
    values.resize(frame->channel_count());
    present.resize(frame->channel_count());
    if (frame->keyframe() && frame->page() == 0) {
      std::fill(present.begin(), present.end(), 0);
      has_values = true;
    }
    if (frame->channels() != nullptr) {
      for (std::size_t i = 0; i < frame->channels()->size(); ++i) {
        values[frame->channels()->Get(i)] = frame->values()->Get(i);
        present[frame->channels()->Get(i)] = 1;
      }
    }
    if (frame->absent() != nullptr) {
      for (const std::uint32_t channel : *frame->absent()) present[channel] = 0;
    }
    if (frame->page() + 1 == frame->page_count()) {
      sequence_id = frame->sequence_id();
      ++updates;
    }
  }

  bool Equals(const TelemetryState& state) const {
    if (!has_values || present != state.present) return false;
    for (std::size_t i = 0; i < present.size(); ++i) {
      if (present[i] != 0 && values[i] != state.values[i]) return false;
    }
    return true;
  }
};

TEST(TelemetryJson, SelfDescribesTheLatestTick) {
  const TelemetryState state = SmallState();
  ASSERT_TRUE(state.has_dictionary);
  ASSERT_TRUE(state.has_frame);

  const std::string json = TelemetryJson(state, 1'250'000'000);
  EXPECT_NE(json.find("\"kind\":\"talos.telemetry\""), std::string::npos);
  EXPECT_NE(json.find("\"version\":3"), std::string::npos);
  // uint64 past 2^53 survives as a decimal string.
  EXPECT_NE(json.find("\"stream_id\":\"9007199254740993\""), std::string::npos);
  EXPECT_NE(json.find("\"sequence_id\":\"7\""), std::string::npos);
  EXPECT_NE(json.find("\"dictionary_id\":4242"), std::string::npos);
  EXPECT_NE(json.find("\"age_ms\":250"), std::string::npos);
  EXPECT_NE(json.find("\"x\":1.5"), std::string::npos);
  EXPECT_NE(json.find("\"drive/velocity\":{\"value\":0.75,\"unit\":\"m/s\"}"),
            std::string::npos);
  // A unitless channel still carries "unit", so readers never branch on it.
  EXPECT_NE(json.find("\"pose/valid\":{\"value\":1,\"unit\":\"\"}"),
            std::string::npos);
  // An integer channel past 2^53 keeps every digit.
  EXPECT_NE(json.find("\"hw/status/clock_boot_id\":{\"value\":"
                      "\"9007199254740993\""),
            std::string::npos)
      << json;
}

TEST(TelemetryJson, AbsentLeavesAsNullAndEnumLabelsOnlyInRange) {
  TelemetryState state;
  ASSERT_TRUE(IngestAll(state, DictionaryPages(SmallDictionary())));
  // drive/velocity absent this tick.
  ASSERT_TRUE(IngestAll(state, TickPages(4, 7, {1, 2}, {1, 1})));
  std::string json = TelemetryJson(state, 0);
  EXPECT_NE(json.find("\"drive/velocity\":{\"value\":null,"),
            std::string::npos);
  EXPECT_NE(json.find("\"intake/inhibit\":{\"value\":1,\"unit\":\"\","
                      "\"label\":\"TARGET_STALE\"}"),
            std::string::npos);

  // Out of range: a number with no invented label.
  const TelemetryState out_of_range = SmallState(1.0, 9);
  json = TelemetryJson(out_of_range, 0);
  EXPECT_NE(json.find("\"intake/inhibit\":{\"value\":9,\"unit\":\"\"}"),
            std::string::npos);
  EXPECT_EQ(json.find("\"label\""), std::string::npos);
}

TEST(TelemetryJson, ValuePagesWithoutTheirDictionaryAreNotUsable) {
  TelemetryState state;
  ASSERT_TRUE(IngestAll(state, SmallTick(1.0, 0)));
  EXPECT_FALSE(state.has_dictionary);
  EXPECT_FALSE(state.has_frame);
  // The dictionary for the same stream unlocks later ticks.
  ASSERT_TRUE(IngestAll(state, DictionaryPages(SmallDictionary())));
  ASSERT_TRUE(IngestAll(state, SmallTick(2.0, 0), 5));
  EXPECT_TRUE(state.has_frame);
  EXPECT_EQ(state.values[0], RawFromDouble(2.0));

  // A different dictionary invalidates the values decoded against the old
  // one; a repeat of the same dictionary does not.
  ASSERT_TRUE(IngestAll(state, DictionaryPages(SmallDictionary())));
  EXPECT_TRUE(state.has_frame);
  ASSERT_TRUE(
      IngestAll(state, DictionaryPages(SmallDictionary(), kStream, 99)));
  EXPECT_FALSE(state.has_frame);
}

TEST(TelemetryJson, RejectsInvalidPagesWithoutChangingCachedState) {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::vector<std::string>> invalid{
      // The chassis is a pose the field view draws, never a reading.
      SmallTick(1.0, 0, nan), SmallTick(1.0, 0, infinity),
      // A bool carries 0 or 1.
      TickPages(4, 8, {2}, {2}),
      // A channel index past the dictionary.
      TickPages(4, 8, {4}, {1}),
      // A dictionary past the channel cap: no client could be sent a
      // keyframe of it whole.
      TickPages(talos::telemetry_wire::kMaxChannels + 1, 8, {0}, {1}),
      DictionaryPages(SmallDictionary("drive/velocity"), 55, 99),
      DictionaryPages(SmallDictionary("derived/spoof"), 55, 99)};
  for (const auto& pages : invalid) {
    TelemetryState state = SmallState();
    const std::string before = TelemetryJson(state, 10);
    EXPECT_FALSE(IngestAll(state, pages, 20));
    EXPECT_EQ(TelemetryJson(state, 10), before);
    ASSERT_TRUE(IngestAll(state, SmallTick(2.0, 0), 30));
    EXPECT_EQ(state.values[0], RawFromDouble(2.0));
  }
}

// A tick past one page reassembles channel by channel, and a tick missing a
// page (the reader was lapped) is never half-applied.
TEST(TelemetryPages, PagedTickReassemblesEveryChannel) {
  std::vector<ChannelDef> dictionary;
  std::vector<std::uint32_t> channels;
  std::vector<std::uint64_t> values;
  for (std::uint32_t i = 0; i < 300; ++i) {
    dictionary.push_back({"test/state/channel_" + std::to_string(i),
                          "",
                          {},
                          Talos::Telemetry::ChannelKind_Int});
    channels.push_back(i);
    values.push_back(static_cast<std::uint64_t>(-std::int64_t{i}));
  }
  const auto pages = TickPages(300, 1, channels, values);
  ASSERT_GE(pages.size(), 3u);
  for (const std::string& page : pages) {
    EXPECT_LE(page.size(), talos::telemetry_wire::kMaxValueFrameBytes);
  }

  TelemetryState state;
  ASSERT_TRUE(IngestAll(state, DictionaryPages(dictionary)));
  ASSERT_TRUE(IngestAll(state, pages));
  ASSERT_TRUE(state.has_frame);
  EXPECT_EQ(state.values, values);
  EXPECT_EQ(state.present, std::vector<std::uint8_t>(300, 1));

  // Tick 2 loses its middle page: tick 1 stays the latest, whole.
  std::vector<std::uint64_t> next(300, 7);
  auto lapped = TickPages(300, 2, channels, next);
  lapped.erase(lapped.begin() + 1);
  ASSERT_TRUE(IngestAll(state, lapped));
  EXPECT_EQ(state.sequence_id, 1u);
  EXPECT_EQ(state.values, values);
  EXPECT_EQ(state.incomplete_ticks, 1u);
}

// NaN and the infinities are readings, not absences: they are kept and leave
// as the strings JSON numbers cannot spell.
TEST(TelemetryJson, NonFiniteReadingsAreKeptAndLeaveAsStrings) {
  const std::vector<std::pair<double, std::string>> cases{
      {std::numeric_limits<double>::quiet_NaN(), "\"NaN\""},
      {std::numeric_limits<double>::infinity(), "\"Infinity\""},
      {-std::numeric_limits<double>::infinity(), "\"-Infinity\""}};
  for (const auto& [reading, spelled] : cases) {
    const TelemetryState state = SmallState(reading);
    ASSERT_TRUE(state.has_frame);
    EXPECT_EQ(state.present[0], 1);
    EXPECT_NE(TelemetryJson(state, 0).find("\"drive/velocity\":{\"value\":" +
                                           spelled + ",\"unit\":\"m/s\"}"),
              std::string::npos)
        << spelled;
  }
}

// The decoded value pages one client was sent.
std::vector<const Talos::Telemetry::Frame*> ValuePagesOf(
    const std::vector<std::string>& sent) {
  std::vector<const Talos::Telemetry::Frame*> pages;
  for (const std::string& page : sent) {
    const auto* frame = Talos::Telemetry::GetSizePrefixedFrame(page.data());
    if (frame->dictionary() == nullptr) pages.push_back(frame);
  }
  return pages;
}

// A channel that loses its sample is named in the update's `absent`, so the
// client stops showing its last value; nothing else is re-sent.
TEST(ClientFeed, ChannelThatLosesItsSampleIsSentAsAbsent) {
  TelemetryState state = SmallState();
  ClientFeed feed;
  std::vector<std::string> sent;
  const auto pump = [&](std::int64_t now) {
    feed.Pump(
        state, now, [](std::size_t) { return true; },
        [&sent](std::string_view page) {
          sent.emplace_back(page);
          return true;
        });
  };
  pump(0);
  ASSERT_TRUE(
      IngestAll(state, TickPages(4, 8, {1, 2, 3}, {1, 1, kBeyondDouble})));
  sent.clear();
  pump(1);
  const auto pages = ValuePagesOf(sent);
  ASSERT_EQ(pages.size(), 1u);
  EXPECT_FALSE(pages[0]->keyframe());
  EXPECT_EQ(pages[0]->channels(), nullptr);
  ASSERT_NE(pages[0]->absent(), nullptr);
  ASSERT_EQ(pages[0]->absent()->size(), 1u);
  EXPECT_EQ(pages[0]->absent()->Get(0), 0u);
  EXPECT_TRUE(feed.current(state));
}

// A page the socket refused leaves the client's view unknown: the next update
// is a keyframe, carrying every channel.
TEST(ClientFeed, RefusedPageBringsAKeyframe) {
  TelemetryState state = SmallState();
  ClientFeed feed;
  std::vector<std::string> sent;
  bool refuse = false;
  const auto pump = [&](std::int64_t now) {
    feed.Pump(
        state, now, [](std::size_t) { return true; },
        [&](std::string_view page) {
          if (refuse) return false;
          sent.emplace_back(page);
          return true;
        });
  };
  pump(0);
  ASSERT_TRUE(IngestAll(
      state, TickPages(4, 8, {0, 1, 2, 3}, {RawFromDouble(2.0), 0, 1, 1})));
  refuse = true;
  pump(1);
  EXPECT_FALSE(feed.current(state));
  refuse = false;
  sent.clear();
  pump(2);
  const auto pages = ValuePagesOf(sent);
  ASSERT_EQ(pages.size(), 1u);
  EXPECT_TRUE(pages[0]->keyframe());
  ASSERT_NE(pages[0]->channels(), nullptr);
  EXPECT_EQ(pages[0]->channels()->size(), 4u);
  EXPECT_TRUE(feed.current(state));
}

// With nothing changed, even with the producer silent, the client still hears
// from the stream every kHeartbeatNs, so it can tell a quiet robot from a
// dead link; the update repeats the tick it holds.
TEST(ClientFeed, HeartbeatWhenNothingChanged) {
  const TelemetryState state = SmallState();
  ClientFeed feed;
  std::vector<std::string> sent;
  const auto pump = [&](std::int64_t now) {
    feed.Pump(
        state, now, [](std::size_t) { return true; },
        [&sent](std::string_view page) {
          sent.emplace_back(page);
          return true;
        });
  };
  pump(0);
  sent.clear();
  pump(ClientFeed::kHeartbeatNs - 1);
  EXPECT_TRUE(sent.empty());
  pump(ClientFeed::kHeartbeatNs);
  const auto pages = ValuePagesOf(sent);
  ASSERT_EQ(pages.size(), 1u);
  EXPECT_FALSE(pages[0]->keyframe());
  EXPECT_EQ(pages[0]->channels(), nullptr);
  EXPECT_EQ(pages[0]->absent(), nullptr);
  EXPECT_EQ(pages[0]->sequence_id(), state.sequence_id);
}

// A client that connects mid-stream gets the dictionary and then a keyframe
// of the latest tick, exact integers included.
TEST(ClientFeed, LateJoinerGetsDictionaryThenKeyframe) {
  TelemetryState state = SmallState();
  ASSERT_TRUE(IngestAll(state, SmallTick(0.5, 0), 2));  // a later tick

  ClientFeed feed;
  Mirror mirror;
  feed.Pump(
      state, 0, [](std::size_t) { return true; },
      [&mirror](std::string_view page) {
        mirror.Apply(page);
        return true;
      });
  EXPECT_EQ(mirror.dictionary_pages, state.dictionary_pages.size());
  EXPECT_EQ(mirror.updates, 1u);
  EXPECT_TRUE(mirror.Equals(state));
  EXPECT_EQ(mirror.values[3], kBeyondDouble);
  EXPECT_TRUE(feed.current(state));
}

// Under a starved budget the client skips ticks but never shows a wrong
// value: each update it completes is exactly one producer tick, and once the
// producer stops it converges on the latest tick. Bytes stay within budget.
TEST(ClientFeed, StarvedBudgetDefersChangesAndConvergesWithinBudget) {
  constexpr std::uint32_t kChannels = 200;
  std::vector<ChannelDef> dictionary;
  std::vector<std::uint32_t> channels;
  for (std::uint32_t i = 0; i < kChannels; ++i) {
    dictionary.push_back({"test/state/channel_" + std::to_string(i),
                          "",
                          {},
                          Talos::Telemetry::ChannelKind_Double});
    channels.push_back(i);
  }
  // Every channel changes every tick; its value names the tick.
  const auto tick_values = [&](std::uint64_t tick) {
    std::vector<std::uint64_t> values;
    for (std::uint32_t i = 0; i < kChannels; ++i) {
      values.push_back(RawFromDouble(static_cast<double>(tick * 1000 + i)));
    }
    return values;
  };

  // About 2.5 kB per update against 20 kB/s: under a tenth of 100 Hz ticks.
  constexpr double kRate = 20000;
  TelemetryState state;
  ASSERT_TRUE(IngestAll(state, DictionaryPages(dictionary)));
  ClientFeed feed;
  EgressBudget budget;
  Mirror mirror;
  std::size_t sent = 0;
  std::size_t coherent_updates = 0;
  std::int64_t now = 0;
  const auto pump = [&] {
    feed.Pump(
        state, now,
        [&](std::size_t bytes) { return budget.available(bytes, kRate, now); },
        [&](std::string_view page) {
          const std::size_t updates = mirror.updates;
          mirror.Apply(page);
          budget.spend(EgressBudget::framed(page.size()));
          sent += EgressBudget::framed(page.size());
          if (mirror.updates != updates &&
              mirror.values == tick_values(mirror.sequence_id)) {
            ++coherent_updates;
          }
          return true;
        });
  };

  constexpr std::uint64_t kTicks = 300;  // 3 s at 100 Hz
  for (std::uint64_t tick = 1; tick <= kTicks; ++tick) {
    ASSERT_TRUE(IngestAll(
        state, TickPages(kChannels, tick, channels, tick_values(tick))));
    for (int ms = 0; ms < 10; ++ms, now += 1'000'000) pump();
  }
  EXPECT_LT(mirror.updates, kTicks / 4);
  EXPECT_EQ(coherent_updates, mirror.updates);

  // The producer stops; the deferred changes still arrive.
  while (!feed.current(state) && now < 10'000'000'000) {
    now += 1'000'000;
    pump();
  }
  EXPECT_TRUE(feed.current(state));
  EXPECT_TRUE(mirror.Equals(state));
  EXPECT_EQ(mirror.sequence_id, kTicks);
  EXPECT_LE(
      static_cast<double>(sent),
      EgressBudget::kBurstBytes + kRate * static_cast<double>(now) * 1e-9);
}

// A registry of `nodes` nodes with eight sources each, its counters as of
// `refresh`, and every node's pid as of `restarts`.
SystemGraphInput Registry(int nodes, std::uint64_t refresh,
                          std::uint64_t restarts) {
  SystemGraphInput input;
  input.registry_available = true;
  input.node_capacity = 64;
  input.source_capacity = 64;
  input.wall_ns = 1'700'000'000'000'000'000 +
                  static_cast<std::int64_t>(refresh) * 250'000'000;
  input.monotonic_ns = static_cast<std::int64_t>(refresh) * 250'000'000;
  input.bridge = {.topic = "/talos/telemetry", .clients = 1};
  for (int n = 0; n < nodes; ++n) {
    introspect::NodeSnapshot node;
    node.name = "node_" + std::to_string(n);
    node.target = "//robot/node_" + std::to_string(n) + ":node";
    node.pid = 1000 + static_cast<std::uint64_t>(n) + restarts * 100;
    node.alive = true;
    node.heartbeat_monotonic_ns = input.monotonic_ns;
    node.dispatch_count = refresh * 25;
    for (std::uint16_t i = 0; i < 8; ++i) {
      node.sources.push_back(
          {.id = i,
           .kind =
               i == 0 ? event::SourceKind::SENDER : event::SourceKind::WATCHER,
           .name = "/node_" + std::to_string((n + i) % nodes) + "/state",
           .message_bytes = 64,
           .alignment = 8,
           .events = refresh * 25 + i,
           .sequence = refresh * 25,
           .last_monotonic_ns = input.monotonic_ns,
           .last_latency_ns = 1000 + static_cast<std::int64_t>(refresh % 7)});
    }
    input.nodes.push_back(std::move(node));
  }
  return input;
}

std::string_view Between(std::string_view text, std::string_view open,
                         std::string_view close) {
  const std::size_t begin = text.find(open);
  if (begin == std::string_view::npos) return {};
  const std::size_t from = begin + open.size();
  return text.substr(from, text.find(close, from) - from);
}

// A client whose share the telemetry alone would exhaust: it still holds a
// graph no more than a few refreshes old, hears from the graph often enough
// that Studio never marks it stale while it is merely deferred (including
// across a restart that resends the whole document), converges on the
// current graph and tick, and stays within the budget.
TEST(ClientFeeds, StarvedClientKeepsTheGraphCurrentAndNeverStale) {
  constexpr std::uint32_t kChannels = 200;
  std::vector<ChannelDef> dictionary;
  std::vector<std::uint32_t> channels;
  for (std::uint32_t i = 0; i < kChannels; ++i) {
    dictionary.push_back({"test/state/channel_" + std::to_string(i),
                          "",
                          {},
                          Talos::Telemetry::ChannelKind_Double});
    channels.push_back(i);
  }
  const auto tick_values = [&](std::uint64_t tick) {
    std::vector<std::uint64_t> values;
    for (std::uint32_t i = 0; i < kChannels; ++i) {
      values.push_back(RawFromDouble(static_cast<double>(tick * 1000 + i)));
    }
    return values;
  };

  // About 250 kB/s of telemetry changes and a 25 kB graph against a
  // 20 kB/s share: the whole graph alone takes over a second to afford.
  constexpr double kRate = 20000;
  constexpr int kNodes = 9;
  constexpr std::int64_t kStaleNs = 1'000'000'000;  // GRAPH_STALE_AFTER_MS
  TelemetryState state;
  ASSERT_TRUE(IngestAll(state, DictionaryPages(dictionary)));
  std::uint64_t refresh = 0, restarts = 0;
  SystemGraphDocument document = BuildSystemGraph(Registry(kNodes, 0, 0));
  ASSERT_GT(document.json.size(), 24000u);
  std::map<std::string, std::uint64_t> refresh_of{{document.values, 0}};

  ClientFeeds feeds;
  EgressBudget budget;
  Mirror mirror;
  std::string held_structure, held_values;
  std::size_t sent = 0, documents = 0, notices = 0;
  std::int64_t now = 0, last_text = -1, longest_silence = 0, worst_lag = 0;
  const auto spend = [&](std::string_view bytes) {
    budget.spend(EgressBudget::framed(bytes.size()));
    sent += EgressBudget::framed(bytes.size());
  };
  const auto pump = [&] {
    feeds.Pump(
        document, state, now, kRate,
        [&](std::size_t bytes) { return budget.available(bytes, kRate, now); },
        [&](std::string_view text) {
          spend(text);
          if (last_text >= 0) {
            longest_silence = std::max(longest_silence, now - last_text);
          }
          last_text = now;
          if (text == document.json) {
            ++documents;
            held_structure = document.structure;
            held_values = document.values;
          } else if (text.starts_with(
                         "{\"kind\":\"talos.system_graph.update\"")) {
            EXPECT_EQ(Between(text, "\"structure\":\"", "\""), held_structure);
            held_values = Between(text, "\"values\":", "}");
          } else {
            EXPECT_TRUE(
                text.starts_with("{\"kind\":\"talos.system_graph.deferred\""));
            EXPECT_EQ(Between(text, "\"structure\":\"", "\""), held_structure);
            ++notices;
          }
          return true;
        },
        [&](std::string_view page) {
          spend(page);
          mirror.Apply(page);
          return true;
        });
  };

  for (std::uint64_t tick = 1; tick <= 1500; ++tick) {  // 15 s at 100 Hz
    ASSERT_TRUE(IngestAll(
        state, TickPages(kChannels, tick, channels, tick_values(tick))));
    if (tick % 25 == 0) {
      // A refresh every 250 ms; at 8 s every node restarts.
      restarts = tick >= 800 ? 1 : 0;
      document = BuildSystemGraph(Registry(kNodes, ++refresh, restarts));
      refresh_of[document.values] = refresh;
    }
    for (int ms = 0; ms < 10; ++ms, now += 1'000'000) {
      pump();
      if (!held_values.empty()) {
        worst_lag = std::max(
            worst_lag,
            static_cast<std::int64_t>(refresh - refresh_of.at(held_values)) *
                250'000'000);
      }
    }
  }
  EXPECT_LT(longest_silence, kStaleNs);
  // The 25 kB whole graph after the restart gets about half the share while
  // telemetry is thinned beside it (ClientFeeds::kHoldNs), so about 2.5 s,
  // plus one 250 ms refresh.
  EXPECT_LE(worst_lag, 3'000'000'000);
  EXPECT_EQ(documents, 2u);  // on connect, and after the restart
  EXPECT_GT(notices, 0u);
  EXPECT_GT(mirror.updates, 50u);  // telemetry keeps its share

  // Both feeds stop changing: the client converges on each.
  const std::int64_t stopped = now;
  while (now - stopped < 5'000'000'000 &&
         (held_values != document.values || !feeds.telemetry.current(state))) {
    now += 1'000'000;
    pump();
  }
  EXPECT_EQ(held_structure, document.structure);
  EXPECT_EQ(held_values, document.values);
  EXPECT_TRUE(mirror.Equals(state));
  EXPECT_LE(
      static_cast<double>(sent),
      EgressBudget::kBurstBytes + kRate * static_cast<double>(now) * 1e-9);
}

// REQ-RT-21
// The bridge does not poll: it runs a pass when a tick is published and
// otherwise only when its WakeSchedule says (the graph refresh, a client's
// NextDueNs, the instant a deferred send's budget refills). Driven by that
// schedule, as Bridge::service and Bridge::next_wake_ns drive it, a starved
// client still hears from the graph before Studio would mark it stale, gets
// its telemetry heartbeat on time once the producer goes quiet, and
// converges on the latest graph and tick.
TEST(WakeSchedule, PassesOnlyWhenDueKeepGraphHeartbeatAndConvergence) {
  constexpr std::uint32_t kChannels = 200;
  std::vector<ChannelDef> dictionary;
  std::vector<std::uint32_t> channels;
  for (std::uint32_t i = 0; i < kChannels; ++i) {
    dictionary.push_back({"test/state/channel_" + std::to_string(i),
                          "",
                          {},
                          Talos::Telemetry::ChannelKind_Double});
    channels.push_back(i);
  }
  const auto tick_values = [&](std::uint64_t tick) {
    std::vector<std::uint64_t> values;
    for (std::uint32_t i = 0; i < kChannels; ++i) {
      values.push_back(RawFromDouble(static_cast<double>(tick * 1000 + i)));
    }
    return values;
  };

  // As in StarvedClientKeepsTheGraphCurrentAndNeverStale: telemetry alone
  // would exhaust the share, and the whole graph takes over a second.
  constexpr double kRate = 20000;
  constexpr int kNodes = 9;
  constexpr std::int64_t kStaleNs = 1'000'000'000;  // GRAPH_STALE_AFTER_MS
  constexpr std::int64_t kTickNs = 10'000'000;
  constexpr std::int64_t kTicksEndNs = 3'000'000'000;
  constexpr std::int64_t kRefreshesEndNs = 8'000'000'000;
  constexpr std::int64_t kEndNs = 12'000'000'000;
  TelemetryState state;
  ASSERT_TRUE(IngestAll(state, DictionaryPages(dictionary)));
  std::uint64_t tick = 0, refresh = 0;
  SystemGraphDocument document = BuildSystemGraph(Registry(kNodes, 0, 0));

  ClientFeeds feeds;
  EgressBudget budget;
  WakeSchedule schedule;
  schedule.check_due_ns = kNeverNs;  // no segment to check
  Mirror mirror;
  std::string held_values;
  std::int64_t now = 0, last_text = -1, last_page = -1;
  std::int64_t longest_silence = 0, longest_quiet_gap = 0;
  std::size_t passes = 0;
  // One Bridge::service pass: the refresh when due, then the client's pump,
  // a budget deferral recorded in the schedule as Bridge::affordable does.
  const auto pass = [&] {
    ++passes;
    schedule.BeginPass();
    if (schedule.RefreshDue(now)) {
      schedule.refreshed_ns = now;
      if (now <= kRefreshesEndNs) {
        document = BuildSystemGraph(Registry(kNodes, ++refresh, 0));
      }
    }
    feeds.Pump(
        document, state, now, kRate,
        [&](std::size_t bytes) {
          if (budget.available(bytes, kRate, now)) return true;
          schedule.Deferred(now, budget.wait_ns(bytes, kRate));
          return false;
        },
        [&](std::string_view text) {
          budget.spend(EgressBudget::framed(text.size()));
          if (last_text >= 0) {
            longest_silence = std::max(longest_silence, now - last_text);
          }
          last_text = now;
          if (text == document.json) {
            held_values = document.values;
          } else if (text.starts_with(
                         "{\"kind\":\"talos.system_graph.update\"")) {
            held_values = Between(text, "\"values\":", "}");
          }
          return true;
        },
        [&](std::string_view page) {
          budget.spend(EgressBudget::framed(page.size()));
          mirror.Apply(page);
          // Once the producer has been quiet long enough for the client to
          // catch up, its updates are heartbeats.
          if (now >= kTicksEndNs + kStaleNs && last_page >= 0) {
            longest_quiet_gap = std::max(longest_quiet_gap, now - last_page);
          }
          last_page = now;
          return true;
        });
  };

  pass();  // the client connects
  while (now < kEndNs) {
    std::int64_t next = schedule.NextWakeNs(
        now, false, document, [&](const auto& visit) { visit(feeds); });
    const auto next_tick = static_cast<std::int64_t>(tick + 1) * kTickNs;
    const bool published = next_tick <= kTicksEndNs && next_tick <= next;
    if (published) next = next_tick;  // the publication wakes the bridge
    now = next;
    if (published) {
      ++tick;
      ASSERT_TRUE(IngestAll(
          state, TickPages(kChannels, tick, channels, tick_values(tick))));
    }
    pass();
  }
  EXPECT_LT(longest_silence, kStaleNs);
  EXPECT_LE(longest_quiet_gap, ClientFeed::kHeartbeatNs);
  EXPECT_EQ(held_values, document.values);
  EXPECT_TRUE(mirror.Equals(state));
  EXPECT_EQ(mirror.sequence_id, tick);
  // Not a poll: about one pass per tick, refresh and due message, against
  // the 12000 a 1 ms timer would make.
  EXPECT_LT(passes, 1500u) << "passes over " << kEndNs / 1'000'000 << " ms";
}

// REQ-RT-21
// An update the egress budget holds back is due again the instant the
// budget affords it, not at the bridge's next 250 ms refresh: the schedule
// wakes the bridge then, and the held update goes out on that pass.
TEST(WakeSchedule, ASendHeldByTheBudgetIsDueWhenTheBudgetAffordsIt) {
  constexpr double kRate = 4000;
  const TelemetryState state = SmallState();
  const SystemGraphDocument no_graph;
  ClientFeeds feeds;
  EgressBudget budget;
  ASSERT_TRUE(budget.available(0, kRate, 0));
  budget.spend(static_cast<std::size_t>(EgressBudget::kBurstBytes));  // dry
  WakeSchedule schedule;
  schedule.check_due_ns = kNeverNs;
  std::size_t pages = 0;
  const auto pass = [&](std::int64_t now) {
    schedule.BeginPass();
    feeds.Pump(
        no_graph, state, now, kRate,
        [&](std::size_t bytes) {
          if (budget.available(bytes, kRate, now)) return true;
          schedule.Deferred(now, budget.wait_ns(bytes, kRate));
          return false;
        },
        [](std::string_view) { return true; },
        [&](std::string_view page) {
          budget.spend(EgressBudget::framed(page.size()));
          ++pages;
          return true;
        });
    return schedule.NextWakeNs(now, false, no_graph,
                               [&](const auto& visit) { visit(feeds); });
  };

  const std::int64_t due = pass(0);
  ASSERT_EQ(pages, 0u) << "the dry budget sent a page";
  EXPECT_LT(due, WakeSchedule::kSystemRefreshNs)
      << "a held send waits for the next refresh";
  pass(due - 1);
  EXPECT_EQ(pages, 0u) << "affordable before the wake the schedule set, "
                       << due;
  pass(due);
  EXPECT_GE(pages, 1u) << "not sent at the wake the schedule set, " << due;
}

// The largest graph a client can be sent is the one whose framed size is the
// burst. One byte larger it can never be afforded, so it must never be
// treated as waiting for budget either: that would hold the client's
// telemetry behind a graph that never goes.
TEST(ClientFeeds, GraphAtTheBurstGoesAndOneByteLargerNeverHoldsTelemetry) {
  for (const std::size_t size :
       {EgressBudget::kMaxPayloadBytes, EgressBudget::kMaxPayloadBytes + 1}) {
    SCOPED_TRACE(size);
    const SystemGraphDocument document{
        .json = std::string(size, ' '), .structure = "s", .values = "[]"};
    const TelemetryState state = SmallState();
    ClientFeeds feeds;
    EgressBudget budget;
    std::size_t documents = 0;
    std::int64_t last_page = 0, longest_gap = 0;
    for (std::int64_t now = 0; now <= 3'000'000'000; now += 1'000'000) {
      feeds.Pump(
          document, state, now, 40000,
          [&](std::size_t bytes) {
            return budget.available(bytes, 40000, now);
          },
          [&](std::string_view text) {
            budget.spend(EgressBudget::framed(text.size()));
            documents += text == document.json ? 1 : 0;
            return true;
          },
          [&](std::string_view page) {
            budget.spend(EgressBudget::framed(page.size()));
            longest_gap = std::max(longest_gap, now - last_page);
            last_page = now;
            return true;
          });
    }
    EXPECT_EQ(documents, size == EgressBudget::kMaxPayloadBytes ? 1u : 0u);
    if (size > EgressBudget::kMaxPayloadBytes) {
      EXPECT_LE(longest_gap, ClientFeed::kHeartbeatNs);
    }
  }
}

// A node restarting every second changes the graph's structure every second,
// and each change needs the whole graph, about the client's whole share for a
// second. Whole graphs go at most every GraphFeed::kResendNs and telemetry is
// held at most ClientFeeds::kHoldNs at a time, so neither the client's
// readings nor its graph go stale in Studio (one second each) during the
// crash loop, and the graph converges once it stops.
TEST(ClientFeeds, CrashLoopKeepsTelemetryAndGraphFresh) {
  constexpr std::uint32_t kChannels = 200;
  std::vector<ChannelDef> dictionary;
  std::vector<std::uint32_t> channels;
  for (std::uint32_t i = 0; i < kChannels; ++i) {
    dictionary.push_back({"test/state/channel_" + std::to_string(i),
                          "",
                          {},
                          Talos::Telemetry::ChannelKind_Double});
    channels.push_back(i);
  }
  // A third of the channels change every tick: about 80 kB/s of demand,
  // twice the default 4-client share.
  const auto tick_values = [&](std::uint64_t tick) {
    std::vector<std::uint64_t> values;
    for (std::uint32_t i = 0; i < kChannels; ++i) {
      values.push_back(
          RawFromDouble(static_cast<double>(i % 3 == 0 ? tick * 1000 + i : i)));
    }
    return values;
  };
  constexpr double kRate = 40000;
  constexpr std::int64_t kStaleNs = 1'000'000'000;  // both Studio limits
  TelemetryState state;
  ASSERT_TRUE(IngestAll(state, DictionaryPages(dictionary)));
  std::uint64_t refresh = 0, restarts = 0;
  SystemGraphDocument document = BuildSystemGraph(Registry(15, 0, 0));
  ASSERT_GT(document.json.size(), 38000u);

  ClientFeeds feeds;
  EgressBudget budget;
  Mirror mirror;
  std::string held_structure, held_values;
  std::size_t documents = 0;
  std::int64_t now = 0, last_text = 0, last_page = 0;
  std::int64_t text_gap = 0, page_gap = 0;
  const auto pump = [&] {
    feeds.Pump(
        document, state, now, kRate,
        [&](std::size_t bytes) { return budget.available(bytes, kRate, now); },
        [&](std::string_view text) {
          budget.spend(EgressBudget::framed(text.size()));
          text_gap = std::max(text_gap, now - last_text);
          last_text = now;
          if (text == document.json) {
            ++documents;
            held_structure = document.structure;
            held_values = document.values;
          } else if (text.starts_with(
                         "{\"kind\":\"talos.system_graph.update\"")) {
            EXPECT_EQ(Between(text, "\"structure\":\"", "\""), held_structure);
            held_values = Between(text, "\"values\":", "}");
          }
          return true;
        },
        [&](std::string_view page) {
          budget.spend(EgressBudget::framed(page.size()));
          page_gap = std::max(page_gap, now - last_page);
          last_page = now;
          mirror.Apply(page);
          return true;
        });
  };

  for (std::uint64_t tick = 1; tick <= 1200; ++tick) {  // 12 s at 100 Hz
    ASSERT_TRUE(IngestAll(
        state, TickPages(kChannels, tick, channels, tick_values(tick))));
    if (tick % 25 == 0) {
      // A refresh every 250 ms; from 1 s to 10 s every node restarts each
      // second.
      if (tick % 100 == 0 && tick >= 100 && tick <= 1000) ++restarts;
      document = BuildSystemGraph(Registry(15, ++refresh, restarts));
    }
    for (int ms = 0; ms < 10; ++ms, now += 1'000'000) pump();
  }
  EXPECT_LT(page_gap, kStaleNs);
  EXPECT_LT(text_gap, kStaleNs);
  EXPECT_GE(documents, 3u);
  EXPECT_LE(documents, 7u);  // at most one per kResendNs, not one a second
  // Spared a whole graph a second, telemetry keeps over 25 updates/s (235 in
  // all when every restart resends the whole graph).
  EXPECT_GT(mirror.updates, 300u);

  const std::int64_t stopped = now;
  while (now - stopped < 5'000'000'000 &&
         (held_values != document.values || !feeds.telemetry.current(state))) {
    now += 1'000'000;
    pump();
  }
  EXPECT_EQ(held_structure, document.structure);
  EXPECT_EQ(held_values, document.values);
  EXPECT_TRUE(mirror.Equals(state));
}

// One telemetry update costs more than a hold window's refill (here 850
// changing channels, about 10.8 kB, against the 10 kB a 40 kB/s share refills
// in ClientFeeds::kHoldNs) but less than the waiting whole graph: telemetry
// alone could still afford one update per window and drain the bucket every
// time, so the graph never went and the client kept the old structure
// forever. The graph reserves its share of the refill instead, so it goes
// within the derived bound, while telemetry keeps flowing beside it and every
// update the client completes is still one whole producer tick.
TEST(ClientFeeds, GraphGoesWithinItsBoundWhenAnUpdateOutcostsTheHoldWindow) {
  constexpr std::uint32_t kChannels = 850;
  std::vector<ChannelDef> dictionary;
  std::vector<std::uint32_t> channels;
  for (std::uint32_t i = 0; i < kChannels; ++i) {
    dictionary.push_back({"test/state/channel_" + std::to_string(i),
                          "",
                          {},
                          Talos::Telemetry::ChannelKind_Double});
    channels.push_back(i);
  }
  const auto tick_values = [&](std::uint64_t tick) {
    std::vector<std::uint64_t> values;
    for (std::uint32_t i = 0; i < kChannels; ++i) {
      values.push_back(RawFromDouble(static_cast<double>(tick * 1000 + i)));
    }
    return values;
  };
  constexpr double kRate = 40000;  // the default 4-client share
  const talos::telemetry::PageBudget pages =
      talos::telemetry::MeasurePages(dictionary);
  const double update_bytes = static_cast<double>(
      pages.value_bytes + pages.value_pages * EgressBudget::kMaxHeaderBytes);
  constexpr std::size_t kGraphBytes = 40000;
  ASSERT_GE(update_bytes, kRate * 1e-9 * ClientFeeds::kHoldNs);
  ASSERT_LT(update_bytes, static_cast<double>(kGraphBytes));

  // The client holds a small graph; at 3 s a node restart replaces it with a
  // whole 40 kB one while telemetry has the bucket drained.
  const SystemGraphDocument small{.json = std::string(1000, ' '),
                                  .structure = "0123456789abcdef",
                                  .values = "[]"};
  const SystemGraphDocument large{.json = std::string(kGraphBytes, ' '),
                                  .structure = "fedcba9876543210",
                                  .values = "[]"};
  // Bound: the graph reserves kGraphShare of the refill from the moment it
  // waits, less the deferral notices it sends meanwhile.
  const double notice_bytes = static_cast<double>(EgressBudget::framed(
      std::string_view("{\"kind\":\"talos.system_graph.deferred\","
                       "\"structure\":\"0123456789abcdef\"}")
          .size()));
  const double bound_s =
      (static_cast<double>(EgressBudget::framed(kGraphBytes)) + notice_bytes) /
      (ClientFeeds::kGraphShare * kRate -
       notice_bytes / (GraphFeed::kNoticeNs * 1e-9));
  const auto bound_ns = static_cast<std::int64_t>(bound_s * 1e9) + 1'000'000;
  ASSERT_LT(bound_ns, 2'100'000'000);

  TelemetryState state;
  ASSERT_TRUE(IngestAll(state, DictionaryPages(dictionary)));
  const SystemGraphDocument* document = &small;
  ClientFeeds feeds;
  EgressBudget budget;
  Mirror mirror;
  std::size_t coherent_updates = 0;
  std::size_t updates_while_waiting = 0, updates_after = 0;
  std::int64_t now = 0, large_sent = -1;
  constexpr std::int64_t kChange = 3'000'000'000;
  const auto pump = [&] {
    feeds.Pump(
        *document, state, now, kRate,
        [&](std::size_t bytes) { return budget.available(bytes, kRate, now); },
        [&](std::string_view text) {
          budget.spend(EgressBudget::framed(text.size()));
          if (text == large.json) large_sent = now;
          return true;
        },
        [&](std::string_view page) {
          budget.spend(EgressBudget::framed(page.size()));
          const std::size_t updates = mirror.updates;
          mirror.Apply(page);
          if (mirror.updates != updates) {
            if (mirror.values == tick_values(mirror.sequence_id)) {
              ++coherent_updates;
            }
            if (large_sent >= 0) {
              ++updates_after;
            } else if (now >= kChange) {
              ++updates_while_waiting;
            }
          }
          return true;
        });
  };

  for (std::uint64_t tick = 1; now < kChange + 2 * bound_ns; ++tick) {
    if (now >= kChange) document = &large;
    ASSERT_TRUE(IngestAll(
        state, TickPages(kChannels, tick, channels, tick_values(tick))));
    for (int ms = 0; ms < 10; ++ms, now += 1'000'000) pump();
  }
  ASSERT_GE(large_sent, kChange);
  EXPECT_LE(large_sent - kChange, bound_ns);
  EXPECT_GT(updates_while_waiting, 0u);  // thinned, not stopped
  EXPECT_GT(updates_after, 0u);
  EXPECT_EQ(coherent_updates, mirror.updates);
}

TEST(ChannelsJson, DescribesTheDictionaryOnItsOwn) {
  const TelemetryState state = SmallState();
  const std::string json = ChannelsJson(state);
  EXPECT_NE(json.find("\"kind\":\"talos.telemetry.channels\""),
            std::string::npos);
  EXPECT_NE(json.find("\"version\":3"), std::string::npos);
  EXPECT_NE(json.find("\"name\":\"drive/velocity\",\"unit\":\"m/s\","
                      "\"kind\":\"double\",\"labels\":[]"),
            std::string::npos);
  EXPECT_NE(json.find("\"kind\":\"uint\",\"labels\":[\"DRIVE\","
                      "\"TARGET_STALE\"]"),
            std::string::npos);
}

// REQ-OPS-37
// Each channel carries its alert level, so Studio and the driver dashboard
// read levels from /channels.json without decoding dictionary pages or
// parsing field names.
TEST(ChannelsJson, CarriesEachChannelsAlertLevel) {
  std::vector<ChannelDef> dictionary = SmallDictionary();
  const std::vector<std::pair<std::string, AlertLevel>> alerts{
      {"hw/status/alert_info_ready", Talos::Telemetry::AlertLevel_Info},
      {"hw/status/alert_warning_brownout",
       Talos::Telemetry::AlertLevel_Warning},
      {"hw/status/alert_error_link_lost", Talos::Telemetry::AlertLevel_Error}};
  dictionary.reserve(dictionary.size() + alerts.size());
  for (const auto& [name, level] : alerts) {
    dictionary.push_back(
        {name, "", {}, Talos::Telemetry::ChannelKind_Bool, level});
  }
  TelemetryState state;
  ASSERT_TRUE(IngestAll(state, DictionaryPages(dictionary)));
  const std::string json = ChannelsJson(state);
  EXPECT_NE(json.find("\"name\":\"drive/velocity\",\"unit\":\"m/s\","
                      "\"kind\":\"double\",\"labels\":[],\"alert\":\"none\"}"),
            std::string::npos)
      << json;
  for (const auto& [name, spelled] :
       std::vector<std::pair<std::string, std::string>>{
           {"hw/status/alert_info_ready", "info"},
           {"hw/status/alert_warning_brownout", "warning"},
           {"hw/status/alert_error_link_lost", "error"}}) {
    EXPECT_NE(json.find("\"name\":\"" + name +
                        "\",\"unit\":\"\",\"kind\":\"bool\",\"labels\":[],"
                        "\"alert\":\"" +
                        spelled + "\"}"),
              std::string::npos)
        << name << " in " << json;
  }
}

TEST(TelemetryJson, RejectsBytesThatAreNotV3Pages) {
  TelemetryState state;
  EXPECT_FALSE(IngestTelemetry(state, "not a frame", 0));
  EXPECT_FALSE(state.has_dictionary);
  EXPECT_FALSE(state.has_frame);
}

// REQ-OPS-23
TEST(ClientSendBuffer, HoldsAQuarterSecondOfTheShareAboveAFloor) {
  // max_clients = 1: the whole budget, so a 250 ms round trip still
  // carries the client's share.
  EXPECT_EQ(ClientSendBuffer::Bytes(160000), 40000);
  EXPECT_EQ(ClientSendBuffer::Bytes(80000), 20000);
  // At and below 65,536 B/s (max_clients >= 3) the floor holds.
  EXPECT_EQ(ClientSendBuffer::Bytes(65536), 16 * 1024);
  EXPECT_EQ(ClientSendBuffer::Bytes(40000), 16 * 1024);
  EXPECT_EQ(ClientSendBuffer::Bytes(2500), 16 * 1024);
}

// Clients that each take one send when the bucket holds a token, in the
// order the bridge offered them.
struct Grants {
  int tokens = 0;
  std::string order;
  bool operator()(char client) {
    if (tokens == 0) return false;
    --tokens;
    order.push_back(client);
    return true;
  }
};

// REQ-OPS-23
TEST(ClientRotation, ADrainedClientWaitsForItsTurn) {
  ClientRotation<char> rotation;
  for (const char client : {'a', 'b', 'c'}) rotation.Add(client);
  Grants grants;
  for (int pass = 0; pass < 4; ++pass) {
    grants.tokens = 1;
    rotation.Pass(grants);
  }
  // `a` was just sent to, its output was partly buffered, and it drained
  // with one send's budget in the bucket: `b` is owed that send.
  grants.tokens = 1;
  rotation.Drained('a', grants);
  EXPECT_EQ(grants.order, "abcab");
}

// REQ-OPS-23
TEST(ClientRotation, ADisconnectDoesNotSkipTheClientOwedTheTurn) {
  ClientRotation<char> rotation;
  for (const char client : {'a', 'b', 'c'}) rotation.Add(client);
  Grants grants;
  for (int pass = 0; pass < 2; ++pass) {
    grants.tokens = 1;
    rotation.Pass(grants);
  }
  // `c` is owed the next send when `b`, ahead of it, disconnects.
  rotation.Remove('b');
  grants.tokens = 1;
  rotation.Pass(grants);
  EXPECT_EQ(grants.order, "abc");
}

}  // namespace
}  // namespace studio
