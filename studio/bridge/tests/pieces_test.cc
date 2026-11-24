// REQ-DEV-43: the bridge's game pieces feed (pieces.h). A programmer client
// is sent a keyframe of every piece, then only the pieces whose position,
// in whole millimetres, or in-play flag changed; one the budget holds back
// gets one later delta carrying the latest values, never a backlog; and the
// pieces wait while the client's graph or telemetry has a message waiting.

#include "studio/bridge/pieces.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "sim/sim_generated.h"
#include "studio/bridge/drive.h"
#include "studio/bridge/system.h"
#include "studio/bridge/telemetry.h"
#include "talOS/telemetry/frames.h"

namespace studio {
namespace {

// One message as Studio reads it (studio/src/data/pieces.ts), from the
// layout README.md states.
struct Decoded {
  bool keyframe{false};
  std::uint32_t count{0};
  float radius_m{0};
  std::uint64_t sample_time_ns{0};
  struct Record {
    bool in_play{false};
    std::int32_t x_mm{0}, y_mm{0}, z_mm{0};
    bool operator==(const Record&) const = default;
  };
  std::map<std::uint16_t, Record> records;
};

template <typename T>
T Read(std::string_view bytes, std::size_t at) {
  T value{};
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    value = static_cast<T>(
        value | static_cast<T>(static_cast<std::uint64_t>(
                                   static_cast<unsigned char>(bytes[at + i]))
                               << (8 * i)));
  }
  return value;
}

Decoded Decode(std::string_view bytes) {
  Decoded out;
  EXPECT_EQ(bytes.substr(0, 4), "PCS1");
  EXPECT_EQ(static_cast<unsigned char>(bytes[4]) & ~kPiecesKeyframe, 0u);
  EXPECT_EQ(static_cast<unsigned char>(bytes[5]), 0u);
  out.keyframe = (bytes[4] & 1) != 0;
  const auto records = Read<std::uint16_t>(bytes, 6);
  out.count = Read<std::uint32_t>(bytes, 8);
  out.radius_m = std::bit_cast<float>(Read<std::uint32_t>(bytes, 12));
  out.sample_time_ns = Read<std::uint64_t>(bytes, 16);
  EXPECT_EQ(bytes.size(), 24u + 16u * records);
  for (std::size_t r = 0; r < records; ++r) {
    const std::size_t at = 24 + 16 * r;
    EXPECT_EQ(static_cast<unsigned char>(bytes[at + 2]) & ~kPieceInPlay, 0u);
    EXPECT_EQ(static_cast<unsigned char>(bytes[at + 3]), 0u);
    out.records[Read<std::uint16_t>(bytes, at)] = {
        .in_play = (bytes[at + 2] & 1) != 0,
        .x_mm = static_cast<std::int32_t>(Read<std::uint32_t>(bytes, at + 4)),
        .y_mm = static_cast<std::int32_t>(Read<std::uint32_t>(bytes, at + 8)),
        .z_mm = static_cast<std::int32_t>(Read<std::uint32_t>(bytes, at + 12))};
  }
  return out;
}

// A message of `count` pieces: piece i at (i, -2 i, 0.075) m, in play.
::sim::GamePieces Message(std::uint32_t count, std::uint64_t time_ns) {
  ::sim::GamePieces message{};
  message.mutate_sample_time_ns(time_ns);
  message.mutate_pieces_count(count);
  message.mutate_radius_m(0.075F);
  for (std::uint32_t i = 0; i < count; ++i) {
    const auto at = static_cast<flatbuffers::uoffset_t>(i);
    message.mutable_x_m()->Mutate(at, static_cast<float>(i));
    message.mutable_y_m()->Mutate(at, -2.0F * static_cast<float>(i));
    message.mutable_z_m()->Mutate(at, 0.075F);
    message.mutable_in_play()->Mutate(at, true);
  }
  return message;
}

// For the tests of the message alone: a rate whose allowance covers any
// message a second later, and that second.
constexpr double kAmpleRate = 1e9;
std::int64_t clock_ns = 0;
std::int64_t NextSecond() { return clock_ns += 1'000'000'000; }

void Move(::sim::GamePieces& message, std::uint32_t piece, float dx_m) {
  const auto at = static_cast<flatbuffers::uoffset_t>(piece);
  message.mutable_x_m()->Mutate(at, message.x_m()->Get(at) + dx_m);
}

// REQ-DEV-43
TEST(PiecesFeed, KeyframeThenOnlyChangedPiecesInMillimetres) {
  PiecesState state;
  ::sim::GamePieces message = Message(3, 1000);
  message.mutable_x_m()->Mutate(1, 1.2344F);  // 1234.4 mm: 1234
  ASSERT_TRUE(IngestPieces(state, message));
  PiecesFeed feed;
  std::vector<std::string> sent;
  const auto pump = [&] {
    return feed.Pump(
        state, NextSecond(), kAmpleRate, [](std::size_t) { return true; },
        [&](std::string_view bytes) {
          sent.emplace_back(bytes);
          return true;
        });
  };

  ASSERT_TRUE(pump());
  const Decoded key = Decode(sent.back());
  EXPECT_TRUE(key.keyframe);
  EXPECT_EQ(key.count, 3u);
  EXPECT_EQ(key.radius_m, 0.075F);
  EXPECT_EQ(key.sample_time_ns, 1000u);
  ASSERT_EQ(key.records.size(), 3u);
  EXPECT_EQ(key.records.at(1), (Decoded::Record{true, 1234, -2000, 75}));
  EXPECT_EQ(key.records.at(2), (Decoded::Record{true, 2000, -4000, 75}));
  EXPECT_FALSE(pump()) << "the client holds the latest message";

  // 0.4 mm moves nothing a millimetre shows; 0.6 mm and a piece leaving
  // play do.
  message.mutate_sample_time_ns(2000);
  Move(message, 0, 0.0004F);
  Move(message, 2, 0.0006F);
  message.mutable_in_play()->Mutate(1, false);
  ASSERT_TRUE(IngestPieces(state, message));
  ASSERT_TRUE(pump());
  const Decoded delta = Decode(sent.back());
  EXPECT_FALSE(delta.keyframe);
  EXPECT_EQ(delta.sample_time_ns, 2000u);
  ASSERT_EQ(delta.records.size(), 2u);
  EXPECT_EQ(delta.records.at(1), (Decoded::Record{false, 1234, -2000, 75}));
  EXPECT_EQ(delta.records.at(2), (Decoded::Record{true, 2001, -4000, 75}));

  // A message with nothing a millimetre shows sends nothing.
  Move(message, 0, 0.0001F);
  ASSERT_TRUE(IngestPieces(state, message));
  EXPECT_FALSE(pump());
  EXPECT_EQ(sent.size(), 2u);

  // The field changed (another count): a keyframe.
  ASSERT_TRUE(IngestPieces(state, Message(2, 3000)));
  ASSERT_TRUE(pump());
  EXPECT_TRUE(Decode(sent.back()).keyframe);
  EXPECT_EQ(Decode(sent.back()).records.size(), 2u);

  // A message Studio cannot draw leaves the latest as it was.
  ::sim::GamePieces bad = Message(2, 4000);
  bad.mutable_y_m()->Mutate(0, std::numeric_limits<float>::infinity());
  EXPECT_FALSE(IngestPieces(state, bad));
  EXPECT_EQ(state.sample_time_ns, 3000u);
}

// REQ-DEV-43: a client the budget holds back while pieces move is sent one
// later message with every piece that changed since it was last sent one,
// at its latest position, whatever happened in between; its bytes are that
// message's records, at most a whole keyframe of 640.
TEST(PiecesFeed, HeldBackClientGetsOneLaterDeltaNotABacklog) {
  PiecesState state;
  ::sim::GamePieces message = Message(640, 0);
  ASSERT_TRUE(IngestPieces(state, message));
  PiecesFeed feed;
  bool affordable = true;
  std::vector<std::string> sent;
  std::size_t asked = 0;
  const auto pump = [&] {
    return feed.Pump(
        state, NextSecond(), kAmpleRate,
        [&](std::size_t framed) {
          asked = framed;
          return affordable;
        },
        [&](std::string_view bytes) {
          sent.emplace_back(bytes);
          return true;
        });
  };
  ASSERT_TRUE(pump());
  EXPECT_EQ(sent.back().size(), kMaxPiecesMessageBytes);
  EXPECT_EQ(EgressBudget::framed(sent.back().size()), 10274u);

  // Twenty messages while the budget refuses: pieces 0 to 19, one each,
  // and piece 5 again by 10 cm.
  affordable = false;
  for (std::uint32_t i = 0; i < 20; ++i) {
    message.mutate_sample_time_ns(i + 1);
    Move(message, i, 0.01F);
    if (i == 10) Move(message, 5, 0.1F);
    ASSERT_TRUE(IngestPieces(state, message));
    EXPECT_FALSE(pump());
  }
  EXPECT_EQ(sent.size(), 1u);
  EXPECT_EQ(asked, EgressBudget::framed(24 + 20 * 16));

  affordable = true;
  ASSERT_TRUE(pump());
  ASSERT_EQ(sent.size(), 2u);
  const Decoded delta = Decode(sent.back());
  EXPECT_FALSE(delta.keyframe);
  EXPECT_EQ(delta.sample_time_ns, 20u);
  ASSERT_EQ(delta.records.size(), 20u);
  EXPECT_EQ(delta.records.at(5).x_mm, 5000 + 10 + 100);
  EXPECT_EQ(delta.records.at(19).x_mm, 19000 + 10);
  EXPECT_FALSE(pump());

  // A send the socket refused leaves the client's view unknown: a keyframe.
  message.mutate_sample_time_ns(99);
  Move(message, 0, 0.01F);
  ASSERT_TRUE(IngestPieces(state, message));
  EXPECT_FALSE(feed.Pump(
      state, NextSecond(), kAmpleRate, [](std::size_t) { return true; },
      [](std::string_view) { return false; }));
  ASSERT_TRUE(pump());
  EXPECT_TRUE(Decode(sent.back()).keyframe);
}

// REQ-DEV-43: each refusal leaves the latest message as it was, and a
// position rounds to the nearest millimetre either side of zero.
TEST(IngestPieces, RefusesWhatStudioCannotDrawAndRoundsToTheMillimetre) {
  PiecesState state;
  ::sim::GamePieces good = Message(2, 1);
  good.mutable_x_m()->Mutate(0, -0.0006F);
  ASSERT_TRUE(IngestPieces(state, good));
  EXPECT_EQ(state.mm[0], -1);
  const std::uint64_t generation = state.generation;

  ::sim::GamePieces many = Message(2, 2);
  many.mutate_pieces_count(kMaxPieces + 1);
  ::sim::GamePieces nan_radius = Message(2, 3);
  nan_radius.mutate_radius_m(std::numeric_limits<float>::quiet_NaN());
  ::sim::GamePieces negative_radius = Message(2, 4);
  negative_radius.mutate_radius_m(-0.01F);
  ::sim::GamePieces far = Message(2, 5);
  far.mutable_z_m()->Mutate(1, 2.2e6F);  // 2.2e9 mm, past an int32
  for (const ::sim::GamePieces* refused :
       {&many, &nan_radius, &negative_radius, &far}) {
    EXPECT_FALSE(IngestPieces(state, *refused)) << refused->sample_time_ns();
  }
  EXPECT_EQ(state.generation, generation);
  EXPECT_EQ(state.sample_time_ns, 1u);
}

// A dictionary of `count` channels as `id`, ingested; its pages' count.
std::size_t IngestDictionary(TelemetryState& state, std::uint32_t count,
                             std::uint32_t id) {
  std::vector<talos::telemetry::ChannelDef> dictionary;
  for (std::uint32_t i = 0; i < count; ++i) {
    dictionary.push_back({"test/state/c" + std::to_string(i),
                          "m",
                          {},
                          Talos::Telemetry::ChannelKind_Double});
  }
  std::vector<std::string> pages;
  flatbuffers::FlatBufferBuilder builder;
  talos::telemetry::EncodeDictionaryPages(
      builder, talos::telemetry::PageHeader{7, 0, 1, id, count}, dictionary,
      [&](std::span<const std::uint8_t> page) {
        EXPECT_TRUE(IngestTelemetry(
            state,
            std::string_view{reinterpret_cast<const char*>(page.data()),
                             page.size()},
            0));
        pages.emplace_back();
      });
  return pages.size();
}

// A tick of dictionary `id` in which all `count` channels hold `value`.
void IngestWideTick(TelemetryState& state, std::uint32_t count,
                    std::uint32_t id, std::uint64_t sequence, double value) {
  std::vector<std::uint32_t> channels;
  std::vector<std::uint64_t> values;
  for (std::uint32_t i = 0; i < count; ++i) {
    channels.push_back(i);
    values.push_back(talos::telemetry::RawFromDouble(value + i));
  }
  flatbuffers::FlatBufferBuilder builder;
  const Talos::Telemetry::Pose chassis{1, 2, 0, 0, 0, 0};
  ASSERT_TRUE(talos::telemetry::EncodeValuePages(
      builder, talos::telemetry::PageHeader{7, sequence, sequence, id, count},
      chassis, true, channels, values, {},
      [&](std::span<const std::uint8_t> page,
          const talos::telemetry::PageSpan&) {
        ASSERT_TRUE(IngestTelemetry(
            state,
            std::string_view{reinterpret_cast<const char*>(page.data()),
                             page.size()},
            0));
      }));
}

// A programmer client's budget as the bridge charges it
// (AffordProgrammerSend): its own bucket at `rate` and the shared one at
// four times it, the room for an owed pieces message asked of its own.
struct ClientBudget {
  explicit ClientBudget(double rate) : egress(Plan(rate)) {}
  static EgressPlan Plan(double rate) {
    EgressPlan plan;
    plan.programmer_bytes_per_s = 4 * rate;
    plan.programmer_client_bytes_per_s = rate;
    return plan;
  }
  bool Afford(std::size_t bytes, std::size_t room, ClientFeeds::Feed feed,
              std::int64_t now) {
    return AffordProgrammerSend(reservation, egress, schedule, 1, own, bytes,
                                room, feed, now, false, held);
  }
  void Spend(std::size_t framed) { egress.Spend(own, false, framed); }

  SharedReservation<int> reservation;
  BridgeEgress egress;
  WakeSchedule schedule;
  EgressBudget own;
  std::uint64_t held{0};
};

// One programmer client of rate `rate`, sent everything due every 5 ms as
// the bridge's pass does (PiecesRoom, ClientFeeds::Pump, PumpPieces), its
// own bucket the only one. Telemetry saturates it: a tick of 150 changing
// channels every 5 ms (about 380 kB/s wanted). The 504 pieces are published
// every 50 ms, the first `moving` of them 1 cm further each time until
// `stop_ns`.
struct Scenario {
  double rate{28000};
  std::uint32_t moving{10};
  std::int64_t stop_ns{std::numeric_limits<std::int64_t>::max()};
  std::int64_t end_ns{5'000'000'000};
  std::int64_t dictionary_at_ns{-1};  // a new dictionary arrives
  std::int64_t graph_at_ns{-1};       // the graph's values change
};

struct Outcome {
  std::vector<std::int64_t> pieces_ns;  // each pieces message's send
  std::size_t pieces_bytes{0}, telemetry_bytes{0};
  std::int64_t dictionary_ns{-1};  // its first page sent
  std::int64_t graph_ns{-1};       // the graph update sent
  std::int64_t converged_ns{-1};   // after stop_ns, the client holds them
};

Outcome Simulate(const Scenario& scenario) {
  constexpr std::uint32_t kChannels = 150;
  constexpr std::uint32_t kPieces = 504;
  TelemetryState telemetry;
  std::uint32_t dictionary = 1;
  EXPECT_EQ(IngestDictionary(telemetry, kChannels, dictionary), 1u);
  SystemGraphDocument document = BuildSystemGraph(SystemGraphInput{});
  ClientFeeds feeds;
  PiecesFeed feed;
  PiecesState pieces;
  ::sim::GamePieces message = Message(kPieces, 0);
  ClientBudget budget(scenario.rate);
  Outcome out;
  std::map<std::uint16_t, Decoded::Record> mirror;
  std::uint64_t sequence = 0;
  std::int64_t now = 0;
  const auto afford = [&](std::size_t bytes) {
    return budget.Afford(bytes, 0, ClientFeeds::Feed::kPieces, now);
  };
  for (now = 0; now <= scenario.end_ns; now += 5'000'000) {
    if (now == scenario.dictionary_at_ns) {
      dictionary = 2;
      EXPECT_EQ(IngestDictionary(telemetry, kChannels, dictionary), 1u);
    }
    if (now == scenario.graph_at_ns) {
      document = BuildSystemGraph(SystemGraphInput{.wall_ns = 5'000'000'000});
    }
    ++sequence;
    IngestWideTick(telemetry, kChannels, dictionary, sequence,
                   static_cast<double>(sequence));
    if (now % 50'000'000 == 0) {
      if (now < scenario.stop_ns) {
        for (std::uint32_t i = 0; i < scenario.moving; ++i) {
          Move(message, i, 0.01F);
        }
      }
      message.mutate_sample_time_ns(static_cast<std::uint64_t>(now));
      EXPECT_TRUE(IngestPieces(pieces, message));
    }
    const std::size_t room = PiecesRoom(feeds, telemetry, feed, pieces);
    feeds.Pump(
        document, telemetry, now, scenario.rate,
        [&](std::size_t bytes, ClientFeeds::Feed feed, std::size_t room) {
          return budget.Afford(bytes, room, feed, now);
        },
        [&](std::string_view text) {
          budget.Spend(EgressBudget::framed(text.size()));
          if (scenario.graph_at_ns >= 0 && now >= scenario.graph_at_ns &&
              out.graph_ns < 0) {
            out.graph_ns = now;
          }
          return true;
        },
        [&](std::string_view page) {
          budget.Spend(EgressBudget::framed(page.size()));
          out.telemetry_bytes += EgressBudget::framed(page.size());
          const auto* frame =
              Talos::Telemetry::GetSizePrefixedFrame(page.data());
          if (scenario.dictionary_at_ns >= 0 &&
              now >= scenario.dictionary_at_ns && out.dictionary_ns < 0 &&
              frame->dictionary() != nullptr) {
            out.dictionary_ns = now;
          }
          return true;
        },
        false, room);
    PumpPieces(feeds, telemetry, feed, pieces, now, scenario.rate, afford,
               [&](std::string_view bytes) {
                 budget.Spend(EgressBudget::framed(bytes.size()));
                 out.pieces_bytes += EgressBudget::framed(bytes.size());
                 out.pieces_ns.push_back(now);
                 const Decoded decoded = Decode(bytes);
                 if (decoded.keyframe) mirror.clear();
                 for (const auto& [index, record] : decoded.records) {
                   mirror[index] = record;
                 }
                 return true;
               });
    if (now >= scenario.stop_ns && out.converged_ns < 0) {
      bool holds = mirror.size() == kPieces;
      for (std::uint16_t i = 0; holds && i < kPieces; ++i) {
        const Decoded::Record& held = mirror.at(i);
        holds = held.x_mm == pieces.mm[3 * std::size_t{i}] &&
                held.y_mm == pieces.mm[3 * std::size_t{i} + 1] &&
                held.z_mm == pieces.mm[3 * std::size_t{i} + 2] &&
                held.in_play == (pieces.in_play[i] != 0);
      }
      if (holds) out.converged_ns = now;
    }
  }
  return out;
}

// REQ-DEV-43: with telemetry wanting far more than the client's share, ten
// moving pieces still reach it at 10 Hz or more in every second, and
// telemetry keeps at least 3/4 of the share.
TEST(PumpPieces, SaturatingTelemetryLeavesTenMovingPiecesTenHertz) {
  const Scenario scenario;
  const Outcome out = Simulate(scenario);
  for (std::int64_t second = 0; second < 5; ++second) {
    const auto sent = std::count_if(out.pieces_ns.begin(), out.pieces_ns.end(),
                                    [second](std::int64_t t) {
                                      return t >= second * 1'000'000'000 &&
                                             t < (second + 1) * 1'000'000'000;
                                    });
    EXPECT_GE(sent, 10) << "second " << second;
  }
  EXPECT_GE(static_cast<double>(out.telemetry_bytes),
            0.75 * scenario.rate * 5.0);
}

// REQ-DEV-43: with every piece moving, the pieces take at most a quarter of
// the share plus one allowance, still go (a 504-piece delta about once a
// second), and once motion stops the client holds the latest positions
// within 8,098 B / 7,000 B/s plus 50 ms.
TEST(PumpPieces, EveryPieceMovingTakesAQuarterAndConverges) {
  const Scenario scenario{.moving = 504, .stop_ns = 3'000'000'000};
  const Outcome out = Simulate(scenario);
  EXPECT_LE(static_cast<double>(out.pieces_bytes),
            kPiecesShare * scenario.rate * 5.0 +
                static_cast<double>(kPiecesAllowanceBytes));
  const auto before_stop =
      std::count_if(out.pieces_ns.begin(), out.pieces_ns.end(),
                    [](std::int64_t t) { return t < 3'000'000'000; });
  EXPECT_GE(before_stop, 3);
  ASSERT_GE(out.converged_ns, 0);
  EXPECT_LE(out.converged_ns - scenario.stop_ns, 1'210'000'000);
}

// REQ-DEV-43: a pending dictionary page and a pending graph message go
// before a pending pieces message: none goes from their arrival until
// they are sent.
TEST(PumpPieces, DictionaryAndGraphGoBeforePieces) {
  const Outcome out = Simulate(
      {.dictionary_at_ns = 1'000'000'000, .graph_at_ns = 2'500'000'000});
  ASSERT_GE(out.dictionary_ns, 0);
  ASSERT_GE(out.graph_ns, 0);
  for (const std::int64_t t : out.pieces_ns) {
    EXPECT_FALSE(t >= 1'000'000'000 && t < out.dictionary_ns) << t;
    EXPECT_FALSE(t >= 2'500'000'000 && t < out.graph_ns) << t;
  }
  // The dictionary page (5 kB) waited behind the saturated budget; the
  // small graph update goes first in its pass.
  EXPECT_GT(out.dictionary_ns, 1'000'000'000);
}

// REQ-DEV-43: a pieces message held back by the shared bucket never
// reserves it, however long it waits, so it never denies another client.
TEST(SharedReservation, PiecesNeverReserveTheSharedBucket) {
  SharedReservation<int> reservation;
  ClientRotation<int> rotation;
  rotation.Add(1);
  rotation.Add(2);
  EgressBudget own;
  for (std::int64_t now = 0; now <= 2'000'000'000; now += 100'000'000) {
    reservation.Pass(rotation, now, [&](int client) {
      if (client == 1) {
        reservation.BlockedOnShared(1, EgressBudget::framed(10'264),
                                    ClientFeeds::Feed::kPieces);
      }
      return false;
    });
    EXPECT_FALSE(reservation.reserved().has_value()) << now;
    EXPECT_TRUE(reservation.MaySpend(2, 1000, own, 28000, now)) << now;
  }
}

// REQ-DEV-43: a pass in which the pieces may not go (a dictionary the
// client has not been sent) asks for no wake, though a message waits on its
// allowance, so a long wait for telemetry never spins the bridge.
TEST(PumpPieces, NoWakeWhileThePiecesMayNotGo) {
  constexpr double kRate = 28000;
  PiecesState pieces;
  ::sim::GamePieces message = Message(504, 0);
  ASSERT_TRUE(IngestPieces(pieces, message));
  PiecesFeed feed;
  const auto yes = [](std::size_t) { return true; };
  const auto sent = [](std::string_view) { return true; };
  ASSERT_TRUE(feed.Pump(pieces, 0, kRate, yes, sent));  // the keyframe
  for (std::uint32_t i = 0; i < 504; ++i) Move(message, i, 0.01F);
  ASSERT_TRUE(IngestPieces(pieces, message));
  EXPECT_FALSE(feed.Pump(pieces, 5'000'000, kRate, yes, sent));
  ASSERT_NE(feed.wait_ns(), kNeverNs) << "the case needs an allowance short";

  TelemetryState telemetry;
  ASSERT_EQ(IngestDictionary(telemetry, 150, 1), 1u);
  const ClientFeeds feeds;  // has sent nothing: not Settled
  int wakes = 0;
  for (std::int64_t now = 10'000'000; now <= 2'000'000'000; now += 5'000'000) {
    EXPECT_FALSE(
        PumpPieces(feeds, telemetry, feed, pieces, now, kRate, yes, sent));
    wakes += feed.wait_ns() != kNeverNs ? 1 : 0;
  }
  EXPECT_EQ(wakes, 0);
}

// REQ-DEV-43: the room a telemetry update leaves an owed pieces message is
// asked of the client's own bucket only: an update short of only the shared
// bucket waits, and is reserved for, at its own size.
TEST(AffordProgrammerSend, RoomNeverReachesTheSharedReservation) {
  SharedReservation<int> reservation;
  ClientRotation<int> rotation;
  rotation.Add(1);
  rotation.Add(2);
  BridgeEgress egress(EgressPlan::Of(4, 0, 0));
  WakeSchedule schedule;
  std::uint64_t held = 0;
  EgressBudget own, other;
  std::int64_t wait = 0;
  // Another client has nearly emptied the shared bucket.
  ASSERT_TRUE(egress.Affordable(other, false, 63'000, 0, wait));
  egress.Spend(other, false, 63'000);
  constexpr std::size_t kUpdate = 2'000, kRoom = 8'098;
  std::size_t reserved = 0;
  const auto pump = [&](std::int64_t now) {
    reservation.Pass(rotation, now, [&](int client) {
      if (client != 1) return false;
      if (reservation.reserved() == 1) reserved = reservation.reserved_bytes();
      return AffordProgrammerSend(reservation, egress, schedule, 1, own,
                                  kUpdate, kRoom, ClientFeeds::Feed::kTelemetry,
                                  now, false, held);
    });
  };
  pump(0);
  // Past kSharedReserveDelayNs the wait becomes a reservation of its size.
  pump(kSharedReserveDelayNs + 1);
  EXPECT_EQ(reserved, kUpdate);
}

// REQ-DEV-43: one telemetry update that arrives while a pieces message is
// owed (its allowance short) is sent no later than that message's bytes / R
// after it would be with no pieces.
TEST(PumpPieces, AnUpdateWaitsAtMostOneMessageForThePieces) {
  constexpr double kRate = 28000;
  constexpr std::uint32_t kChannels = 150;
  const auto delivered = [&](bool pieces_owed) {
    TelemetryState telemetry;
    EXPECT_EQ(IngestDictionary(telemetry, kChannels, 1), 1u);
    IngestWideTick(telemetry, kChannels, 1, 1, 0.0);
    const SystemGraphDocument document = BuildSystemGraph(SystemGraphInput{});
    ClientFeeds feeds;
    PiecesFeed feed;
    PiecesState pieces;
    ClientBudget budget(kRate);
    std::int64_t now = 0;
    const auto afford = [&](std::size_t bytes) {
      return budget.Afford(bytes, 0, ClientFeeds::Feed::kPieces, now);
    };
    const auto afford_feeds = [&](std::size_t bytes, ClientFeeds::Feed feed,
                                  std::size_t room) {
      return budget.Afford(bytes, room, feed, now);
    };
    const auto spend = [&](std::string_view bytes) {
      budget.Spend(EgressBudget::framed(bytes.size()));
      return true;
    };
    feeds.Pump(document, telemetry, now, kRate, afford_feeds, spend, spend);
    EXPECT_TRUE(feeds.telemetry.current(telemetry));
    std::size_t owed = 0;
    if (pieces_owed) {
      ::sim::GamePieces message = Message(504, 0);
      EXPECT_TRUE(IngestPieces(pieces, message));
      EXPECT_TRUE(feed.Pump(pieces, now, kRate, afford, spend));  // keyframe
      for (std::uint32_t i = 0; i < 504; ++i) Move(message, i, 0.01F);
      EXPECT_TRUE(IngestPieces(pieces, message));
      owed = feed.Owed(pieces);
      EXPECT_EQ(owed, 8'098u);
    }
    budget.own.available(0, kRate, now);  // its own bucket, emptied
    budget.Spend(static_cast<std::size_t>(budget.own.tokens));
    IngestWideTick(telemetry, kChannels, 1, 2, 5.0);
    for (now = 5'000'000; now <= 2'000'000'000; now += 5'000'000) {
      const std::size_t room = PiecesRoom(feeds, telemetry, feed, pieces);
      feeds.Pump(document, telemetry, now, kRate, afford_feeds, spend, spend,
                 false, room);
      if (feeds.telemetry.current(telemetry)) return std::pair{now, owed};
      PumpPieces(feeds, telemetry, feed, pieces, now, kRate, afford, spend);
    }
    return std::pair{std::int64_t{-1}, owed};
  };
  const auto [alone, none] = delivered(false);
  const auto [beside, owed] = delivered(true);
  ASSERT_GT(alone, 0);
  ASSERT_GT(beside, 0);
  EXPECT_GT(beside, alone) << "the case needs the room to delay the update";
  EXPECT_LE(beside - alone,
            static_cast<std::int64_t>(static_cast<double>(owed) / kRate * 1e9) +
                5'000'000);
}

// REQ-DEV-43: -0.0F radius canonicalizes to +0.0F on the wire, non-finite and
// negative-overflow inputs are refused without mutating state, and a zero-piece
// message clears the client via a 24-byte keyframe with zeroed padding.
TEST(IngestPieces, CanonicalizesNegativeZeroRadiusAndRejectsNonFiniteEdges) {
  PiecesState state;
  ::sim::GamePieces neg_zero = Message(1, 10);
  neg_zero.mutate_radius_m(-0.0F);
  ASSERT_TRUE(IngestPieces(state, neg_zero));
  EXPECT_EQ(std::bit_cast<std::uint32_t>(state.radius_m), 0u);

  PiecesFeed feed;
  std::vector<std::string> sent;
  const auto pump = [&](std::int64_t now) {
    return feed.Pump(
        state, now, kAmpleRate, [](std::size_t) { return true; },
        [&](std::string_view bytes) {
          sent.emplace_back(bytes);
          return true;
        });
  };
  ASSERT_TRUE(pump(0));
  EXPECT_EQ(Read<std::uint32_t>(sent.back(), 12), 0u)
      << "wire radius must not carry -0.0F sign bit";
  const Decoded key = Decode(sent.back());
  EXPECT_TRUE(key.keyframe);
  EXPECT_EQ(key.count, 1u);

  const std::uint64_t generation = state.generation;
  ::sim::GamePieces inf_radius = Message(1, 11);
  inf_radius.mutate_radius_m(std::numeric_limits<float>::infinity());
  ::sim::GamePieces neg_inf_coord = Message(1, 12);
  neg_inf_coord.mutable_x_m()->Mutate(0,
                                      -std::numeric_limits<float>::infinity());
  ::sim::GamePieces nan_coord = Message(1, 13);
  nan_coord.mutable_y_m()->Mutate(0, std::numeric_limits<float>::quiet_NaN());
  ::sim::GamePieces neg_far = Message(1, 14);
  neg_far.mutable_z_m()->Mutate(0, -2.2e6F);
  for (const ::sim::GamePieces* refused :
       {&inf_radius, &neg_inf_coord, &nan_coord, &neg_far}) {
    EXPECT_FALSE(IngestPieces(state, *refused)) << refused->sample_time_ns();
  }
  EXPECT_EQ(state.generation, generation);
  EXPECT_EQ(state.sample_time_ns, 10u);

  ::sim::GamePieces empty = Message(0, 15);
  empty.mutate_radius_m(0.0F);
  ASSERT_TRUE(IngestPieces(state, empty));
  ASSERT_TRUE(pump(1'000'000'000));
  const Decoded cleared = Decode(sent.back());
  EXPECT_TRUE(cleared.keyframe);
  EXPECT_EQ(cleared.count, 0u);
  EXPECT_TRUE(cleared.records.empty());
  EXPECT_EQ(sent.back().size(), kPiecesHeaderBytes);
}

// REQ-DEV-43: PiecesFeed refuses out-of-range or non-finite PiecesState without
// out-of-bounds access, clears stale tail entries on shrink, and resends a
// keyframe after PiecesState is reset even if generation restarts at 1.
TEST(PiecesFeed, RejectsInvalidStateAndClearsStaleTailAndResetState) {
  PiecesFeed feed;
  std::vector<std::string> sent;
  const auto yes = [](std::size_t) { return true; };
  const auto record = [&](std::string_view bytes) {
    sent.emplace_back(bytes);
    return true;
  };

  const auto expect_refused = [&](const PiecesState& invalid) {
    EXPECT_EQ(feed.Owed(invalid), 0u);
    EXPECT_FALSE(feed.Pump(invalid, 0, kAmpleRate, yes, record));
    EXPECT_EQ(feed.wait_ns(), kNeverNs);
  };
  expect_refused(PiecesState{.has = true,
                             .count = kMaxPieces + 1,
                             .radius_m = 0.075F,
                             .generation = 1});
  expect_refused(
      PiecesState{.has = true,
                  .count = 1,
                  .radius_m = std::numeric_limits<float>::quiet_NaN(),
                  .generation = 2});
  expect_refused(PiecesState{.has = true,
                             .count = 1,
                             .radius_m = std::numeric_limits<float>::infinity(),
                             .generation = 3});
  expect_refused(PiecesState{
      .has = true, .count = 1, .radius_m = -0.01F, .generation = 4});
  EXPECT_TRUE(sent.empty());

  PiecesState state;
  ASSERT_TRUE(IngestPieces(state, Message(4, 100)));
  ASSERT_TRUE(feed.Pump(state, 0, kAmpleRate, yes, record));
  EXPECT_EQ(Decode(sent.back()).records.size(), 4u);

  ASSERT_TRUE(IngestPieces(state, Message(1, 200)));
  ASSERT_TRUE(feed.Pump(state, 1'000'000'000, kAmpleRate, yes, record));
  EXPECT_TRUE(Decode(sent.back()).keyframe);
  EXPECT_EQ(Decode(sent.back()).count, 1u);

  // Resetting PiecesState clears feed's cached generation so a fresh session
  // starting at generation 1 sends a new keyframe instead of aliasing.
  state = PiecesState{};
  EXPECT_EQ(feed.Owed(state), 0u);
  ASSERT_TRUE(IngestPieces(state, Message(2, 300)));
  ASSERT_EQ(state.generation, 1u);
  ASSERT_TRUE(feed.Pump(state, 2'000'000'000, kAmpleRate, yes, record));
  const Decoded fresh = Decode(sent.back());
  EXPECT_TRUE(fresh.keyframe);
  EXPECT_EQ(fresh.count, 2u);
  EXPECT_EQ(fresh.records.size(), 2u);
}

// REQ-DEV-43: non-positive, non-finite, or overflowing wait rates never divide
// by zero, corrupt the allowance, or overflow int64 wait deadlines.
TEST(PiecesFeed, NonPositiveNonFiniteAndTinyRatesNeverOverflowOrSpin) {
  PiecesState state;
  ::sim::GamePieces message = Message(640, 1);
  ASSERT_TRUE(IngestPieces(state, message));
  PiecesFeed feed;
  std::size_t sends = 0;
  const auto yes = [](std::size_t) { return true; };
  const auto count_send = [&](std::string_view) {
    ++sends;
    return true;
  };

  for (const double bad_rate :
       {0.0, -100.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(feed.Pump(state, 0, bad_rate, yes, count_send));
    EXPECT_EQ(feed.wait_ns(), kNeverNs);
  }
  EXPECT_EQ(sends, 0u);

  // Allowance stayed intact: a valid rate still sends the full keyframe.
  ASSERT_TRUE(feed.Pump(state, 0, 28000.0, yes, count_send));
  EXPECT_EQ(sends, 1u);

  // Deplete allowance (just spent 10,274 B) and request a tiny rate whose wait
  // exceeds INT64_MAX ns.
  Move(message, 0, 0.1F);
  ASSERT_TRUE(IngestPieces(state, message));
  EXPECT_FALSE(feed.Pump(state, 1'000, 1e-12, yes, count_send));
  EXPECT_EQ(feed.wait_ns(), kNeverNs);

  // A timestamp near kNeverNs where now_ns + wait_ns would overflow signed
  // int64 also reports kNeverNs instead of overflowing WakeSchedule::Deferred.
  PiecesFeed feed_near_max;
  PiecesState near_state;
  ::sim::GamePieces near_msg = Message(640, 1);
  ASSERT_TRUE(IngestPieces(near_state, near_msg));
  ASSERT_TRUE(
      feed_near_max.Pump(near_state, kNeverNs - 100, 28000.0, yes, count_send));
  Move(near_msg, 0, 0.1F);
  ASSERT_TRUE(IngestPieces(near_state, near_msg));
  EXPECT_FALSE(
      feed_near_max.Pump(near_state, kNeverNs - 50, 28000.0, yes, count_send));
  EXPECT_EQ(feed_near_max.wait_ns(), kNeverNs);
}

// REQ-DEV-43, REQ-OPS-35: AffordProgrammerSend refuses size_t overflow in
// framed_bytes + own_room, EgressPlan/BridgeEgress gate invalid or zero rates
// without UB, and duplicate DriveSlots::Admit preserves slot invariants.
TEST(AffordProgrammerSend, GuardsOverflowInvalidRatesAndDuplicateDriveSlots) {
  SharedReservation<int> reservation;
  ClientRotation<int> rotation;
  rotation.Add(1);
  BridgeEgress egress(EgressPlan::Of(4, 0, 0));
  WakeSchedule schedule;
  EgressBudget own;
  std::uint64_t held = 0;

  constexpr std::size_t kUpdate = 2'000;
  constexpr std::size_t kWrapRoom =
      std::numeric_limits<std::size_t>::max() - kUpdate + 1;
  schedule.BeginPass();
  EXPECT_FALSE(AffordProgrammerSend(
      reservation, egress, schedule, 1, own, kUpdate, kWrapRoom,
      ClientFeeds::Feed::kTelemetry, 0, false, held));
  EXPECT_EQ(schedule.retry_ns, kNeverNs);

  EXPECT_THROW((void)EgressPlan::Of(4, 1, 0), std::invalid_argument);

  // Default EgressPlan has 0 B/s drive rate; BridgeEgress must refuse with
  // wait_ns == kNeverNs instead of dividing by zero.
  BridgeEgress default_egress;
  EgressBudget drive_own;
  std::int64_t wait_ns = 0;
  EXPECT_FALSE(default_egress.Affordable(drive_own, true, 100, 0, wait_ns));
  EXPECT_EQ(wait_ns, kNeverNs);

  // Invalid programmer/shared rates never reserve the shared bucket.
  EgressPlan bad_plan;
  bad_plan.programmer_bytes_per_s = 0.0;
  bad_plan.programmer_client_bytes_per_s = 28000.0;
  BridgeEgress zero_shared(bad_plan);
  reservation.Pass(rotation, 0, [&](int client) {
    return AffordProgrammerSend(reservation, zero_shared, schedule, client, own,
                                100, 0, ClientFeeds::Feed::kTelemetry, 0, false,
                                held);
  });
  reservation.Pass(rotation, kSharedReserveDelayNs + 1, [&](int client) {
    return AffordProgrammerSend(reservation, zero_shared, schedule, client, own,
                                100, 0, ClientFeeds::Feed::kTelemetry,
                                kSharedReserveDelayNs + 1, false, held);
  });
  EXPECT_FALSE(reservation.reserved().has_value());

  // Duplicate DriveSlots::Admit refreshes recency without duplicating slots.
  DriveSlots<char> slots(2);
  const auto a1 = slots.Admit('a');
  const EgressBudget* a_budget = slots.Budget('a');
  ASSERT_NE(a_budget, nullptr);
  EXPECT_FALSE(slots.Admit('b').evicted.has_value());
  const auto a2 = slots.Admit('a');
  EXPECT_EQ(a2.slot, a1.slot);
  EXPECT_FALSE(a2.evicted.has_value());
  EXPECT_EQ(slots.Budget('a'), a_budget);
  EXPECT_EQ(slots.clients(), (std::vector<char>{'b', 'a'}));
  // Next admit evicts 'b' (now oldest), and Remove/Admit stay consistent.
  EXPECT_EQ(slots.Admit('c').evicted, 'b');
  slots.Remove('a');
  EXPECT_EQ(slots.Budget('a'), nullptr);
  EXPECT_FALSE(slots.Admit('d').evicted.has_value());
  EXPECT_EQ(slots.clients(), (std::vector<char>{'c', 'd'}));
}

}  // namespace
}  // namespace studio
