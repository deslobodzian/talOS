#include "studio/review/review.h"

#include <gtest/gtest.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <mach/mach.h>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection_generated.h"
#include "nlohmann/json.hpp"
#include "review_test/review_alert_descriptor.h"
#include "review_test/review_alert_generated.h"
#include "review_test/review_message_descriptor.h"
#include "review_test/review_message_generated.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "studio/review/hardware_channels.h"
#include "studio/review/http.h"
#include "studio/review/series_cache.h"
#include "studio/review/testdata/fixture.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/context.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/gap_testing.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/launcher/hardware_record.h"
#include "talOS/launcher/session_retention.h"
#include "talOS/protocol/types.h"
#include "talOS/telemetry/frames.h"

namespace studio::review {
namespace {

using nlohmann::json;
using testing::kStartNs;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kSecond = 1'000 * kMs;

std::string Directory() {
  return ::testing::TempDir() + "/review_" + std::to_string(getpid());
}

// `log_dump --format jsonl <dir>`, one object per record.
std::vector<json> LogDump(const std::string& dir) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  if (runfiles == nullptr) throw std::runtime_error(error);
  const char* location = std::getenv("TALOS_LOG_DUMP");
  if (location == nullptr) throw std::runtime_error("TALOS_LOG_DUMP unset");
  const std::string binary = runfiles->Rlocation(location);
  const std::string output = Directory() + "/log_dump.jsonl";
  const pid_t child = fork();
  if (child == 0) {
    if (freopen(output.c_str(), "w", stdout) == nullptr) _exit(126);
    execl(binary.c_str(), binary.c_str(), "--format", "jsonl", dir.c_str(),
          static_cast<char*>(nullptr));
    _exit(127);
  }
  int status = 0;
  if (child < 0 || waitpid(child, &status, 0) != child || !WIFEXITED(status) ||
      WEXITSTATUS(status) != 0) {
    throw std::runtime_error("log_dump failed");
  }
  std::vector<json> records;
  std::ifstream stream{output};
  for (std::string line; std::getline(stream, line);) {
    records.push_back(json::parse(line));
  }
  return records;
}

// One value of a channel as log_dump decoded it: the record's publication
// time (its publish_ms, from the earliest event time, kStartNs) and value.
struct Expected {
  std::int64_t publish_ns;
  double value;
};

// `{position_m: 0.125, shot_count: 0, mode: IDLE, ready: false}` by field.
std::map<std::string, std::string> Fields(const std::string& decoded) {
  std::map<std::string, std::string> fields;
  std::string body = decoded.substr(1, decoded.size() - 2);
  std::size_t at = 0;
  while (at < body.size()) {
    const std::size_t end = std::min(body.find(", ", at), body.size());
    const std::string item = body.substr(at, end - at);
    const std::size_t colon = item.find(": ");
    fields[item.substr(0, colon)] = item.substr(colon + 2);
    at = end + 2;
  }
  return fields;
}

// The values of the records of `topic` that `process` logged as `kind`, by
// `value` (which returns false for a record without one); a SEND only when
// its transport accepted it (aux 0), as only those reached a reader.
std::vector<Expected> Values(
    const std::vector<json>& records, const std::string& process,
    const std::string& kind, const std::string& topic,
    const std::function<bool(const json&, double&)>& value) {
  std::vector<Expected> out;
  for (const json& r : records) {
    if (r["process"] != process || r["kind"] != kind || r["topic"] != topic ||
        (kind == "SEND" && r["aux"] != 0)) {
      continue;
    }
    double v = 0;
    if (!value(r, v)) continue;
    out.push_back(
        {kStartNs + std::llround(r["publish_ms"].get<double>() * 1e6), v});
  }
  return out;
}

class ReviewTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    std::filesystem::remove_all(Directory());
    testing::WriteFixtureSession(Directory() + "/library/s1");
    records_ = new std::vector<json>(LogDump(Directory() + "/library/s1"));
  }
  static void TearDownTestSuite() {
    delete records_;
    std::filesystem::remove_all(Directory());
  }
  static std::string Library() { return Directory() + "/library"; }

  // Shot positions, Driver Station match times and request demands, as
  // log_dump decodes them from each channel's canonical records: the
  // shooter's SENDs and, with no writer logged, the observer's receipts.
  static std::vector<Expected> Positions() {
    return Values(*records_, "shooter", "SEND", "/shooter/state",
                  [](const json& r, double& v) {
                    v = std::stod(Fields(r["decoded"])["position_m"]);
                    return true;
                  });
  }
  static std::vector<Expected> MatchTimes() {
    return Values(
        *records_, "observer", "MESSAGE", "/hw/state/driver_station",
        [](const json& r, double& v) {
          // Printed with 9 digits: a float's exact value.
          v = static_cast<float>(r["hardware"]["match_time_s"].get<double>());
          return true;
        });
  }
  static std::vector<Expected> Demands() {
    return Values(*records_, "shooter", "SEND", "/hw/request/shooter",
                  [](const json& r, double& v) {
                    v = r["hardware"]["motors"][0]["demand"].get<double>();
                    return true;
                  });
  }

  static std::vector<json>* records_;
};

std::vector<json>* ReviewTest::records_ = nullptr;

// The value an expected series' bucket holds: count, min, max, last and the
// last's publication time, over [start, end) cut into `points` equal
// buckets.
struct Bucket {
  std::uint32_t count{0};
  double min{NAN}, max{NAN}, last{NAN}, t_ns{NAN};
};
std::vector<Bucket> Bucketed(const std::vector<Expected>& values,
                             std::int64_t start, std::int64_t end,
                             std::uint32_t points) {
  std::vector<Bucket> buckets(points);
  std::vector<std::int64_t> last_ns(points, INT64_MIN);
  for (const Expected& e : values) {
    if (e.publish_ns < start || e.publish_ns >= end) continue;
    const auto b = static_cast<std::size_t>(
        static_cast<__int128>(e.publish_ns - start) * points / (end - start));
    Bucket& bucket = buckets[b];
    bucket.min = bucket.count == 0 ? e.value : std::min(bucket.min, e.value);
    bucket.max = bucket.count == 0 ? e.value : std::max(bucket.max, e.value);
    ++bucket.count;
    if (e.publish_ns >= last_ns[b]) {
      last_ns[b] = e.publish_ns;
      bucket.last = e.value;
      bucket.t_ns = static_cast<double>(e.publish_ns);
    }
  }
  return buckets;
}

// The series body (review.h) as buckets, per channel.
std::vector<std::vector<Bucket>> ParseSeries(const std::string& body) {
  std::uint32_t channels = 0;
  std::uint32_t points = 0;
  std::memcpy(&channels, body.data(), 4);
  std::memcpy(&points, body.data() + 4, 4);
  EXPECT_EQ(body.size(), 8 + std::size_t{channels} * points * 36);
  std::vector<std::vector<Bucket>> out(channels, std::vector<Bucket>(points));
  const char* at = body.data() + 8;
  for (auto& series : out) {
    for (double Bucket::*field :
         {&Bucket::t_ns, &Bucket::min, &Bucket::max, &Bucket::last}) {
      for (Bucket& bucket : series) {
        std::memcpy(&(bucket.*field), at, 8);
        at += 8;
      }
    }
    for (Bucket& bucket : series) {
      std::memcpy(&bucket.count, at, 4);
      at += 4;
    }
  }
  return out;
}

void ExpectSameBuckets(const std::vector<Bucket>& actual,
                       const std::vector<Bucket>& expected,
                       const std::string& channel) {
  ASSERT_EQ(actual.size(), expected.size()) << channel;
  const auto same = [](double a, double b) {
    return (std::isnan(a) && std::isnan(b)) || a == b;
  };
  for (std::size_t b = 0; b < actual.size(); ++b) {
    SCOPED_TRACE(channel + " bucket " + std::to_string(b));
    EXPECT_EQ(actual[b].count, expected[b].count);
    if (expected[b].count == 0) continue;  // the rest is ignored (NaN)
    EXPECT_TRUE(same(actual[b].min, expected[b].min))
        << actual[b].min << " vs " << expected[b].min;
    EXPECT_TRUE(same(actual[b].max, expected[b].max))
        << actual[b].max << " vs " << expected[b].max;
    EXPECT_TRUE(same(actual[b].last, expected[b].last))
        << actual[b].last << " vs " << expected[b].last;
    EXPECT_TRUE(same(actual[b].t_ns, expected[b].t_ns))
        << actual[b].t_ns << " vs " << expected[b].t_ns;
  }
}

// Guards that a reviewed value is log_dump's decoding of the same record,
// with its publication time, across a rotated log, a restart and a damaged
// record: a schema'd struct field, a Driver Station channel under its live
// telemetry name, and a decoded hardware request. Reading a reader's copy
// instead of the writer's, a receipt time instead of the publication time,
// a skipped segment or restart, a damaged record decoded anyway, or a failed
// send (tick kFailedShot's, logged after the accepted one, at 4 s) taken as
// a value, changes a count, a value or a time. Every channel shares the same
// bucket edges, dividing [start_ns, end_ns) evenly: a shot published exactly at
// end_ns is in no bucket. REQ-OPS-41
TEST_F(ReviewTest, SnapshotAndSeriesEqualLogDump) {
  Review review{Library(), 1u << 20};
  const std::vector<std::pair<std::string, std::vector<Expected>>> channels{
      {"shooter/state/position_m", Positions()},
      {"hw/state/driver_station/match_time_s", MatchTimes()},
      {"hw/request/shooter/motors/1/demand", Demands()}};
  for (const auto& [name, values] : channels) {
    ASSERT_FALSE(values.empty()) << name;
  }

  // Just after packet 40, whose record is damaged: both readers skip it.
  for (const std::int64_t t :
       {kStartNs + 4 * kSecond + 5 * kMs, kStartNs + 8 * kSecond + 13 * kMs}) {
    SCOPED_TRACE("t_ns " + std::to_string(t));
    const Response response =
        review.Get("/review/s1/snapshot", "t_ns=" + std::to_string(t));
    ASSERT_EQ(response.status, 200) << response.body;
    const json snapshot = json::parse(response.body);
    EXPECT_EQ(snapshot["t_ns"], std::to_string(t));
    for (const auto& [name, values] : channels) {
      const Expected* latest = nullptr;
      for (const Expected& e : values) {
        if (e.publish_ns <= t &&
            (latest == nullptr || e.publish_ns >= latest->publish_ns)) {
          latest = &e;
        }
      }
      ASSERT_NE(latest, nullptr) << name;
      ASSERT_TRUE(snapshot["channels"].contains(name)) << name;
      EXPECT_EQ(snapshot["channels"][name]["value"].get<double>(),
                latest->value)
          << name;
      EXPECT_EQ(snapshot["channels"][name]["sample_ns"],
                std::to_string(latest->publish_ns))
          << name;
    }
  }

  // Both runs, the gap between them, and the rotated segments.
  const std::int64_t start = kStartNs + kSecond + 7 * kMs;
  // Shot 450's publication time.
  const std::int64_t end = kStartNs + 9 * kSecond + kMs / 2;
  constexpr std::uint32_t kPoints = 17;
  std::string query = "start_ns=" + std::to_string(start) +
                      "&end_ns=" + std::to_string(end) +
                      "&points=" + std::to_string(kPoints);
  query += "&c=";
  for (const auto& [name, values] : channels) query += name + ",";
  const Response response = review.Get("/review/s1/series", query);
  ASSERT_EQ(response.status, 200) << response.body;
  EXPECT_EQ(response.content_type, "application/octet-stream");
  const auto series = ParseSeries(response.body);
  ASSERT_EQ(series.size(), channels.size());
  for (std::size_t i = 0; i < channels.size(); ++i) {
    ExpectSameBuckets(series[i],
                      Bucketed(channels[i].second, start, end, kPoints),
                      channels[i].first);
  }
}

// Guards what a session time means on the wall clock: origin_wall_ns is the
// wall time of session time zero, from the header of the log whose loop
// started first (the observer's, half a second before any record), so wall
// time = origin_wall_ns + t_ns. Taking the wall time at start_ns, or another
// log's header, shifts it.
// REQ-OPS-41
TEST_F(ReviewTest, OriginWallIsTheWallClockAtSessionTimeZero) {
  talos::event::log::FileHeader header{};
  std::ifstream stream{Library() + "/s1/observer.tlog", std::ios::binary};
  ASSERT_TRUE(stream.read(reinterpret_cast<char*>(&header), sizeof header));
  ASSERT_EQ(header.start_monotonic_ns, kStartNs + testing::kObserverOriginNs);
  Review review{Library(), 1u << 20};
  const Response response = review.Get("/review/s1/bounds.json", "");
  ASSERT_EQ(response.status, 200) << response.body;
  const json bounds = json::parse(response.body);
  EXPECT_EQ(bounds["origin_wall_ns"],
            std::to_string(header.start_wall_ns - header.start_monotonic_ns));
  EXPECT_EQ(bounds["start_ns"], std::to_string(kStartNs));
  EXPECT_EQ(bounds["end_ns"],
            std::to_string(kStartNs + 10 * kSecond + 2 * kMs));
}

// Guards the snapshot's sample format, the bridge's /telemetry.json one: a
// NaN reading is the string "NaN", not a number JSON cannot hold or an
// absent channel.
// REQ-OPS-41
TEST_F(ReviewTest, SnapshotWritesANonFiniteReadingAsTheBridgeDoes) {
  Review review{Library(), 1u << 20};
  const std::int64_t t = kStartNs + 9 * kSecond + 990 * kMs;
  const Response response =
      review.Get("/review/s1/snapshot", "t_ns=" + std::to_string(t));
  ASSERT_EQ(response.status, 200) << response.body;
  const json channels = json::parse(response.body)["channels"];
  ASSERT_TRUE(channels.contains("shooter/state/position_m")) << response.body;
  EXPECT_EQ(channels["shooter/state/position_m"]["value"], "NaN");
  EXPECT_EQ(channels["shooter/state/shot_count"]["value"], testing::kLastShot);
}

// Guards that a request reads only the seconds it covers: a series late in
// the session starts at its index checkpoint, so it walks the last seconds
// of the restart's log and none of the first run's rotated segments.
// Reading from each file's start, or ignoring which sources a second holds,
// walks more records.
// REQ-OPS-42
TEST_F(ReviewTest, LateRangeReadsOnlyItsSeconds) {
  Review review{Library(), 1u << 20};
  const std::int64_t start = kStartNs + 9 * kSecond;
  const std::int64_t end = kStartNs + 9 * kSecond + 900 * kMs;
  const Response response = review.Get(
      "/review/s1/series",
      "c=shooter/state/position_m&start_ns=" + std::to_string(start) +
          "&end_ns=" + std::to_string(end) + "&points=4");
  ASSERT_EQ(response.status, 200) << response.body;
  // Seconds 8 and 9 of the restart: 100 ticks of a timer record and two
  // sends, out of 450 such ticks in the shooter's logs.
  EXPECT_EQ(review.last_read().seeks, 1u);
  EXPECT_LE(review.last_read().records, 300u);
  EXPECT_GT(ParseSeries(response.body)[0][0].count, 0u);
}

// Guards a bucket whose every sample is NaN (here only the last shot's,
// published at 9.9805 s): it counts its sample and reports NaN for min, max
// and last, not +Inf and -Inf (fmin and fmax skip a NaN, leaving their
// starting values).
// REQ-OPS-41
TEST_F(ReviewTest, AllNaNBucketHasNaNExtremes) {
  Review review{Library(), 1u << 20};
  const Response response = review.Get(
      "/review/s1/series",
      "c=shooter/state/position_m&start_ns=" +
          std::to_string(kStartNs + 9 * kSecond + 970 * kMs) + "&end_ns=" +
          std::to_string(kStartNs + 9 * kSecond + 990 * kMs) + "&points=1");
  ASSERT_EQ(response.status, 200) << response.body;
  const Bucket bucket = ParseSeries(response.body)[0][0];
  EXPECT_EQ(bucket.count, 1u);
  EXPECT_TRUE(std::isnan(bucket.min)) << bucket.min;
  EXPECT_TRUE(std::isnan(bucket.max)) << bucket.max;
  EXPECT_TRUE(std::isnan(bucket.last)) << bucket.last;
}

// Guards the Host check on port 80, where a browser omits the default port
// from Host: a bare loopback name is this server only there.
// REQ-OPS-41
TEST(HostTest, BareLoopbackNameIsThisServerOnlyOnPort80) {
  const struct {
    std::string_view host;
    std::uint16_t port;
    bool allowed;
  } rows[] = {
      {"127.0.0.1:5805", 5805, true},
      {"127.0.0.1", 5805, false},
      {"127.0.0.1", 80, true},
      {"localhost", 80, true},
      {"[::1]", 80, true},
      {"127.0.0.1:80", 80, true},
      {"evil.example", 80, false},
      {"127.0.0.1:81", 80, false},
  };
  for (const auto& row : rows) {
    SCOPED_TRACE(std::string{row.host} + " on " + std::to_string(row.port));
    EXPECT_EQ(AllowedHost(row.host, row.port), row.allowed);
  }
}

// Guards the cache's bound (--cache-mb): its accounted bytes never pass the
// capacity, the least recently used entry goes first, and an entry larger
// than the whole capacity is not kept.
// REQ-OPS-42
TEST(SeriesCacheTest, StaysUnderItsCapacityEvictingLeastRecentlyUsed) {
  const auto entry = [](std::uint32_t points) {
    auto series = std::make_shared<Series>();
    series->t_ns.assign(points, 0);
    series->min = series->max = series->last = series->t_ns;
    series->count.assign(points, 0);
    return series;
  };
  constexpr std::size_t kCapacity = std::size_t{16} * 1024;
  SeriesCache cache{kCapacity};
  cache.Put("first", entry(64));
  for (int i = 0; i < 100; ++i) {
    cache.Put("key" + std::to_string(i), entry(64));
    ASSERT_NE(cache.Get("first"), nullptr) << "evicted after " << i;
    EXPECT_LE(cache.bytes(), kCapacity) << "after " << i;
  }
  EXPECT_EQ(cache.Get("key0"), nullptr);
  EXPECT_NE(cache.Get("key99"), nullptr);
  cache.Put("huge", entry(kCapacity / 36 + 1));
  EXPECT_EQ(cache.Get("huge"), nullptr);
  EXPECT_LE(cache.bytes(), kCapacity);
}

// Guards one series request's share of the heap bound (README "Reviewing a
// session"): a request past kMaxSeriesValues (channels x points; here one
// channel named 14 times at 20,000 points) is refused naming the limit, and
// one at it answers within kMaxSeriesRequestBytes. Without the cap, 256
// channels at 20,000 points held ~400 MB however small --cache-mb was.
// REQ-OPS-42
TEST_F(ReviewTest, SeriesPastItsValueCeilingIsRefused) {
  Review review{Library(), 1u << 20};
  const auto request = [&review](std::size_t channels) {
    std::string query = "start_ns=" + std::to_string(kStartNs) +
                        "&end_ns=" + std::to_string(kStartNs + 10 * kSecond) +
                        "&points=" + std::to_string(kMaxSeriesPoints) + "&c=";
    for (std::size_t i = 0; i < channels; ++i) {
      query += "shooter/state/position_m,";
    }
    return review.Get("/review/s1/series", query);
  };
  constexpr std::size_t kFits = kMaxSeriesValues / kMaxSeriesPoints;
  const Response over = request(kFits + 1);
  ASSERT_EQ(over.status, 400);
  EXPECT_NE(over.body.find(std::to_string(kMaxSeriesValues)), std::string::npos)
      << over.body;
  const Response at = request(kFits);
  ASSERT_EQ(at.status, 200) << at.body;
  EXPECT_LE(SeriesRequestBytes(kFits, kMaxSeriesPoints),
            kMaxSeriesRequestBytes);
  EXPECT_EQ(at.body.size(), 8 + kFits * kMaxSeriesPoints * 36);
}

// Guards the bound on open sessions: opening one past kMaxOpenSessions
// closes the least recently used, and a closed session reopens (its index
// rebuilt) with the same answers. Without it every session ever opened
// kept its index for the server's life.
// REQ-OPS-42
TEST_F(ReviewTest, OpenSessionsStayWithinTheirCap) {
  const std::filesystem::path library = Directory() + "/many";
  std::filesystem::create_directories(library);
  for (std::size_t i = 0; i <= kMaxOpenSessions; ++i) {
    std::filesystem::copy(Library() + "/s1",
                          library / ("s" + std::to_string(i)),
                          std::filesystem::copy_options::recursive);
  }
  Review review{library, 1u << 20};
  const Response first = review.Get("/review/s0/bounds.json", "");
  ASSERT_EQ(first.status, 200) << first.body;
  for (std::size_t i = 1; i <= kMaxOpenSessions; ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(
        review.Get("/review/s" + std::to_string(i) + "/bounds.json", "").status,
        200);
    EXPECT_LE(review.open_sessions(), kMaxOpenSessions);
  }
  const Response again = review.Get("/review/s0/bounds.json", "");
  EXPECT_EQ(again.status, 200);
  EXPECT_EQ(again.body, first.body);
  EXPECT_EQ(review.open_sessions(), kMaxOpenSessions);
  std::filesystem::remove_all(library);
}

// Guards that a request names only a session directory inside the library:
// a parent, a hidden or encoded name, and a symlink out of the library are
// refused, never opened.
// REQ-OPS-41
TEST_F(ReviewTest, SessionIdOutsideTheLibraryIsRefused) {
  const std::string outside = Directory() + "/outside";
  testing::WriteFixtureSession(outside);
  std::filesystem::create_directory_symlink(outside, Library() + "/link");
  Review review{Library(), 1u << 20};
  EXPECT_EQ(review.Get("/review/s1/bounds.json", "").status, 200);
  for (const std::string id :
       {"..", ".", "%2e%2e", ".hidden", "a\\b", "link", "missing"}) {
    SCOPED_TRACE(id);
    const Response response = review.Get("/review/" + id + "/bounds.json", "");
    EXPECT_TRUE(response.status == 400 || response.status == 404)
        << response.status;
  }
  const json sessions =
      json::parse(review.Get("/review/sessions.json", "").body);
  ASSERT_EQ(sessions.size(), 1u) << sessions.dump();
  EXPECT_EQ(sessions[0]["id"], "s1");
  std::filesystem::remove(Library() + "/link");
  std::filesystem::remove_all(outside);
}

// Guards that a log of another format version is refused, naming the file
// and both versions, rather than read as this version.
// REQ-OPS-41
TEST_F(ReviewTest, UnknownLogVersionIsRefusedNamingFileAndVersions) {
  const std::string dir = Library() + "/old";
  testing::WriteFixtureSession(dir);
  const std::string path = dir + "/observer.tlog";
  {
    std::fstream file{path, std::ios::in | std::ios::out | std::ios::binary};
    const std::uint32_t version =
        talos::event::log::OLDEST_READABLE_VERSION - 1;
    file.seekp(offsetof(talos::event::log::FileHeader, version));
    file.write(reinterpret_cast<const char*>(&version), sizeof version);
  }
  Review review{Library(), 1u << 20};
  const Response response = review.Get("/review/old/bounds.json", "");
  EXPECT_EQ(response.status, 422);
  EXPECT_NE(response.body.find(path), std::string::npos) << response.body;
  // The version read, then those this build reads.
  for (const std::string& version :
       {"version " +
            std::to_string(talos::event::log::OLDEST_READABLE_VERSION - 1),
        std::to_string(talos::event::log::OLDEST_READABLE_VERSION) + " to " +
            std::to_string(talos::event::log::FORMAT_VERSION)}) {
    EXPECT_NE(response.body.find(version), std::string::npos) << response.body;
  }
  std::filesystem::remove_all(dir);
}

// Guards: a session recorded under another hardware protocol (its manifest
// says so, though every packet decodes under this build) serves no hardware
// channel, the Driver Station's included, while its schema'd topics are
// served; bounds.json carries log_dump's line naming both versions, and the
// fixture session, of this build's protocol, carries null.
// REQ-OPS-51, REQ-OPS-41
TEST_F(ReviewTest, SessionOfAnotherHardwareProtocolServesNoHardwareChannel) {
  const std::string dir = Library() + "/older";
  testing::WriteFixtureSession(dir);
  const std::uint16_t other = talos::protocol::kProtocolVersion - 1;
  {
    std::ifstream in{dir + "/manifest.json"};
    std::string text{std::istreambuf_iterator<char>{in}, {}};
    text.insert(
        1, "\n  \"hardware_protocol_version\": " + std::to_string(other) + ",");
    std::ofstream{dir + "/manifest.json"} << text;
  }
  Review review{Library(), 1u << 20};
  const Response channels = review.Get("/review/older/channels.json", "");
  ASSERT_EQ(channels.status, 200) << channels.body;
  // Held in a local: a range-for over a member of the temporary would read
  // the document after its destruction.
  const json listed = json::parse(channels.body);
  std::vector<std::string> names;
  for (const auto& channel : listed["channels"]) {
    names.push_back(channel["name"]);
  }
  EXPECT_EQ(std::count_if(names.begin(), names.end(),
                          [](const std::string& name) {
                            return name.starts_with("hw/");
                          }),
            0);
  EXPECT_EQ(std::count(names.begin(), names.end(), "shooter/state/position_m"),
            1);
  const json bounds =
      json::parse(review.Get("/review/older/bounds.json", "").body);
  ASSERT_TRUE(bounds["hardware_not_decoded"].is_string()) << bounds;
  const std::string line = bounds["hardware_not_decoded"];
  for (const std::uint16_t version :
       {other, talos::protocol::kProtocolVersion}) {
    EXPECT_NE(line.find(" " + std::to_string(version)), std::string::npos)
        << line;
  }
  EXPECT_TRUE(
      json::parse(
          review.Get("/review/s1/bounds.json", "").body)["hardware_not_decoded"]
          .is_null());
  std::filesystem::remove_all(dir);
}

// A reflection schema whose one struct, "t.Loop", holds a field of its own
// type: one a crafted log can carry, never flatc.
std::string SelfContainingSchema() {
  flatbuffers::FlatBufferBuilder fbb;
  const auto type = reflection::CreateType(fbb, reflection::BaseType::Obj,
                                           reflection::BaseType::None, 0, 0, 1);
  std::vector<flatbuffers::Offset<reflection::Field>> fields{
      reflection::CreateFieldDirect(fbb, "inner", type, 0, 0)};
  std::vector<flatbuffers::Offset<reflection::Object>> objects{
      reflection::CreateObjectDirect(fbb, "t.Loop", &fields, true, 1, 1)};
  std::vector<flatbuffers::Offset<reflection::Enum>> enums;
  reflection::FinishSchemaBuffer(
      fbb, reflection::CreateSchemaDirect(fbb, &objects, &enums));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

// Guards that a crafted log (a library may come from a USB stick) fails only
// its own session: "huge" holds one record claiming 4 GiB of payload with a
// checksum its author computed, so it holds no intact record and is refused
// (422) without allocating what the record claims; "loop" sends a topic
// whose schema contains itself, which is not reviewed (no channels) instead
// of recursing until the stack overflows. The server answers s1 throughout.
// REQ-OPS-41
TEST_F(ReviewTest, CraftedLogFailsOnlyItsSession) {
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::EventKind;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;
  const auto write = [](const std::string& dir,
                        talos::event::Registration source,
                        std::span<const std::byte> payload) {
    std::filesystem::create_directories(dir);
    source.id = 2;
    source.kind = SourceKind::SENDER;
    tlog::LogWriter writer{dir + "/crafted.tlog", "crafted",
                           tlog::LogWriterOptions{8192, 1, false}, 1};
    writer.start({source}, MonotonicTime::from_nanos(kStartNs));
    if (!payload.empty()) {
      Context tick{};
      tick.dispatch_index = 1;
      tick.event_time = tick.now = MonotonicTime::from_nanos(kStartNs);
      writer.send(tick, 2, payload, 0, 1);
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  };

  talos::event::Registration shot;
  shot.name = "/huge/state";
  shot.message_bytes = sizeof(review_test::Shot);
  {
    using D = talos::Descriptor<review_test::Shot>;
    shot.schema = {std::string{D::kName}, D::kHash,
                   std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                               D::kBfbs.size()}};
  }
  write(Library() + "/huge", shot, {});
  tlog::RecordHeader huge{};
  huge.magic = tlog::RECORD_MAGIC;
  huge.kind = static_cast<std::uint16_t>(EventKind::SEND);
  huge.source_id = 2;
  huge.dispatch_index = 1;
  huge.event_time_ns = huge.now_ns = huge.publish_time_ns = kStartNs;
  huge.payload_bytes = 0xFFFF'FFFFu;
  huge.crc32 = tlog::crc32(std::span<const std::byte>{
      reinterpret_cast<const std::byte*>(&huge), sizeof huge});
  std::ofstream{Library() + "/huge/crafted.tlog",
                std::ios::binary | std::ios::app}
      .write(reinterpret_cast<const char*>(&huge), sizeof huge);

  talos::event::Registration loop;
  loop.name = "/loop/state";
  loop.message_bytes = 1;
  loop.schema = {"t.Loop", 1, SelfContainingSchema()};
  const std::byte one{1};
  write(Library() + "/loop", loop, std::span{&one, 1});

  Review review{Library(), 1u << 20};
  EXPECT_EQ(review.Get("/review/huge/bounds.json", "").status, 422);
  const Response channels = review.Get("/review/loop/channels.json", "");
  ASSERT_EQ(channels.status, 200) << channels.body;
  EXPECT_EQ(channels.body.find("loop/state"), std::string::npos)
      << channels.body;
  EXPECT_EQ(review.Get("/review/s1/bounds.json", "").status, 200);
  EXPECT_EQ(review.Get("/review/sessions.json", "").status, 200);
  std::filesystem::remove_all(Library() + "/huge");
  std::filesystem::remove_all(Library() + "/loop");
}

// Guards that a reviewed session's dictionary names each alert channel's
// level exactly as the live bridge's /channels.json does ("alert" after
// "labels", `studio::AlertLevelName`): a `status` topic's
// `alert_error_<name>` bool is "error", every other channel "none", so the
// page that lists alerts reads a recorded session like a live one.
// REQ-OPS-37
TEST_F(ReviewTest, ChannelsCarryEachAlertLevelAsTheBridgeDoes) {
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;
  const std::string dir = Library() + "/alerts";
  std::filesystem::create_directories(dir);
  talos::event::Registration health;
  health.id = 2;
  health.kind = SourceKind::SENDER;
  health.name = "/thing/status";
  health.message_bytes = sizeof(review_test::Health);
  {
    using D = talos::Descriptor<review_test::Health>;
    health.schema = {std::string{D::kName}, D::kHash,
                     std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                                 D::kBfbs.size()}};
  }
  {
    tlog::LogWriter writer{dir + "/alerts.tlog", "thing",
                           tlog::LogWriterOptions{8192, 1, false}, 1};
    writer.start({health}, MonotonicTime::from_nanos(kStartNs));
    const review_test::Health message{true, false};
    Context tick{};
    tick.dispatch_index = 1;
    tick.event_time = tick.now = MonotonicTime::from_nanos(kStartNs);
    writer.send(
        tick, 2,
        std::span{reinterpret_cast<const std::byte*>(&message), sizeof message},
        0, 1);
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }

  Review review{Library(), 1u << 20};
  const Response channels = review.Get("/review/alerts/channels.json", "");
  std::filesystem::remove_all(dir);
  ASSERT_EQ(channels.status, 200) << channels.body;
  EXPECT_NE(channels.body.find(
                R"({"name":"thing/status/alert_error_link_lost","unit":"",)"
                R"("kind":"bool","labels":[],"alert":"error"})"),
            std::string::npos)
      << channels.body;
  EXPECT_NE(channels.body.find(R"({"name":"thing/status/ready","unit":"",)"
                               R"("kind":"bool","labels":[],"alert":"none"})"),
            std::string::npos)
      << channels.body;
}

// The 0-based index of the first intact record of the log at `path` that
// `match` accepts (CopyWithGap's record numbering).
std::size_t RecordIndex(
    const std::string& path,
    const std::function<bool(const talos::event::log::RecordHeader&)>& match) {
  talos::event::log::LogReader reader{path};
  talos::event::log::LogReader::Record record;
  for (std::size_t i = 0; reader.next(record); ++i) {
    if (match(record.header)) return i;
  }
  throw std::runtime_error("no such record in " + path);
}

// The fixture session copied to `dir` with two gaps, made as the writer
// would make them (CopyWithGap): the observer's records of Driver Station
// packets 60..64 (published 6.0 s to 6.4 s; capture resumed with packet 65,
// received at 6.502 s), and the shooter restart's ticks 490..499 to its end
// (a gap closed at the stop: 30 records, ending at tick 499's 9.9805 s).
// Returns the session directory.
std::string WriteGappedSession(const std::string& dir) {
  namespace tlog = talos::event::log;
  const std::string source = dir + ".source";
  testing::WriteFixtureSession(source);
  std::filesystem::create_directories(dir);
  for (const auto& entry : std::filesystem::directory_iterator{source}) {
    std::filesystem::copy(entry.path(),
                          std::filesystem::path{dir} / entry.path().filename());
  }
  const std::string observer = source + "/observer.tlog";
  const std::size_t packet60 =
      RecordIndex(observer, [](const tlog::RecordHeader& h) {
        return h.publish_time_ns == kStartNs + 6 * kSecond;
      });
  tlog::CopyWithGap(observer, dir + "/observer.tlog", packet60, packet60 + 5);
  const std::string restart = source + "/shooter.restart1.tlog";
  const std::size_t tick490 =
      RecordIndex(restart, [](const tlog::RecordHeader& h) {
        return h.kind ==
                   static_cast<std::uint16_t>(talos::event::EventKind::TIMER) &&
               h.event_time_ns == kStartNs + 490 * (20 * kMs);
      });
  std::size_t records = 0;
  {
    tlog::LogReader reader{restart};
    tlog::LogReader::Record record;
    while (reader.next(record)) ++records;
  }
  tlog::CopyWithGap(restart, dir + "/shooter.restart1.tlog", tick490, records);
  std::filesystem::remove_all(source);
  return dir;
}

// Guards REQ-OPS-41's gap list and REQ-OPS-42's per-gap heap term:
// bounds.json lists every GAP record of the session, from two processes, in
// order of start (then process), each with the process, its start and end
// (resumed, or the stop) as decimal strings, the records, dispatches and
// sends it lost and whether the stop closed it; a session without one has
// an empty list (the golden bounds.json). A gap's records are never values:
// a series bucket inside the shooter's gap holds no sample, and a snapshot
// in it holds the last value published before it (tick 489's), as for any
// stretch a topic was not published, never one from the gap's own record.
// The index keeps 48 B per gap beside its checkpoints.
// REQ-OPS-41, REQ-OPS-42
TEST_F(ReviewTest, BoundsListEveryGap) {
  const std::string dir = WriteGappedSession(Library() + "/gapped");
  Review review{Library(), 1u << 20};
  const Response response = review.Get("/review/gapped/bounds.json", "");
  ASSERT_EQ(response.status, 200) << response.body;
  const json bounds = json::parse(response.body);
  ASSERT_TRUE(bounds.contains("gaps")) << response.body;
  const json expected = json::parse(
      R"([{"process":"observer","start_ns":"1006000000000",)"
      R"("end_ns":"1006502000000","records":5,"dispatches":5,"sends":0,)"
      R"("closed_at_flush":false},)"
      R"({"process":"shooter","start_ns":"1009800000000",)"
      R"("end_ns":"1009980500000","records":30,"dispatches":10,"sends":20,)"
      R"("closed_at_flush":true}])");
  EXPECT_EQ(bounds["gaps"], expected);
  EXPECT_EQ(bounds["start_ns"], std::to_string(kStartNs));
  EXPECT_EQ(bounds["end_ns"],
            std::to_string(kStartNs + 10 * kSecond + 2 * kMs));

  const std::int64_t gap_start = kStartNs + 9 * kSecond + 800 * kMs;
  const Response series = review.Get(
      "/review/gapped/series",
      "c=shooter/state/position_m&start_ns=" + std::to_string(gap_start) +
          "&end_ns=" + std::to_string(gap_start + 200 * kMs) + "&points=2");
  ASSERT_EQ(series.status, 200) << series.body;
  const auto parsed = ParseSeries(series.body);
  ASSERT_EQ(parsed.size(), 1u);
  for (const Bucket& bucket : parsed[0]) {
    EXPECT_EQ(bucket.count, 0u);
  }
  const Response snapshot =
      review.Get("/review/gapped/snapshot",
                 "t_ns=" + std::to_string(kStartNs + 9 * kSecond + 950 * kMs));
  ASSERT_EQ(snapshot.status, 200) << snapshot.body;
  const json position =
      json::parse(snapshot.body)["channels"]["shooter/state/position_m"];
  EXPECT_EQ(position["value"], testing::ShotPosition(489));
  EXPECT_EQ(position["sample_ns"],
            std::to_string(kStartNs + 489 * (20 * kMs) + kMs / 2));

  const auto session = Session::Open(dir);
  EXPECT_EQ(session->index_bytes(),
            session->log_seconds() * sizeof(Checkpoint) + std::size_t{2} * 48);
  std::filesystem::remove_all(dir);
}

// Guards that a damaged GAP record is damage, as log_dump reads it: skipped
// and warned about naming its log, never listed as a gap (log_dump returns
// no GAP record either), and the rest of the session is served.
// REQ-OPS-41
TEST_F(ReviewTest, DamagedGapIsDamageNotAGap) {
  namespace tlog = talos::event::log;
  const std::string dir = Library() + "/damaged_gap";
  testing::WriteFixtureSession(dir);
  const std::string path = dir + "/shooter.restart1.tlog";
  const std::string copy = dir + "/copy.tmp";
  tlog::CopyWithGap(path, copy, 30, 60);  // ticks 360..369
  std::filesystem::rename(copy, path);
  std::uint64_t offset = 0;
  {
    tlog::WindowedLogReader reader{path};
    tlog::LogReader::Record record;
    while (reader.next(record)) {
      if (record.header.kind ==
          static_cast<std::uint16_t>(talos::event::EventKind::GAP)) {
        offset = record.offset;
      }
    }
  }
  ASSERT_NE(offset, 0u);
  {
    std::fstream file{path, std::ios::in | std::ios::out | std::ios::binary};
    const auto at =
        static_cast<std::streamoff>(offset + sizeof(tlog::RecordHeader) +
                                    offsetof(tlog::GapRecord, records));
    file.seekp(at);
    const char byte = 0x7F;
    file.write(&byte, 1);
    ASSERT_TRUE(file);
  }

  Review review{Library(), 1u << 20};
  ::testing::internal::CaptureStderr();
  const Response response = review.Get("/review/damaged_gap/bounds.json", "");
  const std::string stderr_text = ::testing::internal::GetCapturedStderr();
  ASSERT_EQ(response.status, 200) << response.body;
  EXPECT_EQ(json::parse(response.body)["gaps"], json::array()) << response.body;
  EXPECT_NE(stderr_text.find("warning: damaged log " + path + ": "),
            std::string::npos)
      << stderr_text;
  for (const json& record : LogDump(dir)) {
    EXPECT_NE(record["kind"], "GAP") << record.dump();
  }
  std::filesystem::remove_all(dir);
}

// The golden responses (testdata/golden), by file name, and the request each
// answers. The series covers [9.9 s, 10.1 s) in four buckets: the position
// ends its second bucket on the last shot's NaN and has two empty buckets
// after the last shot, shot_id is past 2^53, and the status, sent once at
// 0 s, has no data in it.
std::vector<std::pair<std::string, std::pair<std::string, std::string>>>
GoldenRequests() {
  const std::string end = std::to_string(kStartNs + 10 * kSecond + 2 * kMs);
  return {
      {"sessions.json", {"/review/sessions.json", ""}},
      {"channels.json", {"/review/s1/channels.json", ""}},
      {"bounds.json", {"/review/s1/bounds.json", ""}},
      {"timeline.json", {"/review/s1/timeline.json", ""}},
      {"declared.json", {"/review/s1/declared.json", ""}},
      {"snapshot.json", {"/review/s1/snapshot", "t_ns=" + end}},
      {"series.bin",
       {"/review/s1/series",
        "c=shooter/state/position_m,shooter/state/shot_id,"
        "shooter/status/boot_id&start_ns=" +
            std::to_string(kStartNs + 9 * kSecond + 900 * kMs) + "&end_ns=" +
            std::to_string(kStartNs + 10 * kSecond + 100 * kMs) + "&points=4"}},
  };
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream stream{path, std::ios::binary};
  return {std::istreambuf_iterator<char>{stream}, {}};
}

// Guards the review API's shapes against the checked-in golden responses,
// which //studio:review_golden_test decodes with Studio's review.ts: a field
// renamed, reordered or re-encoded on the server changes these bytes, and
// the same change on the browser side fails that test. An intentional change
// regenerates them (studio/README.md, "Reviewing a session"):
//   TALOS_UPDATE_GOLDEN=1 bazel run //studio/review:review_test --
//       --gtest_filter=ReviewTest.ResponsesEqualTheGoldenFiles
// REQ-OPS-41
TEST_F(ReviewTest, ResponsesEqualTheGoldenFiles) {
  Review review{Library(), 1u << 20};
  const char* update = std::getenv("TALOS_UPDATE_GOLDEN");
  std::filesystem::path golden;
  if (update != nullptr && std::string_view{update} == "1") {
    const char* workspace = std::getenv("BUILD_WORKSPACE_DIRECTORY");
    ASSERT_NE(workspace, nullptr) << "update the golden files with bazel run";
    golden = std::filesystem::path{workspace} / "studio/review/testdata/golden";
  } else {
    std::string error;
    std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
        rules_cc::cc::runfiles::Runfiles::CreateForTest(
            BAZEL_CURRENT_REPOSITORY, &error));
    ASSERT_NE(runfiles, nullptr) << error;
    const char* bounds = std::getenv("TALOS_GOLDEN_BOUNDS");
    if (bounds == nullptr) FAIL() << "TALOS_GOLDEN_BOUNDS is not set";
    golden = std::filesystem::path{runfiles->Rlocation(bounds)}.parent_path();
  }
  for (const auto& [file, request] : GoldenRequests()) {
    SCOPED_TRACE(file);
    const Response response = review.Get(request.first, request.second);
    ASSERT_EQ(response.status, 200) << response.body;
    if (update != nullptr && std::string_view{update} == "1") {
      std::ofstream{golden / file, std::ios::binary} << response.body;
      continue;
    }
    EXPECT_EQ(response.body, ReadFile(golden / file))
        << "the response differs from " << (golden / file).string();
  }
}

// One file's name, size and modification time.
using Listing =
    std::map<std::string,
             std::pair<std::uintmax_t, std::filesystem::file_time_type>>;
Listing List(const std::filesystem::path& root) {
  Listing listing;
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator{root}) {
    listing[entry.path().string()] = {
        entry.is_regular_file() ? entry.file_size() : 0,
        entry.last_write_time()};
  }
  return listing;
}

// Guards that review never writes into the library it reads: a library may
// be a read-only copy, and classifying a session is the launcher's
// retention's business. A non-simulation session with no session_class
// (so "unknown") is served by every endpoint and its files are unchanged,
// no session_class added; then, with every mode bit made read-only, every
// endpoint still answers.
// REQ-OPS-41
TEST_F(ReviewTest, LibraryIsNeverWritten) {
  const std::filesystem::path library = Directory() + "/read_only";
  testing::WriteFixtureSession((library / "s1").string());
  std::string manifest = ReadFile(library / "s1/manifest.json");
  const auto at = manifest.find("\"simulation\": true");
  ASSERT_NE(at, std::string::npos);
  manifest.replace(at, 18, "\"simulation\": false");
  std::ofstream{library / "s1/manifest.json"} << manifest;

  const auto every_endpoint = [&library] {
    Review review{library, 1u << 20};
    for (const auto& [file, request] : GoldenRequests()) {
      SCOPED_TRACE(file);
      const Response response = review.Get(request.first, request.second);
      EXPECT_EQ(response.status, 200) << response.body;
    }
    const json sessions =
        json::parse(review.Get("/review/sessions.json", "").body);
    ASSERT_EQ(sessions.size(), 1u);
    EXPECT_EQ(sessions[0]["class"], "unknown");
  };
  const Listing before = List(library);
  every_endpoint();
  EXPECT_EQ(List(library), before);
  EXPECT_FALSE(std::filesystem::exists(library / "s1/session_class"));

  using std::filesystem::perms;
  const auto read_only = [&library](perms dir, perms file) {
    for (const auto& entry :
         std::filesystem::recursive_directory_iterator{library}) {
      std::filesystem::permissions(entry.path(),
                                   entry.is_directory() ? dir : file);
    }
    std::filesystem::permissions(library, dir);
  };
  read_only(perms::owner_read | perms::owner_exec, perms::owner_read);
  every_endpoint();
  read_only(perms::owner_all, perms::owner_read | perms::owner_write);
  EXPECT_EQ(List(library), before);
  std::filesystem::remove_all(library);
}

// Guards that listing sessions reads no event log (README "Reviewing a
// session": no log file is read whole): a non-simulation session's class is
// the launcher's session_class when current, else "unknown", even when its
// logs would answer. Reading the logs (RecordedMatch) lists the fixture,
// which has no FMS-attached packet, as "other" without a cache.
// REQ-OPS-42
TEST_F(ReviewTest, ListingClassifiesFromTheLauncherCacheOnly) {
  const std::filesystem::path library = Directory() + "/classes";
  testing::WriteFixtureSession((library / "s1").string());
  std::string manifest = ReadFile(library / "s1/manifest.json");
  const auto at = manifest.find("\"simulation\": true");
  ASSERT_NE(at, std::string::npos);
  manifest.replace(at, 18, "\"simulation\": false");
  std::ofstream{library / "s1/manifest.json"} << manifest;
  std::uintmax_t log_bytes = 0;
  for (const auto& entry :
       std::filesystem::directory_iterator{library / "s1"}) {
    if (entry.path().extension() == ".tlog") log_bytes += entry.file_size();
  }
  const struct {
    const char* cache;  // session_class's content, or null for none
    const char* expected;
  } cases[] = {
      {nullptr, "unknown"},        {"match 0\n", "match"}, {"stale", "unknown"},
      {"stale_reader", "unknown"}, {"current", "other"},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.cache == nullptr ? "no cache" : c.cache);
    const auto file = library / "s1/session_class";
    if (c.cache == nullptr) {
      std::filesystem::remove(file);
    } else if (std::string_view{c.cache} == "stale") {
      std::ofstream{file} << "other " << log_bytes + 1 << '\n';
    } else if (std::string_view{c.cache} == "stale_reader") {
      std::ofstream{file} << "other " << log_bytes << " old.reader\n";
    } else if (std::string_view{c.cache} == "current") {
      std::filesystem::remove(file);
      ASSERT_EQ(talos::launcher::RecordedMatch(
                    library / "s1", talos::launcher::ClassCache::kReadWrite),
                talos::launcher::MatchVerdict::kOther);
    } else {
      std::ofstream{file} << c.cache;
    }
    Review review{library, 1u << 20};
    const json sessions =
        json::parse(review.Get("/review/sessions.json", "").body);
    ASSERT_EQ(sessions.size(), 1u);
    EXPECT_EQ(sessions[0]["class"], std::string{c.expected});
  }
  std::filesystem::remove_all(library);
}

// Guards which reader a topic with no writer log is read from: the one whose
// log path sorts first (session.h), whatever its records' times. Here
// b_reader's log starts first in merge order and logged the same message
// with another position; picking the first log in merge order shows 2.
// REQ-OPS-41
TEST(ReaderChoiceTest, TopicIsReadFromTheFirstReaderByLogPath) {
  using talos::event::Context;
  using talos::event::EventKind;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;
  const std::string library = Directory() + "/readers";
  std::filesystem::create_directories(library + "/s1");
  const auto write = [&library](const std::string& name, std::int64_t at_ns,
                                double position) {
    using D = talos::Descriptor<review_test::Shot>;
    talos::event::Registration source;
    source.id = 2;
    source.kind = SourceKind::WATCHER;
    source.name = "/x/state";
    source.message_bytes = sizeof(review_test::Shot);
    source.schema = {std::string{D::kName}, D::kHash,
                     std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                                 D::kBfbs.size()}};
    talos::event::log::LogWriter writer{
        library + "/s1/" + name + ".tlog", name,
        talos::event::log::LogWriterOptions{8192, 1, false}, 1};
    writer.start({source}, MonotonicTime::from_nanos(kStartNs));
    const review_test::Shot value{position, 1, review_test::Mode::IDLE, false,
                                  7};
    Context context{};
    context.kind = EventKind::MESSAGE;
    context.source_id = 2;
    context.dispatch_index = 1;
    context.event_time = context.now = MonotonicTime::from_nanos(at_ns);
    context.publish_time = MonotonicTime::from_nanos(kStartNs);
    context.sequence = 1;
    writer.dispatch(context, std::as_bytes(std::span{&value, 1}));
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  };
  write("a_reader", kStartNs + 2 * kMs, 1.0);
  write("b_reader", kStartNs + kMs, 2.0);
  Review review{library, 1u << 20};
  const Response response = review.Get(
      "/review/s1/snapshot", "t_ns=" + std::to_string(kStartNs + kSecond));
  ASSERT_EQ(response.status, 200) << response.body;
  const json channels = json::parse(response.body)["channels"];
  ASSERT_TRUE(channels.contains("x/state/position_m")) << response.body;
  EXPECT_EQ(channels["x/state/position_m"]["value"], 1.0);
  std::filesystem::remove_all(library);
}

// Guards the bound on a snapshot's reads (studio/README.md, "Reviewing a
// session"): a topic published once at the session's start, then only read
// every tick (a watcher's READ records, which hold no value), costs a
// snapshot at the end one second of reading, not a walk back over every
// second its reader touched it.
// REQ-OPS-42
TEST(SnapshotCostTest, TopicPublishedOnceIsReadFromItsOneSecond) {
  using talos::event::Context;
  using talos::event::EventKind;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;
  const std::string library = Directory() + "/once";
  std::filesystem::create_directories(library + "/s1");
  const auto shot = [](std::uint16_t id, std::string name) {
    using D = talos::Descriptor<review_test::Shot>;
    talos::event::Registration source;
    source.id = id;
    source.kind = SourceKind::WATCHER;
    source.name = std::move(name);
    source.message_bytes = sizeof(review_test::Shot);
    source.schema = {std::string{D::kName}, D::kHash,
                     std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                                 D::kBfbs.size()}};
    return source;
  };
  constexpr int kSeconds = 60;
  constexpr int kTicksPerSecond = 50;
  {
    talos::event::Registration tick;
    tick.id = 1;
    tick.kind = SourceKind::TIMER;
    tick.name = "update";
    talos::event::log::LogWriter writer{
        library + "/s1/reader.tlog", "reader",
        talos::event::log::LogWriterOptions{8192, 1, false}, 1};
    writer.start({tick, shot(2, "/once/state"), shot(3, "/every/state")},
                 MonotonicTime::from_nanos(kStartNs));
    const review_test::Shot value{1.5, 1, review_test::Mode::IDLE, false, 7};
    const auto bytes = std::as_bytes(std::span{&value, 1});
    std::uint64_t index = 0;
    for (int k = 0; k < kSeconds * kTicksPerSecond; ++k) {
      const std::int64_t t = kStartNs + k * (kSecond / kTicksPerSecond);
      Context context{};
      context.dispatch_index = ++index;
      context.event_time = context.now = MonotonicTime::from_nanos(t);
      context.publish_time = MonotonicTime::from_nanos(t);
      context.sequence = static_cast<std::uint64_t>(k) + 1;
      context.kind = EventKind::MESSAGE;
      context.source_id = k == 0 ? 2 : 3;
      writer.dispatch(context, bytes);
      context.kind = EventKind::READ;
      context.source_id = 2;
      context.publish_time = MonotonicTime::from_nanos(kStartNs);
      context.sequence = 1;
      writer.dispatch(context, {});
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }
  Review review{library, 1u << 20};
  const Response response =
      review.Get("/review/s1/snapshot",
                 "t_ns=" + std::to_string(kStartNs + kSeconds * kSecond));
  ASSERT_EQ(response.status, 200) << response.body;
  const json channels = json::parse(response.body)["channels"];
  ASSERT_TRUE(channels.contains("once/state/position_m")) << response.body;
  EXPECT_EQ(channels["once/state/position_m"]["sample_ns"],
            std::to_string(kStartNs));
  // Each second holds 50 ticks of two records; the once topic's second and
  // the every topic's last are read, one seek each.
  const std::uint64_t second_bytes =
      kTicksPerSecond *
      (2 * sizeof(talos::event::log::RecordHeader) + sizeof(review_test::Shot));
  EXPECT_EQ(review.last_read().seeks, 2u);
  EXPECT_LE(review.last_read().bytes, 2 * second_bytes);
  std::filesystem::remove_all(library);
}

// Guards the static handler that serves the built Studio: "/" is
// index.html, a file has its content type, and nothing outside the root is
// served, whether named by ".." or encoded.
// REQ-OPS-41
TEST(StaticFileTest, ServesOnlyFilesInsideTheRoot) {
  namespace fs = std::filesystem;
  const fs::path dir = Directory() + "/static";
  fs::create_directories(dir / "root/assets");
  std::ofstream{dir / "root/index.html"} << "<div id=\"root\"></div>";
  std::ofstream{dir / "root/assets/app.js"} << "export {};";
  std::ofstream{dir / "secret.txt"} << "secret";

  const Response index = StaticFile(dir / "root", "/");
  EXPECT_EQ(index.status, 200);
  EXPECT_EQ(index.content_type, "text/html; charset=utf-8");
  EXPECT_EQ(index.body, "<div id=\"root\"></div>");
  const Response script = StaticFile(dir / "root", "/assets/app.js");
  EXPECT_EQ(script.status, 200);
  EXPECT_EQ(script.content_type, "text/javascript");
  for (const std::string url : {"/../secret.txt", "/assets/../../secret.txt",
                                "//etc/passwd", "/missing", "/assets"}) {
    SCOPED_TRACE(url);
    EXPECT_EQ(StaticFile(dir / "root", url).status, 404);
  }
  EXPECT_EQ(StaticFile(dir / "root", "/%2e%2e/secret.txt").status, 400);
  EXPECT_EQ(StaticFile({}, "/").status, 404);
  fs::remove_all(dir);
}

// Off (no pulse) and a centre pulse both have an output of 0; review reads
// them apart by the request's mode channel, its wire value. Dropping the
// mode, as the flattener did for every string, shows both as 0.
// REQ-OPS-41, REQ-SAFE-03
TEST(HardwareChannelsTest, PwmModeReadsOffApartFromACentrePulse) {
  talos::hardware::Command command;
  command.pwm_output_count = 2;
  command.pwm_outputs[0] = {41, talos::hardware::PwmMode::kOff, 0.0};
  command.pwm_outputs[1] = {42, talos::hardware::PwmMode::kPulse, 0.0};
  talos::hardware::Packet packet{};
  packet.size =
      static_cast<std::uint32_t>(talos::hardware::Encode(command, packet.data));
  ASSERT_GT(packet.size, 0u);
  std::vector<HardwareSample> samples;
  FlattenHardware(talos::hardware::kCommandTopic,
                  talos::launcher::HardwareRecord::kCommand,
                  std::as_bytes(std::span{&packet, 1}), samples);
  std::map<std::string, double> values;
  for (const HardwareSample& sample : samples) {
    EXPECT_EQ(sample.kind, Talos::Telemetry::ChannelKind_Double) << sample.name;
    values[sample.name] = talos::telemetry::DoubleFromRaw(sample.raw);
  }
  const std::map<std::string, double> expected{
      {"hw/command/pwm_outputs/41/mode", 0.0},
      {"hw/command/pwm_outputs/41/output", 0.0},
      {"hw/command/pwm_outputs/42/mode", 1.0},
      {"hw/command/pwm_outputs/42/output", 0.0}};
  for (const auto& [name, value] : expected) {
    ASSERT_TRUE(values.contains(name)) << name;
    EXPECT_EQ(values[name], value) << name;
  }
}

// A neutral motor request and a 0 V voltage request both carry demand 0;
// review tells them apart by the request's mode channel, its wire value.
// REQ-OPS-41
TEST(HardwareChannelsTest, MotorModeReadsNeutralApartFromZeroVolts) {
  talos::hardware::Command command;
  command.count = 2;
  command.motors[0] = {1, talos::hardware::Mode::kNeutral, 0, 0.0, 0.0};
  command.motors[1] = {2, talos::hardware::Mode::kVoltage, 0, 0.0, 0.0};
  talos::hardware::Packet packet{};
  packet.size =
      static_cast<std::uint32_t>(talos::hardware::Encode(command, packet.data));
  ASSERT_GT(packet.size, 0u);
  std::vector<HardwareSample> samples;
  FlattenHardware(talos::hardware::kCommandTopic,
                  talos::launcher::HardwareRecord::kCommand,
                  std::as_bytes(std::span{&packet, 1}), samples);
  std::map<std::string, double> values;
  for (const HardwareSample& sample : samples) {
    values[sample.name] = talos::telemetry::DoubleFromRaw(sample.raw);
  }
  const std::map<std::string, double> expected{
      {"hw/command/motors/1/mode",
       static_cast<double>(talos::hardware::Mode::kNeutral)},
      {"hw/command/motors/1/demand", 0.0},
      {"hw/command/motors/2/mode",
       static_cast<double>(talos::hardware::Mode::kVoltage)},
      {"hw/command/motors/2/demand", 0.0}};
  for (const auto& [name, value] : expected) {
    ASSERT_TRUE(values.contains(name)) << name;
    EXPECT_EQ(values[name], value) << name;
  }
}

// Guards timeline.json mode transitions (all 5 modes + FMS flag) and
// REQ-OPS-51: when hardware_not_decoded is true, modes is empty so consumers
// never misread undecoded hardware packets as "disabled".
// REQ-OPS-41, REQ-OPS-51
TEST_F(ReviewTest, TimelineModesAndHardwareNotDecoded) {
  namespace ds = talos::driver_station;
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::EventKind;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;

  // All 5 modes + FMS transition in a small synthetic session.
  const std::string dir = Library() + "/modes";
  std::filesystem::create_directories(dir);
  {
    using P = talos::Descriptor<talos::hardware::Packet>;
    talos::event::Registration source;
    source.id = 2;
    source.kind = SourceKind::SENDER;
    source.name = "/hw/state/driver_station";
    source.message_bytes = sizeof(talos::hardware::Packet);
    source.schema = {std::string{P::kName}, P::kHash, {}};
    tlog::LogWriter writer{dir + "/bridge.tlog", "bridge",
                           tlog::LogWriterOptions{8192, 1, false}, 1};
    writer.start({source}, MonotonicTime::from_nanos(kStartNs));
    const struct {
      std::int64_t offset_ms;
      std::uint32_t flags;
    } packets[] = {
        {0, ds::kDsAttached},
        {1000, ds::kDsAttached | ds::kFmsAttached},
        {2000,
         ds::kDsAttached | ds::kFmsAttached | ds::kEnabled | ds::kAutonomous},
        {3000, ds::kDsAttached | ds::kFmsAttached | ds::kEnabled | ds::kTeleop},
        {4000, ds::kDsAttached | ds::kEnabled | ds::kTest},
        {5000, ds::kDsAttached | ds::kEStop},
    };
    std::uint64_t seq = 0;
    for (const auto& p : packets) {
      ds::DriverStationData data;
      data.fms.flags = p.flags;
      talos::hardware::Packet packet{};
      packet.size = static_cast<std::uint32_t>(ds::Encode(data, packet.data));
      Context ctx{};
      ctx.dispatch_index = ++seq;
      ctx.event_time = ctx.now =
          MonotonicTime::from_nanos(kStartNs + p.offset_ms * kMs);
      writer.send(ctx, 2, std::as_bytes(std::span{&packet, 1}), 0, seq);
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }
  Review review{Library(), 1u << 20};
  const Response modes_resp = review.Get("/review/modes/timeline.json", "");
  ASSERT_EQ(modes_resp.status, 200) << modes_resp.body;
  const json tl = json::parse(modes_resp.body);
  EXPECT_EQ(tl["hardware_not_decoded"], false);
  EXPECT_EQ(tl["modes_dropped"], 0);
  ASSERT_EQ(tl["modes"].size(), 6u);
  EXPECT_EQ(tl["modes"][0]["mode"], "disabled");
  EXPECT_EQ(tl["modes"][0]["fms"], false);
  EXPECT_EQ(tl["modes"][1]["mode"], "disabled");
  EXPECT_EQ(tl["modes"][1]["fms"], true);
  EXPECT_EQ(tl["modes"][2]["mode"], "auto");
  EXPECT_EQ(tl["modes"][2]["fms"], true);
  EXPECT_EQ(tl["modes"][3]["mode"], "teleop");
  EXPECT_EQ(tl["modes"][3]["fms"], true);
  EXPECT_EQ(tl["modes"][4]["mode"], "test");
  EXPECT_EQ(tl["modes"][4]["fms"], false);
  EXPECT_EQ(tl["modes"][5]["mode"], "estop");
  EXPECT_EQ(tl["modes"][5]["start_ns"], std::to_string(kStartNs + 5 * kSecond));
  EXPECT_EQ(tl["modes"][5]["end_ns"], std::to_string(kStartNs + 5 * kSecond));
  std::filesystem::remove_all(dir);

  // Hardware protocol mismatch: hardware_not_decoded is true and modes is [].
  const std::string foreign = Library() + "/foreign_tl";
  testing::WriteFixtureSession(foreign);
  const std::uint16_t other = talos::protocol::kProtocolVersion - 1;
  {
    std::ifstream in{foreign + "/manifest.json"};
    std::string text{std::istreambuf_iterator<char>{in}, {}};
    text.insert(
        1, "\n  \"hardware_protocol_version\": " + std::to_string(other) + ",");
    std::ofstream{foreign + "/manifest.json"} << text;
  }
  const Response foreign_resp =
      review.Get("/review/foreign_tl/timeline.json", "");
  ASSERT_EQ(foreign_resp.status, 200) << foreign_resp.body;
  const json foreign_tl = json::parse(foreign_resp.body);
  EXPECT_EQ(foreign_tl["hardware_not_decoded"], true);
  EXPECT_TRUE(foreign_tl["modes"].empty());
  EXPECT_EQ(foreign_tl["modes_dropped"], 0);
  // Non-hardware alerts and manifest runs are still served.
  EXPECT_EQ(foreign_tl["alerts"].size(), 1u);
  EXPECT_EQ(foreign_tl["runs"].size(), 3u);
  std::filesystem::remove_all(foreign);
}

// Guards alert intervals in timeline.json: an alert raised and cleared closes
// at the clearing record's publish_ns; one still active at session end closes
// at session end_ns.
// REQ-OPS-37, REQ-OPS-41
TEST_F(ReviewTest, TimelineAlertsCoverClearedAndActiveToEnd) {
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;

  const std::string dir = Library() + "/alert_tl";
  std::filesystem::create_directories(dir);
  {
    using D = talos::Descriptor<review_test::Health>;
    talos::event::Registration health;
    health.id = 2;
    health.kind = SourceKind::SENDER;
    health.name = "/thing/status";
    health.message_bytes = sizeof(review_test::Health);
    health.schema = {std::string{D::kName}, D::kHash,
                     std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                                 D::kBfbs.size()}};
    tlog::LogWriter writer{dir + "/thing.tlog", "thing",
                           tlog::LogWriterOptions{8192, 1, false}, 1};
    writer.start({health}, MonotonicTime::from_nanos(kStartNs));
    const struct {
      std::int64_t offset_ms;
      bool active;
    } samples[] = {
        {0, false},    {1000, true}, {2000, true},
        {3000, false}, {5000, true}, {8000, true},
    };
    std::uint64_t seq = 0;
    for (const auto& s : samples) {
      const review_test::Health msg{s.active, !s.active};
      Context ctx{};
      ctx.dispatch_index = ++seq;
      ctx.event_time = ctx.now =
          MonotonicTime::from_nanos(kStartNs + s.offset_ms * kMs);
      writer.send(ctx, 2, std::as_bytes(std::span{&msg, 1}), 0, seq);
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }
  Review review{Library(), 1u << 20};
  const Response resp = review.Get("/review/alert_tl/timeline.json", "");
  ASSERT_EQ(resp.status, 200) << resp.body;
  const json tl = json::parse(resp.body);
  ASSERT_EQ(tl["alerts"].size(), 2u);
  EXPECT_EQ(tl["alerts"][0]["channel"], "thing/status/alert_error_link_lost");
  EXPECT_EQ(tl["alerts"][0]["level"], "error");
  EXPECT_EQ(tl["alerts"][0]["start_ns"],
            std::to_string(kStartNs + 1 * kSecond));
  EXPECT_EQ(tl["alerts"][0]["end_ns"], std::to_string(kStartNs + 3 * kSecond));
  EXPECT_EQ(tl["alerts"][1]["start_ns"],
            std::to_string(kStartNs + 5 * kSecond));
  EXPECT_EQ(tl["alerts"][1]["end_ns"], std::to_string(kStartNs + 8 * kSecond));
  std::filesystem::remove_all(dir);
}

// Guards manifest runs[] mapping via origin_wall_ns, end_wall_ns == 0 -> null,
// runs_dropped preservation, and never guessing runs from .restart<N>.tlog
// filenames when manifest.json has no runs.
// REQ-OPS-41
TEST_F(ReviewTest, TimelineRunsMapFromManifestAndHandleUnfinishedRuns) {
  const std::string dir = Library() + "/runs_tl";
  testing::WriteFixtureSession(dir);
  // Overwrite manifest with an unfinished run (end_wall_ns == 0) and
  // runs_dropped > 0.
  std::ofstream{dir + "/manifest.json"}
      << "{\n  \"session_id\": " << testing::kSessionId
      << ",\n  \"simulation\": true,\n  \"start_wall_ns\": "
      << testing::kStartWallNs << ",\n  \"end_wall_ns\": 0,\n  \"nodes\": [\n"
      << "    {\"name\": \"shooter\", \"runs_dropped\": 3, \"runs\": [\n"
      << "      {\"pid\": 1, \"start_wall_ns\": " << testing::kStartWallNs
      << ", \"end_wall_ns\": " << (testing::kStartWallNs + 2 * kSecond)
      << ", \"reason\": \"stalled\"},\n"
      << "      {\"pid\": 2, \"start_wall_ns\": "
      << (testing::kStartWallNs + 4 * kSecond)
      << ", \"end_wall_ns\": 0, \"reason\": \"running\"}\n"
      << "    ]}\n  ]\n}\n";
  Review review{Library(), 1u << 20};
  const Response resp = review.Get("/review/runs_tl/timeline.json", "");
  ASSERT_EQ(resp.status, 200) << resp.body;
  const json tl = json::parse(resp.body);
  EXPECT_EQ(tl["runs_dropped"], 3);
  ASSERT_EQ(tl["runs"].size(), 2u);
  EXPECT_EQ(tl["runs"][0]["node"], "shooter");
  EXPECT_EQ(tl["runs"][0]["start_ns"], std::to_string(kStartNs));
  EXPECT_EQ(tl["runs"][0]["end_ns"], std::to_string(kStartNs + 2 * kSecond));
  EXPECT_EQ(tl["runs"][0]["reason"], "stalled");
  EXPECT_EQ(tl["runs"][1]["start_ns"], std::to_string(kStartNs + 4 * kSecond));
  EXPECT_TRUE(tl["runs"][1]["end_ns"].is_null());
  EXPECT_EQ(tl["runs"][1]["reason"], "running");
  std::filesystem::remove_all(dir);

  // Removing manifest.json leaves runs empty even though shooter.restart1.tlog
  // is present: never guess restart reasons from filenames.
  const std::string no_manifest = Library() + "/no_manifest_tl";
  testing::WriteFixtureSession(no_manifest);
  std::filesystem::remove(no_manifest + "/manifest.json");
  const Response no_m = review.Get("/review/no_manifest_tl/timeline.json", "");
  ASSERT_EQ(no_m.status, 200) << no_m.body;
  const json no_m_tl = json::parse(no_m.body);
  EXPECT_TRUE(no_m_tl["runs"].empty());
  EXPECT_EQ(no_m_tl["runs_dropped"], 0);
  std::filesystem::remove_all(no_manifest);
}

// Guards REQ-OPS-42 timeline heap caps: pathological flapping past
// kMaxTimelineModes and kMaxTimelineAlerts stays capped at the exact ceiling
// and reports the dropped counts, closing the last kept mode at its actual
// transition rather than falsely extending it to session end.
// REQ-OPS-42
TEST_F(ReviewTest, TimelineCapsFlappingModesAndAlerts) {
  namespace ds = talos::driver_station;
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;

  const std::string dir = Library() + "/flap";
  std::filesystem::create_directories(dir);
  constexpr std::size_t kExtraModes = 5;
  constexpr std::size_t kExtraAlerts = 7;
  {
    using P = talos::Descriptor<talos::hardware::Packet>;
    using H = talos::Descriptor<review_test::Health>;
    talos::event::Registration ds_src;
    ds_src.id = 2;
    ds_src.kind = SourceKind::SENDER;
    ds_src.name = "/hw/state/driver_station";
    ds_src.message_bytes = sizeof(talos::hardware::Packet);
    ds_src.schema = {std::string{P::kName}, P::kHash, {}};
    talos::event::Registration health_src;
    health_src.id = 3;
    health_src.kind = SourceKind::SENDER;
    health_src.name = "/thing/status";
    health_src.message_bytes = sizeof(review_test::Health);
    health_src.schema = {
        std::string{H::kName}, H::kHash,
        std::string{reinterpret_cast<const char*>(H::kBfbs.data()),
                    H::kBfbs.size()}};
    tlog::LogWriter writer{dir + "/flap.tlog", "flap",
                           tlog::LogWriterOptions{65536, 1, false}, 1};
    writer.start({ds_src, health_src}, MonotonicTime::from_nanos(kStartNs));

    std::uint64_t seq = 0;
    const std::size_t total_modes = kMaxTimelineModes + kExtraModes;
    const std::size_t total_alert_edges =
        (kMaxTimelineAlerts + kExtraAlerts) * 2;
    const std::size_t steps = std::max(total_modes, total_alert_edges);
    for (std::size_t i = 0; i < steps; ++i) {
      Context ctx{};
      ctx.dispatch_index = ++seq;
      ctx.event_time = ctx.now = MonotonicTime::from_nanos(
          kStartNs + static_cast<std::int64_t>(i) * kMs);
      if (i < total_modes) {
        ds::DriverStationData data;
        data.fms.flags =
            ds::kDsAttached | ((i % 2 == 1) ? (ds::kEnabled | ds::kTeleop) : 0);
        talos::hardware::Packet packet{};
        packet.size = static_cast<std::uint32_t>(ds::Encode(data, packet.data));
        writer.send(ctx, 2, std::as_bytes(std::span{&packet, 1}), 0, seq);
      }
      if (i < total_alert_edges) {
        const review_test::Health msg{i % 2 == 0, i % 2 == 1};
        writer.send(ctx, 3, std::as_bytes(std::span{&msg, 1}), 0, seq);
      }
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }

  Review review{Library(), 1u << 20};
  const Response resp = review.Get("/review/flap/timeline.json", "");
  ASSERT_EQ(resp.status, 200) << resp.body;
  const json tl = json::parse(resp.body);
  EXPECT_EQ(tl["modes"].size(), kMaxTimelineModes);
  EXPECT_EQ(tl["modes_dropped"], kExtraModes);
  // Last kept mode closed when the first dropped mode began (index
  // kMaxTimelineModes), not stretched to the end of the session.
  EXPECT_EQ(tl["modes"].back()["end_ns"],
            std::to_string(kStartNs +
                           static_cast<std::int64_t>(kMaxTimelineModes) * kMs));
  EXPECT_EQ(tl["alerts"].size(), kMaxTimelineAlerts);
  EXPECT_EQ(tl["alerts_dropped"], kExtraAlerts);
  std::filesystem::remove_all(dir);
}

// A reflection schema for one status struct with three alert bools (offsets
// 0, 1, 2), used to test multi-alert status topics without altering the
// checked-in fixture schema.
std::string MultiAlertSchema() {
  flatbuffers::FlatBufferBuilder fbb;
  const auto bool_type = reflection::CreateType(
      fbb, reflection::BaseType::Bool, reflection::BaseType::None, -1, 0, 1);
  // Fields sorted by name (matching flatc reflection schemas) and id 0..2.
  std::vector<flatbuffers::Offset<reflection::Field>> fields{
      reflection::CreateFieldDirect(fbb, "alert_error_held", bool_type,
                                    /*id=*/0, /*offset=*/0),
      reflection::CreateFieldDirect(fbb, "alert_error_late", bool_type,
                                    /*id=*/1, /*offset=*/1),
      reflection::CreateFieldDirect(fbb, "alert_warning_flap", bool_type,
                                    /*id=*/2, /*offset=*/2),
  };
  std::vector<flatbuffers::Offset<reflection::Object>> objects{
      reflection::CreateObjectDirect(fbb, "review_test.MultiAlert", &fields,
                                     /*is_struct=*/true, /*minalign=*/1,
                                     /*bytesize=*/3),
  };
  std::vector<flatbuffers::Offset<reflection::Enum>> enums;
  reflection::FinishSchemaBuffer(
      fbb, reflection::CreateSchemaDirect(fbb, &objects, &enums));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

// Guards the per-channel alert cap on a multi-alert status topic: a peer
// alert flapping past kMaxTimelineAlerts cannot starve an earlier long-lived
// alert that closes after the flaps or stays active until session end, and
// alerts_dropped counts every dropped interval across per-channel and global
// truncation.
// REQ-OPS-37, REQ-OPS-41, REQ-OPS-42
TEST_F(ReviewTest, TimelineAlertCapPreservesEarlyAlertDespiteFlappingPeer) {
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;

  const std::string dir = Library() + "/multi_alert_flap";
  std::filesystem::create_directories(dir);
  constexpr std::size_t kExtraFlaps = 5;
  constexpr std::size_t kTotalFlaps = kMaxTimelineAlerts + kExtraFlaps;
  const std::int64_t last_ms = 2 + static_cast<std::int64_t>(kTotalFlaps) * 2;
  {
    talos::event::Registration status_src;
    status_src.id = 2;
    status_src.kind = SourceKind::SENDER;
    status_src.name = "/multi/status";
    status_src.message_bytes = 3;
    status_src.schema = {"review_test.MultiAlert", 2, MultiAlertSchema()};
    tlog::LogWriter writer{dir + "/multi.tlog", "multi",
                           tlog::LogWriterOptions{65536, 1, false}, 1};
    writer.start({status_src}, MonotonicTime::from_nanos(kStartNs));

    std::uint64_t seq = 0;
    const auto send = [&](std::int64_t offset_ms, bool held, bool late,
                          bool flap) {
      const std::array<std::byte, 3> payload{
          static_cast<std::byte>(held ? 1 : 0),
          static_cast<std::byte>(late ? 1 : 0),
          static_cast<std::byte>(flap ? 1 : 0),
      };
      Context ctx{};
      ctx.dispatch_index = ++seq;
      ctx.event_time = ctx.now =
          MonotonicTime::from_nanos(kStartNs + offset_ms * kMs);
      writer.send(ctx, 2, payload, 0, seq);
    };

    // alert_error_held opens at 0 ms and stays active through session end.
    // alert_error_late opens at 1 ms and closes at last_ms after all flaps.
    send(0, true, false, false);
    send(1, true, true, false);
    for (std::size_t i = 0; i < kTotalFlaps; ++i) {
      const std::int64_t base_ms = 2 + static_cast<std::int64_t>(i) * 2;
      send(base_ms, true, true, true);
      send(base_ms + 1, true, true, false);
    }
    send(last_ms, true, false, false);
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }

  Review review{Library(), 1u << 20};
  const Response resp =
      review.Get("/review/multi_alert_flap/timeline.json", "");
  std::filesystem::remove_all(dir);
  ASSERT_EQ(resp.status, 200) << resp.body;
  const json tl = json::parse(resp.body);
  ASSERT_EQ(tl["alerts"].size(), kMaxTimelineAlerts);
  // Total intervals = 1 (held) + 1 (late) + kTotalFlaps (flap).
  // Kept = kMaxTimelineAlerts; dropped = kExtraFlaps + 2.
  EXPECT_EQ(tl["alerts_dropped"], kExtraFlaps + 2);

  EXPECT_EQ(tl["alerts"][0]["channel"], "multi/status/alert_error_held");
  EXPECT_EQ(tl["alerts"][0]["level"], "error");
  EXPECT_EQ(tl["alerts"][0]["start_ns"], std::to_string(kStartNs));
  EXPECT_EQ(tl["alerts"][0]["end_ns"],
            std::to_string(kStartNs + last_ms * kMs));

  EXPECT_EQ(tl["alerts"][1]["channel"], "multi/status/alert_error_late");
  EXPECT_EQ(tl["alerts"][1]["level"], "error");
  EXPECT_EQ(tl["alerts"][1]["start_ns"], std::to_string(kStartNs + 1 * kMs));
  EXPECT_EQ(tl["alerts"][1]["end_ns"],
            std::to_string(kStartNs + last_ms * kMs));

  EXPECT_EQ(tl["alerts"][2]["channel"], "multi/status/alert_warning_flap");
  EXPECT_EQ(tl["alerts"][2]["level"], "warning");
  EXPECT_EQ(tl["alerts"][2]["start_ns"], std::to_string(kStartNs + 2 * kMs));
  EXPECT_EQ(tl["alerts"][2]["end_ns"], std::to_string(kStartNs + 3 * kMs));

  const std::int64_t last_kept_flap_ms =
      2 + static_cast<std::int64_t>(kMaxTimelineAlerts - 3) * 2;
  EXPECT_EQ(tl["alerts"].back()["channel"], "multi/status/alert_warning_flap");
  EXPECT_EQ(tl["alerts"].back()["start_ns"],
            std::to_string(kStartNs + last_kept_flap_ms * kMs));
  EXPECT_EQ(tl["alerts"].back()["end_ns"],
            std::to_string(kStartNs + (last_kept_flap_ms + 1) * kMs));
}

// Guards GET /review/<id>/declared.json: serves graph.json verbatim when
// present and 404 plain text when absent.
// REQ-OPS-41, REQ-CONTRACT-29
TEST_F(ReviewTest, DeclaredServesGraphJsonOr404WhenAbsent) {
  Review review{Library(), 1u << 20};
  const Response present = review.Get("/review/s1/declared.json", "");
  ASSERT_EQ(present.status, 200) << present.body;
  EXPECT_EQ(present.content_type, "application/json");
  EXPECT_EQ(present.body, ReadFile(Library() + "/s1/graph.json"));

  const std::string dir = Library() + "/no_graph";
  testing::WriteFixtureSession(dir);
  std::filesystem::remove(dir + "/graph.json");
  const Response absent = review.Get("/review/no_graph/declared.json", "");
  EXPECT_EQ(absent.status, 404);
  EXPECT_EQ(absent.content_type, "text/plain; charset=utf-8");
  std::filesystem::remove_all(dir);
}

// Guards that GET /review/<id>/declared.json refuses a graph.json symlink
// pointing outside the session library with 404 instead of reading it.
// REQ-OPS-41
TEST_F(ReviewTest, DeclaredRefusesSymlinkOutsideLibrary) {
  const std::filesystem::path outside = Directory() + "/outside_graph.json";
  std::ofstream{outside} << R"({"secret":true})";
  const std::string dir = Library() + "/symlink_graph";
  testing::WriteFixtureSession(dir);
  std::filesystem::remove(dir + "/graph.json");
  std::filesystem::create_symlink(outside, dir + "/graph.json");

  Review review{Library(), 1u << 20};
  const Response resp = review.Get("/review/symlink_graph/declared.json", "");
  std::filesystem::remove_all(dir);
  std::filesystem::remove(outside);

  EXPECT_EQ(resp.status, 404);
  EXPECT_EQ(resp.content_type, "text/plain; charset=utf-8");
  EXPECT_EQ(resp.body.find("secret"), std::string::npos) << resp.body;
}

// Guards canonical alert stream selection during Session::Open(): a sender's
// SEND stream beats receive-side MESSAGE streams even when a receiver logs
// earlier or its path sorts first, and when only receivers exist the
// lexicographically smallest log path wins while keeping intervals across that
// process's restarted log files.
// REQ-OPS-37, REQ-OPS-41, REQ-OPS-42
TEST_F(ReviewTest, TimelineAlertsFollowCanonicalStreamAndSkipNonCanonical) {
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::EventKind;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;

  using D = talos::Descriptor<review_test::Health>;
  const auto make_reg = [](SourceKind kind) {
    talos::event::Registration reg;
    reg.id = 2;
    reg.kind = kind;
    reg.name = "/thing/status";
    reg.message_bytes = sizeof(review_test::Health);
    reg.schema = {std::string{D::kName}, D::kHash,
                  std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                              D::kBfbs.size()}};
    return reg;
  };
  const auto write_messages =
      [&](const std::string& path, const std::string& process,
          const std::vector<std::pair<std::int64_t, bool>>& samples) {
        tlog::LogWriter writer{path, process,
                               tlog::LogWriterOptions{8192, 1, false}, 1};
        writer.start({make_reg(SourceKind::WATCHER)},
                     MonotonicTime::from_nanos(kStartNs));
        std::uint64_t seq = 0;
        for (const auto& [offset_ms, active] : samples) {
          const review_test::Health msg{active, !active};
          Context ctx{};
          ctx.kind = EventKind::MESSAGE;
          ctx.source_id = 2;
          ctx.dispatch_index = ++seq;
          ctx.sequence = seq;
          ctx.event_time = ctx.now = ctx.publish_time =
              MonotonicTime::from_nanos(kStartNs + offset_ms * kMs);
          writer.dispatch(ctx, std::as_bytes(std::span{&msg, 1}));
        }
        writer.flush();
        ASSERT_FALSE(writer.failed()) << writer.error();
      };

  // Subcase 1: SEND beats MESSAGE even when receiver path sorts earlier and
  // receives a record before the sender's first SEND.
  {
    const std::string dir = Library() + "/canon_sender";
    std::filesystem::create_directories(dir);
    write_messages(dir + "/alpha.tlog", "alpha",
                   {{0, true}, {500, false}, {1500, true}, {2500, false}});
    {
      tlog::LogWriter writer{dir + "/thing.tlog", "thing",
                             tlog::LogWriterOptions{8192, 1, false}, 1};
      writer.start({make_reg(SourceKind::SENDER)},
                   MonotonicTime::from_nanos(kStartNs));
      std::uint64_t seq = 0;
      for (const auto& [offset_ms, active] :
           std::vector<std::pair<std::int64_t, bool>>{{1000, true},
                                                      {3000, false}}) {
        const review_test::Health msg{active, !active};
        Context ctx{};
        ctx.dispatch_index = ++seq;
        ctx.event_time = ctx.now =
            MonotonicTime::from_nanos(kStartNs + offset_ms * kMs);
        writer.send(ctx, 2, std::as_bytes(std::span{&msg, 1}), 0, seq);
      }
      writer.flush();
      ASSERT_FALSE(writer.failed()) << writer.error();
    }
    Review review{Library(), 1u << 20};
    const Response resp = review.Get("/review/canon_sender/timeline.json", "");
    std::filesystem::remove_all(dir);
    ASSERT_EQ(resp.status, 200) << resp.body;
    const json tl = json::parse(resp.body);
    EXPECT_EQ(tl["alerts_dropped"], 0);
    ASSERT_EQ(tl["alerts"].size(), 1u);
    EXPECT_EQ(tl["alerts"][0]["start_ns"],
              std::to_string(kStartNs + 1 * kSecond));
    EXPECT_EQ(tl["alerts"][0]["end_ns"],
              std::to_string(kStartNs + 3 * kSecond));
  }

  // Subcase 2: Receiver-only topic picks lexicographically smallest log path
  // (alpha) even when beta and gamma log earlier, and preserves alpha's prior
  // intervals when alpha.restart1.tlog (even smaller path, same process) logs.
  {
    const std::string dir = Library() + "/canon_receiver";
    std::filesystem::create_directories(dir);
    write_messages(dir + "/beta.tlog", "beta",
                   {{0, true}, {1000, false}, {4000, true}, {4500, false}});
    write_messages(dir + "/gamma.tlog", "gamma",
                   {{500, true}, {1500, false}, {6500, true}, {7000, false}});
    write_messages(dir + "/alpha.tlog", "alpha", {{2000, true}, {3000, false}});
    write_messages(dir + "/alpha.restart1.tlog", "alpha",
                   {{5000, true}, {6000, false}});
    Review review{Library(), 1u << 20};
    const Response resp =
        review.Get("/review/canon_receiver/timeline.json", "");
    std::filesystem::remove_all(dir);
    ASSERT_EQ(resp.status, 200) << resp.body;
    const json tl = json::parse(resp.body);
    EXPECT_EQ(tl["alerts_dropped"], 0);
    ASSERT_EQ(tl["alerts"].size(), 2u);
    EXPECT_EQ(tl["alerts"][0]["start_ns"],
              std::to_string(kStartNs + 2 * kSecond));
    EXPECT_EQ(tl["alerts"][0]["end_ns"],
              std::to_string(kStartNs + 3 * kSecond));
    EXPECT_EQ(tl["alerts"][1]["start_ns"],
              std::to_string(kStartNs + 5 * kSecond));
    EXPECT_EQ(tl["alerts"][1]["end_ns"],
              std::to_string(kStartNs + 6 * kSecond));
  }
}

// A reflection schema for one struct of `count` bool fields, each named
// `<prefix><0..count-1>`, used to exercise snapshot channel and byte caps.
std::string ManyBoolsSchema(const std::string& struct_name,
                            const std::string& prefix, std::uint16_t count) {
  flatbuffers::FlatBufferBuilder fbb;
  const auto bool_type = reflection::CreateType(
      fbb, reflection::BaseType::Bool, reflection::BaseType::None, -1, 0, 1);
  std::vector<std::pair<std::string, std::uint16_t>> names;
  names.reserve(count);
  for (std::uint16_t i = 0; i < count; ++i) {
    names.emplace_back(prefix + std::to_string(i), i);
  }
  std::sort(names.begin(), names.end());
  std::vector<flatbuffers::Offset<reflection::Field>> fields;
  fields.reserve(count);
  for (const auto& [name, id] : names) {
    fields.push_back(
        reflection::CreateFieldDirect(fbb, name.c_str(), bool_type, id, id));
  }
  std::vector<flatbuffers::Offset<reflection::Object>> objects{
      reflection::CreateObjectDirect(fbb, struct_name.c_str(), &fields,
                                     /*is_struct=*/true, /*minalign=*/1,
                                     /*bytesize=*/count),
  };
  std::vector<flatbuffers::Offset<reflection::Enum>> enums;
  reflection::FinishSchemaBuffer(
      fbb, reflection::CreateSchemaDirect(fbb, &objects, &enums));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

// Guards REQ-OPS-42's snapshot ceiling: a session with more than
// kMaxSnapshotChannels channels returns valid JSON capped at
// kMaxSnapshotChannels, and one whose channel names would exceed
// kMaxSnapshotBytes stops before exceeding kMaxSnapshotBytes while keeping
// valid JSON.
// REQ-OPS-42
TEST_F(ReviewTest, SnapshotStaysWithinChannelAndByteCaps) {
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;

  const auto write_many = [&](const std::string& dir, const std::string& name,
                              const std::string& prefix, std::size_t topics,
                              std::uint16_t fields_per_topic) {
    std::filesystem::create_directories(dir);
    const std::string schema_name = "review_test." + name;
    const std::string bfbs =
        ManyBoolsSchema(schema_name, prefix, fields_per_topic);
    std::vector<talos::event::Registration> sources;
    sources.reserve(topics);
    for (std::size_t t = 0; t < topics; ++t) {
      talos::event::Registration reg;
      reg.id = static_cast<std::uint16_t>(t + 1);
      reg.kind = SourceKind::SENDER;
      reg.name = "/t" + std::to_string(t) + "/state";
      reg.message_bytes = fields_per_topic;
      reg.schema = {schema_name, 100 + t, bfbs};
      sources.push_back(std::move(reg));
    }
    tlog::LogWriter writer{dir + "/many.tlog", "many",
                           tlog::LogWriterOptions{65536, 1, false}, 1};
    writer.start(sources, MonotonicTime::from_nanos(kStartNs));
    const std::vector<std::byte> payload(fields_per_topic, std::byte{1});
    std::uint64_t seq = 0;
    for (std::size_t t = 0; t < topics; ++t) {
      Context ctx{};
      ctx.dispatch_index = ++seq;
      ctx.event_time = ctx.now = MonotonicTime::from_nanos(kStartNs);
      writer.send(ctx, static_cast<std::uint16_t>(t + 1), payload, 0, seq);
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  };

  // Subcase 1: 9,000 short-named channels hit kMaxSnapshotChannels (8,192).
  {
    const std::string dir = Library() + "/snap_channels";
    write_many(dir, "snap_channels", "f", 9, 1000);
    Review review{Library(), 1u << 20};
    const Response resp = review.Get("/review/snap_channels/snapshot",
                                     "t_ns=" + std::to_string(kStartNs));
    std::filesystem::remove_all(dir);
    ASSERT_EQ(resp.status, 200) << resp.body;
    EXPECT_LE(resp.body.size(), kMaxSnapshotBytes);
    const json parsed = json::parse(resp.body);
    EXPECT_EQ(parsed["channels"].size(), kMaxSnapshotChannels);
  }

  // Subcase 2: 9,000 long-named channels (~300 B JSON each) hit
  // kMaxSnapshotBytes (2 MiB) before kMaxSnapshotChannels and stay valid JSON.
  {
    const std::string dir = Library() + "/snap_bytes";
    write_many(dir, "snap_bytes", std::string(240, 'x') + "_", 9, 1000);
    Review review{Library(), 1u << 20};
    const Response resp = review.Get("/review/snap_bytes/snapshot",
                                     "t_ns=" + std::to_string(kStartNs));
    std::filesystem::remove_all(dir);
    ASSERT_EQ(resp.status, 200) << resp.body;
    EXPECT_LE(resp.body.size(), kMaxSnapshotBytes);
    const json parsed = json::parse(resp.body);
    EXPECT_GT(parsed["channels"].size(), 0u);
    EXPECT_LT(parsed["channels"].size(), kMaxSnapshotChannels);
  }
}

// Guards the end_ns == bounds.end_ns alert ambiguity: an alert that genuinely
// clears on the exact final timestamp reports active_at_end == false, while one
// still active at session end reports active_at_end == true (both share the
// same end_ns == bounds.end_ns).
// REQ-OPS-37, REQ-OPS-41
TEST_F(ReviewTest, TimelineAlertsDistinguishClearedAtEndFromActiveAtEnd) {
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;

  const std::string dir = Library() + "/alert_end_ambiguity";
  std::filesystem::create_directories(dir);
  constexpr std::int64_t kEndMs = 4000;
  {
    talos::event::Registration status_src;
    status_src.id = 2;
    status_src.kind = SourceKind::SENDER;
    status_src.name = "/multi/status";
    status_src.message_bytes = 3;
    status_src.schema = {"review_test.MultiAlert", 2, MultiAlertSchema()};
    tlog::LogWriter writer{dir + "/multi.tlog", "multi",
                           tlog::LogWriterOptions{8192, 1, false}, 1};
    writer.start({status_src}, MonotonicTime::from_nanos(kStartNs));
    std::uint64_t seq = 0;
    const auto send = [&](std::int64_t offset_ms, bool held, bool late,
                          bool flap) {
      const std::array<std::byte, 3> payload{
          static_cast<std::byte>(held ? 1 : 0),
          static_cast<std::byte>(late ? 1 : 0),
          static_cast<std::byte>(flap ? 1 : 0),
      };
      Context ctx{};
      ctx.dispatch_index = ++seq;
      ctx.event_time = ctx.now =
          MonotonicTime::from_nanos(kStartNs + offset_ms * kMs);
      writer.send(ctx, 2, payload, 0, seq);
    };
    send(0, true, false, false);
    send(1000, true, true, true);
    send(2000, true, true, false);
    send(kEndMs, true, false, false);  // late clears on exact bounds.end_ns
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }

  Review review{Library(), 1u << 20};
  const Response bounds_resp =
      review.Get("/review/alert_end_ambiguity/bounds.json", "");
  const Response tl_resp =
      review.Get("/review/alert_end_ambiguity/timeline.json", "");
  std::filesystem::remove_all(dir);
  ASSERT_EQ(bounds_resp.status, 200) << bounds_resp.body;
  ASSERT_EQ(tl_resp.status, 200) << tl_resp.body;
  const std::string end_ns = json::parse(bounds_resp.body)["end_ns"];
  ASSERT_EQ(end_ns, std::to_string(kStartNs + kEndMs * kMs));
  const json alerts = json::parse(tl_resp.body)["alerts"];
  ASSERT_EQ(alerts.size(), 3u) << alerts.dump();

  // alert_error_held: [0, 4s], still active at session end.
  EXPECT_EQ(alerts[0]["channel"], "multi/status/alert_error_held");
  EXPECT_EQ(alerts[0]["end_ns"], end_ns);
  ASSERT_TRUE(alerts[0].contains("active_at_end")) << alerts[0].dump();
  EXPECT_EQ(alerts[0]["active_at_end"], true);

  // alert_warning_flap: [1s, 2s], cleared mid-session.
  EXPECT_EQ(alerts[1]["channel"], "multi/status/alert_warning_flap");
  EXPECT_EQ(alerts[1]["end_ns"], std::to_string(kStartNs + 2 * kSecond));
  ASSERT_TRUE(alerts[1].contains("active_at_end")) << alerts[1].dump();
  EXPECT_EQ(alerts[1]["active_at_end"], false);

  // alert_error_late: [1s, 4s], cleared on exact bounds.end_ns.
  EXPECT_EQ(alerts[2]["channel"], "multi/status/alert_error_late");
  EXPECT_EQ(alerts[2]["end_ns"], end_ns);
  ASSERT_TRUE(alerts[2].contains("active_at_end")) << alerts[2].dump();
  EXPECT_EQ(alerts[2]["active_at_end"], false);
}

// Guards bounds.json pose_topic: reads [studio] pose_topic from the session's
// manifest config_path when readable, and reports null without failing open
// when absent or unreadable (e.g. a session copied from a robot).
// REQ-OPS-41
TEST_F(ReviewTest, BoundsExposeConfiguredStudioPoseTopic) {
  const std::string dir = Library() + "/pose_cfg";
  testing::WriteFixtureSession(dir);
  const std::string config_path = dir + "/robot.toml";
  std::ofstream{config_path}
      << "[studio]\npose_topic = \"/alt_odom/state\"\n\n[subsystems]\n";
  {
    std::string manifest = ReadFile(dir + "/manifest.json");
    manifest.insert(1, "\n  \"config_path\": \"" + config_path + "\",");
    std::ofstream{dir + "/manifest.json"} << manifest;
  }

  {
    Review review{Library(), 1u << 20};
    const Response resp = review.Get("/review/pose_cfg/bounds.json", "");
    ASSERT_EQ(resp.status, 200) << resp.body;
    const json bounds = json::parse(resp.body);
    ASSERT_TRUE(bounds.contains("pose_topic")) << resp.body;
    EXPECT_EQ(bounds["pose_topic"], "/alt_odom/state");
  }

  // Removing the config file falls back to null without breaking Session::Open.
  std::filesystem::remove(config_path);
  {
    Review review{Library(), 1u << 20};
    const Response resp = review.Get("/review/pose_cfg/bounds.json", "");
    ASSERT_EQ(resp.status, 200) << resp.body;
    const json bounds = json::parse(resp.body);
    ASSERT_TRUE(bounds.contains("pose_topic")) << resp.body;
    EXPECT_TRUE(bounds["pose_topic"].is_null());
  }
  std::filesystem::remove_all(dir);
}

// Guards REQ-OPS-42 on manifest.json's config_path: Session::Open refuses
// oversized (> 1 MiB) and non-regular (FIFO, symlink to FIFO) configs without
// hanging or allocating unbounded memory, returning pose_topic == null while
// still reading [studio] pose_topic through a single bounded include.
// REQ-OPS-42, REQ-OPS-41
TEST_F(ReviewTest, BoundedConfigReadRefusesOversizedAndNonRegularConfigPath) {
  const auto write_manifest_config = [](const std::string& dir,
                                        const std::string& cfg_path) {
    testing::WriteFixtureSession(dir);
    std::string manifest = ReadFile(dir + "/manifest.json");
    manifest.insert(1, "\n  \"config_path\": \"" + cfg_path + "\",");
    std::ofstream{dir + "/manifest.json"} << manifest;
  };

  // 1. Oversized (> 1 MiB) valid robot.toml yields pose_topic == null.
  {
    const std::string dir = Library() + "/pose_huge";
    const std::string cfg = dir + "/robot.toml";
    write_manifest_config(dir, cfg);
    {
      std::ofstream out{cfg, std::ios::binary};
      out << "[studio]\npose_topic = \"/huge/state\"\n\n[subsystems]\n";
      const std::string pad(1024, 'x');
      for (int i = 0; i < 1025; ++i) out << "# " << pad << '\n';
    }
    Review review{Library(), 1u << 20};
    const Response resp = review.Get("/review/pose_huge/bounds.json", "");
    std::filesystem::remove_all(dir);
    ASSERT_EQ(resp.status, 200) << resp.body;
    const json bounds = json::parse(resp.body);
    ASSERT_TRUE(bounds.contains("pose_topic")) << resp.body;
    EXPECT_TRUE(bounds["pose_topic"].is_null()) << bounds["pose_topic"];
  }

  // 2. Non-regular FIFO (no writer open), symlink to FIFO, and a valid config
  //    whose `include` targets a FIFO all return pose_topic == null without
  //    blocking.
  {
    const std::string dir = Library() + "/pose_fifo";
    const std::string fifo = dir + "/config.fifo";
    const std::string link = dir + "/link.toml";
    const std::string inc_cfg = dir + "/inc.toml";
    write_manifest_config(dir, fifo);
    ASSERT_EQ(mkfifo(fifo.c_str(), 0600), 0);
    std::filesystem::create_symlink(fifo, link);
    std::ofstream{inc_cfg} << "include = \"config.fifo\"\n";

    const auto check_null = [&](const std::string& path) {
      std::string manifest = ReadFile(dir + "/manifest.json");
      const auto pos = manifest.find("\"config_path\"");
      ASSERT_NE(pos, std::string::npos);
      const auto end = manifest.find('\n', pos);
      manifest.replace(pos, end - pos, "\"config_path\": \"" + path + "\",");
      std::ofstream{dir + "/manifest.json"} << manifest;
      Review review{Library(), 1u << 20};
      const Response resp = review.Get("/review/pose_fifo/bounds.json", "");
      ASSERT_EQ(resp.status, 200) << resp.body;
      const json bounds = json::parse(resp.body);
      ASSERT_TRUE(bounds.contains("pose_topic")) << resp.body;
      EXPECT_TRUE(bounds["pose_topic"].is_null()) << path << ": " << bounds;
    };
    check_null(fifo);
    check_null(link);
    check_null(inc_cfg);
    std::filesystem::remove_all(dir);
  }

  // 3. Single bounded `include` still resolves `[studio] pose_topic` from the
  //    base file.
  {
    const std::string dir = Library() + "/pose_inc";
    const std::string cfg = dir + "/robot.toml";
    write_manifest_config(dir, cfg);
    std::ofstream{dir + "/base.toml"}
        << "[studio]\npose_topic = \"/base_odom/state\"\n";
    std::ofstream{cfg} << "include = \"base.toml\"\n";
    Review review{Library(), 1u << 20};
    const Response resp = review.Get("/review/pose_inc/bounds.json", "");
    std::filesystem::remove_all(dir);
    ASSERT_EQ(resp.status, 200) << resp.body;
    const json bounds = json::parse(resp.body);
    EXPECT_EQ(bounds["pose_topic"], "/base_odom/state");
  }
}

// Guards REQ-OPS-41/REQ-OPS-42 publish/receipt skew > 1 s: a message published
// 5 s before or after its event/receipt time is returned by both ReadSeries
// and Snapshot, and Snapshot checks overlapping seconds instead of stopping at
// the first (older-published) record it sees.
// REQ-OPS-41, REQ-OPS-42
TEST_F(ReviewTest, PublishReceiptSkewOverOneSecond) {
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::EventKind;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;

  using D = talos::Descriptor<review_test::Shot>;
  const auto make_reg = [](std::uint16_t id, const char* name) {
    talos::event::Registration reg;
    reg.id = id;
    reg.kind = SourceKind::WATCHER;
    reg.name = name;
    reg.message_bytes = sizeof(review_test::Shot);
    reg.schema = {std::string{D::kName}, D::kHash,
                  std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                              D::kBfbs.size()}};
    return reg;
  };
  const auto write_msg = [](tlog::LogWriter& writer, std::uint16_t id,
                            std::uint64_t& seq, std::int64_t event_ns,
                            std::int64_t publish_ns, double pos) {
    review_test::Shot shot{pos, static_cast<std::uint32_t>(seq),
                           review_test::Mode::IDLE, true, seq};
    Context ctx{};
    ctx.kind = EventKind::MESSAGE;
    ctx.source_id = id;
    ctx.dispatch_index = ++seq;
    ctx.sequence = seq;
    ctx.event_time = ctx.now = MonotonicTime::from_nanos(event_ns);
    ctx.publish_time = MonotonicTime::from_nanos(publish_ns);
    writer.dispatch(ctx, std::as_bytes(std::span{&shot, 1}));
  };

  // Subcase 1: Lead skew (received 5 s AFTER publication) + overlapping
  // earlier-received record with a newer publication time.
  {
    const std::string dir = Library() + "/skew_lead";
    std::filesystem::create_directories(dir);
    {
      tlog::LogWriter writer{dir + "/obs.tlog", "obs",
                             tlog::LogWriterOptions{8192, 1, false}, 1};
      writer.start({make_reg(2, "/lead/state")},
                   MonotonicTime::from_nanos(kStartNs));
      std::uint64_t seq = 0;
      // Received at 4 s, published at 4 s (pos = 40.0).
      write_msg(writer, 2, seq, kStartNs + 4 * kSecond, kStartNs + 4 * kSecond,
                40.0);
      // Received at 7 s, published at 2 s (5 s lead skew, pos = 20.0).
      write_msg(writer, 2, seq, kStartNs + 7 * kSecond, kStartNs + 2 * kSecond,
                20.0);
      writer.flush();
      ASSERT_FALSE(writer.failed()) << writer.error();
    }
    const auto session = Session::Open(dir);
    ASSERT_EQ(session->files().size(), 1u);
    EXPECT_EQ(session->files()[0].max_lead_s, 5);
    EXPECT_EQ(session->files()[0].max_lag_s, 0);
    const auto ref = session->Find("lead/state/position_m");
    ASSERT_TRUE(ref.has_value());

    // ReadSeries over [1s, 3s) finds the record published at 2s (event at 7s).
    const auto series = session->ReadSeries(
        std::span{&*ref, 1}, kStartNs + 1 * kSecond, kStartNs + 3 * kSecond, 2);
    ASSERT_EQ(series.size(), 1u);
    EXPECT_EQ(series[0].count[1], 1u);
    EXPECT_EQ(series[0].last[1], 20.0);

    // Snapshot at 3s finds the record published at 2s (event at 7s).
    const auto snap3 = session->Snapshot(kStartNs + 3 * kSecond);
    const auto it3 =
        std::find_if(snap3.begin(), snap3.end(), [&](const Sample& s) {
          return s.channel.topic == ref->topic &&
                 s.channel.channel == ref->channel;
        });
    ASSERT_NE(it3, snap3.end());
    EXPECT_EQ(ValueOf(session->channel(*ref).kind, it3->raw), 20.0);
    EXPECT_EQ(it3->sample_ns, kStartNs + 2 * kSecond);

    // Snapshot at 7s must NOT stop at event second 7 (publish 2s): it checks
    // earlier overlapping seconds and returns the newer publish at 4s (40.0).
    const auto snap7 = session->Snapshot(kStartNs + 7 * kSecond);
    const auto it7 =
        std::find_if(snap7.begin(), snap7.end(), [&](const Sample& s) {
          return s.channel.topic == ref->topic &&
                 s.channel.channel == ref->channel;
        });
    ASSERT_NE(it7, snap7.end());
    EXPECT_EQ(ValueOf(session->channel(*ref).kind, it7->raw), 40.0);
    EXPECT_EQ(it7->sample_ns, kStartNs + 4 * kSecond);
    std::filesystem::remove_all(dir);
  }

  // Subcase 2: Lag skew (published 5 s AFTER event time) + later event with
  // an older publication time.
  {
    const std::string dir = Library() + "/skew_lag";
    std::filesystem::create_directories(dir);
    {
      tlog::LogWriter writer{dir + "/obs.tlog", "obs",
                             tlog::LogWriterOptions{8192, 1, false}, 1};
      writer.start({make_reg(2, "/lag/state")},
                   MonotonicTime::from_nanos(kStartNs));
      std::uint64_t seq = 0;
      // Event at 7 s, published at 12 s (5 s lag skew, pos = 120.0).
      write_msg(writer, 2, seq, kStartNs + 7 * kSecond, kStartNs + 12 * kSecond,
                120.0);
      // Event at 9 s, published at 9 s (pos = 90.0).
      write_msg(writer, 2, seq, kStartNs + 9 * kSecond, kStartNs + 9 * kSecond,
                90.0);
      writer.flush();
      ASSERT_FALSE(writer.failed()) << writer.error();
    }
    const auto session = Session::Open(dir);
    ASSERT_EQ(session->files().size(), 1u);
    EXPECT_EQ(session->files()[0].max_lead_s, 0);
    EXPECT_EQ(session->files()[0].max_lag_s, 5);
    const auto ref = session->Find("lag/state/position_m");
    ASSERT_TRUE(ref.has_value());

    // ReadSeries over [11s, 13s) widens first_second by max_lag_s to find it.
    const auto series =
        session->ReadSeries(std::span{&*ref, 1}, kStartNs + 11 * kSecond,
                            kStartNs + 13 * kSecond, 2);
    ASSERT_EQ(series.size(), 1u);
    EXPECT_EQ(series[0].count[1], 1u);
    EXPECT_EQ(series[0].last[1], 120.0);

    // Snapshot at 13s must NOT stop at event second 9 (publish 9s): it checks
    // event second 7 (7 + max_lag_s >= 9) and returns publish 12s (120.0).
    const auto snap13 = session->Snapshot(kStartNs + 13 * kSecond);
    const auto it13 =
        std::find_if(snap13.begin(), snap13.end(), [&](const Sample& s) {
          return s.channel.topic == ref->topic &&
                 s.channel.channel == ref->channel;
        });
    ASSERT_NE(it13, snap13.end());
    EXPECT_EQ(ValueOf(session->channel(*ref).kind, it13->raw), 120.0);
    EXPECT_EQ(it13->sample_ns, kStartNs + 12 * kSecond);
    std::filesystem::remove_all(dir);
  }
}

// Guards REQ-OPS-42 lazy overflow bitset for source ids >= 63: Checkpoint stays
// 24 B, logs with <= 63 sources allocate zero overflow memory, and source ids
// >= 63 (including across multiple 64-bit overflow words) are indexed
// independently without aliasing on bit 63.
// REQ-OPS-41, REQ-OPS-42
TEST_F(ReviewTest, SourceIdsAtOrAbove63DoNotAlias) {
  static_assert(sizeof(Checkpoint) == 24);
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;
  using D = talos::Descriptor<review_test::Shot>;

  // Fixture session uses only source ids < 63 -> zero overflow allocation.
  {
    const auto fixture_session = Session::Open(Library() + "/s1");
    for (const LogFile& file : fixture_session->files()) {
      EXPECT_TRUE(file.overflow_source_ids.empty());
      EXPECT_TRUE(file.overflow_bits.empty());
      EXPECT_EQ(file.overflow_words_per_cp, 0u);
    }
  }

  const std::string dir = Library() + "/high_sources";
  std::filesystem::create_directories(dir);
  const auto make_reg = [](std::uint16_t id, const std::string& name) {
    talos::event::Registration reg;
    reg.id = id;
    reg.kind = SourceKind::SENDER;
    reg.name = name;
    reg.message_bytes = sizeof(review_test::Shot);
    reg.schema = {std::string{D::kName}, D::kHash,
                  std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                              D::kBfbs.size()}};
    return reg;
  };
  {
    std::vector<talos::event::Registration> regs;
    regs.push_back(make_reg(64, "/src64/state"));
    regs.push_back(make_reg(70, "/src70/state"));
    // Add 65 extra overflow sources (ids 80..144) to cross a 64-bit overflow
    // word boundary, verifying multi-word bitset growth too.
    for (std::uint16_t id = 80; id <= 144; ++id) {
      regs.push_back(make_reg(id, "/pad" + std::to_string(id) + "/state"));
    }
    tlog::LogWriter writer{dir + "/high.tlog", "high",
                           tlog::LogWriterOptions{65536, 1, false}, 1};
    writer.start(regs, MonotonicTime::from_nanos(kStartNs));
    std::uint64_t seq = 0;
    const auto send = [&](std::uint16_t id, std::int64_t offset_s, double pos) {
      review_test::Shot shot{pos, static_cast<std::uint32_t>(seq),
                             review_test::Mode::IDLE, true, seq};
      Context ctx{};
      ctx.dispatch_index = ++seq;
      ctx.event_time = ctx.now =
          MonotonicTime::from_nanos(kStartNs + offset_s * kSecond);
      writer.send(ctx, id, std::as_bytes(std::span{&shot, 1}), 0, seq);
    };
    // Source 64 publishes ONLY at second 1.
    send(64, 1, 64.5);
    // Source 70 publishes at every second 2..10; pad sources (ids 80..144,
    // crossing a 64-bit overflow word boundary) publish at second 3 in order.
    for (std::int64_t s = 2; s <= 10; ++s) {
      send(70, s, 70.0 + static_cast<double>(s));
      if (s == 3) {
        for (std::uint16_t id = 80; id <= 144; ++id) {
          send(id, 3, static_cast<double>(id));
        }
      }
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }

  const auto session = Session::Open(dir);
  ASSERT_EQ(session->files().size(), 1u);
  EXPECT_GE(session->files()[0].overflow_words_per_cp, 2u);

  const auto ref64 = session->Find("src64/state/position_m");
  const auto ref70 = session->Find("src70/state/position_m");
  const auto ref144 = session->Find("pad144/state/position_m");
  ASSERT_TRUE(ref64.has_value());
  ASSERT_TRUE(ref70.has_value());
  ASSERT_TRUE(ref144.has_value());

  // ReadSeries for source 64 over [5s, 10s) must skip completely (0 seeks)
  // even though source 70 (>= 63) is present in every second 5..10.
  {
    ReadStats stats{};
    const auto series =
        session->ReadSeries(std::span{&*ref64, 1}, kStartNs + 5 * kSecond,
                            kStartNs + 10 * kSecond, 5, &stats);
    EXPECT_EQ(stats.seeks, 0u);
    EXPECT_EQ(stats.records, 0u);
    for (const auto c : series[0].count) EXPECT_EQ(c, 0u);
  }

  // ReadSeries for source 144 (in second overflow word) over [5s, 10s) skips
  // too, and over [3s, 4s) finds its exact value.
  {
    ReadStats skip_stats{};
    (void)session->ReadSeries(std::span{&*ref144, 1}, kStartNs + 5 * kSecond,
                              kStartNs + 10 * kSecond, 2, &skip_stats);
    EXPECT_EQ(skip_stats.seeks, 0u);

    const auto hit =
        session->ReadSeries(std::span{&*ref144, 1}, kStartNs + 3 * kSecond,
                            kStartNs + 4 * kSecond, 1);
    EXPECT_EQ(hit[0].count[0], 1u);
    EXPECT_EQ(hit[0].last[0], 144.0);
  }

  // Snapshot at 10s jumps directly to each source's latest checkpoint:
  // second 10 (src70), second 3 (pad80..144), second 1 (src64) = 3 seeks total
  // instead of walking seconds 9, 8, 7, 6, 5, 4, 2!
  {
    ReadStats stats{};
    const auto snap = session->Snapshot(kStartNs + 10 * kSecond, &stats);
    EXPECT_EQ(stats.seeks, 3u);
    const auto get_val = [&](ChannelRef ref) -> std::optional<double> {
      const auto it =
          std::find_if(snap.begin(), snap.end(), [&](const Sample& s) {
            return s.channel.topic == ref.topic &&
                   s.channel.channel == ref.channel;
          });
      if (it == snap.end()) return std::nullopt;
      return ValueOf(session->channel(ref).kind, it->raw);
    };
    EXPECT_EQ(get_val(*ref64), 64.5);
    EXPECT_EQ(get_val(*ref70), 80.0);
    EXPECT_EQ(get_val(*ref144), 144.0);
  }
  std::filesystem::remove_all(dir);
}

// Guards GET /review/<id>/manifest.json: serves manifest.json verbatim when
// present, 404 when absent, symlinked, or oversized (> kMaxManifestBytes), and
// 400 on path traversal.
// REQ-OPS-41, REQ-OPS-42
TEST_F(ReviewTest, ManifestServesManifestJsonOr404WhenAbsentOrUnsafe) {
  Review review{Library(), 1u << 20};
  const Response present = review.Get("/review/s1/manifest.json", "");
  ASSERT_EQ(present.status, 200) << present.body;
  EXPECT_EQ(present.content_type, "application/json");
  EXPECT_EQ(present.body, ReadFile(Library() + "/s1/manifest.json"));

  // Missing manifest.json -> 404 plain text.
  {
    const std::string dir = Library() + "/no_manifest";
    testing::WriteFixtureSession(dir);
    std::filesystem::remove(dir + "/manifest.json");
    const Response absent = review.Get("/review/no_manifest/manifest.json", "");
    std::filesystem::remove_all(dir);
    EXPECT_EQ(absent.status, 404);
    EXPECT_EQ(absent.content_type, "text/plain; charset=utf-8");
  }

  // Symlink pointing outside library -> 404 without leaking target contents.
  {
    const std::filesystem::path outside =
        Directory() + "/outside_manifest.json";
    std::ofstream{outside} << R"({"secret":true})";
    const std::string dir = Library() + "/symlink_manifest";
    testing::WriteFixtureSession(dir);
    std::filesystem::remove(dir + "/manifest.json");
    std::filesystem::create_symlink(outside, dir + "/manifest.json");
    const Response sym =
        review.Get("/review/symlink_manifest/manifest.json", "");
    std::filesystem::remove_all(dir);
    std::filesystem::remove(outside);
    EXPECT_EQ(sym.status, 404);
    EXPECT_EQ(sym.content_type, "text/plain; charset=utf-8");
    EXPECT_EQ(sym.body.find("secret"), std::string::npos) << sym.body;
  }

  // Oversized manifest.json (> kMaxManifestBytes) -> 404.
  {
    const std::string dir = Library() + "/huge_manifest";
    testing::WriteFixtureSession(dir);
    {
      std::ofstream out{dir + "/manifest.json", std::ios::binary};
      out << "{\"pad\":\"";
      const std::string chunk(65536, 'a');
      for (std::size_t i = 0; i < 65; ++i) out << chunk;
      out << "\"}";
    }
    const Response huge = review.Get("/review/huge_manifest/manifest.json", "");
    std::filesystem::remove_all(dir);
    EXPECT_EQ(huge.status, 404);
    EXPECT_EQ(huge.content_type, "text/plain; charset=utf-8");
  }

  // Path traversal session id -> 400.
  EXPECT_EQ(review.Get("/review/../manifest.json", "").status, 400);
}

// Guards REQ-OPS-41 version-7 compatibility: a session whose `.tlog` files
// carry FileHeader::version == 7 opens with no conversion and serves HTTP 200
// with identical decoded bounds, channels, timeline, snapshot and series as
// the version-8 session.
// REQ-OPS-41
TEST_F(ReviewTest, Version7LogsOpenAndDecodeIdenticallyToVersion8) {
  const std::string dir = Library() + "/v7";
  testing::WriteFixtureSession(dir);
  std::size_t rewritten = 0;
  for (const auto& entry : std::filesystem::directory_iterator{dir}) {
    if (entry.path().extension() != ".tlog") continue;
    std::fstream file{entry.path(),
                      std::ios::in | std::ios::out | std::ios::binary};
    const std::uint32_t version = 7;
    static_assert(talos::event::log::OLDEST_READABLE_VERSION == 7);
    file.seekp(offsetof(talos::event::log::FileHeader, version));
    file.write(reinterpret_cast<const char*>(&version), sizeof version);
    ASSERT_TRUE(file) << entry.path();
    ++rewritten;
  }
  ASSERT_GT(rewritten, 0u);

  Review review{Library(), 1u << 20};
  const std::string end = std::to_string(kStartNs + 10 * kSecond + 2 * kMs);
  const std::vector<std::pair<std::string, std::string>> endpoints{
      {"bounds.json", ""},
      {"channels.json", ""},
      {"timeline.json", ""},
      {"snapshot", "t_ns=" + end},
      {"series",
       "c=shooter/state/position_m,hw/state/driver_station/"
       "match_time_s,hw/request/shooter/motors/1/demand&start_ns=" +
           std::to_string(kStartNs) + "&end_ns=" + end + "&points=16"},
  };
  for (const auto& [endpoint, query] : endpoints) {
    SCOPED_TRACE(endpoint);
    const Response v8 = review.Get("/review/s1/" + endpoint, query);
    const Response v7 = review.Get("/review/v7/" + endpoint, query);
    ASSERT_EQ(v8.status, 200) << v8.body;
    ASSERT_EQ(v7.status, 200) << v7.body;
    EXPECT_EQ(v7.body, v8.body);
  }
  std::filesystem::remove_all(dir);
}

// Reflection schema with `depth` levels of nested structs: t.D00 (root, depth
// 0) -> t.D01 (depth 1) -> ... -> t.D<depth> (depth `depth`, holding one bool
// field `leaf`). Object names are zero-padded so they stay sorted for
// FlatBuffers LookupByKey.
std::string NestedStructSchema(std::size_t depth) {
  flatbuffers::FlatBufferBuilder fbb;
  std::vector<flatbuffers::Offset<reflection::Object>> objects;
  objects.reserve(depth + 1);
  const auto obj_name = [](std::size_t i) {
    return std::string{"t.D"} + (i < 10 ? "0" : "") + std::to_string(i);
  };
  for (std::size_t i = 0; i <= depth; ++i) {
    std::vector<flatbuffers::Offset<reflection::Field>> fields;
    if (i < depth) {
      const auto child_type = reflection::CreateType(
          fbb, reflection::BaseType::Obj, reflection::BaseType::None,
          static_cast<std::int32_t>(i + 1), 0, 1);
      fields.push_back(
          reflection::CreateFieldDirect(fbb, "inner", child_type, 0, 0));
    } else {
      const auto bool_type =
          reflection::CreateType(fbb, reflection::BaseType::Bool,
                                 reflection::BaseType::None, -1, 0, 1);
      fields.push_back(
          reflection::CreateFieldDirect(fbb, "leaf", bool_type, 0, 0));
    }
    objects.push_back(reflection::CreateObjectDirect(
        fbb, obj_name(i).c_str(), &fields, /*is_struct=*/true, /*minalign=*/1,
        /*bytesize=*/1));
  }
  std::vector<flatbuffers::Offset<reflection::Enum>> enums;
  reflection::FinishSchemaBuffer(
      fbb, reflection::CreateSchemaDirect(fbb, &objects, &enums));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

// Guards REQ-OPS-42 schema and record payload caps through Review:
//   - nesting depth 17 is refused (no channels) while depth 16 is accepted;
//   - 2049 topic channels are refused (no channels) while 2048 are accepted;
//   - a record whose payload_bytes is message_bytes + 1 (even with a valid
//     CRC32) is rejected, failing that session (422).
// Throughout, the healthy sibling session `s1` still returns 200.
// REQ-OPS-42
TEST_F(ReviewTest, SchemaDepthChannelAndRecordPayloadCapsFailOnlyTheirSession) {
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::EventKind;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;

  const auto write_single = [](const std::string& dir,
                               talos::event::Registration source,
                               std::span<const std::byte> payload) {
    std::filesystem::create_directories(dir);
    source.id = 2;
    source.kind = SourceKind::SENDER;
    tlog::LogWriter writer{dir + "/node.tlog", "node",
                           tlog::LogWriterOptions{65536, 1, false}, 1};
    writer.start({source}, MonotonicTime::from_nanos(kStartNs));
    if (!payload.empty()) {
      Context tick{};
      tick.dispatch_index = 1;
      tick.event_time = tick.now = MonotonicTime::from_nanos(kStartNs);
      writer.send(tick, 2, payload, 0, 1);
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  };

  // 1. Nesting depth 16 (accepted) vs depth 17 (refused).
  {
    const std::byte one{1};
    talos::event::Registration d16;
    d16.name = "/depth16/state";
    d16.message_bytes = 1;
    d16.schema = {"t.D00", 2016, NestedStructSchema(16)};
    write_single(Library() + "/depth16", d16, std::span{&one, 1});

    talos::event::Registration d17;
    d17.name = "/depth17/state";
    d17.message_bytes = 1;
    d17.schema = {"t.D00", 2017, NestedStructSchema(17)};
    write_single(Library() + "/depth17", d17, std::span{&one, 1});
  }

  // 2. Topic channels: 2048 (kMaxTopicChannels, accepted) vs 2049 (refused).
  {
    constexpr std::uint16_t kCap =
        static_cast<std::uint16_t>(talos::telemetry::kMaxTopicChannels);
    static_assert(kCap == 2048);

    talos::event::Registration at_cap;
    at_cap.name = "/cap2048/state";
    at_cap.message_bytes = kCap;
    at_cap.schema = {"review_test.Cap2048", 2048,
                     ManyBoolsSchema("review_test.Cap2048", "f", kCap)};
    const std::vector<std::byte> payload2048(kCap, std::byte{1});
    write_single(Library() + "/cap2048", at_cap, payload2048);

    talos::event::Registration over_cap;
    over_cap.name = "/cap2049/state";
    over_cap.message_bytes = kCap + 1;
    over_cap.schema = {"review_test.Cap2049", 2049,
                       ManyBoolsSchema("review_test.Cap2049", "f", kCap + 1)};
    const std::vector<std::byte> payload2049(kCap + 1, std::byte{1});
    write_single(Library() + "/cap2049", over_cap, payload2049);
  }

  // 3. Record claiming message_bytes + 1 with a valid CRC32.
  {
    using D = talos::Descriptor<review_test::Shot>;
    talos::event::Registration shot;
    shot.name = "/over/state";
    shot.message_bytes = sizeof(review_test::Shot);
    shot.schema = {std::string{D::kName}, D::kHash,
                   std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                               D::kBfbs.size()}};
    write_single(Library() + "/over_msg", shot, {});

    std::vector<std::byte> oversized(sizeof(review_test::Shot) + 1,
                                     std::byte{0x2A});
    tlog::RecordHeader hdr{};
    hdr.kind = static_cast<std::uint16_t>(EventKind::SEND);
    hdr.source_id = 2;
    hdr.dispatch_index = 1;
    hdr.event_time_ns = hdr.now_ns = hdr.publish_time_ns = kStartNs;
    hdr.sequence = 1;
    const auto stored = tlog::SealRecord(hdr, oversized);
    ASSERT_EQ(hdr.payload_bytes, sizeof(review_test::Shot) + 1);
    std::ofstream out{Library() + "/over_msg/node.tlog",
                      std::ios::binary | std::ios::app};
    out.write(reinterpret_cast<const char*>(&hdr), sizeof hdr);
    out.write(reinterpret_cast<const char*>(stored.data()),
              static_cast<std::streamsize>(stored.size()));
    ASSERT_TRUE(out);
  }

  Review review{Library(), 1u << 20};

  const Response ok_d16 = review.Get("/review/depth16/channels.json", "");
  ASSERT_EQ(ok_d16.status, 200) << ok_d16.body;
  const json d16_channels = json::parse(ok_d16.body)["channels"];
  ASSERT_EQ(d16_channels.size(), 1u) << ok_d16.body;
  const std::string d16_name = d16_channels[0]["name"];
  EXPECT_TRUE(d16_name.starts_with("depth16/state/")) << d16_name;
  EXPECT_TRUE(d16_name.ends_with("/leaf")) << d16_name;

  ::testing::internal::CaptureStderr();
  const Response bad_d17 = review.Get("/review/depth17/channels.json", "");
  const std::string d17_err = ::testing::internal::GetCapturedStderr();
  ASSERT_EQ(bad_d17.status, 200) << bad_d17.body;
  EXPECT_TRUE(json::parse(bad_d17.body)["channels"].empty()) << bad_d17.body;
  EXPECT_NE(d17_err.find("is not reviewed"), std::string::npos) << d17_err;

  const Response ok_2048 = review.Get("/review/cap2048/channels.json", "");
  ASSERT_EQ(ok_2048.status, 200) << ok_2048.body;
  EXPECT_EQ(json::parse(ok_2048.body)["channels"].size(), 2048u);

  ::testing::internal::CaptureStderr();
  const Response bad_2049 = review.Get("/review/cap2049/channels.json", "");
  const std::string cap_err = ::testing::internal::GetCapturedStderr();
  ASSERT_EQ(bad_2049.status, 200) << bad_2049.body;
  EXPECT_TRUE(json::parse(bad_2049.body)["channels"].empty()) << bad_2049.body;
  EXPECT_NE(cap_err.find("is not reviewed"), std::string::npos) << cap_err;

  EXPECT_EQ(review.Get("/review/over_msg/bounds.json", "").status, 422);

  // Healthy sibling session stays 200 throughout.
  EXPECT_EQ(review.Get("/review/s1/bounds.json", "").status, 200);
  EXPECT_EQ(review.Get("/review/s1/channels.json", "").status, 200);

  for (const char* name :
       {"depth16", "depth17", "cap2048", "cap2049", "over_msg"}) {
    std::filesystem::remove_all(Library() + "/" + name);
  }
}

// Guards REQ-OPS-51 codec-rejection refusal when a session has no recorded
// protocol version or records this build's kProtocolVersion: a single
// codec-rejected `/hw/state` packet beside a decodable Driver Station packet
// refuses every hardware channel (no `hw/` channels in channels.json,
// `hardware_not_decoded` naming kProtocolVersion in bounds.json,
// `hardware_not_decoded: true` and `modes: []` in timeline.json, and one
// refusal line on stderr), while still serving non-hardware channels.
// REQ-OPS-51
TEST_F(ReviewTest, CodecRejectedHardwarePacketRefusesAllHardwareChannels) {
  namespace ds = talos::driver_station;
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;
  using P = talos::Descriptor<talos::hardware::Packet>;
  using S = talos::Descriptor<review_test::Shot>;

  const std::string build_ver =
      std::to_string(talos::protocol::kProtocolVersion);

  for (const bool with_matching_manifest : {false, true}) {
    const std::string id =
        with_matching_manifest ? "hw_reject_match" : "hw_reject_noversion";
    SCOPED_TRACE(id);
    const std::string dir = Library() + "/" + id;
    std::filesystem::create_directories(dir);
    if (with_matching_manifest) {
      std::ofstream{dir + "/manifest.json"}
          << "{\n  \"session_id\": 1,\n  \"hardware_protocol_version\": "
          << build_ver << ",\n  \"nodes\": []\n}\n";
    }
    {
      talos::event::Registration ds_src;
      ds_src.id = 2;
      ds_src.kind = SourceKind::SENDER;
      ds_src.name = "/hw/state/driver_station";
      ds_src.message_bytes = sizeof(talos::hardware::Packet);
      ds_src.schema = {std::string{P::kName}, P::kHash, {}};

      talos::event::Registration hw_src;
      hw_src.id = 3;
      hw_src.kind = SourceKind::SENDER;
      hw_src.name = "/hw/state";
      hw_src.message_bytes = sizeof(talos::hardware::Packet);
      hw_src.schema = {std::string{P::kName}, P::kHash, {}};

      talos::event::Registration shot_src;
      shot_src.id = 4;
      shot_src.kind = SourceKind::SENDER;
      shot_src.name = "/shooter/state";
      shot_src.message_bytes = sizeof(review_test::Shot);
      shot_src.schema = {
          std::string{S::kName}, S::kHash,
          std::string{reinterpret_cast<const char*>(S::kBfbs.data()),
                      S::kBfbs.size()}};

      tlog::LogWriter writer{dir + "/bridge.tlog", "bridge",
                             tlog::LogWriterOptions{8192, 1, false}, 1};
      writer.start({ds_src, hw_src, shot_src},
                   MonotonicTime::from_nanos(kStartNs));

      // Decodable Driver Station packet (enabled in teleop).
      ds::DriverStationData ds_data{};
      ds_data.fms.flags = ds::kDsAttached | ds::kEnabled | ds::kTeleop;
      talos::hardware::Packet ds_pkt{};
      ds_pkt.size =
          static_cast<std::uint32_t>(ds::Encode(ds_data, ds_pkt.data));
      ASSERT_GT(ds_pkt.size, 0u);

      // Codec-rejected /hw/state packet (truncated / invalid payload).
      talos::hardware::Packet bad_state{};
      bad_state.size = 4;
      bad_state.data[0] = bad_state.data[1] = bad_state.data[2] =
          bad_state.data[3] = 0xFF;

      // Healthy non-hardware topic.
      const review_test::Shot shot{2.5, 1, review_test::Mode::IDLE, true, 1};

      Context ctx{};
      ctx.dispatch_index = 1;
      ctx.event_time = ctx.now = MonotonicTime::from_nanos(kStartNs);
      writer.send(ctx, 2, std::as_bytes(std::span{&ds_pkt, 1}), 0, 1);
      ctx.dispatch_index = 2;
      ctx.event_time = ctx.now = MonotonicTime::from_nanos(kStartNs + kMs);
      writer.send(ctx, 3, std::as_bytes(std::span{&bad_state, 1}), 0, 1);
      ctx.dispatch_index = 3;
      ctx.event_time = ctx.now = MonotonicTime::from_nanos(kStartNs + 2 * kMs);
      writer.send(ctx, 4, std::as_bytes(std::span{&shot, 1}), 0, 1);
      writer.flush();
      ASSERT_FALSE(writer.failed()) << writer.error();
    }

    Review review{Library(), 1u << 20};
    ::testing::internal::CaptureStderr();
    const Response channels =
        review.Get("/review/" + id + "/channels.json", "");
    const std::string stderr_text = ::testing::internal::GetCapturedStderr();
    ASSERT_EQ(channels.status, 200) << channels.body;

    // Exactly one refusal line on stderr naming kProtocolVersion and /hw/state.
    EXPECT_EQ(std::count(stderr_text.begin(), stderr_text.end(), '\n'), 1)
        << stderr_text;
    EXPECT_NE(stderr_text.find("hardware packets not decoded:"),
              std::string::npos)
        << stderr_text;
    EXPECT_NE(stderr_text.find(build_ver), std::string::npos) << stderr_text;
    EXPECT_NE(stderr_text.find("/hw/state"), std::string::npos) << stderr_text;

    const json listed = json::parse(channels.body);
    bool has_shooter = false;
    for (const auto& ch : listed["channels"]) {
      const std::string name = ch["name"];
      EXPECT_FALSE(name.starts_with("hw/")) << name;
      if (name == "shooter/state/position_m") has_shooter = true;
    }
    EXPECT_TRUE(has_shooter) << channels.body;

    const Response bounds_resp =
        review.Get("/review/" + id + "/bounds.json", "");
    ASSERT_EQ(bounds_resp.status, 200) << bounds_resp.body;
    const json bounds = json::parse(bounds_resp.body);
    ASSERT_TRUE(bounds["hardware_not_decoded"].is_string()) << bounds;
    const std::string reason = bounds["hardware_not_decoded"];
    EXPECT_NE(reason.find(build_ver), std::string::npos) << reason;
    EXPECT_NE(reason.find("/hw/state"), std::string::npos) << reason;

    const Response tl_resp = review.Get("/review/" + id + "/timeline.json", "");
    ASSERT_EQ(tl_resp.status, 200) << tl_resp.body;
    const json tl = json::parse(tl_resp.body);
    EXPECT_EQ(tl["hardware_not_decoded"], true);
    EXPECT_TRUE(tl["modes"].empty()) << tl.dump();

    std::filesystem::remove_all(dir);
  }
}

// Current resident set size of this process in bytes, or 0 if unavailable.
std::uint64_t CurrentResidentBytes() {
#if defined(__linux__)
  std::ifstream statm{"/proc/self/statm"};
  std::uint64_t size_pages = 0;
  std::uint64_t rss_pages = 0;
  if (!(statm >> size_pages >> rss_pages)) return 0;
  const long page_bytes = ::sysconf(_SC_PAGESIZE);
  if (page_bytes <= 0) return 0;
  return rss_pages * static_cast<std::uint64_t>(page_bytes);
#elif defined(__APPLE__)
  mach_task_basic_info_data_t info{};
  mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
  if (::task_info(::mach_task_self(), MACH_TASK_BASIC_INFO,
                  reinterpret_cast<task_info_t>(&info),
                  &count) != KERN_SUCCESS) {
    return 0;
  }
  return info.resident_size;
#else
  return 0;
#endif
}

// A reflection schema for a 1024-byte struct with one double field at offset 0.
std::string WideStructSchema(const std::string& struct_name) {
  flatbuffers::FlatBufferBuilder fbb;
  const auto double_type = reflection::CreateType(
      fbb, reflection::BaseType::Double, reflection::BaseType::None, -1, 0, 8);
  std::vector<flatbuffers::Offset<reflection::Field>> fields{
      reflection::CreateFieldDirect(fbb, "position_m", double_type, 0, 0)};
  std::vector<flatbuffers::Offset<reflection::Object>> objects{
      reflection::CreateObjectDirect(fbb, struct_name.c_str(), &fields,
                                     /*is_struct=*/true, /*minalign=*/8,
                                     /*bytesize=*/1024)};
  std::vector<flatbuffers::Offset<reflection::Enum>> enums;
  reflection::FinishSchemaBuffer(
      fbb, reflection::CreateSchemaDirect(fbb, &objects, &enums));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

// REQ-OPS-42: opening and querying a multi-MiB `.tlog` streams through
// a fixed read window and never reads the file whole, so both the session's
// index and the process's RSS delta stay far below the file size on disk.
TEST_F(ReviewTest, MultiMiBLogStreamsWithoutReadingFileWhole) {
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;

  const std::string dir = Library() + "/multi_mib";
  std::filesystem::create_directories(dir);
  constexpr std::size_t kPayloadBytes = 1024;
  constexpr std::uint64_t kRecords = 24'000;  // ~25.7 MiB on disk
  const std::string log_path = dir + "/wide.tlog";
  {
    talos::event::Registration reg;
    reg.id = 2;
    reg.kind = SourceKind::SENDER;
    reg.name = "/wide/state";
    reg.message_bytes = kPayloadBytes;
    reg.schema = {"review_test.Wide", 991024,
                  WideStructSchema("review_test.Wide")};
    tlog::LogWriter writer{log_path, "wide",
                           tlog::LogWriterOptions{1u << 20, 4, false}, 1};
    writer.start({reg}, MonotonicTime::from_nanos(kStartNs));
    // Fill with non-zero bytes so trailing-zero stripping keeps all 1024 bytes.
    std::array<std::byte, kPayloadBytes> payload;
    payload.fill(std::byte{0x5A});
    for (std::uint64_t i = 0; i < kRecords; ++i) {
      const double pos = static_cast<double>(i) * 0.25;
      std::memcpy(payload.data(), &pos, sizeof pos);
      Context ctx{};
      ctx.dispatch_index = i + 1;
      ctx.event_time = ctx.now = MonotonicTime::from_nanos(
          kStartNs + static_cast<std::int64_t>(i) * kMs);
      writer.send(ctx, 2, payload, 0, i + 1);
    }
    writer.flush();
    ASSERT_FALSE(writer.failed()) << writer.error();
  }

  const std::uint64_t file_bytes = std::filesystem::file_size(log_path);
  ASSERT_GE(file_bytes, std::uint64_t{24} << 20);

  const std::uint64_t rss_before = CurrentResidentBytes();
  const auto session = Session::Open(dir);
  const auto ref = session->Find("wide/state/position_m");
  ASSERT_TRUE(ref.has_value());
  const auto series = session->ReadSeries(
      std::span{&*ref, 1}, session->start_ns(), session->end_ns() + 1, 16);
  ASSERT_EQ(series.size(), 1u);
  EXPECT_DOUBLE_EQ(series[0].last.back(),
                   static_cast<double>(kRecords - 1) * 0.25);
  const auto snap = session->Snapshot(session->end_ns());
  ASSERT_FALSE(snap.empty());
  const std::uint64_t rss_after = CurrentResidentBytes();

  // Index is 24 B per log-file second (~24 seconds -> ~576 B), far below the
  // multi-MiB file size.
  EXPECT_LE(session->index_bytes(), session->log_seconds() * 24u + 1024u);
  EXPECT_LT(session->index_bytes(), file_bytes / 1000u);
  if (rss_before > 0 && rss_after > rss_before) {
    // Reading the file whole would grow RSS by >= file_bytes (~25+ MiB);
    // streaming stays far below a quarter of the file size.
    EXPECT_LT(rss_after - rss_before, file_bytes / 4u)
        << "rss_before=" << rss_before << " rss_after=" << rss_after
        << " file_bytes=" << file_bytes;
  }

  std::filesystem::remove_all(dir);
}

// Reflection schema for `/hw/status` carrying a `protocol_version_id` ushort.
std::string BridgeStatusProtocolSchema() {
  flatbuffers::FlatBufferBuilder fbb;
  const auto ushort_type = reflection::CreateType(
      fbb, reflection::BaseType::UShort, reflection::BaseType::None, -1, 0, 2);
  std::vector<flatbuffers::Offset<reflection::Field>> fields{
      reflection::CreateFieldDirect(fbb, "protocol_version_id", ushort_type, 0,
                                    0)};
  std::vector<flatbuffers::Offset<reflection::Object>> objects{
      reflection::CreateObjectDirect(fbb, "bridge.BridgeStatus", &fields,
                                     /*is_struct=*/true, /*minalign=*/2,
                                     /*bytesize=*/2)};
  std::vector<flatbuffers::Offset<reflection::Enum>> enums;
  reflection::FinishSchemaBuffer(
      fbb, reflection::CreateSchemaDirect(fbb, &objects, &enums));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

// Guards REQ-OPS-51: when a session's bridge `/hw/status` protocol_version_id
// disagrees with its manifest's hardware_protocol_version, studio_review prints
// one warning naming both versions and reads the session under the bridge's
// version (decoding hardware channels when the bridge matches this build even
// if the manifest disagrees, and refusing them when the bridge is foreign even
// if the manifest matches this build).
// REQ-OPS-51
TEST_F(ReviewTest, BridgeVsManifestProtocolDisagreementWarnsAndUsesBridge) {
  namespace ds = talos::driver_station;
  namespace tlog = talos::event::log;
  using talos::event::Context;
  using talos::event::MonotonicTime;
  using talos::event::SourceKind;
  using P = talos::Descriptor<talos::hardware::Packet>;

  const std::uint16_t build = talos::protocol::kProtocolVersion;
  const std::uint16_t other = static_cast<std::uint16_t>(build - 1);
  const std::string status_bfbs = BridgeStatusProtocolSchema();

  struct Case {
    const char* id;
    std::uint16_t bridge_ver;
    std::uint16_t manifest_ver;
    bool expect_decoded;
  };
  for (const Case& c : {
           Case{"disagree_bridge_ok", build, other, true},
           Case{"disagree_bridge_foreign", other, build, false},
       }) {
    SCOPED_TRACE(c.id);
    const std::string dir = Library() + "/" + c.id;
    std::filesystem::create_directories(dir);
    std::ofstream{dir + "/manifest.json"}
        << "{\n  \"session_id\": 1,\n  \"hardware_protocol_version\": "
        << c.manifest_ver << ",\n  \"nodes\": []\n}\n";
    {
      talos::event::Registration status_src;
      status_src.id = 2;
      status_src.kind = SourceKind::SENDER;
      status_src.name = "/hw/status";
      status_src.message_bytes = sizeof(std::uint16_t);
      status_src.schema = {"bridge.BridgeStatus", 77001, status_bfbs};

      talos::event::Registration ds_src;
      ds_src.id = 3;
      ds_src.kind = SourceKind::SENDER;
      ds_src.name = "/hw/state/driver_station";
      ds_src.message_bytes = sizeof(talos::hardware::Packet);
      ds_src.schema = {std::string{P::kName}, P::kHash, {}};

      tlog::LogWriter writer{dir + "/hardware_node.tlog", "hardware_node",
                             tlog::LogWriterOptions{8192, 1, false}, 1};
      writer.start({status_src, ds_src}, MonotonicTime::from_nanos(kStartNs));

      const std::uint16_t ver = c.bridge_ver;
      Context ctx{};
      ctx.dispatch_index = 1;
      ctx.event_time = ctx.now = MonotonicTime::from_nanos(kStartNs);
      writer.send(ctx, 2, std::as_bytes(std::span{&ver, 1}), 0, 1);

      ds::DriverStationData ds_data{};
      ds_data.fms.flags = ds::kDsAttached | ds::kEnabled | ds::kTeleop;
      ds_data.fms.match_time_s = 42.0f;
      talos::hardware::Packet ds_pkt{};
      ds_pkt.size =
          static_cast<std::uint32_t>(ds::Encode(ds_data, ds_pkt.data));
      ASSERT_GT(ds_pkt.size, 0u);
      ctx.dispatch_index = 2;
      ctx.event_time = ctx.now = MonotonicTime::from_nanos(kStartNs + kMs);
      writer.send(ctx, 3, std::as_bytes(std::span{&ds_pkt, 1}), 0, 1);

      writer.flush();
      ASSERT_FALSE(writer.failed()) << writer.error();
    }

    Review review{Library(), 1u << 20};
    ::testing::internal::CaptureStderr();
    const Response channels =
        review.Get("/review/" + std::string{c.id} + "/channels.json", "");
    const std::string stderr_text = ::testing::internal::GetCapturedStderr();
    ASSERT_EQ(channels.status, 200) << channels.body;

    // One disagreement warning line naming both versions and reading under the
    // bridge's version.
    const std::string bridge_str = std::to_string(c.bridge_ver);
    const std::string manifest_str = std::to_string(c.manifest_ver);
    EXPECT_NE(
        stderr_text.find("bridge recorded hardware protocol " + bridge_str),
        std::string::npos)
        << stderr_text;
    EXPECT_NE(stderr_text.find("its launcher " + manifest_str),
              std::string::npos)
        << stderr_text;
    EXPECT_NE(stderr_text.find("read under the bridge's"), std::string::npos)
        << stderr_text;
    if (c.expect_decoded) {
      EXPECT_EQ(std::count(stderr_text.begin(), stderr_text.end(), '\n'), 1)
          << stderr_text;
    }

    const json listed = json::parse(channels.body);
    bool has_ds_channel = false;
    bool has_status_channel = false;
    for (const auto& ch : listed["channels"]) {
      const std::string name = ch["name"];
      if (name == "hw/state/driver_station/match_time_s") {
        has_ds_channel = true;
      }
      if (name == "hw/status/protocol_version_id") {
        has_status_channel = true;
      }
      if (!c.expect_decoded) {
        EXPECT_FALSE(name.starts_with("hw/state/") ||
                     name.starts_with("hw/request/") ||
                     name.starts_with("hw/command"))
            << name;
      }
    }
    EXPECT_TRUE(has_status_channel) << channels.body;
    EXPECT_EQ(has_ds_channel, c.expect_decoded) << channels.body;

    const json bounds = json::parse(
        review.Get("/review/" + std::string{c.id} + "/bounds.json", "").body);
    if (c.expect_decoded) {
      EXPECT_TRUE(bounds["hardware_not_decoded"].is_null()) << bounds;
    } else {
      ASSERT_TRUE(bounds["hardware_not_decoded"].is_string()) << bounds;
      const std::string reason = bounds["hardware_not_decoded"];
      EXPECT_NE(reason.find(bridge_str), std::string::npos) << reason;
      EXPECT_NE(reason.find(std::to_string(build)), std::string::npos)
          << reason;
    }
    std::filesystem::remove_all(dir);
  }
}

}  // namespace
}  // namespace studio::review
