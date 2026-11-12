#include "studio/review/review.h"

#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cmath>
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

#include "nlohmann/json.hpp"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "studio/review/series_cache.h"
#include "studio/review/testdata/fixture.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_writer.h"

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
// `value` (which returns false for a record without one).
std::vector<Expected> Values(
    const std::vector<json>& records, const std::string& process,
    const std::string& kind, const std::string& topic,
    const std::function<bool(const json&, double&)>& value) {
  std::vector<Expected> out;
  for (const json& r : records) {
    if (r["process"] != process || r["kind"] != kind || r["topic"] != topic) {
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
// a skipped segment or restart, or a damaged record decoded anyway, changes
// a count, a value or a time. Every channel shares the same bucket edges,
// dividing [start_ns, end_ns) evenly: a shot published exactly at end_ns is
// in no bucket.
// REQ-OPS-41
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
  constexpr std::size_t kCapacity = 16 * 1024;
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
    const std::uint32_t version = talos::event::log::FORMAT_VERSION - 1;
    file.seekp(offsetof(talos::event::log::FileHeader, version));
    file.write(reinterpret_cast<const char*>(&version), sizeof version);
  }
  Review review{Library(), 1u << 20};
  const Response response = review.Get("/review/old/bounds.json", "");
  EXPECT_EQ(response.status, 422);
  EXPECT_NE(response.body.find(path), std::string::npos) << response.body;
  for (const std::uint32_t version : {talos::event::log::FORMAT_VERSION - 1,
                                      talos::event::log::FORMAT_VERSION}) {
    EXPECT_NE(response.body.find("version " + std::to_string(version)),
              std::string::npos)
        << response.body;
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
    golden = std::filesystem::path{runfiles->Rlocation(
                                       std::getenv("TALOS_GOLDEN_BOUNDS"))}
                 .parent_path();
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

}  // namespace
}  // namespace studio::review
