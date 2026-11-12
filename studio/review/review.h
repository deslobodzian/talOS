#pragma once

// The review API (studio/README.md "Reviewing a session"), as a function of
// a request's path and query: no socket, so all of it is tested in process.
// main.cc carries /review/* requests to Review::Get (after http.h's Host
// check) and its answers back; every other path is a static file.
//
//   GET /review/sessions.json
//       [{"id","start_wall_ns","end_wall_ns","class","files":[{"path","bytes"}]}]
//       newest first; class is match, other or unknown, from the manifest
//       and the launcher's session_class file only (no log is read, and
//       nothing is ever written into the library)
//   GET /review/<id>/channels.json
//       the bridge's /channels.json document ("alert" comes with slice S3a)
//   GET /review/<id>/bounds.json       {"start_ns","end_ns","origin_wall_ns"}
//       wall time of a session time t_ns is origin_wall_ns + t_ns
//   GET /review/<id>/snapshot?t_ns=
//       {"t_ns","channels":{name:{"value","sample_ns"}}}
//       value as the bridge's /telemetry.json writes a sample, or null for
//       no reading; a channel with no sample at or before t_ns is absent
//   GET /review/<id>/series?c=a,b&start_ns=&end_ns=&points=N
//       application/octet-stream, little-endian: u32 channels, u32 buckets,
//       then per channel f64 t_ns[buckets], f64 min[], f64 max[], f64 last[],
//       u32 count[]; buckets divide [start_ns, end_ns) evenly, the same for
//       every channel; an empty bucket has count 0 and NaN elsewhere, and
//       one whose every sample is NaN its count and NaN elsewhere but t_ns;
//       400 past kMaxSeriesChannels, kMaxSeriesPoints or kMaxSeriesValues
//
// A session that cannot be read (a log of another format version, no
// intact record) answers 422 with the reason; other sessions are unaffected.
//
// Times are nanoseconds on the session's monotonic clock (session.h); a
// value's time is its publication time. In JSON every 64-bit nanosecond
// value is a decimal string, as the bridge sends timestamp_ns.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "studio/review/series_cache.h"
#include "studio/review/session.h"

namespace studio::review {

// The most channels one series request may name, and the most buckets.
inline constexpr std::size_t kMaxSeriesChannels = 256;
inline constexpr std::uint32_t kMaxSeriesPoints = 20'000;
// The most values (channels x points) one series request may ask for: a
// plot of 16 channels at 16,384 px. A request past it is refused (400).
inline constexpr std::size_t kMaxSeriesValues = std::size_t{1} << 18;

// The heap one series request of `channels` x `points` holds while it is
// answered, beyond the cache: each value's decoded bucket (Series, 36 B) and
// last-sample time (8 B), then its 36 B in the response, after an 8 B header.
inline constexpr std::size_t SeriesRequestBytes(std::size_t channels,
                                                std::size_t points) {
  return 8 + channels * points *
                 (4 * sizeof(double) + sizeof(std::uint32_t) +
                  sizeof(std::int64_t) + 36);
}
// One request's ceiling: 20 MiB and 8 B.
inline constexpr std::size_t kMaxSeriesRequestBytes =
    SeriesRequestBytes(1, kMaxSeriesValues);

// The most sessions kept open (indexed) at once; opening one more closes
// the least recently used, whose index is rebuilt if it is opened again.
inline constexpr std::size_t kMaxOpenSessions = 4;

struct Response {
  int status{200};
  std::string content_type{"application/json"};
  std::string body;
};

// Whether `id` can name a session directory of a library: one path segment
// of letters, digits, '_', '-' and '.', not starting with '.', so it never
// names a parent, a hidden file or a path outside the library.
bool ValidSessionId(std::string_view id);

// The query's parameters in order, names and values percent-decoded ('+'
// as a space). False for a malformed escape.
bool ParseQuery(std::string_view query,
                std::vector<std::pair<std::string, std::string>>& out);

class Review {
 public:
  Review(std::filesystem::path library, std::size_t cache_bytes);

  Response Get(std::string_view path, std::string_view query);

  const SeriesCache& cache() const { return cache_; }
  // The sessions open now: at most kMaxOpenSessions.
  std::size_t open_sessions() const { return sessions_.size(); }
  // What the last snapshot or series request read from the logs.
  const ReadStats& last_read() const { return last_read_; }

 private:
  // The opened session `id`, indexing it on first use; null with `error`
  // set to the response for an unknown id or a session that cannot be read.
  const Session* Open(std::string_view id, Response& error);

  Response Sessions();
  Response Channels(const Session& session) const;
  Response Bounds(const Session& session) const;
  Response Snapshot(const Session& session, std::string_view query);
  Response Series(std::string_view id, const Session& session,
                  std::string_view query);

  std::filesystem::path library_;
  SeriesCache cache_;
  ReadStats last_read_;
  struct OpenSession {
    std::unique_ptr<Session> session;
    std::uint64_t used{0};  // the request that last used it
  };
  std::map<std::string, OpenSession, std::less<>> sessions_;
  std::uint64_t requests_{0};
};

}  // namespace studio::review
