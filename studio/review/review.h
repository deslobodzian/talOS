#pragma once

// The review API (studio/README.md "Reviewing a session"), as a function of
// a request's path and query: no socket, so all of it is tested in process.
// main.cc only carries requests to Review::Get and its answers back.
//
//   GET /review/sessions.json
//       [{"id","start_wall_ns","end_wall_ns","class","files":[{"path","bytes"}]}]
//       newest first; class is match, other or unknown (RecordedMatch,
//       read-only: nothing is ever written into the library)
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
//       every channel; an empty bucket has count 0 and NaN elsewhere
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
  std::map<std::string, std::unique_ptr<Session>, std::less<>> sessions_;
  // Per session id: its files' total bytes when classified, and the class.
  std::map<std::string, std::pair<std::uintmax_t, std::string_view>> classes_;
};

}  // namespace studio::review
