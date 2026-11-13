#pragma once

// One recorded session opened for review: its index, its channels, and the
// reads a review request makes (a series over a range, a snapshot at an
// instant), with no socket in sight.
//
// Time. Every time here (start_ns, end_ns, a sample's time, a range) is on
// the clock the merge orders records by: the main processor's monotonic
// clock, which stamps each record's event time and each message's
// publication time (talOS/events/log/format.h). A value's time is always
// its publication time (RecordHeader::publish_time_ns), never when a reader
// received it or when the log was read.
//
// Index. Open() streams the session once, merged (launcher::MergeLogs, one
// read window per log), and keeps per log file one Checkpoint per second of
// event time that holds a record: where that second's first record starts
// (an offset meaningful only with its file's path) and which of the file's
// sources recorded a value (an accepted SEND or a MESSAGE record) that
// second. That is 24 B per log-file second whatever the record rate, and
// nothing per record: every later read seeks to a checkpoint
// (event::log::VisitLog's start_offset) and reads only the seconds it needs,
// through one window.
//
// Values. A topic is read from one process's logs, all its segments and
// restarts: the writer's, whose accepted SEND records
// (event::log::IsAcceptedSend) hold every message it published, else the
// MESSAGE records of the reader whose log path sorts first among those that
// logged one (event::RecordedSession orders logs by path too, but takes the
// first whose manifest registers the topic, values or not). A value is
// decoded as log_dump decodes it:
// a talos_messages struct (and a target envelope's header and message) by
// its recorded schema, named by talOS/telemetry:channels as live telemetry
// names it; the Driver Station packet as telemetry reads it; the other
// hardware packets by log_dump's own decoder (hardware_channels.h).
//
// A read selects records by publication time but walks a file by event
// time, so it reads one second either side of the range: a message received
// or sent more than a second from its publication time is not found.

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "talOS/events/context.h"
#include "talOS/launcher/hardware_record.h"
#include "talOS/telemetry/channels.h"

namespace studio::review {

// One second of one log file: the second (event time, floor of ns / 1e9),
// where its first record starts, and bit i set when source id i (ids from
// 63 on share bit 63) has a value record (a SEND its transport accepted, or a
// MESSAGE) in it; a failed send reached no reader and is no value. A READ or
// HANDLER_EXIT record sets no bit: a watcher read every tick holds no new
// value, so it must not make a snapshot read that second.
struct Checkpoint {
  std::int64_t second{0};
  std::uint64_t offset{0};
  std::uint64_t sources{0};
};
static_assert(sizeof(Checkpoint) == 24, "REQ-OPS-42: 24 B per log-file second");

struct LogFile {
  std::string path;
  std::string process;
  std::vector<Checkpoint> checkpoints;
};

enum class Decoder : std::uint8_t {
  kStruct,         // channels read at their offsets (structs, envelopes)
  kDriverStation,  // telemetry::ReadDriverStation
  kHardware,       // FlattenHardware
};

struct Topic {
  std::string name;
  Decoder decoder{Decoder::kStruct};
  talos::launcher::HardwareRecord hardware{
      talos::launcher::HardwareRecord::kNone};
  // Channels in dictionary order. A hardware channel has no offset.
  std::vector<talos::telemetry::Channel> channels;
  // Which records hold its values: SEND (the writer's) or MESSAGE.
  talos::event::EventKind kind{talos::event::EventKind::SEND};
  std::string process;  // whose logs
  struct Source {
    std::size_t file{0};
    std::uint16_t source_id{0};
  };
  std::vector<Source> sources;  // in file order
};

struct ChannelRef {
  std::uint32_t topic{0};
  std::uint32_t channel{0};
};

// A channel's samples over [start_ns, end_ns), cut into equal buckets (the
// same edges for every channel of a request): per
// bucket the smallest, largest and last sample (by publication time) and
// how many; `t_ns` is the last sample's publication time. An empty bucket
// has count 0 and NaN for the rest; one whose every sample is NaN keeps its
// count and t_ns, with NaN min, max and last.
struct Series {
  std::vector<double> t_ns, min, max, last;
  std::vector<std::uint32_t> count;
  std::size_t bytes() const {
    return count.size() * (4 * sizeof(double) + sizeof(std::uint32_t));
  }
};

// A channel's value at or before an instant, with its publication time.
struct Sample {
  ChannelRef channel;
  std::uint64_t raw{0};  // as a telemetry page carries it
  std::int64_t sample_ns{0};
  // False when the topic's record at sample_ns holds no reading of this
  // channel (a hardware value that is not finite, or a field it lacks).
  bool reading{true};
};

// What a read walked, from its seeks onward.
struct ReadStats {
  std::uint64_t records{0};  // intact records visited
  std::uint64_t bytes{0};    // their headers and stored payloads
  std::uint64_t seeks{0};    // reads started at a checkpoint
};

class Session {
 public:
  // Indexes the session directory `dir` (every *.tlog in it). Throws
  // std::runtime_error naming the file for a log that cannot be read: one
  // of another format version (naming both versions), another session's,
  // or none at all. A damaged log is read as log_dump reads it, its damage
  // printed as log_dump prints it.
  static std::unique_ptr<Session> Open(const std::filesystem::path& dir);

  // The earliest and latest event time of any record.
  std::int64_t start_ns() const { return start_ns_; }
  std::int64_t end_ns() const { return end_ns_; }
  // The wall clock at zero on the session's clock, so a time t_ns is at
  // wall time origin_wall_ns + t_ns: from the header of the log that started
  // first (its start_wall_ns minus its start_monotonic_ns), the clock the
  // launcher's manifest records runs in.
  std::int64_t origin_wall_ns() const { return origin_wall_ns_; }

  const std::vector<LogFile>& files() const { return files_; }
  const std::vector<Topic>& topics() const { return topics_; }
  const talos::telemetry::Channel& channel(ChannelRef ref) const {
    return topics_[ref.topic].channels[ref.channel];
  }
  std::optional<ChannelRef> Find(std::string_view name) const;

  // The index's checkpoints: what grows with the session's length.
  std::size_t index_bytes() const;
  std::size_t log_seconds() const;

  // One Series per channel of `channels`, in that order, of `points`
  // buckets. Requires end_ns > start_ns and points > 0.
  std::vector<Series> ReadSeries(std::span<const ChannelRef> channels,
                                 std::int64_t start_ns, std::int64_t end_ns,
                                 std::uint32_t points,
                                 ReadStats* stats = nullptr) const;

  // Every channel of each topic with a record published at or before
  // `t_ns`, from its latest such record, by topic then channel; a topic with
  // none is absent. Per log file it reads, once each, only the seconds that
  // hold a value record of one of its topics' sources at or before
  // floor(t_ns) + 1 s, latest first, stopping per source at the first that
  // yields its value: normally one second per source however long ago it was
  // last published (studio/README.md, "Reviewing a session").
  std::vector<Sample> Snapshot(std::int64_t t_ns,
                               ReadStats* stats = nullptr) const;

 private:
  Session() = default;

  // Decodes one record of `topic` into raw samples: raw[i] for channel i,
  // present[i] whether it has one.
  bool Decode(const Topic& topic, std::span<const std::byte> payload,
              std::vector<std::uint64_t>& raw,
              std::vector<std::uint8_t>& present) const;

  std::vector<LogFile> files_;
  std::vector<Topic> topics_;
  std::unordered_map<std::string, ChannelRef> by_name_;
  // Per hardware topic: channel name -> index.
  std::vector<std::unordered_map<std::string, std::uint32_t>> hardware_index_;
  std::int64_t start_ns_{0};
  std::int64_t end_ns_{0};
  std::int64_t origin_wall_ns_{0};
};

// The value of a raw telemetry sample of `kind` as a double (a bucket's).
double ValueOf(talos::telemetry::ChannelKind kind, std::uint64_t raw);

}  // namespace studio::review
