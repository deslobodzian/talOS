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
// second, and per GAP record one Gap. That is 24 B per log-file second
// whatever the record rate, plus 48 B per gap, and nothing per record: a
// GAP record makes no checkpoint and holds no value, so a read finds in a
// gap's span only what the log holds there (nothing of the process that
// dropped it), as for any stretch a topic was not published. Every later
// read seeks to a checkpoint
// (event::log::VisitLog's start_offset) and reads only the seconds it needs,
// through one window.
//
// Values. A topic is read from one process's logs, all its segments and
// restarts: the writer's, whose accepted SEND records
// (event::log::IsAcceptedSend) hold every message it published, else the
// MESSAGE records of the reader whose log path sorts first among those that
// logged one (event::RecordedSession orders logs by path too (a log's
// segments by number), but takes the first whose manifest registers the
// topic, values or not). A value is
// decoded as log_dump decodes it:
// a talos_messages struct (and a target envelope's header and message) by
// its recorded schema, named by talOS/telemetry:channels as live telemetry
// names it; the Driver Station packet as telemetry reads it; the other
// hardware packets by log_dump's own decoder (hardware_channels.h).
//
// Hardware packets are served only from a session recorded under this
// build's hardware protocol, decided once while indexing as log_dump decides
// it (talos::launcher::HardwareProtocolCheck: the version the session's
// bridge recorded on /hw/status, else the one manifest.json records, then
// this build's codecs on every packet). Otherwise no hardware topic is
// served, the Driver Station's included, and hardware_not_decoded() holds
// log_dump's line, also printed on stderr.
//
// A read selects records by publication time while walking a file by event
// time: each LogFile tracks its observed maximum second-level lead and lag
// between checkpoint event time and publication time so reads widen their
// event-time scan window to cover any skew, and keeps a bounded, lazy
// out-of-line presence bitset for source ids >= 63 (0 bytes when all source
// ids are < 63; capped at kMaxOverflowSources and kMaxOverflowBytesPerFile per
// file, falling back to Checkpoint::sources bit 63 past either cap) so
// Checkpoint stays 24 B and heap stays bounded (REQ-OPS-42).

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

// One GAP record of a log file (talOS/events/log/format.h): where its writer
// dropped records. start_ns and end_ns are its LogGap's start_ns and
// resumed_ns: every record it dropped named a time in [start_ns, end_ns],
// and end_ns is when capture resumed, or, for one closed at the stop, when
// the writer was stopped. The counts are its GapRecord's (zero for a GAP
// whose payload is not one, which no writer makes). 48 B, beside the
// checkpoints (REQ-OPS-42).
struct Gap {
  std::int64_t start_ns{0};
  std::int64_t end_ns{0};
  std::uint64_t records{0};
  std::uint64_t dispatches{0};
  std::uint64_t sends{0};
  std::uint32_t file{0};  // index into Session::files()
  bool closed_at_flush{false};
};
static_assert(sizeof(Gap) == 48, "REQ-OPS-42: 48 B per GAP record");

// Fixed ceilings on the timeline kept per open session (REQ-OPS-42): 96 KiB
// each for modes and alerts (4096 x 24 B) and at most 4096 runs from
// manifest.json, regardless of session length or flapping rate. While
// Session::Open() scans logs, alerts are capped at kMaxTimelineAlerts per
// alert channel (4096 x 16 B = 64 KiB transient per alert channel on each
// topic's single canonical stream; non-canonical streams are skipped or
// evicted immediately; close order equals start order within one channel)
// before merging into the global earliest kMaxTimelineAlerts intervals by
// start_ns, so a flapping alert cannot drop an earlier alert on the same
// status topic. Excess items are dropped and counted in *_dropped().
inline constexpr std::size_t kMaxTimelineModes = 4096;
inline constexpr std::size_t kMaxTimelineAlerts = 4096;
inline constexpr std::size_t kMaxTimelineRuns = 4096;
// Maximum UTF-8 byte length kept for each RunInterval's node name and reason
// read from manifest.json (REQ-OPS-42; 5x NAMING.md's 24 B node name cap).
inline constexpr std::size_t kMaxRunStringBytes = 128;

// Fixed ceilings on total channels and channel metadata bytes (name + unit +
// labels) across all topics in one open session (REQ-OPS-42): well above live
// telemetry's 2048-channel dictionary cap and kMaxSnapshotChannels (8192),
// while bounding per-session heap and /channels.json size even for crafted
// multi-topic logs. A session exceeding either cap is refused in Session::Open
// (HTTP 422 for that session only).
inline constexpr std::size_t kMaxSessionChannels = 16384;
inline constexpr std::size_t kMaxSessionChannelBytes = std::size_t{8} << 20;

// Fixed ceiling on channels returned by one Session::Snapshot (REQ-OPS-42):
// well above live telemetry's 2048-channel dictionary cap, while bounding
// snapshot memory even for a crafted session with huge channel counts.
inline constexpr std::size_t kMaxSnapshotChannels = 8192;

// Fixed ceilings on LogFile out-of-line presence bitsets for source ids >= 63
// (REQ-OPS-42): at most kMaxOverflowSources distinct sources (4 words = 32
// B/checkpoint) and at most kMaxOverflowBytesPerFile bytes (32 KiB) per file
// (falling back conservatively to Checkpoint::sources bit 63 past either
// per-file ceiling), and at most kMaxSessionOverflowBytes (8 MiB) across all
// files in one session (refusing the session in Session::Open with HTTP 422).
inline constexpr std::size_t kMaxOverflowWordsPerCheckpoint = 4;
inline constexpr std::size_t kMaxOverflowSources =
    kMaxOverflowWordsPerCheckpoint * 64;
inline constexpr std::size_t kMaxOverflowBytesPerFile = std::size_t{32} << 10;
inline constexpr std::size_t kMaxSessionOverflowBytes = std::size_t{8} << 20;

enum class ModeKind : std::uint8_t {
  kDisabled,
  kTeleop,
  kAuto,
  kTest,
  kEstop,
};
std::string_view ModeKindName(ModeKind mode);

struct ModeInterval {
  std::int64_t start_ns{0};
  std::int64_t end_ns{0};
  ModeKind mode{ModeKind::kDisabled};
  bool fms{false};
};
static_assert(sizeof(ModeInterval) == 24,
              "REQ-OPS-42: 24 B per timeline mode interval");

struct LogFile {
  std::string path;
  std::string process;
  std::vector<Checkpoint> checkpoints;
  // Maximum second-level skew observed across value records in this file:
  // max_lead_s = max(0, checkpoint.second - FloorSecond(publish_ns)),
  // max_lag_s  = max(0, FloorSecond(publish_ns) - checkpoint.second).
  std::int64_t max_lead_s{0};
  std::int64_t max_lag_s{0};
  // Lazy, bounded out-of-line presence bitset for source ids >= 63, allocated
  // only when a reviewed value record with source_id >= 63 is indexed (0 bytes
  // for <= 63 sources; capped at kMaxOverflowSources and
  // kMaxOverflowBytesPerFile). When overflow_capped is true, any source_id >=
  // 63 absent from overflow_source_ids or any checkpoint past overflow_bits
  // falls back to Checkpoint::sources bit 63.
  std::vector<std::uint16_t> overflow_source_ids;
  std::vector<std::uint64_t> overflow_bits;
  std::size_t overflow_words_per_cp{0};
  bool overflow_capped{false};

  std::optional<std::size_t> OverflowBit(std::uint16_t source_id) const;
  bool HasSource(std::size_t cp_idx, std::uint16_t source_id,
                 std::optional<std::size_t> overflow_bit) const;
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

struct AlertInterval {
  std::int64_t start_ns{0};
  std::int64_t end_ns{0};
  std::uint32_t topic{0};
  // Low 31 bits: channel index within topic; high bit: active_at_end (true
  // when still active at session end, false when cleared by a false sample,
  // even on exact end_ns).
  std::uint32_t channel_and_active{0};

  static constexpr std::uint32_t kActiveAtEndMask = std::uint32_t{1} << 31;

  static AlertInterval Make(std::int64_t start_ns, std::int64_t end_ns,
                            ChannelRef ref, bool active_at_end) noexcept {
    return {start_ns, end_ns, ref.topic,
            (ref.channel & ~kActiveAtEndMask) |
                (active_at_end ? kActiveAtEndMask : 0u)};
  }
  ChannelRef channel() const noexcept {
    return {topic, channel_and_active & ~kActiveAtEndMask};
  }
  bool active_at_end() const noexcept {
    return (channel_and_active & kActiveAtEndMask) != 0;
  }
};
static_assert(sizeof(AlertInterval) == 24,
              "REQ-OPS-42: 24 B per timeline alert interval");

struct RunInterval {
  std::string node;
  std::int64_t start_ns{0};
  std::optional<std::int64_t> end_ns;  // empty when end_wall_ns == 0
  std::string reason;
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

  // The earliest and latest event time of any record, and the start and
  // end of any gap: a gap's span is time its process ran, though its log
  // holds no record of it.
  std::int64_t start_ns() const { return start_ns_; }
  std::int64_t end_ns() const { return end_ns_; }
  // The wall clock at zero on the session's clock, so a time t_ns is at
  // wall time origin_wall_ns + t_ns: from the header of the log that started
  // first (its start_wall_ns minus its start_monotonic_ns), the clock the
  // launcher's manifest records runs in.
  std::int64_t origin_wall_ns() const { return origin_wall_ns_; }

  const std::vector<LogFile>& files() const { return files_; }
  // Every intact GAP record of the session, by start, then process, then
  // the merge's order. A damaged one is damage (read and warned about as
  // log_dump does) and is not listed.
  const std::vector<Gap>& gaps() const { return gaps_; }
  const std::vector<Topic>& topics() const { return topics_; }
  const talos::telemetry::Channel& channel(ChannelRef ref) const {
    return topics_[ref.topic].channels[ref.channel];
  }
  std::optional<ChannelRef> Find(std::string_view name) const;

  // log_dump's line for a session whose hardware packets are not served
  // (another protocol, or packets this build's codecs reject), without its
  // "warning: " prefix; empty when they are.
  const std::string& hardware_not_decoded() const {
    return hardware_not_decoded_;
  }
  // `[studio] pose_topic` from the session's robot config (manifest.json
  // config_path), or empty when unset or unreadable.
  const std::string& pose_topic() const { return pose_topic_; }

  // Timeline summary derived during Open() (REQ-OPS-41, REQ-OPS-42):
  // Driver Station mode transitions (empty when hardware_not_decoded() is
  // non-empty), active alert intervals across all alert channels, and node
  // runs from manifest.json mapped via origin_wall_ns().
  const std::vector<ModeInterval>& modes() const { return modes_; }
  std::uint64_t modes_dropped() const { return modes_dropped_; }
  const std::vector<AlertInterval>& alerts() const { return alerts_; }
  std::uint64_t alerts_dropped() const { return alerts_dropped_; }
  const std::vector<RunInterval>& runs() const { return runs_; }
  std::uint64_t runs_dropped() const { return runs_dropped_; }

  // The index's checkpoints and gaps: what grows with the session's length.
  std::size_t index_bytes() const;
  std::size_t log_seconds() const;

  // One Series per channel of `channels`, in that order, of `points`
  // buckets. Requires end_ns > start_ns and points > 0.
  std::vector<Series> ReadSeries(std::span<const ChannelRef> channels,
                                 std::int64_t start_ns, std::int64_t end_ns,
                                 std::uint32_t points,
                                 ReadStats* stats = nullptr) const;

  // Every channel of each topic with a record published at or before
  // `t_ns`, from its latest such record, by topic then channel, capped at
  // `max_channels`; a topic with none is absent. Per log file it reads, once
  // each, only the seconds that hold a value record of one of its topics'
  // sources at or before floor(t_ns) + file.max_lead_s, latest first, stopping
  // per source once no earlier checkpoint can hold a newer publication
  // (checkpoint.second < floor(found.publish_ns) - file.max_lag_s): normally
  // one second per source however long ago it was last published
  // (studio/README.md, "Reviewing a session").
  std::vector<Sample> Snapshot(
      std::int64_t t_ns, ReadStats* stats = nullptr,
      std::size_t max_channels = kMaxSnapshotChannels) const;

 private:
  Session() = default;

  // Decodes one record of `topic` into raw samples: raw[i] for channel i,
  // present[i] whether it has one.
  bool Decode(const Topic& topic, std::span<const std::byte> payload,
              std::vector<std::uint64_t>& raw,
              std::vector<std::uint8_t>& present) const;

  std::vector<LogFile> files_;
  std::vector<Gap> gaps_;
  std::vector<Topic> topics_;
  std::unordered_map<std::string, ChannelRef> by_name_;
  // Per hardware topic: channel name -> index.
  std::vector<std::unordered_map<std::string, std::uint32_t>> hardware_index_;
  std::int64_t start_ns_{0};
  std::int64_t end_ns_{0};
  std::int64_t origin_wall_ns_{0};
  std::string hardware_not_decoded_;
  std::string pose_topic_;
  std::vector<ModeInterval> modes_;
  std::uint64_t modes_dropped_{0};
  std::vector<AlertInterval> alerts_;
  std::uint64_t alerts_dropped_{0};
  std::vector<RunInterval> runs_;
  std::uint64_t runs_dropped_{0};
};

// The value of a raw telemetry sample of `kind` as a double (a bucket's).
double ValueOf(talos::telemetry::ChannelKind kind, std::uint64_t raw);

}  // namespace studio::review
