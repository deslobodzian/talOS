#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/events/recorder.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/events/time.h"
#include "talOS/schema/identity.h"

// Input replay: drive a program that was not in a recorded session (or a
// changed version of one that was) from the topics that session logged.
//
// ReplayEventLoop re-runs a program against its own log and fails at the
// first difference. This is the other question -- "what would this program
// have done in that match?" -- so nothing is compared: the program runs on a
// SimulatedEventLoop on the recorded clock, every recorded message on a topic
// it watches is published at its recorded publish time, and whatever it
// sends is its own new output. A program that was in the session (its logs
// carry its process name) runs on its own recorded timeline: its loop's
// origin and run start, so its timers fire at the recorded deadlines, and it
// is handed what it received live, at the drains it received it in.
namespace talos::event {

// A topic as a session's logs recorded it.
struct RecordedTopic {
  std::string name;
  std::uint32_t message_bytes{0};
  schema::Identity schema;
  // Whose log its messages come from: the replayed program's own when it
  // watched the topic live, else its writer's when the writer logged it,
  // otherwise the first reader's that did.
  std::string recorded_by;
  std::size_t message_count{0};
  // The earliest loss in a log this topic is fed from, and that log; empty
  // when none lost records. A loss is a GAP record, from its
  // LogGap::start_ns, or a damaged stretch (records lost between intact
  // ones), from the earliest time a record of this topic lost there could
  // be fed at: for a writer's sends, the now_ns of the last intact record
  // before the stretch (a send is fed at its publication time, its record's
  // now_ns, and now_ns never decreases along a log); for a reader's
  // receipts, the publication time of its last intact receipt of the topic
  // (a watcher receives in sequence order); both counted along the log
  // across its segments (a later segment starts from the one before it,
  // when read). Where nothing bounds it, the log is refused instead (below).
  // A reader's stretch, in a log other than the replayed program's, costs
  // nothing where the session's logs hold every message of the topic whose
  // sequence lies between its intact receipts around the stretch (from 0
  // when none precedes it; never when none follows it in its file): the
  // writer's sends restore all it could have lost.
  std::optional<MonotonicTime> first_gap;
  std::string first_gap_log;
  // Where that loss is a damaged stretch: its byte offset in first_gap_log.
  std::optional<std::uint64_t> first_gap_damage_offset;
  // The first log (in log_paths() order) this topic is fed from that input
  // replay refuses, and why, naming the sign it found: the log's
  // `<log>.capped` marker, the session manifest's `log_capped`, its
  // `log_gap` or `recorder_failed` for a run's log without its EXIT record,
  // or a damaged stretch
  // (its byte offset) before which no receipt bounds the topic's lost
  // records (the replayed program's own receipts, fed at their drains'
  // event times, which need not follow the log's order; a reader's topic
  // with no intact receipt before it, unless an intact receipt of it
  // follows the stretch in its file and the session's logs hold every
  // message of the topic before that receipt's sequence). Empty when none.
  std::string refused_log;
  std::string refused_reason;
};

struct RecordedMessage {
  MonotonicTime time;  // producer's publish time, not a reader's receipt
  // When the replayed program is to see it: `time`, or for its own receipt
  // the deadline of the live drain that delivered it.
  MonotonicTime delivered;
  std::uint32_t topic{0};  // index into RecordedSession::topics()
  std::uint64_t sequence{0};
  std::size_t offset{0};  // of its payload in the session's byte store
};

// Every message one session's logs recorded, by topic.
//
// A topic the replayed program watched live is fed from its own logs'
// receipts alone: what it saw, when it saw it. Each is delivered at the
// drain that delivered it live (its MESSAGE record's event time), with the
// publish time the transport stamped. The replayed loop runs every drain at
// its deadline, without the live loop's wake-up latency, so a receipt
// published after its drain's deadline (during that latency) is published
// at that deadline.
//
// Every other topic is fed from all logs: a message's time is its producer's
// publish time, preserved in both SEND and MESSAGE records even when only a
// delayed reader logged it (a send recorded before its loop's first
// dispatch, with no clock, at its log's origin, the transport's stamp), and
// one message seen in several logs is kept once, by topic and transport
// sequence.
//
// A log with a GAP record (log/format.h), or a damaged stretch (records lost
// between intact ones, log::LogDamage::spans), lost records there: each
// topic notes the earliest such loss in a log it is fed from (first_gap),
// which is where ReplayInputs ends a run that feeds it, except a reader's
// stretch whose lost receipts the session's other logs all hold (first_gap
// says how). When the replayed
// program's own log has a loss before any timer firing, run_start() comes
// from the first firing after it, never before the loss's start less that
// timer's offset: no replayed timer then fires before it, as none did live.
//
// A log may also have lost records nothing in it marks: one the byte cap
// cost records (`<log>.capped`, log::CappedMarkerPath, beside the log a
// `<stem>.segment<k>.tlog` belongs to, or the session manifest's
// `log_capped` for its member: its oldest segment deleted, which may have
// held a GAP record, or capture stopped at the cap), and a run's log without
// its EXIT record of a member the manifest marks `log_gap` or
// `recorder_failed`, whose gap that run's end may have cut short before its
// GAP record was written, or whose recorder failed (a disk error ends
// capture with no GAP record). A run's
// log with its EXIT holds a GAP record for every gap (LogWriter::gaps) and
// is judged by them. Each topic notes the first such log it is fed from
// (refused_log), which ReplayInputs refuses, as it does a damaged stretch
// that nothing bounds. The replayed program's run start is read from its
// first run's log: one whose first segment the byte cap deleted lost the
// firing it is read from, and ReplayInputs refuses it
// (run_start_refused_log), fed topic or not. The manifest is
// `manifest.json` in the directory holding the logs
// (talOS/launcher/README.md, "Session manifest"), used when its session_id
// is the logs'; it names a member's logs by the paths they were recorded at
// (`nodes[].log_paths`), matched here by file name, so a session read from
// another directory still matches. Logs with no such
// manifest beside them (a directory outside a launcher session, another
// session's manifest) are judged by their `.capped` markers alone.
class RecordedSession {
 public:
  // `path` is a session directory (every *.tlog in it, restarts included)
  // or one log; `replayed` is the process name of the program to be
  // replayed, or empty for none. Throws std::runtime_error on an unreadable
  // log, on logs from two sessions, on a topic the logs record with two
  // schemas, and on a manifest.json beside them that is not JSON. A log's
  // damaged records are skipped (log::LogReader) and mark a loss or a
  // refusal (above); what was skipped is printed to stderr as a warning
  // naming the file and byte offsets.
  explicit RecordedSession(const std::string& path,
                           std::string_view replayed = {});

  const std::vector<std::string>& log_paths() const { return log_paths_; }
  std::uint64_t session_id() const { return session_id_; }

  // The earliest log origin, and the latest time any log recorded.
  MonotonicTime start() const { return start_; }
  MonotonicTime end() const { return end_; }

  // Where the replayed program's loop starts (SimulatedEventLoop::Options):
  // its first recorded run's loop origin, and the instant that run's loop
  // began running, which its construction put after the origin (the first
  // timer firing's deadline minus its manifest offset; the origin when it
  // had no timer). For a program the session did not log, start() for both.
  // Where the replay ends: the latest time the program's own logs recorded
  // (its exit), or end().
  MonotonicTime loop_start() const { return loop_start_; }
  MonotonicTime run_start() const { return run_start_; }
  MonotonicTime loop_end() const { return loop_end_; }

  // The file of the replayed program's first run's log its run start would
  // be read from, when that log's manifest arms a timer and the byte cap
  // deleted its first segment (log::LogSegmentPath(log, 0); a rotated log
  // keeps its later ones), and why, naming the sign: its `<log>.capped`
  // marker, the session manifest's `log_capped`, or else only that the
  // segment is gone. The run start's firing
  // was in that segment; the oldest kept one's first firing may be whole
  // periods later, so ReplayInputs refuses the log, whatever it feeds.
  // Empty when none.
  const std::string& run_start_refused_log() const {
    return run_start_refused_log_;
  }
  const std::string& run_start_refused_reason() const {
    return run_start_refused_reason_;
  }

  const std::vector<RecordedTopic>& topics() const { return topics_; }
  // The topic called `name`, or null when no log recorded it.
  const RecordedTopic* find(std::string_view name) const;

  // In feed order: delivered, then time, then topic name, then sequence.
  const std::vector<RecordedMessage>& messages() const { return messages_; }
  std::span<const std::byte> payload(const RecordedMessage& message) const {
    return {bytes_.data() + message.offset,
            topics_[message.topic].message_bytes};
  }

 private:
  std::vector<std::string> log_paths_;
  std::uint64_t session_id_{0};
  MonotonicTime start_;
  MonotonicTime end_;
  MonotonicTime loop_start_;
  MonotonicTime run_start_;
  MonotonicTime loop_end_;
  std::string run_start_refused_log_;
  std::string run_start_refused_reason_;
  std::vector<RecordedTopic> topics_;
  std::vector<RecordedMessage> messages_;
  std::vector<std::byte> bytes_;
};

struct InputReplayReport {
  struct Fed {
    std::string topic;
    std::string recorded_by;
    std::size_t messages{0};
  };
  // Topics the program watches that the session recorded, in registration
  // order, with how many messages each was fed.
  std::vector<Fed> fed;
  // Topics the program watches that no log recorded: nothing arrives on
  // them, as if their writer had never run.
  std::vector<std::string> unrecorded;
  struct Gap {
    std::string topic;
    std::string log;
    MonotonicTime at;
    // Set when the loss is a damaged stretch, not a GAP record: its byte
    // offset in `log`.
    std::optional<std::uint64_t> damage_offset;
  };
  // Set when the run ended before a loss (a gap or damage) in a log it was
  // fed from.
  std::optional<Gap> stopped_at_gap;
};

// The topic `registration` (a watcher of the program) reads, matched against
// what the session recorded: null when nothing was recorded, the recorded
// topic when the schemas agree. Throws ReplayError naming both types when
// they do not.
const RecordedTopic* MatchRecordedTopic(const RecordedSession& session,
                                        const Registration& registration);

// Runs `loop` to session.loop_end(), publishing every recorded message on a
// topic the program watches with its recorded publish time, just before the
// first drain at or after its `delivered` time, which delivers it. It never
// feeds across a loss: when a topic it feeds has a first_gap at or before
// loop_end(), the run ends one nanosecond before the earliest such loss (the
// first fed topic's on a tie), feeding nothing delivered at or after it, and
// the report's stopped_at_gap names it. A topic
// the program sends itself is its own output and is not fed. `loop` must
// have finished registration, not have run, and start at
// session.loop_start() with its run starting at session.run_start()
// (SimulatedEventLoop::Options::start, run_start). Finishes the loop, which
// closes its recorder's log. Throws ReplayError before running on a schema
// mismatch, on a topic it would feed from a refused log (refused_log),
// naming the log, the sign and the topic, and on a run start read from a
// refused log (run_start_refused_log), naming the log and the sign.
template <RecorderPolicy Recorder>
InputReplayReport ReplayInputs(SimulatedEventLoop<Recorder>& loop,
                               SimulationEnvironment& environment,
                               const RecordedSession& session) {
  if (loop.start_time() != session.loop_start()) {
    throw std::logic_error(
        "input replay needs a loop that starts at the recorded origin "
        "(RecordedSession::loop_start)");
  }
  if (!session.run_start_refused_log().empty()) {
    throw ReplayError("input replay refuses " +
                      session.run_start_refused_log() +
                      ", which the replayed program's run start would be "
                      "read from: " +
                      session.run_start_refused_reason());
  }

  InputReplayReport report;
  std::vector<SimulatedChannel*> channels(session.topics().size(), nullptr);
  std::vector<std::size_t> fed_index(session.topics().size(), 0);
  const Manifest& manifest = loop.manifest();
  const auto sends = [&](std::string_view topic) {
    for (const Registration& registration : manifest) {
      if (registration.kind == SourceKind::SENDER &&
          registration.name == topic) {
        return true;
      }
    }
    return false;
  };

  for (const Registration& registration : manifest) {
    if (registration.kind != SourceKind::WATCHER || sends(registration.name)) {
      continue;
    }
    const RecordedTopic* topic = MatchRecordedTopic(session, registration);
    if (topic == nullptr) {
      report.unrecorded.push_back(registration.name);
      continue;
    }
    if (!topic->refused_log.empty()) {
      throw ReplayError("input replay refuses " + topic->refused_log +
                        ", which topic '" + topic->name +
                        "' would be fed from: " + topic->refused_reason);
    }
    const auto index =
        static_cast<std::size_t>(topic - session.topics().data());
    if (channels[index] != nullptr) continue;  // watched twice
    channels[index] = &environment.channel(
        registration.name, registration.message_bytes, registration.schema);
    fed_index[index] = report.fed.size();
    report.fed.push_back({topic->name, topic->recorded_by, 0});
    // The earliest gap among the fed topics' logs, the first fed topic's on
    // a tie, ends the run, unless the run ends before it anyway.
    if (topic->first_gap.has_value() &&
        *topic->first_gap <= session.loop_end() &&
        (!report.stopped_at_gap.has_value() ||
         *topic->first_gap < report.stopped_at_gap->at)) {
      report.stopped_at_gap = InputReplayReport::Gap{
          topic->name, topic->first_gap_log, *topic->first_gap,
          topic->first_gap_damage_offset};
    }
  }
  // Nothing at or past the gap is fed or run: the run ends one nanosecond
  // before it, so no drain there runs.
  const MonotonicTime end = report.stopped_at_gap.has_value()
                                ? report.stopped_at_gap->at - Duration{1}
                                : session.loop_end();

  std::optional<MonotonicTime> instant;
  for (const RecordedMessage& message : session.messages()) {
    if (message.delivered > end) break;
    SimulatedChannel* channel = channels[message.topic];
    if (channel == nullptr) continue;
    // Everything due before the message's delivery happens first; then the
    // clock stands at it, so the drain there, a timer's or a wake's, is the
    // first to see it. Every message of one instant goes out before that
    // drain runs.
    if (instant != message.delivered) {
      loop.run_until(message.delivered - Duration{1});
      if (loop.monotonic_now() < message.delivered) {
        loop.advance_until(message.delivered);
      }
      instant = message.delivered;
    }
    channel->publish(session.payload(message), message.time);
    ++report.fed[fed_index[message.topic]].messages;
  }
  loop.run_until(end);
  loop.finish();
  return report;
}

}  // namespace talos::event
