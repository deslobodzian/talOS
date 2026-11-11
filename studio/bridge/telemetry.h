#pragma once

/*
 * The bridge's side of the telemetry feed (talOS/telemetry/README.md):
 *
 * - IngestTelemetry assembles the producer's v3 pages into the latest
 *   complete tick and its dictionary.
 * - ClientFeed sends one WebSocket client whatever brings it to that tick:
 *   the dictionary, a keyframe when it joins, and after that only the
 *   channels whose value changed, at their exact current value. A send the
 *   egress budget cannot afford is not dropped but deferred: the client's
 *   next update carries every change since what it last received, so it never
 *   shows a stale value as current, only skips intermediate samples.
 * - TelemetryJson / ChannelsJson are the self-describing JSON views. Same
 *   rule as system.h: uint64 header fields leave as decimal strings, and an
 *   integer channel value past 2^53 leaves as a decimal string, because a
 *   JSON number is a double. An absent sample leaves as null.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "studio/bridge/system.h"
#include "talOS/telemetry/frames.h"
#include "talOS/telemetry/telemetry_generated.h"

namespace studio {

inline constexpr int kTelemetryVersion = 3;

using talos::telemetry::ChannelKind;

struct TelemetryChannelInfo {
  std::string name;
  std::string unit;
  std::vector<std::string> labels;  // non-empty: enum channel
  ChannelKind kind{Talos::Telemetry::ChannelKind_Double};
};

struct TelemetryPose {
  double x{0}, y{0}, z{0}, roll{0}, pitch{0}, yaw{0};
  bool operator==(const TelemetryPose&) const = default;
};

// The bridge's decoded view of the feed: the latest complete dictionary (and
// its pages as received, to hand a client) and the latest complete tick.
struct TelemetryState {
  bool has_dictionary{false};
  std::vector<TelemetryChannelInfo> dictionary;
  std::uint32_t dictionary_id{0};
  std::uint64_t stream_id{0};
  std::vector<std::string> dictionary_pages;  // size-prefixed, as received

  bool has_frame{false};
  std::uint64_t sequence_id{0};
  std::uint64_t timestamp_ns{0};
  TelemetryPose chassis{};
  // Per channel: the raw 64-bit sample, read per its kind, and whether there
  // is one this tick.
  std::vector<std::uint64_t> values;
  std::vector<std::uint8_t> present;
  // Monotonic, so age_ms cannot be hidden or inflated by a wall-clock step.
  std::int64_t received_monotonic_ns{0};

  // Ticks whose pages did not all arrive (the reader was lapped mid-tick):
  // never applied, so no channel mixes two ticks.
  std::uint64_t incomplete_ticks{0};

  // Assembly of the dictionary and the tick in progress.
  struct {
    std::uint64_t stream_id{0};
    std::uint32_t dictionary_id{0};
    std::uint32_t page_count{0};
    std::vector<std::string> pages;
  } pending_dictionary;
  struct {
    bool active{false};
    std::uint64_t sequence_id{0};
    std::uint32_t page_count{0};
    std::uint32_t next_page{0};
    std::vector<std::uint64_t> values;
    std::vector<std::uint8_t> present;
  } pending_tick;
};

namespace detail {

inline bool FinitePose(const Talos::Telemetry::Pose& pose) {
  return std::isfinite(pose.x()) && std::isfinite(pose.y()) &&
         std::isfinite(pose.z()) && std::isfinite(pose.roll()) &&
         std::isfinite(pose.pitch()) && std::isfinite(pose.yaw());
}

// A sample its kind can hold: any double (NaN and infinities are readings;
// absence is a missing sample, not a value), or a 0/1 bool.
inline bool ValidSample(ChannelKind kind, std::uint64_t raw) {
  return kind != Talos::Telemetry::ChannelKind_Bool || raw <= 1;
}

// Decodes a complete set of dictionary pages; false when they do not form
// one dictionary Studio can decode.
inline bool DecodeDictionary(const std::vector<std::string>& pages,
                             std::uint32_t channel_count,
                             std::vector<TelemetryChannelInfo>& out) {
  out.clear();
  out.reserve(channel_count);
  std::unordered_set<std::string> names;
  for (const std::string& bytes : pages) {
    const auto* frame = Talos::Telemetry::GetSizePrefixedFrame(bytes.data());
    if (frame->first_channel() != out.size()) return false;
    for (const auto* info : *frame->dictionary()) {
      TelemetryChannelInfo channel;
      channel.name = info->name()->str();
      if (!names.insert(channel.name).second ||
          channel.name.starts_with("derived/") ||
          info->kind() > Talos::Telemetry::ChannelKind_MAX) {
        return false;
      }
      channel.unit = info->unit() != nullptr ? info->unit()->str() : "";
      channel.kind = info->kind();
      if (info->labels() != nullptr) {
        for (const auto* label : *info->labels()) {
          channel.labels.push_back(label != nullptr ? label->str() : "");
        }
      }
      out.push_back(std::move(channel));
    }
  }
  return out.size() == channel_count;
}

}  // namespace detail

// Ingests one size-prefixed v3 page (as telemetry_wire::payload returns it).
// Dictionary pages complete a dictionary when its last page arrives in
// order; value pages complete a tick when its last page arrives in order,
// and only a complete tick that belongs to the cached dictionary becomes the
// latest. Returns false for bytes that are not a valid v3 page, a dictionary
// past telemetry_wire::kMaxChannels, or a dictionary whose pages do not
// decode; such bytes never change the cached dictionary or the latest tick
// (a dictionary being assembled from them is discarded). A valid page that
// cannot be used yet returns true.
inline bool IngestTelemetry(TelemetryState& state, std::string_view payload,
                            std::int64_t monotonic_ns) {
  flatbuffers::Verifier verifier{
      reinterpret_cast<const std::uint8_t*>(payload.data()), payload.size()};
  if (!Talos::Telemetry::VerifySizePrefixedFrameBuffer(verifier)) return false;
  const auto* frame = Talos::Telemetry::GetSizePrefixedFrame(payload.data());
  if (frame == nullptr || frame->page_count() == 0 ||
      frame->page() >= frame->page_count() ||
      frame->channel_count() > talos::telemetry_wire::kMaxChannels) {
    return false;
  }

  if (frame->dictionary() != nullptr) {
    if (frame->values() != nullptr || frame->channels() != nullptr) {
      return false;
    }
    auto& pending = state.pending_dictionary;
    if (frame->page() == 0) {
      pending.stream_id = frame->stream_id();
      pending.dictionary_id = frame->dictionary_id();
      pending.page_count = frame->page_count();
      pending.pages.clear();
    } else if (pending.pages.size() != frame->page() ||
               pending.stream_id != frame->stream_id() ||
               pending.dictionary_id != frame->dictionary_id() ||
               pending.page_count != frame->page_count()) {
      pending.pages.clear();  // A page went missing; wait for the next one.
      return true;
    }
    pending.pages.emplace_back(payload);
    if (pending.pages.size() < pending.page_count) return true;

    std::vector<TelemetryChannelInfo> dictionary;
    std::vector<std::string> pages = std::move(pending.pages);
    pending.pages.clear();
    if (!detail::DecodeDictionary(pages, frame->channel_count(), dictionary)) {
      return false;
    }
    if (!state.has_dictionary || frame->stream_id() != state.stream_id ||
        frame->dictionary_id() != state.dictionary_id) {
      state.has_frame = false;
      state.values.clear();
      state.present.clear();
      state.pending_tick.active = false;
    }
    state.dictionary = std::move(dictionary);
    state.dictionary_id = frame->dictionary_id();
    state.stream_id = frame->stream_id();
    state.dictionary_pages = std::move(pages);
    state.has_dictionary = true;
    return true;
  }

  // A value page.
  const auto* chassis = frame->chassis();
  if (chassis == nullptr || !detail::FinitePose(*chassis)) return false;
  const auto* channels = frame->channels();
  const auto* values = frame->values();
  const std::size_t carried = channels != nullptr ? channels->size() : 0;
  if ((values != nullptr ? values->size() : 0) != carried) return false;
  const std::uint32_t count = frame->channel_count();
  for (std::size_t i = 0; i < carried; ++i) {
    if (channels->Get(i) >= count) return false;
  }
  if (frame->absent() != nullptr) {
    for (const std::uint32_t channel : *frame->absent()) {
      if (channel >= count) return false;
    }
  }
  const bool aligns = state.has_dictionary &&
                      frame->stream_id() == state.stream_id &&
                      frame->dictionary_id() == state.dictionary_id &&
                      count == state.dictionary.size();
  if (!aligns) return true;  // Well-formed; just not usable yet.
  for (std::size_t i = 0; i < carried; ++i) {
    if (!detail::ValidSample(state.dictionary[channels->Get(i)].kind,
                             values->Get(i))) {
      return false;
    }
  }

  auto& tick = state.pending_tick;
  if (frame->page() == 0) {
    if (tick.active) ++state.incomplete_ticks;
    tick.active = true;
    tick.sequence_id = frame->sequence_id();
    tick.page_count = frame->page_count();
    tick.next_page = 0;
    if (frame->keyframe() || !state.has_frame) {
      tick.values.assign(count, 0);
      tick.present.assign(count, 0);
    } else {
      tick.values = state.values;
      tick.present = state.present;
    }
  } else if (!tick.active || tick.sequence_id != frame->sequence_id() ||
             tick.page_count != frame->page_count() ||
             tick.next_page != frame->page()) {
    if (tick.active) ++state.incomplete_ticks;
    tick.active = false;
    return true;
  }
  for (std::size_t i = 0; i < carried; ++i) {
    tick.values[channels->Get(i)] = values->Get(i);
    tick.present[channels->Get(i)] = 1;
  }
  if (frame->absent() != nullptr) {
    for (const std::uint32_t channel : *frame->absent()) {
      tick.present[channel] = 0;
    }
  }
  ++tick.next_page;
  if (tick.next_page < tick.page_count) return true;

  tick.active = false;
  state.sequence_id = frame->sequence_id();
  state.timestamp_ns = frame->timestamp_ns();
  state.chassis = {chassis->x(),    chassis->y(),     chassis->z(),
                   chassis->roll(), chassis->pitch(), chassis->yaw()};
  state.values.swap(tick.values);
  state.present.swap(tick.present);
  state.received_monotonic_ns = monotonic_ns;
  state.has_frame = true;
  return true;
}

// The WebSocket egress token bucket: every send to a client is charged, with
// conservative framing overhead, against a shared bucket and the client's own
// (docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary).
struct EgressBudget {
  static constexpr double kBytesPerSecond = 160000;
  static constexpr double kBurstBytes =
      talos::telemetry_wire::kClientBurstBytes;
  // WebSocket framing is charged too, conservatively: a 2-byte header below
  // 126 payload bytes, otherwise the largest header.
  static constexpr std::size_t kMaxHeaderBytes =
      talos::telemetry_wire::kMaxWebSocketHeaderBytes;
  static constexpr std::size_t framed(std::size_t bytes) {
    return bytes + (bytes < 126 ? 2 : kMaxHeaderBytes);
  }
  // The largest payload a client can ever afford in one send.
  static constexpr std::size_t kMaxPayloadBytes =
      talos::telemetry_wire::kClientBurstBytes - kMaxHeaderBytes;
  double tokens = kBurstBytes;
  std::int64_t updated_ns = 0;
  bool started = false;

  // Refills at `rate` B/s up to the burst; true when `bytes` fit now.
  bool available(std::size_t bytes, double rate, std::int64_t now_ns) {
    if (started && now_ns > updated_ns) {
      tokens = std::min(
          kBurstBytes,
          tokens + static_cast<double>(now_ns - updated_ns) * 1e-9 * rate);
    }
    if (!started || now_ns > updated_ns) updated_ns = now_ns;
    started = true;
    return tokens >= static_cast<double>(bytes);
  }
  void spend(std::size_t bytes) { tokens -= static_cast<double>(bytes); }
  // How long after the last available() `bytes` fit, refilling at `rate`;
  // 0 when they fit now, kNeverNs when they exceed the burst.
  std::int64_t wait_ns(std::size_t bytes, double rate) const {
    const auto needed = static_cast<double>(bytes);
    if (needed > kBurstBytes) return kNeverNs;
    if (tokens >= needed) return 0;
    return static_cast<std::int64_t>(std::ceil((needed - tokens) / rate * 1e9));
  }
};

// What one client has been sent, so its next update carries only what
// changed. Pump is called whenever there may be something to send (each new
// tick, each bridge pass); it never sends what the client already has.
class ClientFeed {
 public:
  // With nothing changed, an empty update still goes out this often, so the
  // client knows the stream is alive and how old its view is.
  static constexpr std::int64_t kHeartbeatNs = 100'000'000;

  // `afford(framed_bytes)` says whether a batch of that many framed bytes may
  // be sent now, and spends nothing; `send(page)` sends one page, is charged
  // for it, and returns false if it was not sent. A whole update is sent only
  // when the budget affords all of it, so the client applies it as one tick.
  // The dictionary is within telemetry_wire::kMaxChannels (IngestTelemetry), so
  // the largest update, a keyframe of every channel, fits the burst and is
  // affordable once the budget refills.
  template <class Afford, class Send>
  void Pump(const TelemetryState& state, std::int64_t now_ns, Afford&& afford,
            Send&& send) {
    if (!state.has_dictionary) return;
    const std::size_t count = state.dictionary.size();
    if (!known_ || stream_id_ != state.stream_id ||
        dictionary_id_ != state.dictionary_id) {
      known_ = true;
      stream_id_ = state.stream_id;
      dictionary_id_ = state.dictionary_id;
      dictionary_pages_sent_ = 0;
      Resync(count);
    }
    while (dictionary_pages_sent_ < state.dictionary_pages.size()) {
      const std::string& page = state.dictionary_pages[dictionary_pages_sent_];
      if (!afford(EgressBudget::framed(page.size())) || !send(page)) return;
      ++dictionary_pages_sent_;
    }
    // The client holds this tick: nothing to send until the heartbeat is due.
    if (!state.has_frame || (has_values_ && sequence_id_ == state.sequence_id &&
                             now_ns - last_send_ns_ < kHeartbeatNs)) {
      return;
    }

    const bool keyframe = !has_values_;
    channels_.clear();
    raw_.clear();
    absent_.clear();
    for (std::size_t i = 0; i < count; ++i) {
      if (state.present[i] != 0) {
        if (keyframe || present_[i] == 0 || values_[i] != state.values[i]) {
          channels_.push_back(static_cast<std::uint32_t>(i));
          raw_.push_back(state.values[i]);
        }
      } else if (!keyframe && present_[i] != 0) {
        absent_.push_back(static_cast<std::uint32_t>(i));
      }
    }
    if (!keyframe && channels_.empty() && absent_.empty() &&
        chassis_ == state.chassis && now_ns - last_send_ns_ < kHeartbeatNs) {
      return;
    }

    pages_.clear();
    spans_.clear();
    std::size_t framed_total = 0;
    const talos::telemetry::PageHeader header{
        .stream_id = state.stream_id,
        .sequence_id = state.sequence_id,
        .timestamp_ns = state.timestamp_ns,
        .dictionary_id = state.dictionary_id,
        .channel_count = static_cast<std::uint32_t>(count)};
    const Talos::Telemetry::Pose chassis{
        state.chassis.x,    state.chassis.y,     state.chassis.z,
        state.chassis.roll, state.chassis.pitch, state.chassis.yaw};
    const bool encoded = talos::telemetry::EncodeValuePages(
        builder_, header, chassis, keyframe, channels_, raw_, absent_,
        [&](std::span<const std::uint8_t> page,
            const talos::telemetry::PageSpan& span) {
          pages_.emplace_back(reinterpret_cast<const char*>(page.data()),
                              page.size());
          spans_.push_back(span);
          framed_total += EgressBudget::framed(page.size());
        });
    // Neither can happen (frames_test; the channel cap); never send what
    // the client could not apply whole.
    if (!encoded ||
        static_cast<double>(framed_total) > EgressBudget::kBurstBytes) {
      return;
    }
    if (!afford(framed_total)) return;
    for (std::size_t i = 0; i < pages_.size(); ++i) {
      if (!send(pages_[i])) {
        Resync(count);
        return;
      }
      Apply(spans_[i], keyframe && i == 0);
    }
    sequence_id_ = state.sequence_id;
    chassis_ = state.chassis;
    last_send_ns_ = now_ns;
  }

  // When an update is next due with nothing new published: the heartbeat,
  // once the client holds a tick; kNeverNs before.
  std::int64_t NextHeartbeatNs() const {
    return has_values_ ? last_send_ns_ + kHeartbeatNs : kNeverNs;
  }

  // True when the client holds exactly the state's latest tick.
  bool current(const TelemetryState& state) const {
    if (!known_ || !has_values_ || !state.has_frame ||
        stream_id_ != state.stream_id ||
        dictionary_id_ != state.dictionary_id || chassis_ != state.chassis) {
      return false;
    }
    for (std::size_t i = 0; i < state.present.size(); ++i) {
      if (present_[i] != state.present[i] ||
          (present_[i] != 0 && values_[i] != state.values[i])) {
        return false;
      }
    }
    return true;
  }

 private:
  // The client's view is unknown (it has not joined, or a send failed): the
  // next update is a keyframe, which makes it start from every channel absent.
  void Resync(std::size_t count) {
    has_values_ = false;
    values_.assign(count, 0);
    present_.assign(count, 0);
  }

  void Apply(const talos::telemetry::PageSpan& span, bool starts_keyframe) {
    if (starts_keyframe) {
      std::fill(present_.begin(), present_.end(), 0);
      has_values_ = true;
    }
    for (std::size_t i = span.first_sample;
         i < span.first_sample + span.sample_count; ++i) {
      values_[channels_[i]] = raw_[i];
      present_[channels_[i]] = 1;
    }
    for (std::size_t i = span.first_absent;
         i < span.first_absent + span.absent_count; ++i) {
      present_[absent_[i]] = 0;
    }
  }

  bool known_{false};
  std::uint64_t stream_id_{0};
  std::uint32_t dictionary_id_{0};
  std::size_t dictionary_pages_sent_{0};
  bool has_values_{false};
  std::uint64_t sequence_id_{0};
  std::vector<std::uint64_t> values_;
  std::vector<std::uint8_t> present_;
  TelemetryPose chassis_{};
  std::int64_t last_send_ns_{0};

  // Scratch, reused per update.
  std::vector<std::uint32_t> channels_;
  std::vector<std::uint64_t> raw_;
  std::vector<std::uint32_t> absent_;
  std::vector<std::string> pages_;
  std::vector<talos::telemetry::PageSpan> spans_;
  flatbuffers::FlatBufferBuilder builder_{2048};
};

// One WebSocket client's two feeds under one budget. The system graph goes
// first: from the moment a graph message waits for budget it reserves
// kGraphShare of the client's refill, and the client's telemetry may spend
// only what the budget affords beyond that reserve, at most one update per
// kHoldNs. Telemetry, whose demand can exceed a client's share, can then
// neither take every byte as it arrives nor drain each window's refill (an
// update larger than a window's refill would otherwise do that forever), so
// a waiting message of G framed bytes goes within
//   (G + n) / (kGraphShare * rate - n / GraphFeed::kNoticeNs)
// of the client's own share, n being one framed deferral notice
// (docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary). Telemetry is
// change-only and coalesces, so the thinning costs it skipped samples, never
// values. `rate` is the client's share in B/s; `afford(framed_bytes)` and the
// senders are as in ClientFeed.
struct ClientFeeds {
  static constexpr std::int64_t kHoldNs = 250'000'000;
  static constexpr double kGraphShare = 0.5;

  GraphFeed graph;
  ClientFeed telemetry;

  template <class Afford, class SendText, class SendBinary>
  void Pump(const SystemGraphDocument& document, const TelemetryState& state,
            std::int64_t now_ns, double rate, Afford&& afford,
            SendText&& send_text, SendBinary&& send_binary) {
    const auto afford_text = [&afford](std::size_t bytes) {
      return afford(EgressBudget::framed(bytes));
    };
    if (!graph.Pump(document, now_ns, EgressBudget::kMaxPayloadBytes,
                    afford_text, send_text)) {
      held_since_ns_ = -1;
      telemetry.Pump(state, now_ns, afford, send_binary);
      return;
    }
    if (held_since_ns_ < 0) held_since_ns_ = held_sent_ns_ = now_ns;
    if (now_ns - held_sent_ns_ < kHoldNs) return;
    // Every byte telemetry spends leaves at least the reserve in the bucket,
    // which grows faster than the reserve, so the bucket never falls below
    // the reserve less the notices sent since: once that reaches G the
    // message is affordable.
    const auto reserve = static_cast<std::size_t>(
        std::min(EgressBudget::kBurstBytes,
                 kGraphShare * rate * 1e-9 *
                     static_cast<double>(now_ns - held_since_ns_)));
    bool sent = false;
    telemetry.Pump(
        state, now_ns,
        [&afford, reserve](std::size_t bytes) {
          return afford(bytes + reserve);
        },
        [&](std::string_view page) {
          const bool ok = send_binary(page);
          sent = sent || ok;
          return ok;
        });
    if (sent) held_sent_ns_ = now_ns;
  }

  // The earliest instant Pump, called with nothing new published, may send:
  // the graph's next due message, the telemetry heartbeat, and the end of
  // the hold on telemetry while a graph message waits. Sends deferred only
  // for budget are due when it refills, which the caller tracks.
  std::int64_t NextDueNs(const SystemGraphDocument& document) const {
    std::int64_t due =
        std::min(graph.NextDueNs(document), telemetry.NextHeartbeatNs());
    if (held_since_ns_ >= 0) due = std::min(due, held_sent_ns_ + kHoldNs);
    return due;
  }

 private:
  std::int64_t held_since_ns_{-1};  // when the waiting graph message began
  std::int64_t held_sent_ns_{0};    // telemetry's last send while it waits
};

// When the bridge must next run with no socket event or publication: the
// graph refresh, the segment check, the instant a send the egress budget
// deferred becomes affordable, and each client's due update or heartbeat
// (ClientFeeds::NextDueNs). The bridge never polls; this is every reason
// it wakes on its own.
struct WakeSchedule {
  // The system graph refreshes every 250 ms, by the clock: Studio marks the
  // graph stale after GRAPH_STALE_AFTER_MS (studio/src/data/freshness.ts)
  // without one.
  static constexpr std::int64_t kSystemRefreshNs = 250'000'000;
  // How soon the loop comes back when it cannot sleep on the wake channel.
  static constexpr std::int64_t kFallbackNs = 1'000'000;

  std::int64_t refreshed_ns = 0;  // the last graph refresh
  std::int64_t check_due_ns = 0;  // the next segment check
  // The earliest instant a deferred send becomes affordable this pass,
  // kNeverNs when nothing waits for budget.
  std::int64_t retry_ns = kNeverNs;

  // Starts a pass: what it defers is recorded afresh.
  void BeginPass() { retry_ns = kNeverNs; }
  bool RefreshDue(std::int64_t now_ns) const {
    return now_ns - refreshed_ns >= kSystemRefreshNs;
  }
  // A send the budget deferred at `now_ns` fits once `wait_ns` passes
  // (EgressBudget::wait_ns of every bucket it is charged to, the largest):
  // it is due again then. kNeverNs: it never fits, so it is never due.
  void Deferred(std::int64_t now_ns, std::int64_t wait_ns) {
    if (wait_ns != kNeverNs) retry_ns = std::min(retry_ns, now_ns + wait_ns);
  }

  // The next instant to run, after `now_ns`. `polling` (no wake channel, or
  // pages left unread) brings it back in kFallbackNs. `for_each_feeds(f)`
  // calls f(const ClientFeeds&) for each client. An instant already past is
  // left out: what it was due for was done this pass or waits for budget,
  // which retry_ns covers.
  template <class ForEachFeeds>
  std::int64_t NextWakeNs(std::int64_t now_ns, bool polling,
                          const SystemGraphDocument& document,
                          ForEachFeeds&& for_each_feeds) const {
    if (polling) return now_ns + kFallbackNs;
    std::int64_t next = refreshed_ns + kSystemRefreshNs;
    const auto consider = [&next, now_ns](std::int64_t due_ns) {
      if (due_ns > now_ns && due_ns < next) next = due_ns;
    };
    consider(check_due_ns);
    consider(retry_ns);
    for_each_feeds(
        [&](const ClientFeeds& feeds) { consider(feeds.NextDueNs(document)); });
    return std::max(next, now_ns + 1);
  }
};

static_assert(EgressBudget::framed(EgressBudget::kMaxPayloadBytes) <=
                      talos::telemetry_wire::kClientBurstBytes &&
                  EgressBudget::framed(EgressBudget::kMaxPayloadBytes + 1) >
                      talos::telemetry_wire::kClientBurstBytes,
              "kMaxPayloadBytes is the largest payload the burst affords");

// A finite double in round-trip form; NaN and the infinities, which JSON
// numbers cannot spell, as the strings "NaN", "Infinity" and "-Infinity".
inline void AppendTelemetryDouble(std::string& out, double value) {
  if (std::isnan(value)) {
    out += "\"NaN\"";
    return;
  }
  if (std::isinf(value)) {
    out += value > 0 ? "\"Infinity\"" : "\"-Infinity\"";
    return;
  }
  char buffer[32];
  std::snprintf(buffer, sizeof buffer, "%.17g", value);
  out += buffer;
}

// One channel sample as JSON: a number, or for an integer past 2^53 (which a
// JSON number, read as a double, would round) its exact decimal string.
inline void AppendSample(std::string& out, ChannelKind kind,
                         std::uint64_t raw) {
  constexpr std::uint64_t kExact = std::uint64_t{1} << 53;
  switch (kind) {
    case Talos::Telemetry::ChannelKind_Int: {
      const auto value = static_cast<std::int64_t>(raw);
      const bool exact = value >= -static_cast<std::int64_t>(kExact) &&
                         value <= static_cast<std::int64_t>(kExact);
      if (!exact) out += '"';
      out += std::to_string(value);
      if (!exact) out += '"';
      return;
    }
    case Talos::Telemetry::ChannelKind_UInt:
    case Talos::Telemetry::ChannelKind_Bool:
      if (raw > kExact) out += '"';
      out += std::to_string(raw);
      if (raw > kExact) out += '"';
      return;
    default:
      AppendTelemetryDouble(out, talos::telemetry::DoubleFromRaw(raw));
  }
}

inline std::string_view ChannelKindName(ChannelKind kind) {
  switch (kind) {
    case Talos::Telemetry::ChannelKind_Int:
      return "int";
    case Talos::Telemetry::ChannelKind_UInt:
      return "uint";
    case Talos::Telemetry::ChannelKind_Bool:
      return "bool";
    default:
      return "double";
  }
}

inline void AppendPose(std::string& out, const TelemetryPose& pose) {
  out += "{\"x\":";
  AppendTelemetryDouble(out, pose.x);
  out += ",\"y\":";
  AppendTelemetryDouble(out, pose.y);
  out += ",\"z\":";
  AppendTelemetryDouble(out, pose.z);
  out += ",\"roll\":";
  AppendTelemetryDouble(out, pose.roll);
  out += ",\"pitch\":";
  AppendTelemetryDouble(out, pose.pitch);
  out += ",\"yaw\":";
  AppendTelemetryDouble(out, pose.yaw);
  out += '}';
}

// GET /telemetry.json. Precondition: has_dictionary && has_frame (the
// route answers 503 until then). `now_monotonic_ns` is on the clock
// IngestTelemetry was given.
inline std::string TelemetryJson(const TelemetryState& state,
                                 std::int64_t now_monotonic_ns) {
  std::string out;
  out.reserve(2048);
  out += "{\"kind\":\"talos.telemetry\",\"version\":";
  out += std::to_string(kTelemetryVersion);
  out += ",\"stream_id\":";
  AppendU64(out, state.stream_id);
  out += ",\"sequence_id\":";
  AppendU64(out, state.sequence_id);
  out += ",\"timestamp_ns\":";
  AppendU64(out, state.timestamp_ns);
  out += ",\"dictionary_id\":";
  out += std::to_string(state.dictionary_id);
  out += ",\"age_ms\":";
  const std::int64_t age_ms =
      now_monotonic_ns > state.received_monotonic_ns
          ? (now_monotonic_ns - state.received_monotonic_ns) / 1'000'000
          : 0;
  out += std::to_string(age_ms);
  out += ",\"chassis\":";
  AppendPose(out, state.chassis);
  // Keyed by name (unique by the producer's construction-time check) so a
  // reader goes straight to
  // doc["channels"]["pose_estimator/state/velocity/vx_mps"]["value"];
  // members stay in dictionary order.
  out += ",\"channels\":{";
  for (std::size_t i = 0; i < state.dictionary.size(); ++i) {
    if (i != 0) out += ',';
    const TelemetryChannelInfo& channel = state.dictionary[i];
    const bool present = i < state.present.size() && state.present[i] != 0;
    AppendEscaped(out, channel.name);
    out += ":{\"value\":";
    if (present) {
      AppendSample(out, channel.kind, state.values[i]);
    } else {
      out += "null";
    }
    out += ",\"unit\":";
    AppendEscaped(out, channel.unit);
    // Enum channels name their value, but only in range: an out-of-range
    // code is a number, not a label waiting to be invented.
    if (present && !channel.labels.empty() &&
        channel.kind != Talos::Telemetry::ChannelKind_Double &&
        state.values[i] < channel.labels.size()) {
      out += ",\"label\":";
      AppendEscaped(out, channel.labels[state.values[i]]);
    }
    out += '}';
  }
  out += "}}";
  return out;
}

// GET /channels.json: the dictionary document on its own, so a client can
// learn the channel set without waiting for values it may not need.
// Precondition: has_dictionary (the route answers 503 until then).
inline std::string ChannelsJson(const TelemetryState& state) {
  std::string out;
  out.reserve(2048);
  out += "{\"kind\":\"talos.telemetry.channels\",\"version\":";
  out += std::to_string(kTelemetryVersion);
  out += ",\"stream_id\":";
  AppendU64(out, state.stream_id);
  out += ",\"dictionary_id\":";
  out += std::to_string(state.dictionary_id);
  out += ",\"channels\":[";
  for (std::size_t i = 0; i < state.dictionary.size(); ++i) {
    if (i != 0) out += ',';
    const TelemetryChannelInfo& channel = state.dictionary[i];
    out += "{\"name\":";
    AppendEscaped(out, channel.name);
    out += ",\"unit\":";
    AppendEscaped(out, channel.unit);
    out += ",\"kind\":";
    AppendEscaped(out, ChannelKindName(channel.kind));
    out += ",\"labels\":[";
    for (std::size_t j = 0; j < channel.labels.size(); ++j) {
      if (j != 0) out += ',';
      AppendEscaped(out, channel.labels[j]);
    }
    out += "]}";
  }
  out += "]}";
  return out;
}

}  // namespace studio
