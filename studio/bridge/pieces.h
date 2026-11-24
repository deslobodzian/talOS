#pragma once

/*
 * Game pieces for Studio's 3D viewport (REQ-DEV-43): the bridge's side of the
 * MuJoCo gateway's /sim/telemetry/pieces (sim.GamePieces,
 * talOS/hardware/sim.fbs).
 *
 * - IngestPieces keeps the latest message, its positions quantised to whole
 *   millimetres.
 * - PiecesFeed sends one programmer client what brings it to that latest
 *   message: a keyframe of every piece when it joins (or the field changes),
 *   then only the pieces whose quantised position or in-play flag changed
 *   since the last message it was sent. A send the budget cannot afford is
 *   not queued: the next one compares against the latest message, so a slow
 *   client gets a later, larger delta and never a backlog. Its memory is
 *   fixed at construction.
 * - PumpPieces offers a client its pieces from an allowance of a quarter
 *   of its rate (kPiecesShare), after any pending graph message, dictionary
 *   page or telemetry keyframe; while a pieces message is owed, telemetry
 *   updates leave room for it (PiecesRoom). Bounds at PumpPieces.
 *
 * The message, one binary WebSocket frame (README.md "Game pieces"), all
 * little-endian:
 *
 *   0  4 B  magic "PCS1"
 *   4  1 B  flags: bit 0 keyframe (start from no piece)
 *   5  1 B  0
 *   6  2 B  u16 records that follow
 *   8  4 B  u32 pieces_count, the field's pieces
 *  12  4 B  f32 radius_m
 *  16  8 B  u64 sample_time_ns, the gateway's node clock
 *  24       records, 16 B each:
 *             u16 piece index, u8 flags (bit 0 in play), u8 0,
 *             i32 x_mm, i32 y_mm, i32 z_mm (field frame)
 */

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "sim/sim_generated.h"
#include "studio/bridge/telemetry.h"
#include "talOS/telemetry/wire.h"

namespace studio {

// sim.GamePieces's arrays (MujocoGatewayExtension's kMaxGamePieces).
inline constexpr std::uint32_t kMaxPieces = 640;
static_assert(kMaxPieces <= std::numeric_limits<std::uint16_t>::max(),
              "piece indices and record counts must fit u16");
static_assert(sizeof(::sim::GamePieces) == 16 + kMaxPieces * 13);

inline constexpr std::string_view kPiecesMagic = "PCS1";
inline constexpr std::uint8_t kPiecesKeyframe = 1;
inline constexpr std::uint8_t kPieceInPlay = 1;
inline constexpr std::size_t kPiecesHeaderBytes = 24;
inline constexpr std::size_t kPieceRecordBytes = 16;
// A keyframe of 640 pieces: 10,264 B, 10,274 framed.
inline constexpr std::size_t kMaxPiecesMessageBytes =
    kPiecesHeaderBytes + kMaxPieces * kPieceRecordBytes;
static_assert(EgressBudget::framed(kMaxPiecesMessageBytes) <=
                  talos::telemetry_wire::kClientBurstBytes,
              "the largest pieces message must fit a client's burst");

// The latest message, positions in whole millimetres.
struct PiecesState {
  bool has{false};
  std::uint64_t sample_time_ns{0};
  std::uint32_t count{0};
  float radius_m{0.0F};
  std::array<std::int32_t, std::size_t{3} * kMaxPieces>
      mm{};  // x, y, z per piece
  std::array<std::uint8_t, kMaxPieces> in_play{};
  // Advances with each message ingested, so a feed that sent this one skips
  // the comparison.
  std::uint64_t generation{0};
};

namespace detail {

// `metres` to the nearest millimetre; false when that is no int32.
inline bool Millimetres(float metres, std::int32_t& out) {
  const double mm = std::round(static_cast<double>(metres) * 1000.0);
  if (!(mm >= -2147483648.0 && mm <= 2147483647.0)) return false;
  out = static_cast<std::int32_t>(mm);
  return true;
}

inline void PutU16(char* at, std::uint16_t value) {
  at[0] = static_cast<char>(value & 0xFFU);
  at[1] = static_cast<char>(value >> 8U);
}
inline void PutU32(char* at, std::uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    at[i] =
        static_cast<char>((value >> (8U * static_cast<unsigned>(i))) & 0xFFU);
  }
}
inline void PutU64(char* at, std::uint64_t value) {
  for (int i = 0; i < 8; ++i) {
    at[i] =
        static_cast<char>((value >> (8U * static_cast<unsigned>(i))) & 0xFFU);
  }
}

}  // namespace detail

// Takes `message` as the latest, quantised. False, leaving `state` as it
// was, for one Studio cannot draw: more than kMaxPieces pieces, a radius
// that is not finite or negative, or a coordinate that is not finite or past
// an int32 of millimetres.
inline bool IngestPieces(PiecesState& state, const ::sim::GamePieces& message) {
  const std::uint32_t count = message.pieces_count();
  const float radius = message.radius_m();
  if (count > kMaxPieces || !std::isfinite(radius) || radius < 0.0F) {
    return false;
  }
  std::array<std::int32_t, std::size_t{3} * kMaxPieces> mm{};
  for (std::size_t i = 0; i < count; ++i) {
    const auto at = static_cast<flatbuffers::uoffset_t>(i);
    if (!detail::Millimetres(message.x_m()->Get(at), mm[3 * i]) ||
        !detail::Millimetres(message.y_m()->Get(at), mm[3 * i + 1]) ||
        !detail::Millimetres(message.z_m()->Get(at), mm[3 * i + 2])) {
      return false;
    }
  }
  state.mm = mm;
  for (std::uint32_t i = 0; i < kMaxPieces; ++i) {
    state.in_play[i] = i < count && message.in_play()->Get(
                                        static_cast<flatbuffers::uoffset_t>(i))
                           ? 1
                           : 0;
  }
  state.count = count;
  state.radius_m = radius == 0.0F ? 0.0F : radius;
  state.sample_time_ns = message.sample_time_ns();
  state.has = true;
  ++state.generation;
  return true;
}

// A programmer client's pieces allowance: pieces accrue kPiecesShare of
// the client's rate, up to one framed keyframe of kMaxPieces
// (kPiecesAllowanceBytes), so the first keyframe goes at once and deltas
// never bank more than one keyframe.
inline constexpr double kPiecesShare = 0.25;
inline constexpr std::size_t kPiecesAllowanceBytes =
    EgressBudget::framed(kMaxPiecesMessageBytes);
// While a pieces message is owed, a telemetry update must fit the client's
// bucket beside it (PiecesRoom); were the largest update and the largest
// pieces message more than a burst, that update could never be sent while
// one is owed.
static_assert(talos::telemetry_wire::kMaxUpdateBytes + kPiecesAllowanceBytes <=
                  talos::telemetry_wire::kClientBurstBytes,
              "the largest update and pieces message must fit one burst");
static_assert(static_cast<double>(talos::telemetry_wire::kMaxUpdateBytes +
                                  kPiecesAllowanceBytes) <=
                  EgressBudget::kBurstBytes,
              "the largest update and pieces message must fit one burst");

// What one client has been sent of the pieces, and its allowance.
class PiecesFeed {
 public:
  PiecesFeed() {
    message_.reserve(kMaxPiecesMessageBytes);
    changed_.reserve(kMaxPieces);
  }

  // The framed size of the message the client is owed for `state` (a
  // keyframe when it holds none, or the count or radius changed; else the
  // pieces whose millimetre position or in-play flag changed since its last
  // message); 0 when it holds `state` already. Rebuilt against the latest
  // message each time that changes: nothing is queued. Every message is at
  // most a keyframe of state.count pieces, so at most the allowance's cap,
  // which the allowance always comes to cover.
  std::size_t Owed(const PiecesState& state) {
    if (!state.has) {
      has_ = false;
      prepared_ = false;
      return 0;
    }
    if (state.count > kMaxPieces || !std::isfinite(state.radius_m) ||
        state.radius_m < 0.0F || (has_ && generation_ == state.generation)) {
      return 0;
    }
    if (!prepared_ || prepared_generation_ != state.generation) {
      Prepare(state);
    }
    if (!keyframe_ && changed_.empty()) {
      generation_ = state.generation;  // it holds this message already
      return 0;
    }
    return EgressBudget::framed(kPiecesHeaderBytes +
                                changed_.size() * kPieceRecordBytes);
  }

  // Sends the owed message (Owed) when the allowance, refilling at
  // kPiecesShare of `client_rate`, covers it and `afford(framed_bytes)`, the
  // client's own and the shared bucket, allow it; `send(bytes)` sends it, is
  // charged for it, and returns false if it was not sent (the next message
  // is then a keyframe). The allowance is spent with the buckets. Returns
  // whether it sent one.
  template <class Afford, class Send>
  bool Pump(const PiecesState& state, std::int64_t now_ns, double client_rate,
            Afford&& afford, Send&& send) {
    wait_ns_ = kNeverNs;
    const std::size_t framed = Owed(state);
    if (framed == 0 || framed > kPiecesAllowanceBytes) return false;
    if (!std::isfinite(client_rate) || client_rate <= 0.0) return false;
    const double rate = kPiecesShare * client_rate;
    if (!std::isfinite(rate) || rate <= 0.0) return false;
    if (!Allowance(framed, rate, now_ns)) {
      const double wait =
          std::ceil((static_cast<double>(framed) - tokens_) / rate * 1e9);
      if (std::isfinite(wait) && wait >= 0.0 &&
          wait < static_cast<double>(kNeverNs)) {
        const auto wait_i64 =
            std::max<std::int64_t>(1, static_cast<std::int64_t>(wait));
        if (now_ns <= 0 || wait_i64 <= kNeverNs - now_ns) {
          wait_ns_ = wait_i64;
        }
      }
      return false;
    }
    if (!afford(framed)) return false;
    Encode(state, keyframe_);
    if (!send(std::string_view{message_})) {
      has_ = false;
      prepared_ = false;
      return false;
    }
    tokens_ -= static_cast<double>(framed);
    for (const std::size_t i : changed_) {
      in_play_[i] = state.in_play[i] != 0 ? std::uint8_t{1} : std::uint8_t{0};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        mm_[3 * i + axis] = state.mm[3 * i + axis];
      }
    }
    if (keyframe_) {
      for (std::size_t i = state.count; i < kMaxPieces; ++i) {
        in_play_[i] = 0;
        mm_[3 * i] = 0;
        mm_[3 * i + 1] = 0;
        mm_[3 * i + 2] = 0;
      }
    }
    has_ = true;
    count_ = state.count;
    radius_m_ = state.radius_m == 0.0F ? 0.0F : state.radius_m;
    generation_ = state.generation;
    prepared_ = false;
    return true;
  }

  // How long after this pass's Pump its allowance covers the owed message;
  // kNeverNs when none waited on the allowance, or no Pump ran this pass
  // (Decline).
  std::int64_t wait_ns() const { return wait_ns_; }
  // A pass in which the pieces may not go: nothing waits on the allowance.
  void Decline() { wait_ns_ = kNeverNs; }

 private:
  void Prepare(const PiecesState& state) {
    const float radius = state.radius_m == 0.0F ? 0.0F : state.radius_m;
    const std::size_t count = std::min<std::size_t>(state.count, kMaxPieces);
    keyframe_ = !has_ || count_ != state.count || radius_m_ != radius;
    changed_.clear();
    for (std::size_t i = 0; i < count; ++i) {
      const std::uint8_t in_play =
          state.in_play[i] != 0 ? std::uint8_t{1} : std::uint8_t{0};
      if (keyframe_ || in_play_[i] != in_play ||
          mm_[3 * i] != state.mm[3 * i] ||
          mm_[3 * i + 1] != state.mm[3 * i + 1] ||
          mm_[3 * i + 2] != state.mm[3 * i + 2]) {
        changed_.push_back(static_cast<std::uint16_t>(i));
      }
    }
    prepared_ = true;
    prepared_generation_ = state.generation;
  }

  // Refills the allowance at `rate` B/s up to its cap; true when `bytes`
  // fit now.
  bool Allowance(std::size_t bytes, double rate, std::int64_t now_ns) {
    if (started_ && now_ns > updated_ns_) {
      const double dt_ns =
          (updated_ns_ >= 0 || now_ns <= kNeverNs + updated_ns_)
              ? static_cast<double>(now_ns - updated_ns_)
              : static_cast<double>(now_ns) - static_cast<double>(updated_ns_);
      tokens_ = std::min(static_cast<double>(kPiecesAllowanceBytes),
                         tokens_ + dt_ns * 1e-9 * rate);
    }
    if (!started_ || now_ns > updated_ns_) updated_ns_ = now_ns;
    started_ = true;
    return tokens_ >= static_cast<double>(bytes);
  }

  void Encode(const PiecesState& state, bool keyframe) {
    message_.resize(kPiecesHeaderBytes + changed_.size() * kPieceRecordBytes);
    char* out = message_.data();
    for (std::size_t i = 0; i < kPiecesMagic.size(); ++i) {
      out[i] = kPiecesMagic[i];
    }
    out[4] = static_cast<char>(keyframe ? kPiecesKeyframe : 0);
    out[5] = 0;
    detail::PutU16(out + 6, static_cast<std::uint16_t>(changed_.size()));
    detail::PutU32(out + 8, state.count);
    const float radius = state.radius_m == 0.0F ? 0.0F : state.radius_m;
    detail::PutU32(out + 12, std::bit_cast<std::uint32_t>(radius));
    detail::PutU64(out + 16, state.sample_time_ns);
    char* record = out + kPiecesHeaderBytes;
    for (const std::size_t i : changed_) {
      detail::PutU16(record, static_cast<std::uint16_t>(i));
      record[2] = static_cast<char>(state.in_play[i] != 0 ? kPieceInPlay : 0);
      record[3] = 0;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        detail::PutU32(record + 4 + 4 * axis,
                       static_cast<std::uint32_t>(state.mm[3 * i + axis]));
      }
      record += kPieceRecordBytes;
    }
  }

  bool has_{false};
  std::uint32_t count_{0};
  float radius_m_{0.0F};
  std::uint64_t generation_{0};
  std::array<std::int32_t, std::size_t{3} * kMaxPieces> mm_{};
  std::array<std::uint8_t, kMaxPieces> in_play_{};
  // The owed message's pieces, for the state generation they were built
  // against; scratch reserved at construction, as is the message.
  bool prepared_{false};
  bool keyframe_{false};
  std::uint64_t prepared_generation_{0};
  std::vector<std::uint16_t> changed_;
  std::string message_;
  // The allowance (Allowance).
  double tokens_{static_cast<double>(kPiecesAllowanceBytes)};
  std::int64_t updated_ns_{0};
  bool started_{false};
  std::int64_t wait_ns_{kNeverNs};
};

// What a client's telemetry updates leave in its own bucket (not the shared
// one: AffordProgrammerSend in drive.h) for the pieces message it is owed
// (ClientFeeds::Pump's `pieces_room`): that message's framed size while one is
// owed and nothing of higher priority waits (ClientFeeds::PiecesMayGo), else 0.
// Held while the allowance refills too, so the buckets hold the message by the
// time the allowance covers it: the pieces get their whole kPiecesShare, and
// elastic telemetry, which would fill the client's share, cannot take the bytes
// first.
inline std::size_t PiecesRoom(const ClientFeeds& feeds,
                              const TelemetryState& telemetry,
                              PiecesFeed& pieces, const PiecesState& state) {
  if (!feeds.PiecesMayGo(telemetry)) return 0;
  return pieces.Owed(state);
}

// Offers a programmer client its pieces after its graph and telemetry, at
// most kPiecesShare of its rate (the allowance). They wait while a graph
// message, a dictionary page or a telemetry keyframe is pending
// (ClientFeeds::PiecesMayGo), and are charged to the client's own and the
// shared bucket like any send; they never take the shared reservation.
// While a pieces message is owed, the client's telemetry updates leave room
// for it (PiecesRoom). Bounds, for a client of rate R and a message of N
// changed records (24 + 16 N B, plus 10 B of framing from 126 B):
// - the pieces spend at most kPiecesShare R over any window, plus one
//   allowance (10,274 B), so saturating telemetry keeps at least 3/4 of R;
// - a telemetry update waits at most one pieces message's bytes / R longer
//   than it would without pieces (a 504-piece keyframe, 8,098 B: 0.29 s at
//   28,000 B/s, 0.20 s at 40,000; ten pieces, 194 B: 7 ms, 5 ms);
// - pieces go at min(20, kPiecesShare R / bytes) messages a second, the
//   gateway's 20 Hz at most: 10 moving FUEL at 20 Hz at either share, 100
//   at 4.3 Hz (28,000 B/s) or 6.1 Hz (40,000), all 504 at 0.86 or 1.2 Hz;
//   after motion stops the client holds the latest positions within one
//   message's bytes / (kPiecesShare R) plus 50 ms (1.21 s for 504 pieces
//   at 28,000 B/s).
// `afford(framed_bytes)` and `send` are as in PiecesFeed::Pump.
template <class Afford, class Send>
bool PumpPieces(const ClientFeeds& feeds, const TelemetryState& telemetry,
                PiecesFeed& pieces, const PiecesState& state,
                std::int64_t now_ns, double client_rate, Afford&& afford,
                Send&& send) {
  if (!feeds.PiecesMayGo(telemetry)) {
    pieces.Decline();
    return false;
  }
  return pieces.Pump(state, now_ns, client_rate, std::forward<Afford>(afford),
                     std::forward<Send>(send));
}

}  // namespace studio
