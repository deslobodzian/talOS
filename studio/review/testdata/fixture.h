#pragma once

// A recorded session for //studio/review's tests, written with the real
// LogWriter:
//
//   shooter.segment<k>.tlog, shooter.tlog  its first run, byte-capped so it
//                                          rotated (no segment deleted)
//   shooter.restart1.tlog                  its restart
//       a tick every 20 ms (shots 0..299 from 0 s, 350..499 from 7 s) that
//       sends /shooter/state (review_test.Shot) and /hw/request/shooter (a
//       hardware Command), each published 0.5 ms after its tick; the last
//       shot's position is NaN
//   observer.tlog                          the reader, whose loop started
//                                          0.5 s before the first record
//       every 5th /shooter/state, received 1 ms after its publication, and
//       a /hw/state/driver_station packet every 100 ms from 0 s to 10 s,
//       received 2 ms after its publication, enabled from 5 s; the record of
//       packet 40 is damaged (a payload byte flipped)
//   manifest.json                          a finished --sim session
//
// Times are kStartNs plus the offsets above, on the recording's monotonic
// clock.

#include <cstdint>
#include <string>

namespace studio::review::testing {

inline constexpr std::int64_t kStartNs = 1'000'000'000'000;  // 1000 s
inline constexpr std::uint64_t kSessionId = 0x5E55;
inline constexpr int kDamagedPacket = 40;
inline constexpr int kLastShot = 499;
inline constexpr std::int64_t kObserverOriginNs = -500'000'000;

// The shot of tick `k`: position_m, shot_count, mode and ready.
double ShotPosition(int k);

// Writes the session into `dir`, which must not exist yet. Throws
// std::runtime_error when a log cannot be written.
void WriteFixtureSession(const std::string& dir);

}  // namespace studio::review::testing
