#pragma once

// The ring of each simulation truth topic that is not the default MAX_SLOTS,
// stated once for its writer and its readers (and the --describe of each,
// from which the launcher creates the segment).

#include <cstddef>

namespace talos::hardware::sim {

// /sim/telemetry/pieces (sim.GamePieces, 8336 B): its one reader takes only
// the newest message, so a ring need only outlast one read. At 25 Hz, its
// fastest rate, eight slots take 320 ms to lap, against a copy of
// microseconds; 1024 would hold 8.5 MB for nothing (REQ-DEV-43).
inline constexpr std::size_t kPiecesSlots = 8;

}  // namespace talos::hardware::sim
