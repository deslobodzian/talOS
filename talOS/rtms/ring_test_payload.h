#pragma once

// A message for the ring's concurrency tests (ring_handoff_test.cc,
// ring_lap_test.cc) that no torn or misplaced copy can pass for a real one.

#include <cstddef>
#include <cstdint>
#include <span>

namespace talos::rtms_test {

// Publication `sequence`: word i holds sequence * words + i, so a copy that
// mixes two publications, or reads the right bytes from the wrong offset,
// matches no single sequence across the whole message width.
inline void FillRingPayload(std::span<std::uint64_t> words,
                            std::uint64_t sequence) {
  for (std::size_t i = 0; i < words.size(); ++i) {
    words[i] = (sequence * words.size()) + i;
  }
}

// The index of the first word that is not publication `sequence`'s, or
// words.size() when the copy is intact.
inline std::size_t FirstForeignWord(std::span<const std::uint64_t> words,
                                    std::uint64_t sequence) {
  for (std::size_t i = 0; i < words.size(); ++i) {
    if (words[i] != (sequence * words.size()) + i) return i;
  }
  return words.size();
}

// The publication time written with publication `sequence`: the slot's
// timestamp must come from the same publication as its payload.
inline std::int64_t RingPublishTime(std::uint64_t sequence) {
  return (static_cast<std::int64_t>(sequence) * 1'000) + 17;
}

}  // namespace talos::rtms_test
