#pragma once

// Whether a process recorded in shared memory still runs.
//
// Shared state outlives the processes that claim parts of it: an RTMS writer
// or reader slot (rtms.h), a live registry row (talOS/introspection). A
// process that dies without releasing its claim -- SIGKILL, a crash -- leaves
// it behind, and only a check against the kernel tells a dead claimant from a
// live one. A pid alone is not enough: a pid reused after its process died
// would keep the dead process's claims. So a claim records a process word,
// and this is the one implementation of reading it back.

#include <cstdint>

// How a process is recorded -- an RTMS segment's writer and each reader slot's
// claimant, a registry row's claimant -- so that a pid reused after its process
// died is not mistaken for that process: the pid in the low kProcessPidBits
// bits and, above them, the low kProcessStartBits bits of the process's start
// time (clock ticks since boot on Linux, microseconds since the epoch on macOS;
// 0 where the platform did not report it, which compares the pid alone). A
// recorded process is alive while a process with its pid runs -- a zombie
// does not -- and started at the recorded time.
inline constexpr unsigned kProcessPidBits = 32;
inline constexpr unsigned kProcessStartBits = 30;
inline constexpr std::uint64_t kProcessPidMask = (1ULL << kProcessPidBits) - 1;
inline constexpr std::uint64_t kProcessStartMask =
    (1ULL << kProcessStartBits) - 1;

namespace talos {

// This process's word. Cached per pid, so a forked child records itself
// rather than its parent.
std::uint64_t OwnProcessWord();

// Whether the process a word records still runs: a process with its pid runs
// and started when the recorded one did. A start either side does not know
// (0) compares the pid alone. A pid that exists but cannot be inspected
// (another user's process hidden from /proc, EPERM) counts as running: only a
// process known to be gone may lose its claims.
bool ProcessWordAlive(std::uint64_t word);

}  // namespace talos
