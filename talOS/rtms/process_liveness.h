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
// died is not mistaken for that process, and a pid from another pid namespace
// is not looked up in this one:
//
//   - the pid in the low kProcessPidBits bits (a Linux pid is below
//     PID_MAX_LIMIT, 2^22; a macOS pid below 100000);
//   - above it, the low kProcessStartBits bits of the process's start time
//     (0 where the platform did not report it, which compares the pid alone);
//   - above that, the code of the pid namespace the pid was issued in
//     (kProcessNamespaceBits; see "Pid namespaces" below).
//
// A recorded process is alive while a process with its pid runs -- a zombie
// does not -- and started at the recorded time.
//
// The start is in clock ticks since boot on Linux (USER_HZ, 100 per second),
// whose low 20 bits wrap every 2.9 hours, and in milliseconds since the epoch
// on macOS, whose low 20 bits wrap every 17.5 minutes. A pid reused within
// one wrap of its dead process's start never matches it (both kernels hand
// out pids in turn, so no pid comes back within one tick or millisecond); a
// reuse later matches only when it started a whole number of wraps after the
// dead process to the tick or millisecond, about one reuse in a million.
inline constexpr unsigned kProcessPidBits = 22;
inline constexpr unsigned kProcessStartBits = 20;
inline constexpr unsigned kProcessNamespaceBits = 20;
inline constexpr unsigned kProcessNamespaceShift =
    kProcessPidBits + kProcessStartBits;
inline constexpr std::uint64_t kProcessPidMask = (1ULL << kProcessPidBits) - 1;
inline constexpr std::uint64_t kProcessStartMask =
    (1ULL << kProcessStartBits) - 1;
inline constexpr std::uint64_t kProcessNamespaceMask =
    (1ULL << kProcessNamespaceBits) - 1;

// Pid namespaces. Processes in different Linux pid namespaces can share
// /dev/shm (a container run with --ipc=host, or the host and such a
// container), but a pid names a process only in the namespace that issued it:
// kill() and /proc in another namespace find another process or none, so a
// live claimant would look dead there. A check therefore probes a pid only
// from the namespace that recorded it, and otherwise counts the claimant as
// running: only a process known to be gone may lose its claims. (A registry
// row also ages out on its heartbeat; an RTMS claim from another namespace is
// freed only when its segment is unlinked, at the end of a session.)
//
// A namespace is recorded as a code. A Linux pid namespace is known by the
// inode of /proc/self/ns/pid, unique among the live namespaces of the one
// kernel that shared memory implies. The initial namespace's is
// kInitialPidNamespaceInode; every other comes from the one allocator that
// also numbers every namespace of every type and every dynamically
// registered /proc entry (per-IRQ /proc/irq/<n>/*, /proc/net/* of each
// network namespace, driver entries), lowest free number first, from
// 0xF0000000 upwards. So a new namespace's offset tracks how many such
// entries the host holds: a few thousand on a container host. The code is
// that inode's offset from the initial one, plus one; 20 bits keep it exact
// up to about a million live entries.
//
//   kProcessNamespaceNone (0): no pid namespaces (macOS); compares the pid
//     alone, like an unknown start.
//   kProcessNamespaceOpaque: a namespace that cannot be compared -- its inode
//     does not fit, /proc/self/ns/pid could not be read, or (as the checking
//     side only) this process's /proc is not its own pid namespace's
//     (/proc/self/status NSpid lists more than one pid, or none). It matches
//     nothing, itself included: a process whose checks are opaque reclaims
//     no claim at all, so in its session a crashed publisher's topic refuses
//     its restart and dead readers' slots stay held.
inline constexpr std::uint64_t kProcessNamespaceNone = 0;
inline constexpr std::uint64_t kProcessNamespaceOpaque = kProcessNamespaceMask;
inline constexpr std::uint64_t kInitialPidNamespaceInode = 0xEFFFFFFCULL;

namespace talos {

// The code of the Linux pid namespace whose /proc/<pid>/ns/pid has `inode`.
constexpr std::uint64_t PidNamespaceCode(std::uint64_t inode) {
  return inode >= kInitialPidNamespaceInode &&
                 inode - kInitialPidNamespaceInode < kProcessNamespaceOpaque - 1
             ? inode - kInitialPidNamespaceInode + 1
             : kProcessNamespaceOpaque;
}

// Whether a process in namespace `checker` can look up a pid recorded in
// namespace `recorded` (both codes).
constexpr bool ProcessNamespaceComparable(std::uint64_t recorded,
                                          std::uint64_t checker) {
  return recorded == kProcessNamespaceNone ||
         (recorded == checker && recorded != kProcessNamespaceOpaque);
}

// This process's word. Cached per process, so a forked child records itself
// rather than its parent.
std::uint64_t OwnProcessWord();

// Whether the process a word records still runs, as seen from this process:
// ProcessWordAliveFrom with this process's namespace code.
bool ProcessWordAlive(std::uint64_t word);

// Whether the process a word records still runs, as seen from a process in
// namespace `checker`. A word from a namespace `checker` cannot compare with
// (ProcessNamespaceComparable) counts as running. Otherwise a process with
// its pid must run and have started when the recorded one did; a start
// either side does not know (0) compares the pid alone. A pid that exists but
// cannot be inspected (another user's process hidden from /proc, EPERM)
// counts as running.
bool ProcessWordAliveFrom(std::uint64_t word, std::uint64_t checker);

}  // namespace talos
