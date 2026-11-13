#pragma once

// Keeping memory resident for a realtime loop: every page faulted in for
// writing up front and, where the OS allows, locked (mlock), so the loop never
// takes a page fault on it. Below every node layer because SharedMemoryPtr,
// which maps every RTMS ring, registers its mappings here.

#include <cstddef>

namespace talos::memory {

// What making a set of ranges resident achieved.
struct Residency {
  std::size_t ranges = 0;  // ranges asked for
  std::size_t bytes = 0;   // their total size
  std::size_t failed = 0;  // ranges mlock refused (they were still prefaulted)
  int first_error = 0;     // errno of the first refusal, 0 when none
};

// Faults in every page of [address, address + size) for writing, without
// changing a byte (an atomic OR with zero per page, safe while other
// processes use the memory), then mlocks the range. Returns 0, or the errno
// mlock set (ENOMEM or EPERM past RLIMIT_MEMLOCK or the wired limit); the
// pages are prefaulted either way. `address` must be page aligned and the
// range mapped readable and writable.
int MakeResident(void* address, std::size_t size) noexcept;

// From now on, every SharedMemoryPtr mapping in this process is made
// resident when it is mapped, and the ones mapped already are made resident
// now; the result covers those. Idempotent. A later mapping that mlock
// refuses is reported once per process on stderr and kept anyway.
Residency KeepMappingsResident() noexcept;

// Whether KeepMappingsResident has been called in this process.
bool MappingsKeptResident() noexcept;

// SharedMemoryPtr's bookkeeping: a mapping registers when mapped (and is
// made resident then if KeepMappingsResident was called) and unregisters
// before it is unmapped.
void TrackMapping(void* address, std::size_t size);
void UntrackMapping(void* address) noexcept;

}  // namespace talos::memory
