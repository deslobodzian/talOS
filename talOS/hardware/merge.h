#pragma once

// The bridge's request merge rules, also run by talos::testing::SimHardware
// when it plays the bridge, so a Sim test sees the owner timeout and merged
// observation the robot has.
//
// Each actuator-owning subsystem (hardware::Devices::owns_actuators) is one
// MergeOwner. Its request is admitted when its observation (the gateway
// sample time of the State it was built from) is no newer than the latest
// State, younger than command_timeout_us against it, and no older than the
// owner's last admitted one. Admitting it puts the owner in the merge until
// its last admitted request is more than OwnerTimeoutUs old on the merging
// side's clock. The merged command carries MergedObservationUs, the oldest
// observation among owners still merged, and the gateway invalidates every
// output (a new epoch) once that is command_timeout_us old: one silent owner
// still merged stops every output.

#include <algorithm>
#include <cstdint>
#include <optional>

namespace talos::hardware {

// How long the merge keeps a silent owner's last request:
// max(2 x period_us, command_timeout_us / 2), never more than
// command_timeout_us. Upper side: a silent owner must leave the merge before
// its last observation reaches the gateway's limit, which holds while that
// observation was younger than command_timeout_us minus this timeout when it
// was admitted. The configuration refuses an actuator owner whose period
// exceeds a quarter of the timeout, so every validated owner's timeout is
// exactly half of it; an owner whose last observation was already older stops
// every output, the cost of tolerating request gaps up to half the timeout.
// Lower side: one late or missed tick must not brake the subsystem, so never
// less than two of its periods; with the clamp, that bound only matters for
// an unvalidated config.
std::uint64_t OwnerTimeoutUs(std::uint64_t period_us,
                             std::uint64_t command_timeout_us);

// One actuator owner's standing in the merge. Seen times are on the merging
// side's clock (the bridge's, or sim time), observations on the gateway's.
struct MergeOwner {
  std::uint64_t timeout_us{0};
  // Whether a request has been admitted since the last Reset().
  bool seen{false};
  std::uint64_t last_seen_us{0};
  // Last admitted observation: a request observed before it is a replay and
  // is refused even within the age bound.
  std::uint64_t last_observed_time_us{0};
  bool timed_out{true};

  // Whether a request observed at `observed_time_us` may merge, given the
  // sample time of the latest State received. Identity (config, boot, epoch)
  // is the caller's check.
  bool Admits(std::uint64_t observed_time_us,
              std::uint64_t latest_sample_time_us,
              std::uint64_t command_timeout_us) const;
  // An admitted request, received at `now_us`.
  void Accept(std::uint64_t now_us, std::uint64_t observed_time_us);
  // Times the owner out once its last admitted request is more than
  // timeout_us old (or there is none); returns timed_out.
  bool Expire(std::uint64_t now_us);
  // The first instant at which Expire() times the owner out, nullopt while
  // it is already timed out: a caller that sleeps between calls wakes then.
  std::optional<std::uint64_t> ExpiresAtUs() const;
  // Forgets every admitted request: a controller transition (boot, epoch,
  // configuration) that makes them obsolete.
  void Reset();
};

// The merged command's observation: the oldest last observation among owners
// still merged, or nullopt when none is. `owner_of` maps an element of
// `owners` to its MergeOwner.
template <class Owners, class OwnerOf>
std::optional<std::uint64_t> MergedObservationUs(const Owners& owners,
                                                 OwnerOf owner_of) {
  std::optional<std::uint64_t> oldest;
  for (const auto& entry : owners) {
    const MergeOwner& owner = owner_of(entry);
    if (owner.timed_out) continue;
    oldest = std::min(oldest.value_or(owner.last_observed_time_us),
                      owner.last_observed_time_us);
  }
  return oldest;
}

}  // namespace talos::hardware
