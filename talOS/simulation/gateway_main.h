#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/manifest.h"
#include "talOS/hardware/sim_physics.h"

// The lightweight simulator's program (README.md): flag parsing, --describe,
// the robot config applied to the physics backend, the UDP endpoint, the
// registry row and the wall-clock loop. `sim_gateway` runs it as is. A
// program that calls RunSimGateway with an extension can configure the
// physics backend before the first Reconfigure, run before and after each
// tick, stamp the Driver Station packets and declare topics of its own.
namespace talos::simulation {

// What BeforeTick sees of the tick about to run.
struct TickInput {
  uint64_t now_us{};                    // this tick's node-clock time
  std::optional<uint64_t> schedule_us;  // since the schedules' time zero
  // The Driver Station data the tick uses: its flags enable the gateway, and
  // it goes out in this tick's packet if one is due. Valid during the call.
  const driver_station::DriverStationData* driver_station{};
};

// The FMS fields a Driver Station packet carries that only an extension sets.
// The gateway keeps one FmsStamp for the whole run, all zero until the first
// BeforeTick writes it; it copies it into every packet it sends until the
// next BeforeTick. Without an extension the packets carry match_time_s 0
// and an empty game_specific_message. The message ends at its first NUL and
// at most at kMaxGameMessageLength - 1 characters (the codec's limit). A
// non-finite match_time_s goes out as 0.
struct FmsStamp {
  float match_time_s{0.0f};
  std::array<char, driver_station::kMaxGameMessageLength>
      game_specific_message{};
};

// One of the extension's own topics, as the registry row reports it (the
// gateway reports /sim/state's the same way): messages written, writes that
// failed, the last write's sequence and node-clock time in ns.
struct TopicCount {
  uint64_t samples{}, dropped{}, sequence{};
  int64_t last_ns{};
};

// Every hook runs on the gateway's only thread. A hook's std::exception ends
// the program with exit 1 and its message on stderr, without the summary. A
// program with an extension always runs the physics backend, as if --physics
// were given. Anything the extension attaches to the backend (an external
// plant) must outlive RunSimGateway.
class GatewayExtension {
 public:
  virtual ~GatewayExtension() = default;
  // After [simulation] is applied to `backend`, before Reconfigure; throw
  // std::invalid_argument to refuse (exit 1 with the message).
  virtual void Configure(const config::RobotConfig& config,
                         const hardware::sim::PhysicsOptions& options,
                         hardware::sim::PhysicsSimBackend& backend) = 0;
  // Every tick, after the Driver Station data is stamped with the tick's mode
  // and before Endpoint::Tick, whether or not a Driver Station packet is due
  // this tick; `fms` is the stamp (FmsStamp) the extension may rewrite.
  // `schedule_us` is set once the schedules' time zero (the loop's start, or
  // --schedule-start-file's time) is known and reached. Throw: exit 1.
  virtual void BeforeTick(const TickInput& tick, FmsStamp& fms) {}
  virtual void AfterTick(uint64_t now_us) = 0;  // throw: exit 1
  // The topics the extension publishes itself (SENDER entries; the gateway
  // does not open them; ids are renumbered after the gateway's own). Called
  // once per program, before Configure, for --describe's manifest as well as
  // the registry row, so it must not depend on the config or Configure: the
  // launcher refuses a run whose endpoints differ from its --describe probe.
  // --describe calls no other hook. The program exits 1, --describe included,
  // when an entry's name repeats /sim/state's or another entry's, or when
  // /sim/state and the entries exceed a registry row's
  // introspect::kMaxSourcesPerNode (64), so at most 63 entries. Throw: exit 1.
  virtual event::Manifest Topics() const { return {}; }
  // After AfterTick, once per tick while the registry row is up: one entry
  // per Topics() entry, in its order, zeroed before the call. Throw: exit 1.
  virtual void Counts(std::span<TopicCount> out) const {}
  // Only on a normal end (the stop signal or --duration-s), after the
  // outputs are neutralised and the summary line is printed; not after a
  // hook or the loop has thrown. Appends to the stdout summary.
  virtual void Finish() = 0;
};

struct GatewayProgramInfo {
  std::string_view target;        // --describe and registry target
  std::string_view backend_name;  // stderr "backend=" for the physics backend
  bool physics_only{false};       // --physics implied and accepted
                                  // (an extension implies it too)
};

// The gateway's whole program; returns main's exit code. `extension` may be
// null. A hook's exception exits 1 with its message on stderr, as every other
// error does.
int RunSimGateway(int argc, char** argv, const GatewayProgramInfo& program,
                  GatewayExtension* extension);

}  // namespace talos::simulation
