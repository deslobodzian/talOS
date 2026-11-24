#pragma once

#include <cstdint>
#include <string_view>

#include "talOS/configuration/config_parser.h"
#include "talOS/hardware/sim_physics.h"

// The lightweight simulator's program (README.md): flag parsing, --describe,
// the robot config applied to the physics backend, the UDP endpoint, the
// registry row and the wall-clock loop. `sim_gateway` runs it as is. A
// program that calls RunSimGateway with an extension can configure the
// physics backend before the first Reconfigure and run after each tick.
namespace talos::simulation {

// Every hook runs on the gateway's only thread. A hook's std::exception ends
// the program with exit 1 and its message on stderr, without the summary.
class GatewayExtension {
 public:
  virtual ~GatewayExtension() = default;
  // After [simulation] is applied to `backend`, before Reconfigure; throw
  // std::invalid_argument to refuse (exit 1 with the message).
  virtual void Configure(const config::RobotConfig& config,
                         const hardware::sim::PhysicsOptions& options,
                         hardware::sim::PhysicsSimBackend& backend) = 0;
  virtual void AfterTick(uint64_t now_us) = 0;  // throw: exit 1
  // Only on a normal end (the stop signal or --duration-s), after the
  // outputs are neutralised and the summary line is printed; not after a
  // hook or the loop has thrown. Appends to the stdout summary.
  virtual void Finish() = 0;
};

struct GatewayProgramInfo {
  std::string_view target;        // --describe and registry target
  std::string_view backend_name;  // stderr "backend=" for the physics backend
  bool physics_only{false};       // --physics implied and accepted
};

// The gateway's whole program; returns main's exit code. `extension` may be
// null. A hook's exception exits 1 with its message on stderr, as every other
// error does.
int RunSimGateway(int argc, char** argv, const GatewayProgramInfo& program,
                  GatewayExtension* extension);

}  // namespace talos::simulation
