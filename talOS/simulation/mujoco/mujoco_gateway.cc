// mujoco_gateway: sim_gateway --physics with a MuJoCo world as the plant
// (README.md).

#include <chrono>
#include <cstdint>
#include <iostream>

#include "talOS/simulation/gateway_main.h"
#include "talOS/simulation/mujoco/gateway_extension.h"
#include "talOS/simulation/mujoco/plant.h"

int main(int argc, char** argv) {
  talos::simulation::mujoco::InstallMujocoProcessHandlers();
  talos::simulation::mujoco::MujocoGatewayExtension extension{{
      .field = talos::simulation::mujoco::RunfilesFieldResolver(
          argc > 0 ? argv[0] : ""),
      .wall_clock_ns =
          [] {
            return static_cast<int64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
          },
      .log = &std::cerr,
      .summary = &std::cout,
  }};
  return talos::simulation::RunSimGateway(
      argc, argv, {"//talOS/simulation/mujoco:mujoco_gateway", "mujoco", true},
      &extension);
}
