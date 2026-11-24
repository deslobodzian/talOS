// The MuJoCo gateway, //talOS/simulation/mujoco:sim_gateway: sim_gateway
// --physics with a MuJoCo world as the plant (README.md).

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>

#include "talOS/simulation/gateway_main.h"
#include "talOS/simulation/mujoco/gateway_extension.h"
#include "talOS/simulation/mujoco/plant.h"

int main(int argc, char** argv) {
  try {
    talos::simulation::mujoco::InstallMujocoProcessHandlers();
    const char* argv0 =
        (argc > 0 && argv != nullptr && argv[0] != nullptr) ? argv[0] : "";
    talos::simulation::mujoco::MujocoGatewayExtension extension{{
        .field = talos::simulation::mujoco::RunfilesFieldResolver(argv0),
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
        argc, argv, {"//talOS/simulation/mujoco:sim_gateway", "mujoco", true},
        &extension);
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
