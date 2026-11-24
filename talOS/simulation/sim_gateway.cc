#include "talOS/simulation/gateway_main.h"

int main(int argc, char** argv) {
  return talos::simulation::RunSimGateway(
      argc, argv, {"//talOS/simulation:sim_gateway", "physics", false},
      nullptr);
}
