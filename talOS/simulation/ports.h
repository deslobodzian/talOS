#ifndef TALOS_SIMULATION_PORTS_H_
#define TALOS_SIMULATION_PORTS_H_

#include <cstdint>
#include <stdexcept>

#include "talOS/configuration/config_parser.h"

namespace talos::simulation {

// The simulator's loopback UDP ports: it stands where the controller does,
// so it listens on the port the bridge sends to and sends to the one the
// bridge binds.
struct GatewayPorts {
  uint16_t listen;  // --port, else [robot] controller_port
  uint16_t send;    // --remote-port, else [robot] local_port
};

// A flag of 0 was not given. The launcher passes both flags (its own
// --remote-port and --local-port may override the config); a hand-run
// simulator takes the config's, as the bridge on that config does.
inline GatewayPorts ResolveGatewayPorts(const config::RobotConfig& config,
                                        uint16_t port_flag,
                                        uint16_t remote_port_flag) {
  const GatewayPorts ports{
      .listen = config::ResolveControllerPort(config, port_flag),
      .send = config::ResolveLocalPort(config, remote_port_flag)};
  if (ports.listen == ports.send) {
    throw std::invalid_argument(
        "simulator --port and --remote-port must differ");
  }
  return ports;
}

}  // namespace talos::simulation

#endif  // TALOS_SIMULATION_PORTS_H_
