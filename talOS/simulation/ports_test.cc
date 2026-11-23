#include "talOS/simulation/ports.h"

#include <gtest/gtest.h>

namespace talos::simulation {
namespace {

// REQ-DEV-31: a simulator run by hand on a config uses that config's ports,
// so the bridge on the same config receives its State; a flag overrides.
TEST(GatewayPorts, ConfigPortsUnlessAFlagOverrides) {
  config::RobotConfig robot;
  robot.controller_port = 5900;
  robot.local_port = 5901;
  const GatewayPorts from_config = ResolveGatewayPorts(robot, 0, 0);
  EXPECT_EQ(from_config.listen, 5900);
  EXPECT_EQ(from_config.send, 5901);
  const GatewayPorts from_flags = ResolveGatewayPorts(robot, 6000, 6001);
  EXPECT_EQ(from_flags.listen, 6000);
  EXPECT_EQ(from_flags.send, 6001);
  EXPECT_THROW(ResolveGatewayPorts(robot, 5901, 0), std::invalid_argument);
  EXPECT_THROW(ResolveGatewayPorts(robot, 6000, 6000), std::invalid_argument);
}

}  // namespace
}  // namespace talos::simulation
