#pragma once

#include <string>

namespace talos::testing {
class Sim;
}  // namespace talos::testing

namespace talos::teleop {

// Adds the declarative teleop node to `sim` without exposing the internal
// talOS/teleop/node.h (and talOS/node/access.h) header to node tests.
void AddTeleop(talos::testing::Sim& sim, const std::string& name = "teleop");

}  // namespace talos::teleop
