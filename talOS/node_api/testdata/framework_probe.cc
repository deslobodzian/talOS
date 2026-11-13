// An exported C++ node, plus where the framework it calls lives, for
// cpp_node_test.py.
#include <bit>
#include <cstddef>
#include <cstdint>

#include "talOS/devices/devices.h"
#include "talOS/hardware/gateway.h"
#include "talOS/node/access.h"
#include "talOS/node/node.h"

namespace {

struct Probe {
  explicit Probe(talos::Node&) {}
};

// A pointer to a non-virtual member function under the Itanium C++ ABI
// (Linux and macOS): the function's address, then the this-adjustment.
struct MemberFunction {
  const void* address;
  std::ptrdiff_t adjustment;
};

}  // namespace

TALOS_EXPORT_NODE(probe, Probe)

// The node and devices functions this library calls, as resolved from it.
extern "C" const void* talos_probe_node_function() {
  return reinterpret_cast<const void*>(
      &talos::detail::NodeAccess::add_extension);
}
extern "C" const void* talos_probe_devices_function() {
  return reinterpret_cast<const void*>(&talos::robot_enabled);
}
// A framework function node_api itself never calls: it is in
// libtalos_node.so only because that library links the hardware package
// whole, and this library does not link without it.
extern "C" const void* talos_probe_hardware_function() {
  return std::bit_cast<MemberFunction>(&talos::hardware::Gateway::Reconfigure)
      .address;
}
