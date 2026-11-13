// Linked into a launcher process test (alwayslink): after its last test,
// unlinks the registry segment of the test's shared-memory namespace.
//
// A launch prepares the registry and every node registers in it, and the
// registry deliberately outlives its sessions (RegistryMapping), so each
// such test target used to leave one ~356 KB segment in /dev/shm. Bazel's
// TEST_TMPDIR is the namespace (ShmObjectName), unique to the target and its
// shard and inherited by every process the test starts, so this removes only
// what the test created, never another process's segment. Process segments
// of a session (its topics) are the launcher's to unlink at session end.
// When the registry's name is not namespaced (run outside Bazel with
// neither variable set, or with TALOS_SHM_NAMESPACE set and empty, which
// ShmObjectName takes over TEST_TMPDIR), it is the production registry's,
// which a live session beside the test (./sim.sh) is using: it is left
// alone then.

#include <gtest/gtest.h>

#include "talOS/introspection/registry.h"
#include "talOS/memory/shared_memory_ptr.h"

namespace {

// Whether the registry's shared-memory name is namespaced, decided by the
// mapping RegistryMapping::unlink uses, never by reading its variables
// again.
bool Namespaced() {
  return ShmObjectName(talos::introspect::kRegistryPath).starts_with("/~");
}

class RegistryCleanup : public ::testing::Environment {
 public:
  void TearDown() override {
    if (Namespaced()) talos::introspect::RegistryMapping::unlink();
  }
};

// gtest owns the environment once it is added.
[[maybe_unused]] ::testing::Environment* const kRegistryCleanup =
    ::testing::AddGlobalTestEnvironment(new RegistryCleanup);

}  // namespace
