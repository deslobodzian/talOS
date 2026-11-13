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
// Run outside Bazel with neither TEST_TMPDIR nor TALOS_SHM_NAMESPACE set,
// the name is the production registry's, which a live session beside the
// test (./sim.sh) is using: it is left alone then.

#include <gtest/gtest.h>

#include <cstdlib>

#include "talOS/introspection/registry.h"

namespace {

// Whether ShmObjectName maps names into a namespace of their own
// (talOS/memory/shared_memory_ptr.h): one of its variables is set and not
// empty.
bool Namespaced() {
  for (const char* name : {"TALOS_SHM_NAMESPACE", "TEST_TMPDIR"}) {
    const char* value = std::getenv(name);
    if (value != nullptr && *value != '\0') return true;
  }
  return false;
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
