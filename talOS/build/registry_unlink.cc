// Unlinks the live registry segment of this process's shared-memory
// namespace, for talos_sim_test (contract.bzl, sim_check.py) after its
// session ends.
//
// The launcher leaves the registry for later sessions (RegistryMapping), so
// each test that launches a session would otherwise leave one segment in
// /dev/shm. Bazel's TEST_TMPDIR is the namespace (ShmObjectName), unique to
// the test target and inherited by every process the test starts, so this
// removes only what that test's sessions created. The name comes from
// ShmObjectName itself, so no copy of the mapping exists outside C++. When
// the name is not namespaced (neither TALOS_SHM_NAMESPACE nor TEST_TMPDIR
// set, or TALOS_SHM_NAMESPACE set and empty) it is the production registry,
// which a live session may be using: it is left alone and the tool exits 2.

#include <cstdio>
#include <string>

#include "talOS/introspection/registry.h"
#include "talOS/memory/shared_memory_ptr.h"

int main() {
  const std::string name = ShmObjectName(talos::introspect::kRegistryPath);
  if (!name.starts_with("/~")) {
    std::fprintf(stderr,
                 "registry_unlink: %s is not in a test namespace; left alone\n",
                 name.c_str());
    return 2;
  }
  talos::introspect::RegistryMapping::unlink();
  return 0;
}
