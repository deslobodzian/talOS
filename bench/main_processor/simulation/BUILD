load("@rules_python//python:defs.bzl", "py_test")
load("//tools:sync_manifest.bzl", "manifest_sync_test")
load(":soak_manifest.bzl", "CONFIG_FILES", "NODES")

exports_files(["soak.toml"])

# soak.toml is robot_physics_sim.toml's graph with a [studio] table; its
# generated soak_manifest.bzl lists what it launches and reads.
manifest_sync_test(
    name = "manifest_sync_test",
    config = "soak.toml",
    config_files = CONFIG_FILES,
    manifest = "soak_manifest.bzl",
)

# Minutes of the bench's simulated graph (Studio bridge included) under
# SIGKILL/SIGSTOP chaos; see soak_test.py. Manual: `bazel test //...` skips
# it, and the CI soak job runs it on its own (TALOS_SOAK_SECONDS sets the
# teleop length).
py_test(
    name = "soak_test",
    size = "large",
    timeout = "long",
    srcs = [
        "soak_test.py",
        "support.py",
    ],
    data = [
        "soak.toml",
        # The bridge refuses to start without its webroot,
        # [studio] webroot = bazel-bin/studio/dist.
        "//studio:web_dist",
        "//studio/bridge:studio_bridge",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
        "//talOS/simulation:sim_gateway",
    ] + CONFIG_FILES + NODES,
    imports = ["."],
    # Launches Python nodes; ctypes cannot load sanitizer runtimes in Python.
    tags = [
        "exclusive",
        "manual",
        "no_asan",
        "no_tsan",
    ],
)
