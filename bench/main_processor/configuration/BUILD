load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_graph_test")
load("//tools:sync_manifest.bzl", "manifest_sync_test")
load(":robot_bench_manifest.bzl", BENCH_CONFIG_FILES = "CONFIG_FILES", BENCH_NODES = "NODES")
load(":robot_manifest.bzl", "CONFIG_FILES", "NODES")
load(":robot_physics_sim_manifest.bzl", PHYSICS_CONFIG_FILES = "CONFIG_FILES", PHYSICS_NODES = "NODES")
load(":robot_py_manifest.bzl", PY_CONFIG_FILES = "CONFIG_FILES", PY_NODES = "NODES")

package(default_visibility = ["//visibility:public"])

# robot.toml launches the C++ motor node, robot_py.toml the Python one (same
# subsystem files, except the motor's motor_py.toml: motor.toml's devices
# with the Python node at 20 ms), and robot_bench.toml
# is robot.toml at the bench's RoboRIO address. robot_physics_sim.toml is
# robot_py.toml's graph for the physics simulator (launcher --sim) only. Each has a generated
# <stem>_manifest.bzl; regenerate with `bazel run //tools:sync_manifest --
# <config>` after editing it.
exports_files([
    "robot.toml",
    "robot_bench.toml",
    "robot_physics_sim.toml",
    "robot_py.toml",
])

exports_files(glob(["subsystems/*.toml"]))

filegroup(
    name = "robot_config",
    srcs = ["robot.toml"] + CONFIG_FILES,
)

filegroup(
    name = "robot_py_config",
    srcs = ["robot_py.toml"] + PY_CONFIG_FILES,
)

cc_test(
    name = "config_golden_test",
    size = "small",
    srcs = ["config_golden_test.cc"],
    data = [
        "robot_bench.toml",
        ":robot_config",
        ":robot_py_config",
    ],
    deps = [
        "//talOS/configuration",
        "@googletest//:gtest_main",
    ],
)

talos_graph_test(
    name = "graph_test",
    config = "robot.toml",
    data = CONFIG_FILES,
    nodes = NODES,
    # Launches a Python node (gyro); ctypes cannot load sanitizer runtimes
    # in Python.
    tags = [
        "no_asan",
        "no_tsan",
    ],
)

talos_graph_test(
    name = "bench_graph_test",
    config = "robot_bench.toml",
    data = BENCH_CONFIG_FILES,
    nodes = BENCH_NODES,
    # Launches a Python node (gyro); ctypes cannot load sanitizer runtimes
    # in Python.
    tags = [
        "no_asan",
        "no_tsan",
    ],
)

talos_graph_test(
    name = "py_graph_test",
    config = "robot_py.toml",
    data = PY_CONFIG_FILES,
    nodes = PY_NODES,
    # Launches a Python node; ctypes cannot load sanitizer runtimes in Python.
    tags = [
        "no_asan",
        "no_tsan",
    ],
)

manifest_sync_test(
    name = "manifest_sync_test",
    config = "robot.toml",
    config_files = CONFIG_FILES,
    manifest = "robot_manifest.bzl",
)

manifest_sync_test(
    name = "bench_manifest_sync_test",
    config = "robot_bench.toml",
    config_files = BENCH_CONFIG_FILES,
    manifest = "robot_bench_manifest.bzl",
)

manifest_sync_test(
    name = "py_manifest_sync_test",
    config = "robot_py.toml",
    config_files = PY_CONFIG_FILES,
    manifest = "robot_py_manifest.bzl",
)

talos_graph_test(
    name = "physics_sim_graph_test",
    config = "robot_physics_sim.toml",
    data = PHYSICS_CONFIG_FILES,
    nodes = PHYSICS_NODES,
    # Launches Python nodes; ctypes cannot load sanitizer runtimes in Python.
    tags = [
        "no_asan",
        "no_tsan",
    ],
)

manifest_sync_test(
    name = "physics_sim_manifest_sync_test",
    config = "robot_physics_sim.toml",
    config_files = PHYSICS_CONFIG_FILES,
    manifest = "robot_physics_sim_manifest.bzl",
)

# studio/tests/browser/robot.toml repeats robot_physics_sim.toml's overrides
# (config load refuses a nested include) plus its own [studio] table.
py_test(
    name = "physics_sim_copy_test",
    size = "small",
    srcs = ["physics_sim_copy_test.py"],
    data = [
        "robot_physics_sim.toml",
        "//studio:tests/browser/robot.toml",
    ],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)
