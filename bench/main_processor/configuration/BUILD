load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_graph_test")
load("//tools:sync_manifest.bzl", "manifest_sync_test")
load(":robot_bench_manifest.bzl", BENCH_CONFIG_FILES = "CONFIG_FILES", BENCH_NODES = "NODES")
load(":robot_manifest.bzl", "CONFIG_FILES", "NODES")
load(":robot_py_manifest.bzl", PY_CONFIG_FILES = "CONFIG_FILES", PY_NODES = "NODES")

package(default_visibility = ["//visibility:public"])

# robot.toml launches the C++ motor node, robot_py.toml the Python one (same
# subsystem files; only the motor's `node =` differs), and robot_bench.toml
# is robot.toml at the bench's RoboRIO address. Each has a generated
# <stem>_manifest.bzl; regenerate with `bazel run //tools:sync_manifest --
# <config>` after editing it.
exports_files([
    "robot.toml",
    "robot_bench.toml",
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
        "robot_py.toml",
        ":robot_config",
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
)

talos_graph_test(
    name = "bench_graph_test",
    config = "robot_bench.toml",
    data = BENCH_CONFIG_FILES,
    nodes = BENCH_NODES,
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
