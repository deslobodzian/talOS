load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_graph_test")
load("//tools:sync_manifest.bzl", "manifest_sync_test")
load(":robot_bench_manifest.bzl", BENCH_CONFIG_FILES = "CONFIG_FILES", BENCH_NODES = "NODES")
load(":robot_manifest.bzl", "CONFIG_FILES", "NODES")

package(default_visibility = ["//visibility:public"])

exports_files([
    "robot.toml",
    "robot_bench.toml",
])

exports_files(glob(["subsystems/*.toml"]))

filegroup(
    name = "robot_config",
    srcs = ["robot.toml"] + CONFIG_FILES,
)

cc_test(
    name = "config_golden_test",
    size = "small",
    srcs = ["config_golden_test.cc"],
    data = [
        "robot_bench.toml",
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
