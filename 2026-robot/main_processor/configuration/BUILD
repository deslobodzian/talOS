load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_graph_test")
load("//tools:sync_manifest.bzl", "manifest_sync_test")
load(":robot_manifest.bzl", "CONFIG_FILES", "NODES")

package(default_visibility = ["//visibility:public"])

exports_files(["robot.toml"])

exports_files(glob(["subsystems/*.toml"]))

# The whole robot config: the manifest plus every file it reads. Any test
# that opens robot.toml must take this, not the manifest alone — sibling
# files resolve relative to robot.toml at parse time, in source and runfiles.
filegroup(
    name = "robot_config",
    srcs = ["robot.toml"] + CONFIG_FILES,
)

cc_test(
    name = "config_golden_test",
    size = "small",
    srcs = ["config_golden_test.cc"],
    data = [":robot_config"],
    deps = [
        "//talOS/configuration",
        "//talOS/hardware:config",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)

talos_graph_test(
    name = "graph_test",
    config = "robot.toml",
    data = CONFIG_FILES,
    nodes = NODES,
)

# robot_manifest.bzl is generated from robot.toml; this fails when it is stale.
manifest_sync_test(
    name = "manifest_sync_test",
    config = "robot.toml",
    config_files = CONFIG_FILES,
    manifest = "robot_manifest.bzl",
)
