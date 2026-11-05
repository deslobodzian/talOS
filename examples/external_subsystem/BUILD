load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_bundle", "talos_graph_test")
load("//tools:sync_manifest.bzl", "manifest_sync_test")
load(":robot_manifest.bzl", "CONFIG_FILES", "NODES")

package(default_visibility = ["//visibility:public"])

exports_files(["robot.toml"])

talos_graph_test(
    name = "graph_test",
    config = "robot.toml",
    data = CONFIG_FILES,
    nodes = NODES,
)

manifest_sync_test(
    name = "manifest_sync_test",
    config = "robot.toml",
    config_files = CONFIG_FILES,
    manifest = "robot_manifest.bzl",
)

talos_bundle(
    name = "robot_bundle",
    config = "robot.toml",
    config_files = CONFIG_FILES,
    nodes = ["//talOS/bridge:hardware_node"] + NODES,
)

py_test(
    name = "bundle_test",
    size = "small",
    srcs = ["bundle_test.py"],
    data = [":robot_bundle"],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)

# The nested module is outside //... (.bazelignore); this runs its own test
# matrix from the consuming workspace.
test_suite(
    name = "wrist_tests",
    tests = ["@wrist//:node_test"],
)
