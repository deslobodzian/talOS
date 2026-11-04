load("//talOS/build:contract.bzl", "talos_graph_test")
load("//tools:sync_manifest.bzl", "manifest_sync_test")
load(":robot_manifest.bzl", "CONFIG_FILES", "NODES")
load(":robot_py_manifest.bzl", PY_CONFIG_FILES = "CONFIG_FILES", PY_NODES = "NODES")

package(default_visibility = ["//visibility:public"])

# The example robot: robot.toml launches the cpp/ nodes, robot_py.toml the
# py/ ones. Both list the same subsystems/*.toml and differ only in `node =`.
# Each has a generated <stem>_manifest.bzl (NODES, CONFIG_FILES); regenerate
# with `bazel run //tools:sync_manifest -- <config>` after editing either.
exports_files([
    "robot.toml",
    "robot_py.toml",
])

# The subsystem files both manifests include. A test that opens either
# manifest takes this too: paths resolve relative to the manifest at parse
# time, in source and runfiles.
filegroup(
    name = "subsystems",
    srcs = glob(["subsystems/*.toml"]),
)

talos_graph_test(
    name = "cpp_graph_test",
    config = "robot.toml",
    data = CONFIG_FILES,
    nodes = NODES,
)

talos_graph_test(
    name = "py_graph_test",
    config = "robot_py.toml",
    data = PY_CONFIG_FILES,
    nodes = PY_NODES,
)

manifest_sync_test(
    name = "cpp_manifest_sync_test",
    config = "robot.toml",
    config_files = CONFIG_FILES,
    manifest = "robot_manifest.bzl",
)

manifest_sync_test(
    name = "py_manifest_sync_test",
    config = "robot_py.toml",
    config_files = PY_CONFIG_FILES,
    manifest = "robot_py_manifest.bzl",
)
