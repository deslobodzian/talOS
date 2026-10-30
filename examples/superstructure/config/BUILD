load("//talOS/build:contract.bzl", "talos_graph_test")

package(default_visibility = ["//visibility:public"])

# The example robot: robot.toml launches the cpp/ nodes, robot_py.toml the
# py/ ones. Both list the same subsystems/*.toml and differ only in `node =`.
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
    data = [":subsystems"],
    nodes = [
        "//examples/superstructure/cpp:elevator",
        "//examples/superstructure/cpp:intake",
        "//examples/superstructure/cpp:superstructure",
    ],
)

talos_graph_test(
    name = "py_graph_test",
    config = "robot_py.toml",
    data = [
        "robot.toml",
        ":subsystems",
    ],
    nodes = [
        "//examples/superstructure/py:elevator",
        "//examples/superstructure/py:intake",
        "//examples/superstructure/py:superstructure",
    ],
)
