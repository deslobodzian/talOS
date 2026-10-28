load("@rules_python//python:defs.bzl", "py_test")

package(default_visibility = ["//visibility:public"])

# The example robot: robot.toml launches the cpp/ nodes, robot_py.toml the
# py/ ones. Everything but the `node =` lines is the same.
exports_files([
    "robot.toml",
    "robot_py.toml",
])

py_test(
    name = "config_test",
    size = "small",
    srcs = ["config_test.py"],
    data = [
        "robot.toml",
        "robot_py.toml",
    ],
)
