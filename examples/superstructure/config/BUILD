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
