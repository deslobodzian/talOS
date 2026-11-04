load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_messages", "talos_py_node")

# The py_localizer's topic. Its state is the shared geometry.PoseEstimate and its
# camera inputs are geometry.PoseObservation (//talOS/msgs:geometry), so the
# package declares no schema of its own. Consumers find the pose through
# config (telemetry's pose_topic), not through this package.
talos_messages(
    name = "messages",
    srcs = [],
    include_prefix = "py_localizer",
    topics = {
        "state": "/py_localizer/state",
    },
    deps = ["//talOS/msgs:geometry"],
)

talos_py_node(
    name = "node",
    class_name = "PyLocalizer",
    main = "py_localizer.py",
    node_name = "py_localizer",
    deps = [":messages_py"],
)

py_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.py"],
    data = [
        "py_localizer.py",
        "testdata/robot.toml",
        ":node",
    ],
    env = {"NODE_BINARY": "$(rlocationpath :node)"},
    legacy_create_init = False,
    deps = [
        ":messages_py",
        "//talOS/python:testing",
        "@rules_python//python/runfiles",
    ],
)

# Golden copy of //tools:new_subsystem output; only this target is not generated.
filegroup(
    name = "golden",
    srcs = glob(["**"]),
    visibility = ["//tools:__pkg__"],
)
