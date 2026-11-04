load("@rules_python//python:defs.bzl", "py_test")
load("//talOS/build:contract.bzl", "talos_messages", "talos_py_node")

# The estimator's topic. Its state is the shared geometry.PoseEstimate and its
# camera inputs are geometry.PoseObservation (//talOS/msgs:geometry), so the
# package declares no schema of its own. Telemetry and Studio find the pose
# through their `pose_topic` config, not through this package.
talos_messages(
    name = "messages",
    srcs = [],
    topics = {
        "state": "/pose_estimator/state",
    },
    deps = ["//talOS/msgs:geometry"],
)

talos_py_node(
    name = "node",
    class_name = "PoseEstimator",
    main = "pose_estimator.py",
    node_name = "pose_estimator",
    deps = [
        ":messages_py",
        "//2026-robot/main_processor/drivetrain:messages_py",
        "@pypi//gtsam",
        "@pypi//numpy",
        "@pypi//robotpy_wpimath",
    ],
)

py_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.py"],
    data = [
        "pose_estimator.py",
        "testdata/robot.toml",
        ":node",
    ],
    env = {"POSE_ESTIMATOR_BINARY": "$(rlocationpath :node)"},
    legacy_create_init = False,
    deps = [
        ":messages_py",
        "//2026-robot/main_processor/drivetrain:messages_py",
        "//talOS/python:testing",
        "@pypi//gtsam",
        "@pypi//numpy",
        "@pypi//robotpy_wpimath",
        "@rules_python//python/runfiles",
    ],
)
