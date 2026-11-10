load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages")

package(default_visibility = ["//visibility:public"])

CAMERAS = [
    "front_left_camera",
    "front_right_camera",
    "rear_left_camera",
    "rear_right_camera",
]

talos_messages(
    name = "messages",
    srcs = ["vision.fbs"],
    topics = {
        camera + "_" + role: "/" + camera + "/" + role
        for camera in CAMERAS
        for role in [
            "state",
            "status",
        ]
    },
    deps = ["//talOS/msgs:geometry"],
)

cc_library(
    name = "vision",
    hdrs = ["vision.h"],
    include_prefix = "vision",
    strip_include_prefix = ".",
    deps = [
        ":messages",
        "//2026-robot/main_processor/vision/data:field_layout_2026",
        "//talOS/geometry",
        "//talOS/hardware:sim_messages",
        "//talOS/msgs:geometry",
        "//talOS/node",
        "//talOS/process",
        "//talOS/vision:camera_source",
        "//talOS/vision:frame_mailbox",
        "//talOS/vision:localization",
        "//talOS/vision:pipeline",
        "//talOS/vision/capture",
        "//talOS/vision/field_sim",
        "//talOS/vision/field_sim:field_sim_patterns",
        "//talOS/vision/recording",
        "//talOS/vision/recording:recorder",
    ],
)

[
    talos_cc_node(
        name = camera,
        srcs = ["vision.cc"],
        node_name = camera,
        deps = [":vision"],
    )
    for camera in CAMERAS
]

cc_test(
    name = "node_test",
    srcs = ["node_test.cc"],
    data = [
        ":front_left_camera",
        "//2026-robot/main_processor/configuration:robot.toml",
        "//2026-robot/main_processor/configuration:robot_config",
    ],
    env = {
        "TALOS_TEST_NODE": "$(rlocationpath :front_left_camera)",
        "TALOS_TEST_CONFIG": "$(rlocationpath //2026-robot/main_processor/configuration:robot.toml)",
    },
    deps = [
        ":vision",
        "//talOS/configuration",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
