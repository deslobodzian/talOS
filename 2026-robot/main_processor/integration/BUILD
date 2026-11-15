load("@rules_cc//cc:cc_test.bzl", "cc_test")

# Tests of several 2026 nodes together, under Sim on the robot's own config.
# No node lives here: only a testonly target outside a node package may link
# more than one node's library (tools/check_layers.py), and each node library
# grants visibility to this package alone.

# The teleop chain: Driver Station packet, driver, drivetrain, module
# requests.
cc_test(
    name = "teleop_chain_test",
    size = "small",
    srcs = ["teleop_chain_test.cc"],
    data = [
        "//2026-robot/main_processor/configuration:robot.toml",
        "//2026-robot/main_processor/configuration:robot_config",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath //2026-robot/main_processor/configuration:robot.toml)",
    },
    deps = [
        "//2026-robot/main_processor/driver",
        "//2026-robot/main_processor/drivetrain",
        "//talOS/configuration",
        "//talOS/devices:testing",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# Autonomous's drive action against the real drivetrain node, with the test
# playing the pose estimator.
cc_test(
    name = "closed_loop_test",
    size = "small",
    srcs = ["closed_loop_test.cc"],
    data = [
        "//2026-robot/main_processor/configuration:robot.toml",
        "//2026-robot/main_processor/configuration:robot_config",
    ],
    env = {
        "TALOS_TEST_CONFIG": "$(rlocationpath //2026-robot/main_processor/configuration:robot.toml)",
    },
    deps = [
        "//2026-robot/main_processor/autonomous",
        "//2026-robot/main_processor/drivetrain",
        "//talOS/configuration",
        "//talOS/devices:testing",
        "//talOS/geometry",
        "//talOS/node:testing",
        "//talOS/trajectory",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
