load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("//talOS/build:contract.bzl", "talos_messages")

# Configured mechanism building blocks over talOS/devices: PositionMechanism
# (elevator, arm, hood, turret), VelocityMechanism (flywheel, roller) and
# RollerWithSensor (intake, indexer). Each is a member of a robot node,
# driven from its update(); states publish as //talOS/msgs:control messages.
# See README.md.
cc_library(
    name = "mechanisms",
    srcs = [
        "position_mechanism.cc",
        "roller_with_sensor.cc",
        "velocity_mechanism.cc",
    ],
    hdrs = [
        "params.h",
        "position_mechanism.h",
        "roller_with_sensor.h",
        "velocity_mechanism.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/configuration",
        "//talOS/devices",
        "//talOS/hardware:config",
        "//talOS/msgs:control",
        "//talOS/node",
    ],
)

cc_test(
    name = "position_mechanism_test",
    size = "small",
    srcs = ["position_mechanism_test.cc"],
    deps = [
        ":mechanisms",
        "//talOS/devices",
        "//talOS/devices:testing",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/msgs:control",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "velocity_mechanism_test",
    size = "small",
    srcs = ["velocity_mechanism_test.cc"],
    deps = [
        ":mechanisms",
        "//talOS/devices",
        "//talOS/devices:testing",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/msgs:control",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
    ],
)

talos_messages(
    name = "roller_test_message",
    srcs = ["roller_test_message.fbs"],
    include_prefix = "roller_test",
    visibility = ["//visibility:private"],
)

cc_test(
    name = "roller_with_sensor_test",
    size = "small",
    srcs = ["roller_with_sensor_test.cc"],
    deps = [
        ":mechanisms",
        ":roller_test_message",
        "//talOS/devices",
        "//talOS/devices:testing",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/node:allocation_probe",
        "//talOS/node:testing",
        "@googletest//:gtest_main",
    ],
)
