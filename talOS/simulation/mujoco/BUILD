load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

# MuJoCo rigid-body physics (README.md). Only this package and its
# subpackages may depend on @mujoco (tools/check_layers.py --source-only).

# MuJoCo as built here steps repeatably and restarts bit-exactly from a saved
# state: a precondition of REQ-DEV-37, whose own test is the gateway's.
cc_test(
    name = "determinism_test",
    srcs = ["determinism_test.cc"],
    deps = [
        "@googletest//:gtest_main",
        "@mujoco",
    ],
)

# The MuJoCo world behind mujoco_gateway (REQ-DEV-37): the robot MJCF
# attached to a field MJCF or a flat carpet, stepped as an ExternalPlant of
# PhysicsSimBackend. Visible only inside this package tree, so @mujoco stays
# behind it (TODO.md SIM-11).
cc_library(
    name = "plant",
    srcs = ["plant.cc"],
    hdrs = ["plant.h"],
    visibility = ["//talOS/simulation/mujoco:__subpackages__"],
    deps = [
        "//talOS/hardware",
        "//talOS/hardware:codecs",
        "@mujoco",
    ],
)

cc_test(
    name = "plant_test",
    size = "small",
    srcs = ["plant_test.cc"],
    deps = [
        ":plant",
        "//talOS/hardware",
        "//talOS/node:allocation_probe",
        "@googletest//:gtest_main",
        "@mujoco",
    ],
)
