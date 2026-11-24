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
