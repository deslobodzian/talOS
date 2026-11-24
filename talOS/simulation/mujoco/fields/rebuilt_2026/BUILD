load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_binary")

# The 2026 FRC field (REBUILT) as a MuJoCo model, generated at build time
# (README.md). REQ-DEV-38.

py_binary(
    name = "generate_field",
    srcs = ["generate_field.py"],
)

genrule(
    name = "rebuilt_2026_xml",
    srcs = ["//talOS/vision/testdata:2026-rebuilt-welded.json"],
    outs = ["rebuilt_2026.xml"],
    cmd = "$(execpath :generate_field) " +
          "--tags $(location //talOS/vision/testdata:2026-rebuilt-welded.json) " +
          "--out $@",
    tools = [":generate_field"],
)

# The model, as `data` for simulators under //talOS/simulation.
filegroup(
    name = "rebuilt_2026",
    srcs = [":rebuilt_2026.xml"],
    visibility = ["//talOS/simulation:__subpackages__"],
)

# REQ-DEV-38: the model's names, symmetry and dimensions, its agreement with
# the official AprilTag poses, and fuel behaviour at rest, through the hub,
# out of the chute and against every wall. About 2 s unsanitized.
cc_test(
    name = "field_test",
    size = "small",
    srcs = ["field_test.cc"],
    data = [
        ":rebuilt_2026.xml",
        "//talOS/vision/testdata:2026-rebuilt-welded.json",
    ],
    env = {
        "FIELD_XML": "$(rlocationpath :rebuilt_2026.xml)",
        "TAGS_JSON": "$(rlocationpath //talOS/vision/testdata:2026-rebuilt-welded.json)",
    },
    deps = [
        "@googletest//:gtest_main",
        "@mujoco",
        "@nlohmann_json//:json",
        "@rules_cc//cc/runfiles",
    ],
)

# The 2026 game's rules on this model (README.md "Rules"; REQ-DEV-40), the
# GameRules a simulator's field table names.
cc_library(
    name = "rules",
    srcs = ["rules.cc"],
    hdrs = ["rules.h"],
    visibility = ["//talOS/simulation/mujoco:__subpackages__"],
    deps = [
        "//talOS/simulation/mujoco:game",
        "@mujoco",
    ],
)

# REQ-DEV-40 on the generated field through MujocoPlant: scoring once,
# parking and release from an empty pocket, the shift order, the grace edge,
# preloads, the match report and Driver Station stamp, the seeded stream,
# bit-identical runs and no allocation.
cc_test(
    name = "rules_test",
    size = "small",
    srcs = ["rules_test.cc"],
    data = [":rebuilt_2026.xml"],
    env = {"FIELD_XML": "$(rlocationpath :rebuilt_2026.xml)"},
    deps = [
        ":rules",
        "//talOS/hardware:sim_messages",
        "//talOS/node:allocation_probe",
        "//talOS/simulation/mujoco:game",
        "//talOS/simulation/mujoco:plant",
        "//talOS/simulation/mujoco:world_rules",
        "@googletest//:gtest_main",
        "@mujoco",
        "@rules_cc//cc/runfiles",
    ],
)
