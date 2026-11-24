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
