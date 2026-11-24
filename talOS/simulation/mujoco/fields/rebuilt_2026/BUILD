load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

# The 2026 FRC field (REBUILT) as a MuJoCo model and as a glTF binary for
# viewers, both generated at build time (README.md). REQ-DEV-38, REQ-DEV-42.

py_library(
    name = "generate_field_lib",
    srcs = ["generate_field.py"],
    imports = ["."],
)

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

# The same field for Studio's 3D viewport ("Load field GLB"), whose
# //studio:field_glb_test loads it:
# bazel-bin/talOS/simulation/mujoco/fields/rebuilt_2026/rebuilt_2026.glb.
genrule(
    name = "rebuilt_2026_glb",
    srcs = ["//talOS/vision/testdata:2026-rebuilt-welded.json"],
    outs = ["rebuilt_2026.glb"],
    cmd = "$(execpath :generate_field) " +
          "--tags $(location //talOS/vision/testdata:2026-rebuilt-welded.json) " +
          "--glb $@",
    tools = [":generate_field"],
    visibility = ["//studio:__pkg__"],
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
# Five shards, so each has its own timeout under --config=tsan: gtest gives
# case i, in run order (the file's TESTs, then its TEST_P instances), to
# shard i % 5, which leaves StagedFuelIsAtRest (case 4, 2 s of the whole
# field) with only two cheap cases and keeps the hub cases apart. A case
# added above it moves every later one.
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
    shard_count = 5,
    deps = [
        "@googletest//:gtest_main",
        "@mujoco",
        "@nlohmann_json//:json",
        "@rules_cc//cc/runfiles",
    ],
)

# REQ-DEV-42: every geom but the fuel is a node of the glTF binary with its
# pose, size and colour in Studio's frame, and the binary is a valid,
# self-contained GLB.
cc_test(
    name = "field_gltf_test",
    size = "small",
    srcs = ["field_gltf_test.cc"],
    data = [
        ":rebuilt_2026.glb",
        ":rebuilt_2026.xml",
    ],
    env = {
        "FIELD_GLB": "$(rlocationpath :rebuilt_2026.glb)",
        "FIELD_XML": "$(rlocationpath :rebuilt_2026.xml)",
    },
    deps = [
        "@googletest//:gtest_main",
        "@mujoco",
        "@nlohmann_json//:json",
        "@rules_cc//cc/runfiles",
    ],
)

# REQ-DEV-42: the generator's output is the same bytes every time.
py_test(
    name = "generate_field_test",
    size = "small",
    srcs = ["generate_field_test.py"],
    data = ["//talOS/vision/testdata:2026-rebuilt-welded.json"],
    env = {
        "TAGS_JSON": "$(rlocationpath //talOS/vision/testdata:2026-rebuilt-welded.json)",
    },
    deps = [
        ":generate_field_lib",
        "@rules_python//python/runfiles",
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
# Six shards, so each has its own timeout under --config=tsan: gtest gives
# case i, in run order (the file's TESTs, then its TEST_P instances), to
# shard i % 6, which puts each two-scenario case (RunsAreBitIdentical,
# TheSeedMatters) beside one cheap case. A case added above them moves
# every later one.
cc_test(
    name = "rules_test",
    size = "small",
    srcs = ["rules_test.cc"],
    data = [":rebuilt_2026.xml"],
    env = {"FIELD_XML": "$(rlocationpath :rebuilt_2026.xml)"},
    shard_count = 6,
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
