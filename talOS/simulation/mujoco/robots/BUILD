load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_binary", "py_test")
load("//talOS/build:robot_model.bzl", "robot_model_sync_test")

# Robot models for the MuJoCo gateway, generated from a parameter file
# (README.md). REQ-DEV-41.

# robot_model_sync_test (//talOS/build:robot_model.bzl) runs the script as
# its main.
exports_files(["generate_robot.py"])

# bazel run //talOS/simulation/mujoco/robots:generate_robot -- \
#   --parameters <robot_params.toml> --out <robot.xml>
# From a robot repository: @talos//talOS/simulation/mujoco/robots:generate_robot.
py_binary(
    name = "generate_robot",
    srcs = ["generate_robot.py"],
    visibility = ["//visibility:public"],
)

# The two example models are checked in beside their parameter files, as a
# robot repository keeps its own.
robot_model_sync_test(
    name = "swerve_robot_sync_test",
    model = "examples/swerve_robot.xml",
    parameters = "examples/swerve_robot.toml",
)

robot_model_sync_test(
    name = "shooter_rig_sync_test",
    model = "examples/shooter_rig.xml",
    parameters = "examples/shooter_rig.toml",
)

# The MuJoCo rig example's model (examples/mujoco_rig), for rig_shot_test;
# the example's model_twin_test holds its own copy to these bytes.
exports_files(["testdata/mujoco_rig/robot.xml"])

robot_model_sync_test(
    name = "mujoco_rig_sync_test",
    model = "testdata/mujoco_rig/robot.xml",
    parameters = "testdata/mujoco_rig/robot_params.toml",
)

# REQ-DEV-37, REQ-DEV-39, REQ-DEV-40, REQ-DEV-41: the rig example's shot
# without processes: its model on the 2026 field, assembled as the MuJoCo
# gateway assembles it and given its nodes' commands, launches all eight
# preloads and scores at least seven in TELEOP, the same every run.
cc_test(
    name = "rig_shot_test",
    size = "small",
    srcs = ["rig_shot_test.cc"],
    data = [
        "testdata/mujoco_rig/robot.xml",
        "//talOS/simulation/mujoco/fields/rebuilt_2026",
    ],
    env = {
        "FIELD_XML": "$(rlocationpath //talOS/simulation/mujoco/fields/rebuilt_2026)",
        "RIG_XML": "$(rlocationpath testdata/mujoco_rig/robot.xml)",
    },
    deps = [
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/hardware:codecs",
        "//talOS/hardware:sim_messages",
        "//talOS/simulation/mujoco:gateway_extension",
        "//talOS/simulation/mujoco:plant",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# REQ-DEV-41: bad parameter files are refused naming the key, config names
# pass through, the examples carry their scripted-shooter parameters, the
# output is the same bytes from run to run, and --check fails on an edited
# parameter.
py_test(
    name = "generate_robot_test",
    size = "small",
    srcs = [
        "generate_robot.py",
        "generate_robot_test.py",
    ],
    data = [
        "examples/shooter_rig.toml",
        "examples/shooter_rig.xml",
        "examples/swerve_robot.toml",
        "examples/swerve_robot.xml",
    ],
    legacy_create_init = False,
    main = "generate_robot_test.py",
)

# REQ-DEV-41: each example model assembles on the flat carpet and the 2026
# field, binds every motor its parameters name, carries a scripted-shooter
# contract the shooter accepts, launches clear of itself, closes each beam
# on a piece in its path, and rests still with its beams open and only its
# wheels on the carpet; the shooter rig feeds every preload to its inlet.
cc_test(
    name = "robot_model_test",
    size = "small",
    srcs = ["robot_model_test.cc"],
    data = [
        "examples/shooter_rig.xml",
        "examples/swerve_robot.xml",
        "//talOS/simulation/mujoco/fields/rebuilt_2026",
    ],
    env = {
        "FIELD_XML": "$(rlocationpath //talOS/simulation/mujoco/fields/rebuilt_2026)",
        "SHOOTER_RIG_XML": "$(rlocationpath examples/shooter_rig.xml)",
        "SWERVE_ROBOT_XML": "$(rlocationpath examples/swerve_robot.xml)",
    },
    deps = [
        "//talOS/hardware",
        "//talOS/simulation/mujoco:plant",
        "//talOS/simulation/mujoco:scripted_shooter",
        "//talOS/simulation/mujoco:world_rules",
        "@googletest//:gtest_main",
        "@mujoco",
        "@rules_cc//cc/runfiles",
    ],
)
