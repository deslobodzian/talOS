load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_test")

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

# The MuJoCo world behind the MuJoCo gateway (REQ-DEV-37): the robot MJCF
# attached to a field MJCF or a flat carpet, stepped as an ExternalPlant of
# PhysicsSimBackend. Visible only inside this package tree, so @mujoco stays
# behind it (TODO.md SIM-11).
cc_library(
    name = "plant",
    srcs = ["plant.cc"],
    hdrs = ["plant.h"],
    visibility = ["//talOS/simulation/mujoco:__subpackages__"],
    deps = [
        ":world_rules",
        "//talOS/hardware",
        "//talOS/hardware:codecs",
        "@mujoco",
    ],
)

# What changes the MuJoCo world after a step besides physics (a scripted
# shot, a game's referee): the interface MujocoPlant::Step runs.
cc_library(
    name = "world_rules",
    hdrs = ["world_rules.h"],
    visibility = ["//talOS/simulation/mujoco:__subpackages__"],
)

# A field's game (game.h): the GameRules a field's rules package implements
# and the gateway's field table names (REQ-DEV-40).
cc_library(
    name = "game",
    hdrs = ["game.h"],
    visibility = ["//talOS/simulation/mujoco:__subpackages__"],
    deps = [
        ":world_rules",
        "//talOS/hardware:sim_messages",
    ],
)

# The robot model's scripted shooters (README.md "Scripted shooter";
# REQ-DEV-39), a WorldRules.
cc_library(
    name = "scripted_shooter",
    srcs = ["scripted_shooter.cc"],
    hdrs = ["scripted_shooter.h"],
    visibility = ["//talOS/simulation/mujoco:__subpackages__"],
    deps = [
        ":world_rules",
        "@mujoco",
    ],
)

# REQ-DEV-39: launches from a stationary and from a moving, turning robot,
# the flywheel's momentum debit, robot bodies never launched, one body at a
# time, the contract's refusals, bit-identical runs and no allocation.
cc_test(
    name = "scripted_shooter_test",
    size = "small",
    srcs = ["scripted_shooter_test.cc"],
    deps = [
        ":plant",
        ":scripted_shooter",
        ":world_rules",
        "//talOS/hardware",
        "//talOS/node:allocation_probe",
        "@googletest//:gtest_main",
        "@mujoco",
    ],
)

cc_test(
    name = "plant_test",
    size = "small",
    srcs = ["plant_test.cc"],
    deps = [
        ":plant",
        ":world_rules",
        "//talOS/hardware",
        "//talOS/node:allocation_probe",
        "@googletest//:gtest_main",
        "@mujoco",
    ],
)

# The MuJoCo gateway (README.md): sim_gateway --physics with the MuJoCo world
# as the plant (REQ-DEV-37). A config names it in [robot] sim_gateway, whose
# roster member is sim_gateway, hence the rule's name (the node label rule,
# talOS/NAMING.md "Nodes").
cc_binary(
    name = "sim_gateway",
    srcs = ["mujoco_gateway.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":gateway_extension",
        ":plant",
        "//talOS/simulation:gateway_main",
    ],
)

# The program's GatewayExtension: assembles the world from the config's
# [simulation.mujoco] with the robot's scripted shooters and the field's game
# rules, finds the field models in its runfiles, feeds the rules the match,
# publishes /sim/state/match and /sim/telemetry/pieces, and reports the
# plant's clock. A library
# so the component tests drive it in process.
cc_library(
    name = "gateway_extension",
    srcs = ["gateway_extension.cc"],
    hdrs = ["gateway_extension.h"],
    data = ["//talOS/simulation/mujoco/fields/rebuilt_2026"],
    local_defines = [
        "TALOS_MUJOCO_FIELD_REBUILT_2026=\\\"$(rlocationpath //talOS/simulation/mujoco/fields/rebuilt_2026)\\\"",
    ],
    visibility = ["//talOS/simulation/mujoco:__subpackages__"],
    deps = [
        ":game",
        ":plant",
        ":scripted_shooter",
        ":world_rules",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/events:events_core",
        "//talOS/hardware",
        "//talOS/hardware:sim_messages",
        "//talOS/hardware:sim_rings",
        "//talOS/ipc",
        "//talOS/simulation:gateway_main",
        "//talOS/simulation/mujoco/fields/rebuilt_2026:rules",
        "@mujoco",
        "@rules_cc//cc/runfiles",
    ],
)

# REQ-DEV-37: a bound motor against the analytic rigid-body mechanism of the
# same inertia, tick for tick through two Gateways (the design note's
# bounds; intended differences in the test's header).
cc_test(
    name = "gateway_differential_test",
    size = "small",
    srcs = ["gateway_differential_test.cc"],
    deps = [
        ":plant",
        "//talOS/hardware",
        "//talOS/hardware:codecs",
        "@googletest//:gtest_main",
        "@mujoco",
    ],
)

# REQ-DEV-37: two compiles of one model, ticked at the same jittered times
# with the same commands and faults, give identical encoded States; a 1 us
# change at one tick does not.
cc_test(
    name = "gateway_bit_identity_test",
    size = "small",
    srcs = ["gateway_bit_identity_test.cc"],
    data = ["//talOS/simulation/mujoco/fields/rebuilt_2026"],
    env = {"FIELD_XML": "$(rlocationpath //talOS/simulation/mujoco/fields/rebuilt_2026)"},
    deps = [
        ":gateway_extension",
        ":plant",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/hardware:codecs",
        "//talOS/hardware:sim_messages",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# REQ-DEV-43: /sim/telemetry/pieces against the stepped model's bodies on
# the 2026 field (positions, count, radius, a scored FUEL out of play), the
# 10th-tick rule, identical bytes across runs, a free robot never a piece,
# and the refusals of a field the message cannot carry.
cc_test(
    name = "pieces_test",
    size = "small",
    srcs = ["pieces_test.cc"],
    data = ["//talOS/simulation/mujoco/fields/rebuilt_2026"],
    env = {"FIELD_XML": "$(rlocationpath //talOS/simulation/mujoco/fields/rebuilt_2026)"},
    deps = [
        ":gateway_extension",
        ":plant",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/hardware:sim_messages",
        "@googletest//:gtest_main",
        "@mujoco",
        "@rules_cc//cc/runfiles",
    ],
)

# REQ-DEV-37: the program as a child process: --describe against
# sim_gateway --physics, refusals naming the key or flag, a model file
# robot_model_files omits, a diverging model, and 50 ms on the 2026 field
# from the config's start pose.
# Integration lane: real-time runs over loopback UDP.
py_test(
    name = "mujoco_gateway_test",
    size = "small",
    srcs = ["mujoco_gateway_test.py"],
    data = [
        ":sim_gateway",
        "//talOS/simulation:sim_gateway",
    ] + glob(["testdata/**"]),
    env = {
        "ARM_XML": "$(rlocationpath testdata/arm/arm.xml)",
        "MUJOCO_GATEWAY": "$(rlocationpath :sim_gateway)",
        "ROVER_CONFIG": "$(rlocationpath testdata/rover/robot.toml)",
        "SIM_GATEWAY": "$(rlocationpath //talOS/simulation:sim_gateway)",
        "UNLISTED_MESH_CONFIG": "$(rlocationpath testdata/unlisted_mesh/robot.toml)",
    },
    legacy_create_init = False,
    tags = ["integration"],
    deps = [
        "//talOS/build:udp_ports",
        "@rules_python//python/runfiles",
    ],
)

# REQ-DEV-37: the launcher accepts a config whose [robot] sim_gateway is
# the program: its roster member passes the node label rule and is
# described with /sim/state/match. Integration lane: the launcher probes
# the bridge and the program (--describe-only).
py_test(
    name = "launcher_describe_test",
    size = "small",
    srcs = ["launcher_describe_test.py"],
    data = [
        "testdata/arm/arm.xml",
        ":sim_gateway",
        "//talOS/bridge:hardware_node",
        "//talOS/launcher",
    ],
    env = {
        "ARM_XML": "$(rlocationpath testdata/arm/arm.xml)",
        "LAUNCHER": "$(rlocationpath //talOS/launcher)",
    },
    legacy_create_init = False,
    tags = ["integration"],
    deps = ["@rules_python//python/runfiles"],
)

# REQ-DEV-37: the field names config load accepts are the ones the gateway
# finds in its runfiles and loads, and no others.
cc_test(
    name = "fields_test",
    size = "small",
    srcs = ["fields_test.cc"],
    deps = [
        ":game",
        ":gateway_extension",
        ":plant",
        "//talOS/configuration",
        "//talOS/hardware:sim_messages",
        "@googletest//:gtest_main",
        "@mujoco",
    ],
)

# REQ-DEV-37: Configure refuses each analytic chassis key and flag when the
# model owns the chassis, and accepts them with a welded root; the robot
# starts at --start-pose, else the config's start pose, else the default.
cc_test(
    name = "gateway_extension_test",
    size = "small",
    srcs = ["gateway_extension_test.cc"],
    data = [
        "testdata/arm/arm.xml",
        "testdata/rover/rover.xml",
    ],
    env = {
        "ARM_XML": "$(rlocationpath testdata/arm/arm.xml)",
        "ROVER_XML": "$(rlocationpath testdata/rover/rover.xml)",
    },
    deps = [
        ":gateway_extension",
        ":plant",
        "//talOS/configuration",
        "//talOS/driver_station",
        "//talOS/hardware",
        "//talOS/simulation:gateway_main",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)

# REQ-DEV-40: the program referees its mode schedule's match on the 2026
# field: /sim/state/match's periods in order and counting down, the Driver
# Station's match time and game message, the live registry row against
# --describe, and (REQ-DEV-43) /sim/telemetry/pieces every 50 ms. Integration lane: fork/exec, loopback UDP, shared memory; one
# run of about 1.3 s of real time.
cc_test(
    name = "match_state_test",
    size = "small",
    srcs = ["match_state_test.cc"],
    data = [
        "testdata/rover/robot.toml",
        "testdata/rover/rover.xml",
        ":sim_gateway",
    ],
    env = {
        "MUJOCO_GATEWAY": "$(rlocationpath :sim_gateway)",
        "ROVER_CONFIG": "$(rlocationpath testdata/rover/robot.toml)",
    },
    tags = ["integration"],
    deps = [
        "//talOS/driver_station",
        "//talOS/hardware:sim_messages",
        "//talOS/hardware:sim_rings",
        "//talOS/introspection:describe",
        "//talOS/introspection:names",
        "//talOS/introspection:registry",
        "//talOS/ipc",
        "//talOS/protocol:frame",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
    ],
)
