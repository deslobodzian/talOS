load("@rules_python//python:defs.bzl", "py_test")
load("//examples/mujoco_rig/main_processor/configuration:robot_sim_manifest.bzl", "CONFIG_FILES", "NODES")
load("//talOS/build:contract.bzl", "talos_sim", "talos_sim_test")

# The rig in simulation against the MuJoCo gateway (README.md):
#   bazel run //examples/mujoco_rig:sim -- --duration-s 20 \
#     --sim-gateway-arg --mode-schedule --sim-gateway-arg disabled:1,teleop:19 ...
# The rig's place on the field is robot_sim.toml's [simulation.start_pose].
talos_sim(
    name = "sim",
    config = "//examples/mujoco_rig/main_processor/configuration:robot_sim.toml",
    data = CONFIG_FILES + ["//talOS/simulation/mujoco:sim_gateway"],
    nodes = NODES,
)

# e2e: the simulated rig starts, runs and stops cleanly, every member once
# (talos_sim_test).
talos_sim_test(
    name = "sim_test",
    sim = ":sim",
)

# e2e (REQ-DEV-37, REQ-DEV-39, REQ-DEV-40, REQ-DEV-41): a launcher session
# of the rig against the MuJoCo gateway, its own nodes driven by a scripted
# joystick, launches all 8 preloads and scores at least 7 in the blue HUB in
# TELEOP, each beam break closing and opening, every member exiting 0.
# Judged from the session's record, on the schedule clock (shot_test.py);
# 7 to 9 s.
py_test(
    name = "shot_test",
    size = "medium",
    srcs = [
        "shot_test.py",
        "//talOS/build:sim_check.py",
    ],
    data = [
        "//examples/mujoco_rig/main_processor/configuration:robot_sim.toml",
        "//talOS/bridge:hardware_node",
        "//talOS/build:registry_unlink",
        "//talOS/launcher",
        "//talOS/launcher:log_dump",
        "//talOS/launcher:session_report",
        "//talOS/simulation/mujoco:sim_gateway",
    ] + CONFIG_FILES + NODES,
    env = {
        "CONFIG": "$(rlocationpath //examples/mujoco_rig/main_processor/configuration:robot_sim.toml)",
        "LAUNCHER": "$(rlocationpath //talOS/launcher)",
        "LOG_DUMP": "$(rlocationpath //talOS/launcher:log_dump)",
        "REGISTRY_UNLINK": "$(rlocationpath //talOS/build:registry_unlink)",
        "SESSION_REPORT": "_main/talOS/launcher/session_report.py",
    },
    legacy_create_init = False,
    main = "shot_test.py",
    tags = [
        "e2e",
        "exclusive",
    ],
    deps = ["@rules_python//python/runfiles"],
)

# REQ-DEV-41: the rig's model is the bytes talOS's rig_shot_test shoots
# (//talOS/simulation/mujoco/robots:testdata/mujoco_rig/robot.xml).
py_test(
    name = "model_twin_test",
    size = "small",
    srcs = ["model_twin_test.py"],
    data = [
        "//examples/mujoco_rig/main_processor/configuration:mujoco/robot.xml",
        "//talOS/simulation/mujoco/robots:testdata/mujoco_rig/robot.xml",
    ],
    env = {
        "EXAMPLE_MODEL": "$(rlocationpath //examples/mujoco_rig/main_processor/configuration:mujoco/robot.xml)",
        "TWIN_MODEL": "$(rlocationpath //talOS/simulation/mujoco/robots:testdata/mujoco_rig/robot.xml)",
    },
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)
