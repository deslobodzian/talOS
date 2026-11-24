"""Generated from robot_sim.toml by //tools:sync_manifest. Do not edit.

Regenerate after changing the config:

    bazel run //tools:sync_manifest -- examples/mujoco_rig/main_processor/configuration/robot_sim.toml
"""

# Every node label robot_sim.toml launches.
NODES = [
    "//examples/mujoco_rig/main_processor/feeder:node",
    "//examples/mujoco_rig/main_processor/flywheel:node",
    "//talOS/telemetry:node",
    "//talOS/teleop:node",
]

# Every file robot_sim.toml reads besides itself.
CONFIG_FILES = [
    "//examples/mujoco_rig/main_processor/configuration:mujoco/robot.xml",
    "//examples/mujoco_rig/main_processor/configuration:robot.toml",
    "//examples/mujoco_rig/main_processor/configuration:subsystems/feeder.toml",
    "//examples/mujoco_rig/main_processor/configuration:subsystems/flywheel.toml",
    "//examples/mujoco_rig/main_processor/configuration:subsystems/telemetry.toml",
    "//examples/mujoco_rig/main_processor/configuration:subsystems/teleop.toml",
]
