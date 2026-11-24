"""Generated from robot.toml by //tools:sync_manifest. Do not edit.

Regenerate after changing the config:

    bazel run //tools:sync_manifest -- examples/mujoco_rig/main_processor/configuration/robot.toml
"""

# Every node label robot.toml launches.
NODES = [
    "//examples/mujoco_rig/main_processor/feeder:node",
    "//examples/mujoco_rig/main_processor/flywheel:node",
    "//talOS/telemetry:node",
    "//talOS/teleop:node",
]

# Every file robot.toml reads besides itself.
CONFIG_FILES = [
    "//examples/mujoco_rig/main_processor/configuration:subsystems/feeder.toml",
    "//examples/mujoco_rig/main_processor/configuration:subsystems/flywheel.toml",
    "//examples/mujoco_rig/main_processor/configuration:subsystems/telemetry.toml",
    "//examples/mujoco_rig/main_processor/configuration:subsystems/teleop.toml",
]
