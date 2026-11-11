"""Generated from robot_py.toml by //tools:sync_manifest. Do not edit.

Regenerate after changing the config:

    bazel run //tools:sync_manifest -- bench/main_processor/configuration/robot_py.toml
"""

# Every node label robot_py.toml launches.
NODES = [
    "//bench/main_processor/motor:node_py",
    "//talOS/telemetry:node",
    "//talOS/teleop:node",
]

# Every file robot_py.toml reads besides itself.
CONFIG_FILES = [
    "//bench/main_processor/configuration:robot.toml",
    "//bench/main_processor/configuration:subsystems/motor.toml",
    "//bench/main_processor/configuration:subsystems/telemetry.toml",
    "//bench/main_processor/configuration:subsystems/teleop.toml",
]
