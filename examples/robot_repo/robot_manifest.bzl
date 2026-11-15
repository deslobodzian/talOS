"""Generated from robot.toml by @talos//tools:sync_manifest. Do not edit.

Regenerate after changing the config:

    bazel run @talos//tools:sync_manifest -- robot.toml
"""

# Every node label robot.toml launches.
NODES = [
    "//gate:node",
    "//roller:node",
    "@talos//talOS/telemetry:node",
    "@talos//talOS/teleop:node",
]

# Every file robot.toml reads besides itself.
CONFIG_FILES = [
    "//:subsystems/gate.toml",
    "//:subsystems/roller.toml",
    "//:subsystems/telemetry.toml",
    "//:subsystems/teleop.toml",
]
