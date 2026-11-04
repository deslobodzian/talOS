"""Generated from robot.toml by //tools:sync_manifest. Do not edit.

Regenerate after changing the config:

    bazel run //tools:sync_manifest -- examples/superstructure/config/robot.toml
"""

# Every node label robot.toml launches.
NODES = [
    "//examples/superstructure/cpp:elevator",
    "//examples/superstructure/cpp:intake",
    "//examples/superstructure/cpp:superstructure",
    "//talOS/telemetry:node",
]

# Every file robot.toml reads besides itself.
CONFIG_FILES = [
    "//examples/superstructure/config:subsystems/elevator.toml",
    "//examples/superstructure/config:subsystems/intake.toml",
    "//examples/superstructure/config:subsystems/superstructure.toml",
    "//examples/superstructure/config:subsystems/telemetry.toml",
]
