"""Generated from robot_py.toml by //tools:sync_manifest. Do not edit.

Regenerate after changing the config:

    bazel run //tools:sync_manifest -- examples/superstructure/config/robot_py.toml
"""

# Every node label robot_py.toml launches.
NODES = [
    "//examples/superstructure/py:elevator",
    "//examples/superstructure/py:intake",
    "//examples/superstructure/py:superstructure",
]

# Every file robot_py.toml reads besides itself.
CONFIG_FILES = [
    "//examples/superstructure/config:robot.toml",
    "//examples/superstructure/config:subsystems/elevator.toml",
    "//examples/superstructure/config:subsystems/intake.toml",
    "//examples/superstructure/config:subsystems/superstructure.toml",
]
