"""Generated from robot.toml by //tools:sync_manifest. Do not edit.

Regenerate after changing the config:

    bazel run //tools:sync_manifest -- examples/external_subsystem/robot.toml
"""

# Every node label robot.toml launches.
NODES = [
    "@wrist//:node",
]

# Every file robot.toml reads besides itself.
CONFIG_FILES = [
    "//examples/external_subsystem:wrist.toml",
]
