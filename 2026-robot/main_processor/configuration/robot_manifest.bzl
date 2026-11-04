"""Generated from robot.toml by //tools:sync_manifest. Do not edit.

Regenerate after changing the config:

    bazel run //tools:sync_manifest -- 2026-robot/main_processor/configuration/robot.toml
"""

# Every node label robot.toml launches.
NODES = [
    "//2026-robot/main_processor/drivetrain:node",
    "//2026-robot/main_processor/intake:node",
    "//2026-robot/main_processor/operator_interface:node",
    "//2026-robot/main_processor/pose_estimator:node",
    "//2026-robot/main_processor/shooter:node",
    "//2026-robot/main_processor/telemetry:node",
]

# Every file robot.toml reads besides itself.
CONFIG_FILES = [
    "//2026-robot/main_processor/configuration:subsystems/drivetrain.toml",
    "//2026-robot/main_processor/configuration:subsystems/intake.toml",
    "//2026-robot/main_processor/configuration:subsystems/operator_interface.toml",
    "//2026-robot/main_processor/configuration:subsystems/pose_estimator.toml",
    "//2026-robot/main_processor/configuration:subsystems/shooter.toml",
    "//2026-robot/main_processor/configuration:subsystems/telemetry.toml",
]
