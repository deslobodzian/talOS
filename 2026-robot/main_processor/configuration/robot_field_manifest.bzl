"""Generated from robot_field.toml by //tools:sync_manifest. Do not edit.

Regenerate after changing the config:

    bazel run //tools:sync_manifest -- 2026-robot/main_processor/configuration/robot_field.toml
"""

# Every node label robot_field.toml launches.
NODES = [
    "//2026-robot/main_processor/autonomous:node",
    "//2026-robot/main_processor/driver:node",
    "//2026-robot/main_processor/drivetrain:node",
    "//2026-robot/main_processor/intake:node",
    "//2026-robot/main_processor/pose_estimator:node",
    "//2026-robot/main_processor/shooter:node",
    "//2026-robot/main_processor/vision:front_left_camera",
    "//2026-robot/main_processor/vision:front_right_camera",
    "//2026-robot/main_processor/vision:rear_left_camera",
    "//2026-robot/main_processor/vision:rear_right_camera",
    "//talOS/telemetry:node",
    "//talOS/teleop:node",
]

# Every file robot_field.toml reads besides itself.
CONFIG_FILES = [
    "//2026-robot/main_processor/configuration:robot.toml",
    "//2026-robot/main_processor/configuration:subsystems/autonomous.toml",
    "//2026-robot/main_processor/configuration:subsystems/driver.toml",
    "//2026-robot/main_processor/configuration:subsystems/drivetrain.toml",
    "//2026-robot/main_processor/configuration:subsystems/front_left_camera.toml",
    "//2026-robot/main_processor/configuration:subsystems/front_right_camera.toml",
    "//2026-robot/main_processor/configuration:subsystems/intake.toml",
    "//2026-robot/main_processor/configuration:subsystems/pose_estimator.toml",
    "//2026-robot/main_processor/configuration:subsystems/rear_left_camera.toml",
    "//2026-robot/main_processor/configuration:subsystems/rear_right_camera.toml",
    "//2026-robot/main_processor/configuration:subsystems/shooter.toml",
    "//2026-robot/main_processor/configuration:subsystems/telemetry.toml",
    "//2026-robot/main_processor/configuration:subsystems/teleop.toml",
    "//2026-robot/main_processor/configuration:subsystems/vision.toml",
]
