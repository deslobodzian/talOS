"""Generated from soak.toml by //tools:sync_manifest. Do not edit.

Regenerate after changing the config:

    bazel run //tools:sync_manifest -- bench/main_processor/simulation/soak.toml
"""

# Every node label soak.toml launches.
NODES = [
    "//bench/main_processor/gyro:node",
    "//bench/main_processor/motor:node_py",
    "//bench/main_processor/swerve_module:node",
    "//talOS/telemetry:node",
    "//talOS/teleop:node",
]

# Every file soak.toml reads besides itself.
CONFIG_FILES = [
    "//bench/main_processor/configuration:robot.toml",
    "//bench/main_processor/configuration:subsystems/gyro.toml",
    "//bench/main_processor/configuration:subsystems/motor.toml",
    "//bench/main_processor/configuration:subsystems/motor_py.toml",
    "//bench/main_processor/configuration:subsystems/swerve_module.toml",
    "//bench/main_processor/configuration:subsystems/telemetry.toml",
    "//bench/main_processor/configuration:subsystems/teleop.toml",
]
