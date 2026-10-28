load("//talOS/build:contract.bzl", "talos_bundle")

package(default_visibility = ["//visibility:public"])

talos_bundle(
    name = "robot_bundle",
    nodes = [
        "//talOS/bridge:hardware_node",
        "//2026-robot/main_processor/arbiter:node",
        "//talOS/driver_station:node",
        "//2026-robot/main_processor/drivetrain:node",
        "//2026-robot/main_processor/intake:node",
        "//2026-robot/main_processor/odometry:node",
        "//2026-robot/main_processor/operator_interface:node",
        "//2026-robot/main_processor/shooter:node",
        "//2026-robot/main_processor/telemetry:node",
        "//2026-robot/main_processor/simulation:sim_gateway",
    ],
    config = "//2026-robot/main_processor/configuration:robot.toml",
    subsystems = [
        "//2026-robot/main_processor/configuration:subsystems/arbiter.toml",
        "//2026-robot/main_processor/configuration:subsystems/driver_station.toml",
        "//2026-robot/main_processor/configuration:subsystems/drivetrain.toml",
        "//2026-robot/main_processor/configuration:subsystems/intake.toml",
        "//2026-robot/main_processor/configuration:subsystems/odometry.toml",
        "//2026-robot/main_processor/configuration:subsystems/operator_interface.toml",
        "//2026-robot/main_processor/configuration:subsystems/shooter.toml",
        "//2026-robot/main_processor/configuration:subsystems/telemetry.toml",
    ],
)
