load("@talos//build:defs.bzl", "talos_bundle")

package(default_visibility = ["//visibility:public"])

talos_bundle(
    name = "robot_bundle",
    nodes = [
        "//talOS/bridge:hardware_node",
        "//2026-robot/main_processor/arbiter:node",
        "@talos_driver_station//:node",
        "//2026-robot/main_processor/drivetrain:node",
        "//2026-robot/main_processor/intake:node",
        "@talos_odometry//:node",
        "//2026-robot/main_processor/operator_interface:node",
        "//2026-robot/main_processor/shooter:node",
        "//2026-robot/main_processor/telemetry:node",
        "//2026-robot/main_processor/simulation:sim_gateway",
    ],
    config = "//2026-robot/main_processor/configuration:robot.toml",
    # Deployment configs: the canonical subsystem files under configuration/.
    # Per-node subsystem.toml examples (in-tree or @talos_* standalone) carry
    # standalone node identities ("//:node") and are not staged; the bundle
    # stages only these deployed copies so config/ and staged paths stay
    # consistent with robot.toml's subsystems/<name>.toml manifest.
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
