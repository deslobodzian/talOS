load("@talos//build:defs.bzl", "talos_bundle")

talos_bundle(
    name = "robot_bundle",
    nodes = [
        "@talos//talOS/bridge:hardware_node",
        "@talos_driver_station//:node",
        "@talos_operator_interface//:node",
        "@talos_odometry//:node",
        "@talos_telemetry//:node",
    ],
    config = ":robot.toml",
    subsystems = [
        "fixture_actuators.toml",
        "driver_station.toml",
        "operator_interface.toml",
        "odometry.toml",
        "telemetry.toml",
    ],
)
