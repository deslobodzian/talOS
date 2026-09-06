# 2026-robot

One robot, built on [talOS](../talOS). Two processors, one wire contract.

| | runs | built with |
|---|---|---|
| `main_processor/` | the talOS node graph — drivetrain, shooter, odometry, driver station | Bazel |
| `controller_processor/rio/` | the HAL that owns the actuators | GradleRIO / WPILib |

They communicate over the UDP framing in `talOS/protocol`, so the controller
processor is replaceable: a SystemCore or Jetson would be a new directory
beside `rio/` speaking the same frames, with no change to `main_processor/`.

## Configuration

`main_processor/configuration/robot.toml` is the single declaration of this
robot's hardware. It names every motor, sensor and subsystem, which talOS node
binary implements each subsystem, and the `sim_gateway` used in simulation.
`config_golden_test` pins that file against a reference config, so an
accidental edit to the hardware declaration fails the build.

## Adding a subsystem

Add a `[subsystems.<name>]` block with a `node = "//2026-robot/main_processor/<name>:node"`
target and its devices; the launcher discovers it from the config and the
hardware bridge starts tracking its requests. Subsystems that own no actuators
(the driver station, for instance) are input-only and get no request tracker.
