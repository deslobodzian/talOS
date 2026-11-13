# Pre-match checklist

For the drive team, at the field before each match, with the robot on and
disabled. The Driver Station's green robot code and communications only say
the RoboRIO program runs; whether the main processor is driving it is on the
driver dashboard.

Open the driver dashboard on the Driver Station laptop:
`http://10.56.87.11:5800/drive` (`http://<main processor>:5800/drive`; what
it draws: [Studio README](../studio/README.md#driver-dashboard)). The boot
service, or `run.sh`, starts the Studio bridge that serves it
([DEPLOYMENT.md](DEPLOYMENT.md#build-a-bundle)). The full Studio at `/` is
for the pit. A widget dimmed and tagged
STALE is not current, and "no data" means never received: neither passes a
check below.

## Driver Station

- [ ] Robot code and communications green, joysticks in the expected slots
  (joystick 0 also selects the autonomous routine).
- [ ] Battery: a freshly charged battery, resting voltage on the Driver
  Station at or above the team's threshold.
- [ ] Message log: no `talos companion link` error since the last check (link
  lost, not heard since robot code start, or another protocol version), and
  no `companion up, but no actuator commands` warning.

## Driver dashboard, top to bottom

- [ ] Status line reads **OK** `Connected: telemetry current`. Otherwise:

  | Status line | Meaning | Do |
  |---|---|---|
  | **CONNECTING** | the page is opening its feed, or waiting for the first telemetry | wait a few seconds |
  | **BRIDGE LINK** `Dashboard to bridge link down` or `nothing heard` | the laptop does not reach the main processor's port 5800: main processor off, still booting, off the robot network, or started with `--no-studio` | check the main processor's power and Ethernet; if it persists, [main processor not reachable](#main-processor-not-reachable) |
  | **TELEMETRY** `Telemetry stale` | the bridge is up but the telemetry node publishes nothing new: every value is stale | check the telemetry node in Studio at `/` (the pit); the robot can drive, but the dashboard shows nothing current |
  | **ROBORIO LINK** `RoboRIO link lost` | the main processor accepts no hardware state from the RoboRIO: RoboRIO off, its program not running, a wrong address, or another protocol version; every output is neutral | read the Driver Station message log; on a protocol error redeploy both sides from one checkout ([DEPLOYMENT.md](DEPLOYMENT.md#roborio)) |
  | **ROBORIO LINK** `RoboRIO link unknown` | no hardware status: the hardware bridge (`hardware_node`) is not publishing | check Subsystems below |
  | **REPLACED** | another `/drive` page took this page's slot (the layout's `drive_clients` are held) | close the other page, then press Reconnect, which takes the oldest page's slot |

- [ ] Match: `DISABLED`; the alliance and station are the assigned ones; at
  the field, the match number and `FMS`. `NO DRIVER STATION`: the main
  processor gets no Driver Station packets through the RoboRIO. `E-STOP`: the
  robot is e-stopped.
- [ ] Battery: a voltage, with no `LOW` and no `BROWNOUT` tag.
- [ ] Auto routine: the intended routine by name. With the robot disabled,
  step it with joystick 0's POV right/left; the name follows. `routine #N`
  means an index outside `routines` in the deployed
  [autonomous config](../2026-robot/main_processor/configuration/subsystems/autonomous.toml)
  ([BRINGUP.md](BRINGUP.md#autonomous-selection)).
- [ ] Alerts: Active reads `none`. Each alert names its owner and its
  condition (`hw link lost`, `hw not configured`, `hw uncommissioned`,
  `hw motor fault`, `hw owner timed out`, `hw brownout`; what each means:
  [bridge_status.fbs](../talOS/bridge/bridge_status.fbs)). `owner timed out`: an actuator node is
  not commanding (down or crash-looping; see Subsystems). The Raised list
  keeps every alert seen this match with its count: read it after the match.
- [ ] Field: the robot drawn at its starting position, with no `POSE NOT
  VALID` (odometry not integrating fresh input) or `NOT LOCALIZED` (the
  pose estimator has neither trusted its seed nor seen a field tag) tag.
- [ ] Subsystems: every node `UP`, none `QUARANTINED`, and `restarts` not
  rising between checks. `DOWN` in red is a critical node. `unknown`: the
  bridge could not read the live registry, so the rows say nothing.
- [ ] Pre-match: every item `OK` (bounds in the
  [layout](../2026-robot/main_processor/configuration/dashboard.toml)):

  | Item | Reads | FAIL means |
  |---|---|---|
  | Battery | `/hw/status` `battery_v` at the layout's bound or above | swap the battery |
  | Hardware configured | `/hw/status` `configured`: the RoboRIO's current boot acknowledged the main processor's hardware configuration, so it hears the main processor | the RoboRIO has no configuration from it yet and holds every output neutral |
  | Commissioned | `/hw/status` `commissioned`: the configuration is marked commissioned | the RoboRIO holds every output neutral; a person commissions the robot ([BRINGUP.md](BRINGUP.md#human-commissioning)) |
  | Swerve modules | `/drivetrain/state` `modules_valid` | a swerve module's motor or CANcoder feedback is invalid |
  | Gyro | `/drivetrain/state` `yaw_valid` | no usable heading: neither the gyro's yaw nor one integrated from valid modules |
  | Localized | `/pose_estimator/state` `localized` | as the Field widget's `NOT LOCALIZED` |

## Main processor not reachable

When the dashboard does not load or stays on **BRIDGE LINK**, the main
processor's own view is unavailable, and only the RoboRIO's remains: the
`SmartDashboard/companion/` values it publishes through NetworkTables (what
each means: [RoboRIO](../talOS/hardware/README.md#roborio)), read in any
NetworkTables dashboard (Shuffleboard, Elastic, Glass) on the Driver Station
laptop. They tell whether the RoboRIO hears the main processor at all:

- `companion/connected` true and `companion/frame_age_s` below 0.5: the main
  processor's hardware bridge reaches the RoboRIO, so only the dashboard's
  link is down (the bridge's port 5800, or `--no-studio`). False: the main
  processor is off, still booting, or not on the robot network; the RoboRIO
  drives nothing without it.
- `companion/command_age_s` above 0.5, or -1, with `connected` true: the
  main processor is up but its actuator nodes are not commanding (down or
  crash-looping).
- `companion/protocol_ok` false: the RoboRIO program and the main processor
  bundle come from different builds (`companion/protocol` and
  `companion/peer_protocol` show both versions); redeploy both from one
  checkout ([DEPLOYMENT.md](DEPLOYMENT.md#roborio)).
- `companion/configured` or `companion/commissioned` false: the RoboRIO has
  no hardware configuration (neither its saved copy nor a push from the main
  processor) or an uncommissioned one, and holds every output neutral.
