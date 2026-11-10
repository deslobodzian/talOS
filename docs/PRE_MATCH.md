# Pre-match checklist

For the drive team, at the field before each match, with the robot on and
disabled. The Driver Station's green robot code and communications only say
the RoboRIO program runs; whether the main processor (the companion) is
driving it is on the dashboard and in the Driver Station message log.

Open any NetworkTables dashboard (Shuffleboard, Elastic, Glass) on the
`SmartDashboard/companion/` values the RoboRIO publishes (what each means:
[RoboRIO](../talOS/hardware/README.md#roborio)).

- [ ] Driver Station: robot code and communications green, joysticks in the
  expected slots (joystick 0 also selects the autonomous routine).
- [ ] Battery: a freshly charged battery, resting voltage on the Driver
  Station at or above the team's threshold.
- [ ] `companion/connected` is true and `companion/frame_age_s` is below 0.5.
  False: the main processor is off, still booting, or not on the robot
  network; the RoboRIO drives nothing without it.
- [ ] `companion/command_age_s` is between 0 and 0.5, disabled or not. Above
  0.5 or -1 with `connected` true: the main processor is up but its actuator
  nodes are not commanding (down or crash-looping); the Driver Station log
  shows a `no actuator commands` warning.
- [ ] `companion/protocol_ok` is true. False: the RoboRIO program and the main
  processor bundle come from different builds (`companion/protocol` and
  `companion/peer_protocol` show both versions); redeploy both from one
  checkout ([DEPLOYMENT.md](DEPLOYMENT.md#roborio)).
- [ ] `companion/configured` and `companion/commissioned` are true. False: the
  RoboRIO has no hardware configuration (neither its saved copy nor a push
  from the main processor) or an uncommissioned one, and holds every output
  neutral.
- [ ] Driver Station message log: no `talos companion link` error since the
  last check (link lost, not heard since robot code start, or another
  protocol version).
- [ ] Autonomous routine: with the robot disabled, step it with joystick 0's
  POV right/left and read the selection in Studio's
  `autonomous/state/routine_index` (the index into `routines` in the
  deployed [autonomous config](../2026-robot/main_processor/configuration/subsystems/autonomous.toml);
  see [BRINGUP.md](BRINGUP.md#autonomous-selection)). The Driver Station does
  not show it yet (TODO.md DEPLOY-2).
