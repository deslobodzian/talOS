# talOS readiness report

Regenerate, never hand-edit:

```bash
bazel build //2026-robot/main_processor:robot_bundle
python3 tools/delivery/readiness.py \
    --config 2026-robot/main_processor/configuration/robot.toml \
    --bundle bazel-bin/2026-robot/main_processor/robot_bundle
```

Captured 2026-09-14 from the real competition config plus the freshly
built `robot_bundle` (no `--vendor`: no vendor/firmware expectations file
exists in the tree yet, so that dimension inhibits by construction).

```
# talOS readiness report

## processes
- [READY] arbiter: bin/arbiter [cpp]
- [READY] drivetrain: bin/drivetrain [cpp]
- [READY] driver_station: bin/driver_station [cpp]
- [READY] intake: bin/intake [python]
- [READY] odometry: bin/odometry [cpp]
- [READY] operator_interface: bin/operator_interface [cpp]
- [READY] shooter: bin/shooter [cpp]
- [READY] telemetry: bin/telemetry [cpp]

## hardware
- [INHIBIT] arbiter: uncommissioned: commands gated on hardware
- [INHIBIT] drivetrain: uncommissioned: commands gated on hardware
- [INHIBIT] driver_station: uncommissioned: commands gated on hardware
- [INHIBIT] intake: uncommissioned: commands gated on hardware
- [INHIBIT] odometry: uncommissioned: commands gated on hardware
- [INHIBIT] operator_interface: uncommissioned: commands gated on hardware
- [INHIBIT] shooter: uncommissioned: commands gated on hardware
- [INHIBIT] telemetry: uncommissioned: commands gated on hardware

## config
- [READY] config_sha256: c2a85e2a77c059a5...

## feedback
- [READY] arbiter: no motors
- [READY] drivetrain.back_left_drive: rotor feedback
- [READY] drivetrain.back_left_steer: remote CANcoder 'back_left_encoder'
- [READY] drivetrain.back_right_drive: rotor feedback
- [READY] drivetrain.back_right_steer: remote CANcoder 'back_right_encoder'
- [READY] drivetrain.front_left_drive: rotor feedback
- [READY] drivetrain.front_left_steer: remote CANcoder 'front_left_encoder'
- [READY] drivetrain.front_right_drive: rotor feedback
- [READY] drivetrain.front_right_steer: remote CANcoder 'front_right_encoder'
- [READY] driver_station: no motors
- [READY] intake.roller: rotor feedback
- [READY] odometry: no motors
- [READY] operator_interface: no motors
- [READY] shooter.flywheel: rotor feedback
- [READY] telemetry: no motors

## vendor/firmware
- [INHIBIT] expectations: no vendor/firmware expectations recorded; pass --vendor vendor_expectations.json

verdict: INHIBITED
```

Reading it: the bundle is complete and internally consistent (every roster
node staged, config hash matches the manifest, every feedback source
resolves), but the robot must not run on hardware yet — the competition
config is uncommissioned, and nobody has recorded which vendor/firmware
versions the field crew must match. Commission (`commissioned = true` in
the competition `robot.toml` after the commissioning sign-off) and add a
`vendor_expectations.json` (`compat_record.py --vendor`), then regenerate
this report to READY.
