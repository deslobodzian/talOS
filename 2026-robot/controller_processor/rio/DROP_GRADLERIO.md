# Dropping GradleRIO: decision doc

Status: **decided — replace GradleRIO with a Bazel cross-compile; no build-system
code changes in this task.** See the `TODO (next step)` in `README.md` and
`TODO(build)` in `rio/build.gradle`.

## Context (what exists today)

- `rio/` is deliberately tiny: `Robot.cpp` (`frc::RobotBase`, 5 ms loop, DS
  watchdog via `HAL_ObserveUserProgram*`), `DriverStationReader.cpp`
  (`frc::DriverStation` -> `talos::driver_station` wire), `PhoenixBackend`
  (TalonFX / CANcoder / Pigeon2 + `*SimState` sim classes, plus
  `frc::{AnalogInput,DigitalInput,DigitalOutput,Encoder,PWM}`), layered on
  `talOS/{protocol,hardware,driver_station}` sources compiled into the same
  binary (see `build.gradle` `srcDir` list).
- GradleRIO supplies four things: (1) WPILib + friends headers/libs,
  (2) Phoenix 6 `26.3.0` natives (CTRE maven, `vendordeps/Phoenix6.json`),
  (3) the `linuxathena` (RoboRIO) cross toolchain + desktop target, with
  `-Wall -Wextra -pedantic -Werror -Wno-unused-parameter`, and
  (4) sim conveniences: `simulateNative`, `wpi.sim.addGui()`,
  `addDriverstation()`.
- `sim.sh` is the Driver Station contract: GUI run = enable + Teleoperated in
  the sim GUI; `--headless` = Gradle `-Pheadless` (skips `addGui()`) plus
  `TALOS_SIM_DS=1`. The `SimDriverStation` stand-in in `Robot.cpp`
  (`TALOS_SIM_DS`, `TALOS_SIM_DS_DISABLE_AFTER_MS`, `TALOS_SIM_RUN_MS`) is
  already build-system agnostic (`#ifndef __FRC_ROBORIO__`) and is also what
  `tools/test_drivetrain.py --wpilib` drives.

## Decision 1 — what the Bazel cross-compile needs

Vendor (or `http_archive`) the exact natives Gradle resolves today, for **two**
platforms:

1. **Desktop sim** (linuxx86-64 / mac): WPILib `wpilibc` + `hal` + `wpiutil`
   (+ transitive `ntcore`, simGUI/driverstation-sim support libs) and Phoenix 6
   `api-cpp` **including** the `sim/` headers and sim-state libs
   (`TalonFXSimState`, `CANcoderSimState`, `Pigeon2SimState`), plus the
   `units` library Phoenix headers require.
2. **`linuxathena` (RoboRIO)**: the same WPILib + Phoenix 6 artifacts built for
   the RIO (cortex-a9/ARMv7) plus a Bazel `cc_toolchain` + sysroot for that
   target; define `__FRC_ROBORIO__` for it (that one define is what gates out
   `SimDriverStation` in `Robot.cpp`), and keep the strict warning set above.
3. Reuse the `talOS/{protocol,hardware,driver_station}` sources that are
   already Bazel targets instead of recompiling them by relative `srcDir`.
4. Out of scope for sim but required before deleting Gradle: a deploy path
   replacing the `frcCpp` artifact + `src/main/deploy` static-file deploy
   (`sim.sh` never deploys; the real robot does).

## Decision 2 — what replaces sim-GUI enable / mode-select

- **GUI case:** keep the WPILib sim GUI — it is ordinary native code
  (`hal` sim + simGUI + the `addDriverstation` extension), not Gradle magic.
  The Bazel-built desktop sim binary links the same libs and opens the same
  window; the user still enables and picks Teleoperated there, and
  `simgui-ds.json` keyboard joysticks carry over unchanged. No behavior change.
- **Headless case:** unchanged. `TALOS_SIM_DS=1` (+ `DISABLE_AFTER_MS` /
  `RUN_MS`) in `Robot.cpp` already forges enable/teleop without any GUI, and
  `send_joystick` supplies stick deflection with the same wire encoding the
  Rio sends. The only thing `-Pheadless` does today is skip `addGui()`; the
  Bazel equivalent is a build flag (e.g. `--define rio_headless=1`) or a
  runtime switch (skip GUI init when `TALOS_SIM_DS` is set) — either way
  `sim.sh --headless` keeps meaning "no GUI, `TALOS_SIM_DS=1`".

## Decision 3 — sequenced cutover (`./sim.sh` works throughout)

0. **Pin current behavior.** Record `simulateNative` / `-Pheadless` log lines
   (`Robot program starting`, `hardware gateway config=...`,
   `sim_robot: active_ticks=...`); `tools/test_drivetrain.py --wpilib` green.
1. **Add Bazel desktop-sim target alongside Gradle.** No `sim.sh` change;
   acceptance: Bazel binary vs Gradle binary produce identical log lines and
   both pass `--wpilib`.
2. **Dual-path `sim.sh`.** New opt-in (e.g. `TALOS_RIO_BUILD=bazel`), default
   stays Gradle; CI runs both paths, GUI and `--headless`.
3. **Flip the default** to Bazel once both are green for a soak period; Gradle
   remains as fallback via the same switch.
4. **Add the `linuxathena` cross target** (`buildRio` parity: same warnings,
   `__FRC_ROBORIO__`, size/behavior check vs Gradle artifact). Still no deploy.
5. **Deploy parity, then delete Gradle** (`build.gradle`, wrapper, `sim.sh`
   Gradle branch). Done when `./sim.sh`, `./sim.sh --headless`, and
   `--wpilib` pass with no Gradle in the tree.

## Non-goals

No RIO program behavior change, no wire-contract change, no `talOS/` changes.
The program was cut down to `RobotBase` + gateway precisely so this stays a
packaging job.
