"""C++ vs Python differential parity test over talos.parity.

Compares C++ nodes hosted through talos.cpp_node (differential_parity_nodes.cc)
against Python mirrors tick-for-tick and byte-for-byte across:
- all 8 device kinds (Motor, CANcoder, Gyro, Encoder, DigitalInput,
  AnalogInput, DigitalOutput, PwmOutput),
- DriverStation, Target, TargetLane, OperatorArming (with ArmingBlocker),
- StateMachine, Node.session_dir / Sim.set_session_dir, Sim.pause / Sim.resume,
- SimHardware.reset_device / SimHardware.reboot,
- all 3 talOS/mechanisms classes (PositionMechanism including hold(),
  VelocityMechanism, RollerWithSensor).

REQ-CONTRACT-14
"""

import os
import unittest

from control.PositionState import PositionState
from control.PositionTarget import PositionTarget
from control.VelocityState import VelocityState
from control.VelocityTarget import VelocityTarget
from demo.Goal import Goal
from demo.Phase import Phase
from demo.Status import Status
from geometry.PoseEstimate import PoseEstimate
from geometry.PoseObservation import PoseObservation
from python.runfiles import runfiles
import talos
from talos import parity
from talos.mechanisms import (PositionMechanism, RollerCommand,
                              RollerWithSensor, VelocityMechanism)
from talos.testing import Sim, SimHardware, Stick

_RUNFILES = runfiles.Create()
_CPP_LIB = _RUNFILES.Rlocation(os.environ["TALOS_PARITY_NODES_LIB"])

CppDevicesRig = talos.cpp_node(_CPP_LIB, "devices_rig")
CppOperatorNode = talos.cpp_node(_CPP_LIB, "operator_node")
CppPositionNode = talos.cpp_node(_CPP_LIB, "position_node")
CppVelocityNode = talos.cpp_node(_CPP_LIB, "velocity_node")
CppRollerNode = talos.cpp_node(_CPP_LIB, "roller_node")

ROBOT_TOML = """
[robot]
period_us = 10000
command_timeout_us = 100000

[subsystems.rig.motors.drive]
type = "TalonFX"
can_id = 1
max_voltage = 12.0
[subsystems.rig.sensors.cc]
type = "CANcoder"
can_id = 2
[subsystems.rig.sensors.imu]
type = "Pigeon2"
can_id = 3
[subsystems.rig.encoders.enc]
type = "QuadratureEncoder"
dio_a = 0
dio_b = 1
[subsystems.rig.digital_inputs.beam]
dio = 2
[subsystems.rig.analog_inputs.pot]
channel = 0
[subsystems.rig.digital_outputs.solenoid]
dio = 3
[subsystems.rig.pwm_outputs.servo]
channel = 0
[subsystems.rig.targets]
teleop = ["teleop"]
autonomous = ["auto"]

[subsystems.operator]

[subsystems.arm.motors.arm]
can_id = 4
feedback = "RemoteCANcoder"
feedback_sensor = "arm_cc"
position_frame = "absolute"
[subsystems.arm.sensors.arm_cc]
type = "CANcoder"
can_id = 5
[subsystems.arm.arm]
tolerance_rot = 0.01
gravity_v = 0.5
units_per_rot = 360.0
stall_time_us = 50000

[subsystems.shooter.motors.flywheel]
can_id = 6
max_velocity_rps = 80.0
[subsystems.shooter.flywheel]
at_speed_entry_tick_count = 3

[subsystems.intake.motors.roller]
can_id = 7
[subsystems.intake.digital_inputs.piece_beam]
dio = 4
[subsystems.intake.roller]
sensor = "piece_beam"
intake_v = 6.0
hold_v = 1.0
eject_v = -6.0
eject_us = 40000
feed_v = 8.0
feed_us = 40000
"""


def _fnv1a64(text):
    h = 14695981039346656037
    for b in text.encode("utf-8"):
        h ^= b
        h = (h * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return h


def _write_config(name):
    path = os.path.join(os.environ["TEST_TMPDIR"], name)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(ROBOT_TOML)
    return path


class PyDevicesRig:
    """Python twin of DevicesRig in differential_parity_nodes.cc."""

    def __init__(self, node):
        self.node = node
        self.link = talos.HardwareLink(node)
        self.drive = talos.Motor(node, "drive")
        self.cc = talos.CANcoder(node, "cc")
        self.imu = talos.Gyro(node, "imu")
        self.enc = talos.Encoder(node, "enc")
        self.beam = talos.DigitalInput(node, "beam")
        self.pot = talos.AnalogInput(node, "pot")
        self.solenoid = talos.DigitalOutput(node, "solenoid")
        self.servo = talos.PwmOutput(node, "servo")
        self.ds = talos.DriverStation(node)
        self.target = talos.Target(node, Goal)
        self.sm = talos.StateMachine(node, Phase.IDLE, states=Phase)
        self.state = node.publish(PoseEstimate, "/rig/state")
        self.status = node.publish(PoseObservation, "/rig/status")
        self.telemetry = node.publish(Status, "/rig/telemetry")

    def update(self):
        goal = self.target.get()
        src = self.target.source

        if goal is None:
            self.sm.go(Phase.IDLE)
        elif goal.enabled:
            self.sm.go(Phase.MOVING)
        else:
            self.sm.go(Phase.HOLDING)

        if goal is not None:
            action = goal.tick_count
            if action == 1:
                self.drive.rezero(goal.height_rot)
                self.imu.rezero(goal.height_rot)
            elif action == 2:
                self.drive.set_position(goal.height_rot)
            elif action == 3:
                self.drive.set_voltage(goal.height_rot)
                self.solenoid.set(goal.enabled)
                self.servo.set(goal.height_rot / 10.0)
            elif action == 4:
                self.drive.stop()
                self.solenoid.set(False)
                self.servo.stop()

        sample_time = self.link.sample_time
        sample_time_ns = 0 if sample_time is None else sample_time
        device_flags = (
            (int(self.drive.connected) << 0)
            | (int(self.cc.connected) << 1)
            | (int(self.imu.connected) << 2)
            | (int(self.enc.connected) << 3)
            | (int(self.beam.connected) << 4)
            | (int(self.pot.connected) << 5)
            | (int(self.drive.zeroed) << 6)
            | (int(self.imu.zeroed) << 7)
            | (int(self.beam.value) << 8)
            | (int(self.drive.forward_limit) << 9)
            | (int(self.drive.reverse_limit) << 10)
            | (int(self.link.enabled) << 11)
            | (int(self.link.fresh()) << 12)
            | (int(self.sm.just_entered) << 13)
        )
        self.state.send(
            sample_time_ns=sample_time_ns,
            pose_x_m=self.drive.position,
            pose_y_m=self.cc.position,
            pose_yaw_rad=self.imu.yaw,
            velocity_vx_mps=self.drive.velocity,
            velocity_vy_mps=self.cc.velocity,
            velocity_omega_radps=self.imu.yaw_rate,
            valid=(self.drive.connected and self.cc.connected
                   and self.imu.connected),
            localized=self.drive.zeroed and self.imu.zeroed,
            observation_accepted_count=device_flags,
            observation_rejected_count=self.drive.clamped_count,
        )

        src_bits = 1 if src == "teleop" else (2 if src == "auto" else 0)
        ds_sm_flags = (
            (int(self.ds.connected) << 0)
            | (int(self.ds.enabled) << 1)
            | (int(self.ds.teleop) << 2)
            | (int(self.ds.autonomous) << 3)
            | (int(self.ds.test) << 4)
            | (int(self.ds.estop) << 5)
            | (int(self.ds.fms_attached) << 6)
            | (int(self.ds.alliance) << 8)
            | (int(self.ds.station) << 10)
            | (src_bits << 14)
            | (int(self.sm.state) << 16)
            | (int(self.sm.previous) << 24)
        )
        self.status.send(
            sample_time_ns=round(self.sm.time_in_state * 1e9),
            pose_x_m=self.enc.position,
            pose_y_m=self.enc.velocity,
            pose_yaw_rad=self.pot.voltage,
            x_std_m=self.drive.temperature_c,
            y_std_m=self.drive.supply_current_a,
            yaw_std_rad=self.ds.match_time,
            tag_count=ds_sm_flags,
            valid=self.sm.is_(Phase.MOVING),
        )

        session_dir = self.node.session_dir
        session_tag = (
            _fnv1a64(session_dir) if session_dir is not None else self.node.tick
        )
        self.telemetry.send(
            height_rot=self.drive.current + self.ds.joystick(0).axis(0),
            at_goal=session_dir is not None,
            tick_count=session_tag,
        )


class PyOperatorNode:
    """Python twin of OperatorNode in differential_parity_nodes.cc."""

    def __init__(self, node):
        self.ds = talos.DriverStation(node)
        self.arming = talos.OperatorArming(node)
        self.lane = talos.TargetLane(node, Goal, "rig", "teleop",
                                     arming=self.arming)
        self.status = node.publish(PoseObservation, "/operator/status")

    def update(self):
        stick = self.ds.joystick(0)
        axis0 = stick.axis(0)
        btn1 = stick.button(1)
        btn2 = stick.button(2)
        btn3 = stick.button(3)
        pov0 = stick.pov(0)

        if btn2:
            self.arming.disarm()

        neutral = True
        input_blocker = talos.ArmingBlocker()
        if abs(axis0) > 0.1:
            neutral = False
            input_blocker = talos.ArmingBlocker(
                talos.ArmingInputKind.AXIS, 0, 0)
        elif btn1:
            neutral = False
            input_blocker = talos.ArmingBlocker(
                talos.ArmingInputKind.BUTTON, 0, 1)
        elif pov0 != -1:
            neutral = False
            input_blocker = talos.ArmingBlocker(
                talos.ArmingInputKind.POV, 0, 0)

        can_send = self.arming.evaluate(neutral, input_blocker)
        sent = False
        if (can_send and not neutral) or btn3:
            sent = self.lane.send(height_rot=axis0, enabled=btn1,
                                  tick_count=pov0)
        else:
            self.lane.release()

        reported = self.arming.blocker
        flags = (
            (int(can_send) << 0)
            | (int(self.arming.armed) << 1)
            | (int(sent) << 2)
            | (int(reported.kind) << 4)
            | (int(stick.connected) << 8)
            | (int(self.ds.enabled) << 9)
        )
        self.status.send(
            sample_time_ns=0,
            pose_x_m=axis0,
            pose_y_m=float(reported.port),
            pose_yaw_rad=float(reported.index),
            x_std_m=float(pov0),
            y_std_m=float(stick.axis_count),
            yaw_std_rad=float(stick.button_count),
            tag_count=flags,
            valid=self.arming.armed,
        )


class PyPositionNode:
    """Python twin of PositionNode in differential_parity_nodes.cc."""

    def __init__(self, node):
        self.arm = PositionMechanism(node, "arm")
        self.target = talos.Target(node, PositionTarget)
        self.mode = node.subscribe(Goal, "/arm/mode", optional=True)
        self.state = node.publish(PositionState, "/arm/state")
        self.status = node.publish(PoseObservation, "/arm/status")

    def update(self):
        action = self.mode.value.tick_count if self.mode.has_value else 0
        if action == 3:
            self.arm.clear_fault()
        t = self.target.get()
        if t is None or action == 3:
            self.arm.stop()
        elif action == 1:
            self.arm.set_goal_units(t.position_rot * 360.0, t.feedforward_v)
        elif action == 2:
            self.arm.hold(t.feedforward_v)
        else:
            self.arm.set_goal(t.position_rot, t.feedforward_v)
        self.arm.update()
        self.state.send(self.arm.state())

        flags = (
            (int(self.arm.phase) << 8)
            | (int(self.arm.fault) << 4)
            | (int(self.arm.homed) << 3)
            | (int(self.arm.at_goal) << 2)
            | (int(self.arm.driving) << 1)
            | int(self.arm.connected)
        )
        self.status.send(
            sample_time_ns=0,
            pose_x_m=self.arm.position_rot,
            pose_y_m=self.arm.position_units,
            pose_yaw_rad=self.arm.velocity_rps,
            x_std_m=self.arm.goal_rot,
            y_std_m=self.arm.gravity_v,
            yaw_std_rad=self.arm.tolerance_rot,
            tag_count=flags,
            valid=self.arm.driving,
        )


class PyVelocityNode:
    """Python twin of VelocityNode in differential_parity_nodes.cc."""

    def __init__(self, node):
        self.flywheel = VelocityMechanism(node, "flywheel")
        self.target = talos.Target(node, VelocityTarget)
        self.state = node.publish(VelocityState, "/shooter/state")
        self.status = node.publish(PoseObservation, "/shooter/status")

    def update(self):
        t = self.target.get()
        if t is None:
            self.flywheel.stop()
        else:
            self.flywheel.set_goal(t.velocity_rps, t.feedforward_v)
        self.flywheel.update()
        self.state.send(self.flywheel.state())

        flags = (
            (int(self.flywheel.phase) << 4)
            | (int(self.flywheel.at_speed) << 2)
            | (int(self.flywheel.driving) << 1)
            | int(self.flywheel.connected)
        )
        self.status.send(
            sample_time_ns=0,
            pose_x_m=self.flywheel.velocity_rps,
            pose_y_m=self.flywheel.goal_rps,
            pose_yaw_rad=self.flywheel.commanded_rps,
            x_std_m=0.0,
            y_std_m=0.0,
            yaw_std_rad=0.0,
            tag_count=flags,
            valid=self.flywheel.at_speed,
        )


class PyRollerNode:
    """Python twin of RollerNode in differential_parity_nodes.cc."""

    def __init__(self, node):
        self.roller = RollerWithSensor(node, "roller")
        self.target = talos.Target(node, Goal)
        self.status = node.publish(Status, "/intake/status")

    def update(self):
        t = self.target.get()
        if t is None:
            self.roller.stop()
        else:
            self.roller.set_goal(RollerCommand(t.tick_count))
        self.roller.update()
        flags = (
            (int(self.roller.phase) << 4)
            | (int(self.roller.sensor_connected) << 1)
            | int(self.roller.driving)
        )
        self.status.send(
            height_rot=self.roller.output_v,
            at_goal=self.roller.has_piece,
            tick_count=flags,
        )


class DifferentialParityTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.config = _write_config("differential_parity_robot.toml")

    # REQ-CONTRACT-14: all 8 device kinds, DriverStation, Target, StateMachine,
    # rezero/zeroed, reset_device, reboot, and pause/resume match byte-for-byte
    # between C++ (cpp_node) and Python across every tick.
    def test_all_devices_ds_target_sm_resets_reboot_and_pause_resume(self):
        def reboot_and_wait(seconds):
            def run(rig):
                rig.hw.reboot()
                rig.advance(seconds)
            return run

        def pause_and_resume(pause_s, wait_before_resume_s, wait_after_s):
            def run(rig):
                rig.sim.pause(rig.node, pause_s)
                rig.advance(wait_before_resume_s)
                rig.sim.resume(rig.node)
                rig.advance(wait_after_s)
            return run

        def goal(action, height_rot=0.0, enabled=True, source="teleop",
                 then=0.02):
            return parity.send(Goal, source, then=then, height_rot=height_rot,
                               enabled=enabled, tick_count=action)

        scenario = [
            ("disabled baseline", parity.wait(0.03),
             parity.require_published()),
            ("enable and inject all 8 devices",
             parity.gateway(True),
             parity.driver_station(
                 enabled=True, teleop=True, fms_attached=True,
                 alliance=talos.Alliance.RED, station=2, match_time=135.5,
                 joysticks=[Stick(axes=(0.25,), buttons=(1,), povs=(90,))]),
             parity.device("set_motor", "drive", 1.5, 2.0, 1000, 500, 38.0,
                           5.5, 9.25),
             parity.device("set_limit_switches", "drive", True, False),
             parity.device("set_cancoder", "cc", 0.375, -1.25, 1000, 1000),
             parity.device("set_gyro", "imu", 0.5, 0.25, 500, 500),
             parity.device("set_encoder", "enc", 12.5, -3.5),
             parity.device("set_digital_input", "beam", True),
             parity.device("set_analog_input", "pot", 2.75),
             parity.wait(0.10)),  # crosses 8-state clock convergence gate
            ("drive voltage + digital_output + pwm_output",
             goal(3, height_rot=5.0, enabled=True)),
            ("clamp voltage above max_voltage",
             goal(3, height_rot=25.0, enabled=True)),
            ("position before rezero stays neutral",
             goal(2, height_rot=1.0, enabled=False)),
            ("rezero motor and gyro",
             goal(1, height_rot=0.25, enabled=False)),
            ("position after rezero drives POSITION",
             goal(2, height_rot=1.0, enabled=True)),
            ("reset_device drops motor zeroed",
             parity.device("reset_device", "drive", then=0.02),
             goal(2, height_rot=1.0, enabled=True)),
            ("reset_device drops gyro zeroed",
             parity.device("reset_device", "imu", then=0.02),
             goal(3, height_rot=3.0, enabled=True)),
            ("rezero again before reboot",
             goal(1, height_rot=0.5, enabled=False)),
            ("reboot drops zeros and neutralizes until new boot",
             reboot_and_wait(0.03),
             goal(2, height_rot=1.0, enabled=True)),
            ("stop motor + digital_output false + pwm stop",
             goal(4, height_rot=0.0, enabled=False)),
            ("pause and resume mid-run",
             goal(3, height_rot=4.0, enabled=True, then=0.01),
             pause_and_resume(0.06, 0.03, 0.03)),
            ("autonomous lane switch",
             parity.driver_station(
                 enabled=True, autonomous=True, alliance=talos.Alliance.BLUE,
                 station=1, match_time=12.0, then=0.02),
             goal(3, height_rot=-4.0, enabled=True, source="auto")),
            ("disconnect sensors and release",
             parity.device("set_connected", "enc", False),
             parity.device("set_connected", "pot", False),
             parity.device("set_connected", "beam", False),
             parity.release(Goal, "auto", then=0.02)),
        ]
        parity.assert_same_behaviour(
            CppDevicesRig, PyDevicesRig, self.config, "rig", scenario,
            topics=[(PoseEstimate, "/rig/state"),
                    (PoseObservation, "/rig/status"),
                    (Status, "/rig/telemetry")],
            motors=["drive"], digital_outputs=["solenoid"], pwms=["servo"])

    # REQ-CONTRACT-14: OperatorArming (evaluate with Axis/Button/POV blockers,
    # armed, disarm, mode transitions, reboot) and TargetLane (armed send vs
    # disarmed send held to release) match between C++ and Python.
    def test_operator_arming_and_target_lane_parity(self):
        def ds_stick(axes=(0.0,), buttons=(), povs=(-1,), *,
                     enabled=True, teleop=True, autonomous=False, then=0.02):
            return parity.driver_station(
                enabled=enabled, teleop=teleop, autonomous=autonomous,
                joysticks=[Stick(axes=axes, buttons=buttons, povs=povs,
                                 button_count=4)],
                then=then)

        def reboot_step(rig):
            rig.hw.reboot()
            rig.advance(0.02)

        def check_latest_target(expected_active):
            def run(rig):
                got = rig.sim.latest_target(Goal, "rig", "teleop")
                if expected_active:
                    if got is None:
                        raise AssertionError("expected active target on lane")
                elif got is not None:
                    raise AssertionError(f"expected released lane, got {got!r}")
            return run

        scenario = [
            ("disabled startup", parity.wait(0.02),
             parity.require_published()),
            ("non-neutral axis blocks arming and btn3 send stays released",
             parity.gateway(True),
             ds_stick(axes=(0.6,), buttons=(3,)),
             check_latest_target(False)),
            ("non-neutral button blocks arming",
             ds_stick(axes=(0.0,), buttons=(1,))),
            ("non-neutral pov blocks arming",
             ds_stick(axes=(0.0,), povs=(90,))),
            ("neutral reading arms operator",
             ds_stick(axes=(0.0,), buttons=(), povs=(-1,)),
             check_latest_target(False)),
            ("armed axis sends active target on lane",
             ds_stick(axes=(-0.75,), buttons=(1,)),
             check_latest_target(True)),
            ("explicit disarm via btn2 blocks active send",
             ds_stick(axes=(0.5,), buttons=(2, 3)),
             check_latest_target(False)),
            ("neutral re-arms then mode switch disarms",
             ds_stick(axes=(0.0,)),
             ds_stick(axes=(0.5,), enabled=True, teleop=False, autonomous=True),
             check_latest_target(False)),
            ("re-arm in teleop then reboot disarms",
             ds_stick(axes=(0.0,)),
             ds_stick(axes=(0.5,)),
             check_latest_target(True),
             reboot_step,
             ds_stick(axes=(0.5,), buttons=(3,)),
             check_latest_target(False)),
        ]
        parity.assert_same_behaviour(
            CppOperatorNode, PyOperatorNode, self.config, "operator", scenario,
            topics=[(PoseObservation, "/operator/status")])

    # REQ-CONTRACT-14: PositionMechanism (set_goal, set_goal_units, hold, stop,
    # stall fault + clear_fault), VelocityMechanism, and RollerWithSensor match
    # byte-for-byte between C++ and Python.
    def test_mechanisms_parity_including_position_hold(self):
        def set_mode(action):
            def run(rig):
                rig.sim.publish(Goal, "/arm/mode", height_rot=0.0,
                                enabled=True, tick_count=action)
            return run

        def ideal(on):
            return lambda rig: rig.hw.ideal_motors(on)

        pos_scenario = [
            ("disabled", parity.wait(0.02), parity.require_published()),
            ("enable and set_goal",
             parity.gateway(True), ideal(True),
             parity.driver_station(enabled=True, teleop=True, then=0.02),
             set_mode(0),
             parity.send(PositionTarget, "teleop", position_rot=0.25,
                         feedforward_v=0.5, then=0.04)),
            ("hold latches current position even if motor moves",
             set_mode(2),
             parity.send(PositionTarget, "teleop", position_rot=0.90,
                         feedforward_v=0.25, then=0.02),
             ideal(False),
             parity.device("set_motor", "arm", 0.30, 0.0),
             parity.send(PositionTarget, "teleop", position_rot=0.90,
                         feedforward_v=0.25, then=0.02)),
            ("set_goal_units moves to 0.5 rot (180 deg)",
             ideal(True), set_mode(1),
             parity.send(PositionTarget, "teleop", position_rot=0.50,
                         feedforward_v=0.0, then=0.04)),
            ("release stops mechanism",
             parity.release(PositionTarget, "teleop", then=0.02)),
            ("stall fault and clear_fault",
             ideal(False), set_mode(0),
             parity.send(PositionTarget, "teleop", position_rot=0.95,
                         feedforward_v=0.0, then=0.08),
             set_mode(3),
             parity.send(PositionTarget, "teleop", position_rot=0.95,
                         feedforward_v=0.0, then=0.02)),
        ]
        parity.assert_same_behaviour(
            CppPositionNode, PyPositionNode, self.config, "arm", pos_scenario,
            topics=[(PositionState, "/arm/state"),
                    (PoseObservation, "/arm/status")],
            motors=["arm"])

        vel_scenario = [
            ("disabled", parity.wait(0.02), parity.require_published()),
            ("spin up to at_speed",
             parity.gateway(True), ideal(True),
             parity.driver_station(enabled=True, teleop=True, then=0.02),
             parity.send(VelocityTarget, "teleop", velocity_rps=60.0,
                         feedforward_v=1.5, then=0.05)),
            ("change setpoint",
             parity.send(VelocityTarget, "teleop", velocity_rps=40.0,
                         feedforward_v=0.5, then=0.04)),
            ("release stops flywheel",
             parity.release(VelocityTarget, "teleop", then=0.02)),
            ("disable while targeted",
             parity.gateway(False),
             parity.send(VelocityTarget, "teleop", velocity_rps=50.0,
                         feedforward_v=0.0, then=0.02)),
        ]
        parity.assert_same_behaviour(
            CppVelocityNode, PyVelocityNode, self.config, "shooter",
            vel_scenario,
            topics=[(VelocityState, "/shooter/state"),
                    (PoseObservation, "/shooter/status")],
            motors=["flywheel"])

        def roller_cmd(cmd, then=0.02):
            return parity.send(Goal, "teleop", height_rot=0.0, enabled=True,
                               tick_count=int(cmd), then=then)

        roller_scenario = [
            ("disabled", parity.wait(0.02), parity.require_published()),
            ("intake until beam sees piece",
             parity.gateway(True),
             parity.driver_station(enabled=True, teleop=True, then=0.02),
             roller_cmd(RollerCommand.INTAKE, then=0.02),
             parity.device("set_digital_input", "piece_beam", True, then=0.02)),
            ("hold piece then eject through timer",
             roller_cmd(RollerCommand.INTAKE, then=0.02),
             roller_cmd(RollerCommand.EJECT, then=0.06)),
            ("intake piece again then feed",
             parity.device("set_digital_input", "piece_beam", False),
             roller_cmd(RollerCommand.INTAKE, then=0.02),
             parity.device("set_digital_input", "piece_beam", True, then=0.02),
             roller_cmd(RollerCommand.FEED, then=0.06)),
            ("disconnected sensor neutralizes intake",
             parity.device("set_connected", "piece_beam", False),
             roller_cmd(RollerCommand.INTAKE, then=0.02)),
            ("release stops roller",
             parity.release(Goal, "teleop", then=0.02)),
        ]
        parity.assert_same_behaviour(
            CppRollerNode, PyRollerNode, self.config, "intake",
            roller_scenario,
            topics=[(Status, "/intake/status")],
            motors=["roller"])

    # REQ-CONTRACT-14: Sim.set_session_dir / Node.session_dir matches between
    # C++ and Python both when set to a path and when cleared to None.
    def test_session_dir_parity(self):
        for path in ("/tmp/talos_differential_parity_session", None):
            with self.subTest(session_dir=path):
                sims = {}
                for label, cls in (("C++", CppDevicesRig),
                                   ("Python", PyDevicesRig)):
                    sim = Sim(self.config)
                    sim.set_session_dir(path)
                    sim.add(cls, "rig")
                    hw = SimHardware(sim)
                    hw.enable()
                    sim.step(0.02)
                    sims[label] = sim.latest(Status, "/rig/telemetry")
                self.assertIsNotNone(sims["C++"])
                self.assertEqual(sims["C++"], sims["Python"])
                self.assertEqual(sims["C++"].at_goal, path is not None)
                if path is not None:
                    self.assertEqual(sims["C++"].tick_count, _fnv1a64(path))


if __name__ == "__main__":
    unittest.main()
