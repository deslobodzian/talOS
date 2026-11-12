"""talos.mechanisms over libtalos_node: marshalling, and the stale and
disabled paths. The behaviour is the C++ classes', proven once in
//talOS/mechanisms (docs/TESTING.md §4); this proves only that Python
reaches it.

REQ-SAFE-36, REQ-SAFE-37, REQ-SAFE-38 (the mechanisms' Python reach).
"""

import unittest

from control.PositionState import PositionState
from control.PositionTarget import PositionTarget
from control.VelocityState import VelocityState
from control.VelocityTarget import VelocityTarget
import talos
from talos._messages import message_type
from talos.mechanisms import (PositionFault, PositionMechanism, PositionPhase,
                              RollerCommand, RollerPhase, RollerWithSensor,
                              VelocityMechanism, VelocityPhase)
from talos.testing import Sim, SimHardware

ROBOT_TOML = """
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.arm.motors.arm]
can_id = 1
feedback = "RemoteCANcoder"
feedback_sensor = "arm_cc"
position_frame = "absolute"
[subsystems.arm.sensors.arm_cc]
type = "CANcoder"
can_id = 2
[subsystems.arm.arm]
tolerance_rot = 0.01
gravity_v = 0.5
units_per_rot = 360.0

[subsystems.shooter.motors.flywheel]
can_id = 3
max_velocity_rps = 80.0
[subsystems.shooter.flywheel]
at_speed_entry_tick_count = 3

[subsystems.intake.motors.roller]
can_id = 4
[subsystems.intake.digital_inputs.beam]
dio = 0
[subsystems.intake.roller]
sensor = "beam"
intake_v = 6.0
hold_v = 1.0
eject_v = -6.0
eject_us = 100000
"""


class Arm:
    def __init__(self, node):
        self.arm = PositionMechanism(node, "arm")
        self.target = talos.Target(node, PositionTarget)
        self.in_units = False
        self.clear = False  # clear_fault() this tick, set by the test
        self.fault_after_clear = None

    def update(self):
        if self.clear:
            self.clear = False
            self.arm.clear_fault()
            self.fault_after_clear = self.arm.fault
        t = self.target.get()
        if t is None:
            self.arm.stop()
        elif self.in_units:
            self.arm.set_goal_units(t.position_rot * 360.0, t.feedforward_v)
        else:
            self.arm.set_goal(t.position_rot, t.feedforward_v)
        self.arm.update()


class Shooter:
    def __init__(self, node):
        self.flywheel = VelocityMechanism(node, "flywheel")
        self.target = talos.Target(node, VelocityTarget)

    def update(self):
        t = self.target.get()
        if t is None:
            self.flywheel.stop()
        else:
            self.flywheel.set_goal(t.velocity_rps, t.feedforward_v)
        self.flywheel.update()


class Intake:
    def __init__(self, node):
        self.roller = RollerWithSensor(node, "roller")
        self.command = None  # set by the test

    def update(self):
        if self.command is None:
            self.roller.stop()
        else:
            self.roller.set_goal(self.command)
        self.roller.update()


class MechanismsTest(unittest.TestCase):
    def setUp(self):
        self.sim = Sim.from_toml(ROBOT_TOML)
        self.arm = self.sim.add(Arm, "arm")
        self.shooter = self.sim.add(Shooter, "shooter")
        self.intake = self.sim.add(Intake, "intake")
        self.hw = SimHardware(self.sim)
        self.hw.enable()
        self.hw.ideal_motors()
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.01)

    def hold(self, msg, subsystem, seconds, **fields):
        for _ in range(round(seconds / 0.005)):
            self.sim.publish_target(msg, subsystem, "teleop", **fields)
            self.sim.step(0.005)

    def test_position_mechanism_requests_and_state(self):
        self.hold(PositionTarget, "arm", 0.05, position_rot=0.25,
                  feedforward_v=0.5)
        applied = self.hw.applied_motor("arm", "arm")
        self.assertEqual(applied.mode, talos.Mode.MOTION_MAGIC)
        self.assertAlmostEqual(applied.demand, 0.25)
        self.assertAlmostEqual(applied.ff_volts, 1.0)  # + gravity_v
        arm = self.arm.arm
        self.assertEqual((arm.phase, arm.fault),
                         (PositionPhase.AT_GOAL, PositionFault.NONE))
        self.assertEqual((arm.homed, arm.at_goal, arm.driving, arm.connected),
                         (True, True, True, True))
        self.assertAlmostEqual(arm.position_rot, 0.25)
        self.assertAlmostEqual(arm.position_units, 90.0)
        self.assertAlmostEqual(arm.goal_rot, 0.25)
        self.assertAlmostEqual(arm.gravity_v, 0.5)
        self.assertAlmostEqual(arm.tolerance_rot, 0.01)
        state = message_type(PositionState).decode
        self.assertEqual(arm.state(), state(message_type(PositionState).encode(
            {"position_rot": arm.position_rot,
             "velocity_rps": arm.velocity_rps, "goal_rot": 0.25,
             "enabled": True, "at_goal": True, "connected": True})))
        self.arm.in_units = True
        self.hold(PositionTarget, "arm", 0.01, position_rot=0.5,
                  feedforward_v=0.0)
        self.assertAlmostEqual(self.hw.applied_motor("arm", "arm").demand,
                               0.5)
        # A stale lane: the node stops, and the mechanism drives nothing.
        self.sim.step(0.11)
        self.assertEqual(self.hw.applied_motor("arm", "arm").mode,
                         talos.Mode.NEUTRAL)
        self.assertEqual(arm.phase, PositionPhase.READY)
        self.assertFalse(arm.driving)

    def test_clear_fault_reaches_the_properties_at_once(self):
        # The arm's motor never moves (no ideal motors): a goal stalls.
        sim = Sim.from_toml(ROBOT_TOML.replace(
            "units_per_rot = 360.0",
            "units_per_rot = 360.0\nstall_time_us = 50000"))
        node = sim.add(Arm, "arm")
        hw = SimHardware(sim)
        hw.enable()
        hw.set_driver_station(enabled=True, teleop=True)
        sim.step(0.01)
        for _ in range(30):
            sim.publish_target(PositionTarget, "arm", "teleop",
                               position_rot=0.25, feedforward_v=0.0)
            sim.step(0.005)
        self.assertEqual((node.arm.phase, node.arm.fault),
                         (PositionPhase.FAULTED, PositionFault.STALL))
        node.clear = True
        sim.step(0.005)
        # Read in update() between clear_fault() and the mechanism's update().
        self.assertEqual(node.fault_after_clear, PositionFault.NONE)

    def test_velocity_mechanism_requests_and_state(self):
        self.hold(VelocityTarget, "shooter", 0.03, velocity_rps=60.0,
                  feedforward_v=1.5)
        applied = self.hw.applied_motor("shooter", "flywheel")
        self.assertEqual((applied.mode, applied.demand, applied.ff_volts),
                         (talos.Mode.VELOCITY, 60.0, 1.5))
        flywheel = self.shooter.flywheel
        self.assertEqual(flywheel.phase, VelocityPhase.AT_SPEED)
        self.assertTrue(flywheel.at_speed and flywheel.driving
                        and flywheel.connected)
        self.assertEqual((flywheel.velocity_rps, flywheel.goal_rps,
                          flywheel.commanded_rps), (60.0, 60.0, 60.0))
        vtype = message_type(VelocityState)
        self.assertEqual(flywheel.state(), vtype.decode(vtype.encode(
            {"velocity_rps": 60.0, "requested_rps": 60.0, "current_a": 0.0,
             "enabled": True, "connected": True})))
        # Disabled: Target.get() is None, and the motor goes neutral.
        self.hw.enable(False)
        self.hold(VelocityTarget, "shooter", 0.02, velocity_rps=60.0,
                  feedforward_v=0.0)
        self.assertEqual(self.hw.applied_motor("shooter", "flywheel").mode,
                         talos.Mode.NEUTRAL)
        self.assertEqual(flywheel.phase, VelocityPhase.STOPPED)
        self.assertFalse(flywheel.at_speed)

    def test_roller_with_sensor_requests_and_state(self):
        roller = self.intake.roller
        self.intake.command = RollerCommand.INTAKE
        self.sim.step(0.01)
        self.assertEqual(roller.phase, RollerPhase.INTAKING)
        self.assertEqual(roller.output_v, 6.0)
        self.assertTrue(roller.driving and roller.sensor_connected)
        self.hw.set_digital_input("intake", "beam", True)
        self.sim.step(0.02)  # the hold request reaches the applied output
        self.assertEqual(roller.phase, RollerPhase.HOLDING)
        self.assertTrue(roller.has_piece)
        self.assertEqual(self.hw.applied_motor("intake", "roller").demand,
                         1.0)
        self.intake.command = None
        self.sim.step(0.01)
        self.assertFalse(roller.driving)
        self.assertEqual(self.hw.applied_motor("intake", "roller").mode,
                         talos.Mode.NEUTRAL)

    def test_errors_reach_python(self):
        # A request outside update() names the mechanism.
        with self.assertRaisesRegex(talos.TalosError,
                                    "PositionMechanism 'arm'"):
            self.arm.arm.clear_fault()
        with self.assertRaisesRegex(talos.TalosError, "unknown roller"):
            talos._abi.ok(talos._abi.lib.talos_roller_with_sensor_set_goal(
                self.intake.roller._node, self.intake.roller._index, 9))
        # An index the node never opened (the shooter has no position
        # mechanism) is an error.
        with self.assertRaisesRegex(talos.TalosError, "no mechanism 0"):
            talos._abi.ok(talos._abi.lib.talos_position_mechanism_stop(
                self.shooter.flywheel._node, 0))
        # A bad parameter fails the open, naming the key.
        sim = Sim.from_toml(ROBOT_TOML.replace("tolerance_rot = 0.01",
                                               "tolerance_rot = -1.0"))
        with self.assertRaisesRegex(talos.TalosError, "tolerance_rot"):
            sim.add(Arm, "arm")


if __name__ == "__main__":
    unittest.main()
