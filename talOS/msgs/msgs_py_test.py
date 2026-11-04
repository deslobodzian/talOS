"""Tests for //talOS/msgs Python bindings over talos.testing.Sim."""

import unittest

import talos
from control.PositionState import PositionState
from control.PositionTarget import PositionTarget
from geometry.OdometrySample import OdometrySample
from talos.testing import Sim, SimHardware

ROBOT_TOML = """
[robot]
period_us = 10000
command_timeout_us = 100000

[subsystems.wrist.motors.m]
can_id = 1
"""


class WristNode:
    def __init__(self, node):
        self.target = talos.Target(node, PositionTarget)
        self.state = node.publish(PositionState, "/wrist/state")
        self.odom = node.publish(OdometrySample, "/wrist/telemetry")

    def update(self):
        t = self.target.get()
        active = t is not None
        goal = t.position_rot if active else 0.0
        self.state.send(position_rot=goal, velocity_rps=0.0, goal_rot=goal,
                        enabled=active, at_goal=active, connected=True)
        self.odom.send(
            sample_time_ns=42,
            pose_x_m=1.25,
            pose_y_m=-0.75,
            pose_yaw_rad=0.5,
            velocity_vx_mps=2.0,
            velocity_vy_mps=-1.0,
            velocity_omega_radps=0.25,
        )


class StandardMessagesTest(unittest.TestCase):
    def test_control_and_nested_geometry_structs_round_trip(self):
        sim = Sim.from_toml(ROBOT_TOML)
        sim.add(WristNode, "wrist")
        hw = SimHardware(sim)
        hw.enable()
        hw.set_driver_station(enabled=True, teleop=True)
        sim.step(0.01)
        sim.publish_target(PositionTarget, "wrist", "teleop",
                           position_rot=0.625, feedforward_v=0.2)
        sim.step(0.02)

        state = sim.latest(PositionState, "/wrist/state")
        self.assertIsNotNone(state)
        self.assertTrue(state.enabled)
        self.assertTrue(state.at_goal)
        self.assertAlmostEqual(state.position_rot, 0.625)
        self.assertAlmostEqual(state.goal_rot, 0.625)

        sample = sim.latest(OdometrySample, "/wrist/telemetry")
        self.assertIsNotNone(sample)
        self.assertEqual(sample.sample_time_ns, 42)
        self.assertAlmostEqual(sample.pose_x_m, 1.25)
        self.assertAlmostEqual(sample.pose_y_m, -0.75)
        self.assertAlmostEqual(sample.pose_yaw_rad, 0.5)
        self.assertAlmostEqual(sample.velocity_vx_mps, 2.0)
        self.assertAlmostEqual(sample.velocity_vy_mps, -1.0)
        self.assertAlmostEqual(sample.velocity_omega_radps, 0.25)


if __name__ == "__main__":
    unittest.main()
