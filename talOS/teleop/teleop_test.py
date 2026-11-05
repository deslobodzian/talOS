"""Configured button bindings through the exported C++ teleop node."""

import unittest

import talos
from control.VelocityTarget import VelocityTarget
from talos.teleop import Teleop
from talos.testing import Sim, SimHardware, Stick


ROBOT_TOML = """
[robot]
period_us = 20000
command_timeout_us = 100000

[subsystems.roller]

[[subsystems.roller.teleop.bindings]]
button = 1
while_held = { velocity_rps = 60.0 }

[subsystems.teleop]

[subsystems.dummy.motors.unused]
can_id = 1
"""


class Roller:
    def __init__(self, node):
        self.target = talos.Target(node, VelocityTarget)
        self.seen = None

    def update(self):
        self.seen = self.target.get()


class TeleopTest(unittest.TestCase):
    def test_button_binding_releases_when_unavailable(self):
        sim = Sim.from_toml(ROBOT_TOML)
        roller = sim.add(Roller, "roller")
        # Graph readers must follow the consumers whose schemas they discover.
        sim.add(Teleop, "teleop")
        hw = SimHardware(sim)
        hw.enable()

        def packet(*, held=True, enabled=True):
            hw.set_driver_station(
                enabled=enabled, teleop=True,
                joysticks=[Stick(buttons=(1,) if held else ())])
            sim.step(0.06)

        def assert_released():
            self.assertIsNone(sim.latest_target(
                VelocityTarget, "roller", "teleop"))
            self.assertIsNone(roller.seen)

        def assert_active():
            lane = sim.latest_target(VelocityTarget, "roller", "teleop")
            self.assertIsNotNone(lane)
            self.assertEqual(lane.velocity_rps, 60.0)
            self.assertIsNotNone(roller.seen)
            self.assertEqual(roller.seen.velocity_rps, 60.0)

        packet(enabled=False)
        assert_released()
        packet()
        assert_active()
        packet(held=False)
        assert_released()
        packet()
        assert_active()
        packet(enabled=False)
        assert_released()
        packet()
        assert_active()
        hw.stop_driver_station()
        sim.step(0.3)
        assert_released()


if __name__ == "__main__":
    unittest.main()
