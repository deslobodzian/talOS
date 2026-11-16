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

[subsystems.roller.teleop]
defaults = { feedforward_v = 0.0 }

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
    def test_pov_and_toggle_edges_and_safety_reset(self):
        for trigger in ("button = 1", "pov = 90\npov_index = 1"):
            for action in ("while_held", "on_toggle"):
                with self.subTest(trigger=trigger, action=action):
                    config = ROBOT_TOML.replace(
                        "button = 1\nwhile_held", trigger + "\n" + action)
                    sim = Sim.from_toml(config)
                    roller = sim.add(Roller, "roller")
                    sim.add(Teleop, "teleop")
                    hw = SimHardware(sim)
                    hw.enable()

                    def packet(pressed=False, enabled=True, teleop=True,
                               connected=True):
                        hw.set_driver_station(
                            enabled=enabled, teleop=teleop,
                            joysticks=[Stick(
                                buttons=(1,) if pressed else (),
                                povs=(-1, 90 if pressed else -1),
                                button_count=1)] if connected else [])
                        sim.step(0.06)

                    packet()
                    self.assertIsNone(roller.seen)
                    packet(True)
                    self.assertEqual(roller.seen.velocity_rps, 60.0)
                    packet(True)
                    self.assertIsNotNone(roller.seen)
                    packet()
                    self.assertEqual(roller.seen is not None,
                                     action == "on_toggle")
                    packet(True)
                    self.assertEqual(roller.seen is not None,
                                     action == "while_held")
                    packet()
                    packet(True)
                    for unavailable in ({"enabled": False},
                                        {"teleop": False},
                                        {"connected": False}):
                        packet(**unavailable)
                        self.assertIsNone(roller.seen)
                        packet()
                        self.assertIsNone(roller.seen)
                        packet(True)
                        self.assertIsNotNone(roller.seen)
                    hw.stop_driver_station()
                    sim.step(0.3)
                    self.assertIsNone(roller.seen)
                    packet()
                    self.assertIsNone(roller.seen)

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

        # A button held while disabled is not a fresh request once enabled.
        packet(enabled=False)
        assert_released()
        packet()
        assert_released()
        packet(held=False)
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
        assert_released()
        packet(held=False)
        packet()
        assert_active()
        hw.stop_driver_station()
        sim.step(0.3)
        assert_released()


if __name__ == "__main__":
    unittest.main()
