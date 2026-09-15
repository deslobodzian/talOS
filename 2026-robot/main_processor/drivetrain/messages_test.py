"""Test Python drivetrain message codecs and FlatBuffers imports."""
import unittest

from talos.drive import ChassisTarget, DrivetrainState
from talos_nodes.drivetrain import packet


class DrivetrainMessagesTest(unittest.TestCase):
    def test_chassis_target_roundtrip(self):
        wire = packet.encode_chassis_target(1.5, -0.5, 3.14, 123456789, True)
        self.assertEqual(len(wire), packet.CHASSIS_TARGET_SIZE)
        vx, vy, omega, issued_ns, enabled = packet.decode_chassis_target(wire)
        self.assertAlmostEqual(vx, 1.5)
        self.assertAlmostEqual(vy, -0.5)
        self.assertAlmostEqual(omega, 3.14)
        self.assertEqual(issued_ns, 123456789)
        self.assertTrue(enabled)

    def test_drivetrain_state_roundtrip(self):
        wire = packet.encode_drivetrain_state(987654321, True, 2.0, -1.0, 0.5, 1.25, -0.2)
        self.assertEqual(len(wire), packet.DRIVETRAIN_STATE_SIZE)
        ts, en, vx, vy, om, yaw, yaw_rate = packet.decode_drivetrain_state(wire)
        self.assertEqual(ts, 987654321)
        self.assertTrue(en)
        self.assertAlmostEqual(vx, 2.0)
        self.assertAlmostEqual(vy, -1.0)
        self.assertAlmostEqual(om, 0.5)
        self.assertAlmostEqual(yaw, 1.25)
        self.assertAlmostEqual(yaw_rate, -0.2)

    def test_describe_sources_structure(self):
        sources = packet.describe_sources(1, 2, 4)
        self.assertEqual(len(sources), 5)
        # Order: 2 watchers, 2 senders, 1 timer "swerve"
        self.assertEqual(sources[0][0], 2)  # watcher
        self.assertEqual(sources[0][1], packet.HW_STATE_TOPIC)
        self.assertEqual(sources[1][0], 2)  # watcher
        self.assertEqual(sources[1][1], packet.TARGET_TOPIC)
        self.assertEqual(sources[2][0], 4)  # sender
        self.assertEqual(sources[2][1], packet.REQUEST_TOPIC)
        self.assertEqual(sources[3][0], 4)  # sender
        self.assertEqual(sources[3][1], packet.STATE_TOPIC)
        self.assertEqual(sources[4][0], 1)  # timer
        self.assertEqual(sources[4][1], "swerve")


if __name__ == "__main__":
    unittest.main()
