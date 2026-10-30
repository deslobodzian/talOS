"""Round-trip of a struct embedded through a talos_messages `deps` edge.

consumer/consumer.fbs includes deps_geometry/geometry.fbs; this checks the
Python modules import across the dep edge, the values survive a build and
read-back through talos.message_type (which maps the flattened nested
fields), and the sizes agree with the C++ test (Pose2d 24, ConsumerState 40).
"""

import unittest

import talos
from deps_consumer import topics
from deps_consumer.ConsumerState import ConsumerState
from deps_geometry.Pose2d import Pose2d



class MessagesDepsTest(unittest.TestCase):
    def test_embedded_struct_round_trip(self):
        mtype = talos.message_type(ConsumerState)
        self.assertEqual(mtype.size, 40)
        self.assertEqual(
            mtype.fields,
            ("pose_x", "pose_y", "pose_heading_rad", "velocity_rps",
             "enabled"))
        data = mtype.encode({"pose_x": 1.0, "pose_y": 2.0,
                             "pose_heading_rad": 0.5, "velocity_rps": 10.0,
                             "enabled": True})
        self.assertEqual(len(data), 40)

        state = mtype.decode(data)
        self.assertEqual(state.pose_x, 1.0)
        self.assertEqual(state.pose_y, 2.0)
        self.assertEqual(state.pose_heading_rad, 0.5)
        self.assertEqual(state.velocity_rps, 10.0)
        self.assertTrue(state.enabled)
        self.assertEqual(
            repr(state),
            "ConsumerState(pose_x=1.0, pose_y=2.0, pose_heading_rad=0.5, "
            "velocity_rps=10.0, enabled=True)")


    def test_layout_matches_cpp(self):
        self.assertEqual(Pose2d.SizeOf(), 24)
        self.assertEqual(ConsumerState.SizeOf(), 40)

    def test_topics(self):
        self.assertEqual(topics.STATE, "/deps_consumer/state")


if __name__ == "__main__":
    unittest.main()
