"""A topics-only talos_messages target: <prefix>.topics, and the dep's
Python modules through the same label."""

import unittest

import talos
from deps_geometry.Pose2d import Pose2d
from deps_topics_only import topics


class TopicsOnlyMessagesTest(unittest.TestCase):
    def test_topics_and_dep_types_come_through_one_label(self):
        self.assertEqual(topics.STATE, "/deps_topics_only/state")
        mtype = talos.message_type(Pose2d)
        self.assertEqual(mtype.fields, ("x_m", "y_m", "heading_rad"))


if __name__ == "__main__":
    unittest.main()
