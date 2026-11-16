"""shm_name against ShmObjectName, the C++ mapping every publisher,
subscriber, launcher and registry uses (REQ-CONTRACT-22): a Python tool that
computes another name unlinks nothing, or the wrong segment."""

import subprocess
import unittest

from python.runfiles import runfiles
from talOS.memory.shm_name import shm_name


class ShmNameTest(unittest.TestCase):
    # REQ-CONTRACT-22
    def test_matches_shm_object_name(self):
        probe = runfiles.Create().Rlocation("_main/talOS/memory/shm_name_probe")
        for namespace, topic in (
                ("/tmp/bazel/_tmp/abc", "/talos_registry.6"),
                ("session-1", "/drivetrain/target/teleop"),
                ("session-1", "//hw/request/lift"),
                ("session-1", "hw/state"),
                ("tést", "/hw/state/driver_station")):
            with self.subTest(namespace=namespace, topic=topic):
                expected = subprocess.run([probe, namespace, topic],
                                          capture_output=True, text=True,
                                          check=True).stdout.strip()
                self.assertRegex(expected, r"^/~[0-9a-f]{16}$")
                self.assertEqual(shm_name(topic, namespace), expected)


if __name__ == "__main__":
    unittest.main()
