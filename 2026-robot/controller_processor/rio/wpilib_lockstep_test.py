"""Keep GradleRIO's WPILib release aligned with Python wpimath wheels."""

import re
import unittest

from python.runfiles import runfiles


def _read(path):
    location = runfiles.Create().Rlocation(path)
    with open(location, encoding="utf-8") as f:
        return f.read()


def _one(pattern, text, what):
    found = re.findall(pattern, text, re.MULTILINE)
    if len(found) != 1:
        raise AssertionError(f"expected one {what}, found {found}")
    return found[0]


class WpilibLockstepTest(unittest.TestCase):
    def test_gradlerio_and_robotpy_pin_one_release(self):
        gradle = _one(r'id "edu\.wpi\.first\.GradleRIO" version "([^"]+)"',
                      _read("_main/2026-robot/controller_processor/rio/build.gradle"),
                      "GradleRIO plugin version")
        requirements = _read("_main/talOS/python/requirements.txt")
        native = _one(r"^robotpy-native-wpimath==(\S+)", requirements,
                      "robotpy-native-wpimath")
        wrapper = _one(r"^robotpy-wpimath==(\S+)", requirements,
                       "robotpy-wpimath")
        self.assertEqual(native, gradle, "robotpy-native-wpimath vs GradleRIO")
        # Wrapper post-releases bind the same underlying native release.
        self.assertTrue(wrapper == gradle or wrapper.startswith(gradle + "."),
                        f"robotpy-wpimath=={wrapper} does not bind WPILib {gradle}")


if __name__ == "__main__":
    unittest.main()
