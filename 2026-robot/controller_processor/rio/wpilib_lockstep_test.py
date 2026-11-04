"""One WPILib release everywhere: GradleRIO (RoboRIO), the C++ Maven pin
(third_party/wpilib/extensions.bzl) and the robotpy native wheels
(talOS/python/requirements.txt). The native wheel and the Maven artifact ship
the same libwpimath, and a process that loads both shares one copy by install
name, so a version skew is an ABI hazard."""

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
    def test_gradlerio_maven_and_robotpy_pin_one_release(self):
        gradle = _one(r'id "edu\.wpi\.first\.GradleRIO" version "([^"]+)"',
                      _read("_main/2026-robot/controller_processor/rio/build.gradle"),
                      "GradleRIO plugin version")
        maven = _one(r'^_VERSION = "([^"]+)"',
                     _read("_main/third_party/wpilib/extensions.bzl"),
                     "_VERSION")
        requirements = _read("_main/talOS/python/requirements.txt")
        native = {
            name: _one(rf"^{name}==(\S+)", requirements, name)
            for name in ("robotpy-native-wpimath", "robotpy-native-wpiutil")
        }
        # robotpy-wpimath post-releases (2026.2.1.1) bind the same native
        # release; the wrapper must start with it.
        wrappers = {
            name: _one(rf"^{name}==(\S+)", requirements, name)
            for name in ("robotpy-wpimath", "robotpy-wpiutil")
        }
        self.assertEqual(maven, gradle, "extensions.bzl _VERSION vs GradleRIO")
        for name, version in native.items():
            self.assertEqual(version, gradle, f"{name} vs GradleRIO")
        for name, version in wrappers.items():
            self.assertTrue(version == gradle or version.startswith(gradle + "."),
                            f"{name}=={version} does not bind WPILib {gradle}")


if __name__ == "__main__":
    unittest.main()
