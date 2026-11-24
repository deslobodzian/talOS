"""Two hand-kept config copies stay equal to what they copy.

robot_physics_sim.toml is robot_py.toml run against the physics simulator.
Config load refuses a nested include, so it repeats robot_py.toml's overrides
on the same include instead of including it, and adds only
[robot] simulation_only. This compares them with only [robot] simulation_only
dropped from robot_physics_sim.toml, so it fails when the include, [studio],
the other [robot] overrides or the [[subsystems]] entries differ.

//studio:drive_browser_test opens Studio's driver dashboard against
studio/tests/browser/robot.toml, which must be the bench's physics
simulation graph plus a [studio] table naming its own dashboard. Config load
refuses a nested include, so that file repeats robot_physics_sim.toml's
overrides on the same include instead of including it. This fails when the
two differ in anything but [studio]: the include, the [robot] overrides, or
the [[subsystems]] entries, with every path resolved from its own file.
"""

import os
import tomllib
import unittest

from python.runfiles import runfiles

PHYSICS = "_main/bench/main_processor/configuration/robot_physics_sim.toml"
PYTHON = "_main/bench/main_processor/configuration/robot_py.toml"
COPIES = ("_main/studio/tests/browser/robot.toml",)


def overrides(path, *, drop_studio=True):
    """The config at `path` as it overlays its include: everything (less
    [studio] when `drop_studio` is true), with `include` and each
    subsystem's `path` and `defaults` resolved from the file's directory."""
    with open(path, "rb") as f:
        config = tomllib.load(f)
    if drop_studio:
        config.pop("studio", None)
    here = os.path.dirname(path)

    def resolve(rel):
        return os.path.normpath(os.path.join(here, rel))

    if "include" in config:
        config["include"] = resolve(config["include"])
    for entry in config.get("subsystems", []):
        for key in ("path", "defaults"):
            if key in entry:
                entry[key] = resolve(entry[key])
    return config


def without_simulation_only(config):
    """`config` less the one override robot_physics_sim.toml adds to
    robot_py.toml: [robot] simulation_only (and [robot] if that empties it)."""
    robot = config.get("robot", {})
    robot.pop("simulation_only", None)
    if not robot:
        config.pop("robot", None)
    return config


class PhysicsSimCopyTest(unittest.TestCase):
    # REQ-CONTRACT-41: as below; here the physics sim graph against the
    # Python bench robot it runs.
    def test_physics_sim_repeats_the_python_bench_overrides(self):
        r = runfiles.Create()
        self.assertEqual(
            without_simulation_only(
                overrides(r.Rlocation(PHYSICS), drop_studio=False)),
            overrides(r.Rlocation(PYTHON), drop_studio=False),
        )

    # REQ-CONTRACT-41: a config derived from another fails a test when it
    # goes stale, as a manifest does; here drive_browser_test's config
    # against the bench's simulated graph it copies.
    def test_copies_repeat_the_physics_sim_overrides(self):
        r = runfiles.Create()
        want = overrides(r.Rlocation(PHYSICS))
        for copy in COPIES:
            with self.subTest(copy):
                self.assertEqual(overrides(r.Rlocation(copy)), want)


if __name__ == "__main__":
    unittest.main()
