"""The rig's model is the one talOS's rig_shot_test shoots.

//talOS/simulation/mujoco/robots:rig_shot_test is this example's shot
without processes, on its own copy of the model (talOS does not depend on an
example). Each copy is generated from its parameter file, which its
robot_model_sync_test holds; this test holds the two models to the same
bytes, so an edit to one parameter file without the other fails here.
"""

import os
import unittest

from python.runfiles import runfiles


def read(env):
    location = os.environ[env]
    resolver = runfiles.Create()
    path = resolver.Rlocation(location, source_repo="") if resolver else None
    if not path or not os.path.exists(path):
        raise FileNotFoundError(f"missing runfile for {env}: {location}")
    with open(path, "rb") as f:
        return f.read()


class ModelTwinTest(unittest.TestCase):
    # REQ-DEV-41
    def test_models_are_the_same_bytes(self):
        self.assertEqual(
            read("EXAMPLE_MODEL"), read("TWIN_MODEL"),
            "examples/mujoco_rig/main_processor/configuration/mujoco/"
            "robot_params.toml and talOS/simulation/mujoco/robots/testdata/"
            "mujoco_rig/robot_params.toml generate different models: make "
            "their tables the same and regenerate both")


if __name__ == "__main__":
    unittest.main()
