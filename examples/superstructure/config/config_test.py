"""robot_py.toml is robot.toml with the py/ nodes: the twins run one robot."""

import os
import tomllib
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))


def load(name):
    with open(os.path.join(HERE, name), "rb") as f:
        return tomllib.load(f)


class ConfigTest(unittest.TestCase):
    def test_python_config_differs_only_in_node_targets(self):
        cpp, py = load("robot.toml"), load("robot_py.toml")
        for name, subsystem in py["subsystems"].items():
            with self.subTest(name):
                self.assertEqual(subsystem.pop("node"),
                                 f"//examples/superstructure/py:{name}")
                self.assertEqual(cpp["subsystems"][name].pop("node"),
                                 f"//examples/superstructure/cpp:{name}")
        self.assertEqual(cpp, py)


if __name__ == "__main__":
    unittest.main()
