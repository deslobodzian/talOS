"""Test verifying the external consumer can import talos_runtime and run native ABI calls."""
import unittest

from py_node import sample_node
from talos_runtime import node_api


class ExternalConsumerPythonTest(unittest.TestCase):
    def test_native_runtime_loaded(self):
        lib_path, version = sample_node.inspect_runtime()
        self.assertIsNotNone(lib_path)
        self.assertGreaterEqual(version, node_api.TALOS_NODE_ABI_VERSION)

    def test_describe_json(self):
        sources = [
            (node_api.TIMER, "external_timer", 0, False, False),
            (node_api.WATCHER, "/external/topic", 64, False, False),
        ]
        desc = node_api.describe_json("external_node", "//py_node:node", sources)
        self.assertIn('"name": "external_node"', desc)
        self.assertIn('"target": "//py_node:node"', desc)
        self.assertIn('"external_timer"', desc)
        self.assertIn('"/external/topic"', desc)


if __name__ == "__main__":
    unittest.main()
