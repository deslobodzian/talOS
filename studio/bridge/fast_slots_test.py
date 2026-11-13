"""The Python harness's fast-value order against the bridge's golden files."""
import json
import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from integration_test import fast_slots


class FastSlotsTest(unittest.TestCase):
    # REQ-OPS-23: integration_test and client_timeout_test read the bridge's
    # live updates through fast_slots. Applied to the golden graph
    # (//studio/bridge:system_test holds the bridge to it byte for byte), the
    # golden update must give the golden later graph, every field of it; a
    # slot out of the bridge's order writes some value into the wrong field.
    def test_update_applied_in_slot_order_gives_the_later_graph(self):
        golden = (Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
                  / "studio/bridge/testdata/golden")
        graph = json.loads((golden / "graph.json").read_text())
        update = json.loads((golden / "update.json").read_text())
        later = json.loads((golden / "later.json").read_text())
        self.assertEqual(update["kind"], "talos.system_graph.update")
        slots = fast_slots(graph)
        self.assertEqual(len(slots), len(update["values"]))
        for (owner, key), value in zip(slots, update["values"]):
            owner[key] = value
        self.assertEqual(graph, later)


if __name__ == "__main__":
    unittest.main()
