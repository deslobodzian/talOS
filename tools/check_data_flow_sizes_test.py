"""Tests of tools/check_data_flow_sizes.py, on fixtures and on the real doc.

RULES.md 9: a docs/DATA_FLOW.md number that differs from the size the code
defines, a marker naming no size, and a size no marker states each fail; the
real doc agrees with //tools:data_flow_sizes. No requirement row covers this
lint yet, so the tests carry no REQ tag.
"""

import os
import subprocess
import unittest

from tools import check_data_flow_sizes

SIZES = "frame.max_payload_bytes 1432\nconfig.max_serialized_bytes 10227\n"


def doc(payload="1432", serialized="10,227"):
    return ("Payload maximum is <!-- size: frame.max_payload_bytes -->"
            f"**{payload} B**.\n"
            "| Config | <!-- size: config.max_serialized_bytes -->"
            f"{serialized} |\n")


class CheckDataFlowSizesTest(unittest.TestCase):
    # REQ-DEV-35, RULES.md 9
    def test_matching_doc_passes(self):
        sizes = check_data_flow_sizes.parse_sizes(SIZES)
        self.assertEqual(check_data_flow_sizes.check(sizes, doc()), [])

    # REQ-DEV-35, RULES.md 9
    def test_differing_missing_and_unknown_sizes_fail(self):
        sizes = check_data_flow_sizes.parse_sizes(SIZES + "state.bytes 1419\n")
        text = doc(payload="1400") + "<!-- size: gone.bytes -->12\n"
        self.assertEqual(
            check_data_flow_sizes.check(sizes, text),
            ["docs/DATA_FLOW.md:1: frame.max_payload_bytes is 1400, "
             "the code's is 1432",
             "docs/DATA_FLOW.md:3: gone.bytes is no size the code prints",
             "docs/DATA_FLOW.md: no marker states state.bytes (1419)"])

    # REQ-DEV-35, RULES.md 9
    def test_marker_without_a_number_fails(self):
        sizes = check_data_flow_sizes.parse_sizes(SIZES)
        text = doc() + "<!-- size: frame.max_payload_bytes --> B\n"
        self.assertEqual(
            check_data_flow_sizes.check(sizes, text),
            ["docs/DATA_FLOW.md:3: frame.max_payload_bytes's marker has no "
             "number"])

    # REQ-DEV-35, RULES.md 9: the real doc against the compiled sizes.
    def test_data_flow_doc_states_the_code_sizes(self):
        sizes_text = subprocess.run(
            [os.environ["DATA_FLOW_SIZES"]], check=True, capture_output=True,
            text=True).stdout
        # A Bazel test's sources are links into the source tree.
        root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        with open(os.path.join(root, check_data_flow_sizes.DOC),
                  encoding="utf-8") as f:
            doc_text = f.read()
        self.assertEqual(
            check_data_flow_sizes.check(
                check_data_flow_sizes.parse_sizes(sizes_text), doc_text), [])

    # REQ-DEV-35, RULES.md 9
    def test_empty_sizes_duplicates_malformed_markers_and_decimals_fail_closed(
            self):
        # Empty sizes output must never pass silently.
        self.assertTrue(check_data_flow_sizes.check({}, ""))

        # Duplicate size names in the sizes output fail closed.
        with self.assertRaises(ValueError):
            check_data_flow_sizes.parse_sizes(
                "frame.max_payload_bytes 1432\n"
                "frame.max_payload_bytes 9999\n")

        sizes = check_data_flow_sizes.parse_sizes(SIZES)
        # Malformed marker names (uppercase, hyphens, extra spaces) must fail
        # instead of being silently ignored.
        malformed = doc() + "<!-- size: Frame.Max-Payload --> 999\n"
        problems = check_data_flow_sizes.check(sizes, malformed)
        self.assertEqual(len(problems), 1, problems)
        self.assertIn("Frame.Max-Payload", problems[0])

        # A fractional value like 1432.5 must not match integer size 1432.
        decimal = doc(payload="1432.5")
        problems = check_data_flow_sizes.check(sizes, decimal)
        self.assertEqual(len(problems), 1, problems)
        self.assertIn("frame.max_payload_bytes", problems[0])


if __name__ == "__main__":
    unittest.main()
