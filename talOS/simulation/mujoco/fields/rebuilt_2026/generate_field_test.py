"""REQ-DEV-42: the generator writes the same bytes every time.

Two generations of the MJCF model and of the glTF binary, in one process,
from the same tag file, are byte-identical: no timestamp, no unordered
iteration, no float formatting that varies between runs.
"""

import os
import sys
import unittest

from python.runfiles import runfiles

import generate_field


class GenerateFieldTest(unittest.TestCase):

    def test_generations_are_byte_identical(self):
        tags = runfiles.Create().Rlocation(os.environ["TAGS_JSON"])
        self.assertEqual(generate_field.build(tags),
                         generate_field.build(tags))
        first = generate_field.build_glb(tags)
        self.assertEqual(first, generate_field.build_glb(tags))
        self.assertEqual(first[:4], b"glTF")


if __name__ == "__main__":
    sys.exit(unittest.main())
