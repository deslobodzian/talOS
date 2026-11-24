"""REQ-DEV-42: the generator writes the same bytes every time, escapes XML
attributes, and rejects invalid tag inputs.

Two generations of the MJCF model and of the glTF binary, in one process,
from the same (or reordered) tag file, are byte-identical: no timestamp, no
unordered iteration, no float formatting that varies between runs.
"""

import json
import math
import os
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

from python.runfiles import runfiles

import generate_field


def _write_json(directory, name, payload):
    path = os.path.join(directory, name)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f)
    return path


class GenerateFieldTest(unittest.TestCase):

    def test_generations_are_byte_identical(self):
        tags = runfiles.Create().Rlocation(os.environ["TAGS_JSON"])
        xml_first = generate_field.build(tags)
        self.assertEqual(xml_first, generate_field.build(tags))
        root = ET.fromstring(xml_first)
        self.assertEqual(root.tag, "mujoco")
        first = generate_field.build_glb(tags)
        self.assertEqual(first, generate_field.build_glb(tags))
        self.assertEqual(first[:4], b"glTF")

    def test_reordered_tags_produce_identical_xml_and_glb(self):
        tags = runfiles.Create().Rlocation(os.environ["TAGS_JSON"])
        with open(tags, encoding="utf-8") as f:
            data = json.load(f)
        reversed_data = dict(data)
        reversed_data["field-tags"] = list(reversed(data["field-tags"]))
        with tempfile.TemporaryDirectory() as tmp:
            reordered = _write_json(tmp, "reordered.json", reversed_data)
            self.assertEqual(generate_field.build(tags),
                             generate_field.build(reordered))
            self.assertEqual(generate_field.build_glb(tags),
                             generate_field.build_glb(reordered))

    def test_xml_escaping_and_non_finite_guard(self):
        out = []
        item = generate_field.Item(
            "geom", 'bad"name&<1>', "box", (1.0, 2.0, 3.0), size=(0.5, 0.5, 0.5)
        )
        generate_field.write_item(out, item)
        elem = ET.fromstring(out[0])
        self.assertEqual(elem.attrib["name"], 'bad"name&<1>')
        with self.assertRaises(ValueError):
            generate_field.fmt([math.nan])
        with self.assertRaises(ValueError):
            generate_field.fmt([math.inf])

    def test_rejects_invalid_tags_and_normalizes_quaternion(self):
        with tempfile.TemporaryDirectory() as tmp:
            # Zero-norm quaternion is refused instead of dividing by zero.
            zero_q = _write_json(tmp, "zero_q.json", {"field-tags": [{
                "ID": 1,
                "pose": {
                    "translation": {"x": 1.0, "y": 2.0, "z": 1.0},
                    "rotation": {"quaternion": {"W": 0.0, "X": 0.0,
                                                "Y": 0.0, "Z": 0.0}},
                },
            }]})
            with self.assertRaises(ValueError):
                generate_field.build(zero_q)
            with self.assertRaises(ValueError):
                generate_field.build_glb(zero_q)

            # Duplicate tag IDs are refused.
            dup = _write_json(tmp, "dup.json", {"field-tags": [
                {"ID": 7, "pose": {
                    "translation": {"x": 1.0, "y": 1.0, "z": 1.0},
                    "rotation": {"quaternion": {"W": 1.0, "X": 0.0,
                                                "Y": 0.0, "Z": 0.0}},
                }},
                {"ID": 7, "pose": {
                    "translation": {"x": 2.0, "y": 1.0, "z": 1.0},
                    "rotation": {"quaternion": {"W": 1.0, "X": 0.0,
                                                "Y": 0.0, "Z": 0.0}},
                }},
            ]})
            with self.assertRaises(ValueError):
                generate_field.build(dup)

            # Non-finite JSON constant is refused.
            nan_path = os.path.join(tmp, "nan.json")
            with open(nan_path, "w", encoding="utf-8") as f:
                f.write('{"field-tags":[{"ID":1,"pose":{"translation":'
                        '{"x":NaN,"y":0,"z":0},"rotation":{"quaternion":'
                        '{"W":1,"X":0,"Y":0,"Z":0}}}}]}')
            with self.assertRaises(ValueError):
                generate_field.build(nan_path)

            # Non-unit quaternion is normalized so the code offset stays 1.5 mm.
            scaled_q = _write_json(tmp, "scaled_q.json", {"field-tags": [{
                "ID": 1,
                "pose": {
                    "translation": {"x": 1.0, "y": 2.0, "z": 1.0},
                    "rotation": {"quaternion": {"W": 0.0, "X": 0.0,
                                                "Y": 0.0, "Z": 2.0}},
                },
            }]})
            items = generate_field.tags(scaled_q)
            self.assertEqual(len(items), 2)
            # Rotated 180 deg around z: normal is (-1, 0, 0), offset -0.0015 m.
            self.assertAlmostEqual(items[1].pos[0], 1.0 - 0.0015, places=12)
            self.assertAlmostEqual(items[1].pos[1], 2.0, places=12)
            self.assertAlmostEqual(items[1].pos[2], 1.0, places=12)


if __name__ == "__main__":
    sys.exit(unittest.main())
