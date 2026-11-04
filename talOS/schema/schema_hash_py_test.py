"""talos_schema.struct_hash (Python) against the hashes the C++ generator
(talos::schema::struct_hash) wrote into each struct class, and over
separately compiled variants of one schema (testdata/variants).
"""

import os
import unittest

import talos_schema
from deps_consumer.ConsumerState import ConsumerState
from deps_geometry.Pose2d import Pose2d
from python.runfiles import runfiles
from schema_test.Arrays import Arrays
from schema_test.Inner import Inner
from schema_test.Mixed import Mixed

_RUNFILES = runfiles.Create()


def _variant(name):
    env = "TALOS_VARIANT_" + name.upper()
    with open(_RUNFILES.Rlocation(os.environ[env]), "rb") as f:
        return f.read()


class ParityTest(unittest.TestCase):
    def test_python_hash_equals_the_generated_cpp_hash(self):
        # Inner: an enum field; Mixed: mixed alignment and a nested struct;
        # Arrays: arrays of scalars, enums and structs.
        for cls in (Inner, Mixed, Arrays):
            with self.subTest(cls=cls.__name__):
                self.assertEqual(cls.TALOS_SCHEMA_NAME,
                                 "schema_test." + cls.__name__)
                self.assertEqual(
                    talos_schema.struct_hash(cls.TALOS_SCHEMA_BFBS,
                                             cls.TALOS_SCHEMA_NAME),
                    cls.TALOS_SCHEMA_HASH)

    def test_hash_does_not_depend_on_the_schema_file_holding_it(self):
        # ConsumerState's .bfbs carries the included deps_geometry.Pose2d.
        self.assertEqual(
            talos_schema.struct_hash(ConsumerState.TALOS_SCHEMA_BFBS,
                                     Pose2d.TALOS_SCHEMA_NAME),
            Pose2d.TALOS_SCHEMA_HASH)
        self.assertEqual(
            talos_schema.struct_hash(ConsumerState.TALOS_SCHEMA_BFBS,
                                     ConsumerState.TALOS_SCHEMA_NAME),
            ConsumerState.TALOS_SCHEMA_HASH)

    def test_distinct_structs_have_distinct_hashes(self):
        hashes = {cls.TALOS_SCHEMA_HASH for cls in (Inner, Mixed, Arrays)}
        self.assertEqual(len(hashes), 3)


class VariantTest(unittest.TestCase):
    NAMES = ("schema_variant.Reading", "schema_variant.Holder")

    def test_comments_and_formatting_do_not_change_it(self):
        base = _variant("float_level")
        reformatted = _variant("float_level_reformatted")
        for name in self.NAMES:
            self.assertEqual(talos_schema.struct_hash(base, name),
                             talos_schema.struct_hash(reformatted, name))

    def test_same_size_changes_change_it_and_those_of_structs_holding_it(self):
        base = _variant("float_level")
        for variant in ("int_level", "renamed_level", "renumbered_kind"):
            changed = _variant(variant)
            for name in self.NAMES:
                with self.subTest(variant=variant, name=name):
                    self.assertIsNotNone(talos_schema.struct_hash(changed, name))
                    self.assertNotEqual(talos_schema.struct_hash(base, name),
                                        talos_schema.struct_hash(changed, name))

    def test_rejects_what_is_not_a_struct_in_a_schema(self):
        base = _variant("float_level")
        self.assertIsNone(talos_schema.struct_hash(base, "schema_variant.Missing"))
        self.assertIsNone(talos_schema.struct_hash(base, "schema_variant.Kind"))
        self.assertIsNone(talos_schema.struct_hash(b"\xa5" * 64,
                                                   "schema_variant.Reading"))


if __name__ == "__main__":
    unittest.main()
