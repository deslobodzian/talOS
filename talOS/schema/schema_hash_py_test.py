"""talos_schema.struct_hash (Python) against the hashes the C++ generator
(talos::schema::struct_hash) wrote into each struct class, and over
separately compiled variants of one schema (testdata/variants).
"""

import os
import re
import subprocess
import tempfile
import unittest

import flatbuffers
import talos_schema
from deps_consumer.ConsumerState import ConsumerState
from deps_geometry.Pose2d import Pose2d
from python.runfiles import runfiles
from reflection import BaseType as ReflectionBaseType
from reflection import Field as ReflectionField
from reflection import Object as ReflectionObject
from reflection import Schema as ReflectionSchema
from reflection import Type as ReflectionType
from schema_test.Arrays import Arrays
from schema_test.Inner import Inner
from schema_test.Mixed import Mixed

_RUNFILES = runfiles.Create()
_DESCRIPTOR_HASH_RE = re.compile(
    r'kName = "([^"]+)";\s*static constexpr std::uint64_t kHash = '
    r'(0x[0-9a-fA-F]+)ULL;')


def _variant_path(name):
    env = "TALOS_VARIANT_" + name.upper()
    return _RUNFILES.Rlocation(os.environ[env])


def _variant(name):
    with open(_variant_path(name), "rb") as f:
        return f.read()


def _cpp_hashes_from_bfbs(bfbs_path, decl_file):
    """Runs descriptor_gen cc (which calls talos::schema::struct_hash in C++)
    and returns {struct_name: hash} for every struct declared in decl_file."""
    gen = _RUNFILES.Rlocation(os.environ["TALOS_DESCRIPTOR_GEN"])
    with tempfile.TemporaryDirectory() as tmp:
        out_h = os.path.join(tmp, "out.h")
        result = subprocess.run(
            [gen, "cc", bfbs_path, decl_file, out_h, "//talOS/schema:variants"],
            capture_output=True, text=True, check=False)
        if result.returncode != 0:
            raise AssertionError(
                f"descriptor_gen failed on {bfbs_path}: {result.stderr}")
        with open(out_h, "r", encoding="utf-8") as f:
            text = f.read()
    return {m.group(1): int(m.group(2), 16)
            for m in _DESCRIPTOR_HASH_RE.finditer(text)}


def _cpp_variant_hashes(variant):
    return _cpp_hashes_from_bfbs(
        _variant_path(variant), f"//variants/{variant}.fbs")


def _synthetic_bytesize_schema(bytesize, minalign=8, decl_file="//synthetic/s.fbs"):
    """Valid reflection.Schema with one Double field at offset 0 and custom
    bytesize, isolating bytesize in canonical_struct."""
    b = flatbuffers.Builder(256)
    name_off = b.CreateString("synthetic.Padded")
    decl_off = b.CreateString(decl_file)
    field_name_off = b.CreateString("field")

    ReflectionType.TypeStart(b)
    ReflectionType.TypeAddBaseType(b, ReflectionBaseType.BaseType.Double)
    ReflectionType.TypeAddIndex(b, -1)
    type_off = ReflectionType.TypeEnd(b)

    ReflectionField.FieldStart(b)
    ReflectionField.FieldAddName(b, field_name_off)
    ReflectionField.FieldAddType(b, type_off)
    ReflectionField.FieldAddOffset(b, 0)
    field_off = ReflectionField.FieldEnd(b)

    ReflectionObject.ObjectStartFieldsVector(b, 1)
    b.PrependUOffsetTRelative(field_off)
    fields_off = b.EndVector()

    ReflectionObject.ObjectStart(b)
    ReflectionObject.ObjectAddName(b, name_off)
    ReflectionObject.ObjectAddFields(b, fields_off)
    ReflectionObject.ObjectAddIsStruct(b, True)
    ReflectionObject.ObjectAddMinalign(b, minalign)
    ReflectionObject.ObjectAddBytesize(b, bytesize)
    ReflectionObject.ObjectAddDeclarationFile(b, decl_off)
    obj_off = ReflectionObject.ObjectEnd(b)

    ReflectionSchema.SchemaStartObjectsVector(b, 1)
    b.PrependUOffsetTRelative(obj_off)
    objs_off = b.EndVector()

    ReflectionSchema.SchemaStartEnumsVector(b, 0)
    enums_off = b.EndVector()

    ReflectionSchema.SchemaStart(b)
    ReflectionSchema.SchemaAddObjects(b, objs_off)
    ReflectionSchema.SchemaAddEnums(b, enums_off)
    schema_off = ReflectionSchema.SchemaEnd(b)
    b.Finish(schema_off, file_identifier=b"BFBS")
    return bytes(b.Output())


class ParityTest(unittest.TestCase):
    # REQ-CONTRACT-07
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


class VariantTest(unittest.TestCase):
    NAMES = ("schema_variant.Reading", "schema_variant.Holder")

    # REQ-CONTRACT-07
    def test_comments_and_formatting_do_not_change_it(self):
        base = _variant("float_level")
        reformatted = _variant("float_level_reformatted")
        base_cpp = _cpp_variant_hashes("float_level")
        reformatted_cpp = _cpp_variant_hashes("float_level_reformatted")
        for name in self.NAMES:
            py_base = talos_schema.struct_hash(base, name)
            py_reformatted = talos_schema.struct_hash(reformatted, name)
            self.assertEqual(py_base, py_reformatted)
            self.assertEqual(py_base, base_cpp[name])
            self.assertEqual(py_reformatted, reformatted_cpp[name])

    # REQ-CONTRACT-07
    def test_same_size_changes_change_it_and_those_of_structs_holding_it(self):
        base = _variant("float_level")
        base_cpp = _cpp_variant_hashes("float_level")
        for variant in (
            "int_level",
            "renamed_level",
            "swapped_fields",
            "aligned_reading",
            "padded_reading",
            "signed_kind",
            "renamed_enum",
            "renamed_enum_value",
            "renumbered_kind",
        ):
            changed = _variant(variant)
            changed_cpp = _cpp_variant_hashes(variant)
            for name in self.NAMES:
                with self.subTest(variant=variant, name=name):
                    py_base = talos_schema.struct_hash(base, name)
                    py_changed = talos_schema.struct_hash(changed, name)
                    self.assertIsNotNone(py_changed)
                    self.assertEqual(py_base, base_cpp[name])
                    self.assertEqual(py_changed, changed_cpp[name])
                    self.assertNotEqual(py_base, py_changed)

    # REQ-CONTRACT-07
    def test_array_length_struct_name_and_bytesize_changes_match_cpp(self):
        arr2 = _variant("array_len_2")
        arr3 = _variant("array_len_3")
        cpp2 = _cpp_variant_hashes("array_len_2")
        cpp3 = _cpp_variant_hashes("array_len_3")
        for name in ("schema_variant.Series", "schema_variant.SeriesHolder"):
            with self.subTest(name=name):
                h2 = talos_schema.struct_hash(arr2, name)
                h3 = talos_schema.struct_hash(arr3, name)
                self.assertEqual(h2, cpp2[name])
                self.assertEqual(h3, cpp3[name])
                self.assertNotEqual(h2, h3)

        base = _variant("float_level")
        renamed_struct = _variant("renamed_struct")
        cpp_base = _cpp_variant_hashes("float_level")
        cpp_renamed = _cpp_variant_hashes("renamed_struct")
        h_base = talos_schema.struct_hash(base, "schema_variant.Holder")
        h_renamed = talos_schema.struct_hash(
            renamed_struct, "schema_variant.Holder")
        self.assertEqual(h_base, cpp_base["schema_variant.Holder"])
        self.assertEqual(h_renamed, cpp_renamed["schema_variant.Holder"])
        self.assertNotEqual(h_base, h_renamed)

        # Isolated bytesize-only change (8 -> 16) with identical minalign (8)
        # and field layout, verified against C++ struct_hash.
        decl = "//synthetic/s.fbs"
        s8 = _synthetic_bytesize_schema(8, minalign=8, decl_file=decl)
        s16 = _synthetic_bytesize_schema(16, minalign=8, decl_file=decl)
        with tempfile.TemporaryDirectory() as tmp:
            p8 = os.path.join(tmp, "s8.bfbs")
            p16 = os.path.join(tmp, "s16.bfbs")
            with open(p8, "wb") as f:
                f.write(s8)
            with open(p16, "wb") as f:
                f.write(s16)
            cpp_s8 = _cpp_hashes_from_bfbs(p8, decl)["synthetic.Padded"]
            cpp_s16 = _cpp_hashes_from_bfbs(p16, decl)["synthetic.Padded"]
        py_s8 = talos_schema.struct_hash(s8, "synthetic.Padded")
        py_s16 = talos_schema.struct_hash(s16, "synthetic.Padded")
        self.assertEqual(py_s8, cpp_s8)
        self.assertEqual(py_s16, cpp_s16)
        self.assertNotEqual(py_s8, py_s16)

    def test_rejects_what_is_not_a_struct_in_a_schema(self):
        base = _variant("float_level")
        self.assertIsNone(talos_schema.struct_hash(base, "schema_variant.Missing"))
        self.assertIsNone(talos_schema.struct_hash(base, "schema_variant.Kind"))
        self.assertIsNone(talos_schema.struct_hash(b"\xa5" * 64,
                                                   "schema_variant.Reading"))


if __name__ == "__main__":
    unittest.main()
