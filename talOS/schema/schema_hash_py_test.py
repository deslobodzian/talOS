"""talos_schema.struct_hash (Python) against the hashes the C++ generator
(talos::schema::struct_hash) wrote into each struct class, and over
separately compiled variants of one schema (testdata/variants).
"""

import os
import re
import struct
import subprocess
import tempfile
import unittest

import flatbuffers
import talos_schema
from deps_consumer.ConsumerState import ConsumerState
from deps_geometry.Pose2d import Pose2d
from python.runfiles import runfiles
from reflection import BaseType as ReflectionBaseType
from reflection import Enum as ReflectionEnum
from reflection import EnumVal as ReflectionEnumVal
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


def _single_field_bfbs(base, index=-1, offset=0, element=0, fixed_length=0,
                       bytesize=8, minalign=8, is_struct=True,
                       with_enum=False, omit_type=False, omit_fields=False):
    b = flatbuffers.Builder(256)
    enums_off = None
    if with_enum:
        en_name = b.CreateString("test.Kind")
        ev_name = b.CreateString("A")
        ReflectionEnumVal.EnumValStart(b)
        ReflectionEnumVal.EnumValAddName(b, ev_name)
        ReflectionEnumVal.EnumValAddValue(b, 0)
        ev_off = ReflectionEnumVal.EnumValEnd(b)
        ReflectionEnum.EnumStartValuesVector(b, 1)
        b.PrependUOffsetTRelative(ev_off)
        evs_off = b.EndVector()
        ReflectionType.TypeStart(b)
        ReflectionType.TypeAddBaseType(b, ReflectionBaseType.BaseType.UByte)
        ut_off = ReflectionType.TypeEnd(b)
        ReflectionEnum.EnumStart(b)
        ReflectionEnum.EnumAddName(b, en_name)
        ReflectionEnum.EnumAddValues(b, evs_off)
        ReflectionEnum.EnumAddUnderlyingType(b, ut_off)
        e_off = ReflectionEnum.EnumEnd(b)
        ReflectionSchema.SchemaStartEnumsVector(b, 1)
        b.PrependUOffsetTRelative(e_off)
        enums_off = b.EndVector()
    else:
        ReflectionSchema.SchemaStartEnumsVector(b, 0)
        enums_off = b.EndVector()

    name_off = b.CreateString("bad.Struct")
    fields_off = None
    if not omit_fields:
        field_name_off = b.CreateString("field")
        type_off = None
        if not omit_type:
            ReflectionType.TypeStart(b)
            ReflectionType.TypeAddBaseType(b, base)
            ReflectionType.TypeAddIndex(b, index)
            ReflectionType.TypeAddElement(b, element)
            ReflectionType.TypeAddFixedLength(b, fixed_length)
            type_off = ReflectionType.TypeEnd(b)
        ReflectionField.FieldStart(b)
        ReflectionField.FieldAddName(b, field_name_off)
        if type_off is not None:
            ReflectionField.FieldAddType(b, type_off)
        ReflectionField.FieldAddOffset(b, offset)
        field_off = ReflectionField.FieldEnd(b)
        ReflectionObject.ObjectStartFieldsVector(b, 1)
        b.PrependUOffsetTRelative(field_off)
        fields_off = b.EndVector()

    ReflectionObject.ObjectStart(b)
    ReflectionObject.ObjectAddName(b, name_off)
    if fields_off is not None:
        ReflectionObject.ObjectAddFields(b, fields_off)
    ReflectionObject.ObjectAddIsStruct(b, is_struct)
    ReflectionObject.ObjectAddMinalign(b, minalign)
    ReflectionObject.ObjectAddBytesize(b, bytesize)
    obj_off = ReflectionObject.ObjectEnd(b)

    ReflectionSchema.SchemaStartObjectsVector(b, 1)
    b.PrependUOffsetTRelative(obj_off)
    objs_off = b.EndVector()

    ReflectionSchema.SchemaStart(b)
    ReflectionSchema.SchemaAddObjects(b, objs_off)
    ReflectionSchema.SchemaAddEnums(b, enums_off)
    b.Finish(ReflectionSchema.SchemaEnd(b), file_identifier=b"BFBS")
    return bytes(b.Output())


class HostileSchemaTest(unittest.TestCase):
    # REQ-CONTRACT-07
    def test_truncated_and_corrupted_bfbs_return_none_and_raise_schema_error(self):
        base = _variant("float_level")
        # Every prefix of a valid .bfbs (including right after "BFBS") must fail
        # cleanly without raising struct.error / IndexError / AssertionError.
        hostile = [
            b"",
            b"BFBS",
            b"\x00\x00\x00\x00BFBS",
            b"\x0c\x00\x00\x00BFBS",
            b"\xff\xff\xff\x7fBFBS" + b"\x00" * 16,
            # Root at offset 12, negative soffset making vtable out of bounds.
            struct.pack("<I4sIi", 12, b"BFBS", 0, 1000),
            # Root at offset 12, vtable at offset 16 with huge vector length.
            struct.pack("<I4sIiHHHI", 12, b"BFBS", 0, -4, 6, 8, 4, 0x7fffffff),
        ]
        # Sample prefixes across the entire buffer, plus every prefix <= 32 bytes.
        for cut in list(range(min(33, len(base)))) + list(
                range(33, len(base), max(1, len(base) // 40))):
            hostile.append(base[:cut])

        for idx, buf in enumerate(hostile):
            with self.subTest(case=idx, length=len(buf)):
                self.assertIsNone(
                    talos_schema.canonical_struct(buf, "schema_variant.Reading"))
                self.assertIsNone(
                    talos_schema.struct_hash(buf, "schema_variant.Reading"))
                with self.assertRaises(talos_schema.SchemaError):
                    talos_schema.find_struct(buf, "schema_variant.Reading")

    # REQ-CONTRACT-07
    def test_cyclic_diamond_and_deep_structs_return_none(self):
        # 1. Direct self-cycle (object 0 references object 0).
        self_cycle = _single_field_bfbs(ReflectionBaseType.BaseType.Obj, index=0)
        self.assertIsNone(talos_schema.canonical_struct(self_cycle, "bad.Struct"))
        self.assertIsNone(talos_schema.struct_hash(self_cycle, "bad.Struct"))

        # 2. 22-level diamond DAG (depth < 32, 2^22 - 1 paths > 4096 visits).
        b = flatbuffers.Builder(1024)
        levels = 22
        obj_offs = []
        for i in range(levels):
            name_off = b.CreateString(f"dag.o{i}")
            field_offs = []
            if i + 1 == levels:
                fn = b.CreateString("leaf_m")
                ReflectionType.TypeStart(b)
                ReflectionType.TypeAddBaseType(b, ReflectionBaseType.BaseType.Double)
                ReflectionType.TypeAddIndex(b, -1)
                ft = ReflectionType.TypeEnd(b)
                ReflectionField.FieldStart(b)
                ReflectionField.FieldAddName(b, fn)
                ReflectionField.FieldAddType(b, ft)
                ReflectionField.FieldAddOffset(b, 0)
                field_offs.append(ReflectionField.FieldEnd(b))
            else:
                for k in range(2):
                    fn = b.CreateString("a" if k == 0 else "b")
                    ReflectionType.TypeStart(b)
                    ReflectionType.TypeAddBaseType(b, ReflectionBaseType.BaseType.Obj)
                    ReflectionType.TypeAddIndex(b, i + 1)
                    ft = ReflectionType.TypeEnd(b)
                    ReflectionField.FieldStart(b)
                    ReflectionField.FieldAddName(b, fn)
                    ReflectionField.FieldAddType(b, ft)
                    ReflectionField.FieldAddId(b, k)
                    ReflectionField.FieldAddOffset(b, 0)
                    field_offs.append(ReflectionField.FieldEnd(b))
            ReflectionObject.ObjectStartFieldsVector(b, len(field_offs))
            for fo in reversed(field_offs):
                b.PrependUOffsetTRelative(fo)
            fvec = b.EndVector()
            ReflectionObject.ObjectStart(b)
            ReflectionObject.ObjectAddName(b, name_off)
            ReflectionObject.ObjectAddFields(b, fvec)
            ReflectionObject.ObjectAddIsStruct(b, True)
            ReflectionObject.ObjectAddMinalign(b, 8)
            ReflectionObject.ObjectAddBytesize(b, 8)
            obj_offs.append(ReflectionObject.ObjectEnd(b))
        ReflectionSchema.SchemaStartObjectsVector(b, len(obj_offs))
        for oo in reversed(obj_offs):
            b.PrependUOffsetTRelative(oo)
        ovec = b.EndVector()
        ReflectionSchema.SchemaStartEnumsVector(b, 0)
        evec = b.EndVector()
        ReflectionSchema.SchemaStart(b)
        ReflectionSchema.SchemaAddObjects(b, ovec)
        ReflectionSchema.SchemaAddEnums(b, evec)
        b.Finish(ReflectionSchema.SchemaEnd(b), file_identifier=b"BFBS")
        dag_bfbs = bytes(b.Output())
        self.assertIsNone(talos_schema.canonical_struct(dag_bfbs, "dag.o0"))
        self.assertIsNone(talos_schema.struct_hash(dag_bfbs, "dag.o0"))

        # 3. Linear chain exceeding max depth (35 > 32).
        b = flatbuffers.Builder(1024)
        chain_len = 35
        obj_offs = []
        for i in range(chain_len):
            name_off = b.CreateString(f"chain.o{i}")
            fn = b.CreateString("x")
            ReflectionType.TypeStart(b)
            if i + 1 == chain_len:
                ReflectionType.TypeAddBaseType(b, ReflectionBaseType.BaseType.Double)
                ReflectionType.TypeAddIndex(b, -1)
            else:
                ReflectionType.TypeAddBaseType(b, ReflectionBaseType.BaseType.Obj)
                ReflectionType.TypeAddIndex(b, i + 1)
            ft = ReflectionType.TypeEnd(b)
            ReflectionField.FieldStart(b)
            ReflectionField.FieldAddName(b, fn)
            ReflectionField.FieldAddType(b, ft)
            ReflectionField.FieldAddOffset(b, 0)
            fo = ReflectionField.FieldEnd(b)
            ReflectionObject.ObjectStartFieldsVector(b, 1)
            b.PrependUOffsetTRelative(fo)
            fvec = b.EndVector()
            ReflectionObject.ObjectStart(b)
            ReflectionObject.ObjectAddName(b, name_off)
            ReflectionObject.ObjectAddFields(b, fvec)
            ReflectionObject.ObjectAddIsStruct(b, True)
            ReflectionObject.ObjectAddMinalign(b, 8)
            ReflectionObject.ObjectAddBytesize(b, 8)
            obj_offs.append(ReflectionObject.ObjectEnd(b))
        ReflectionSchema.SchemaStartObjectsVector(b, len(obj_offs))
        for oo in reversed(obj_offs):
            b.PrependUOffsetTRelative(oo)
        ovec = b.EndVector()
        ReflectionSchema.SchemaStartEnumsVector(b, 0)
        evec = b.EndVector()
        ReflectionSchema.SchemaStart(b)
        ReflectionSchema.SchemaAddObjects(b, ovec)
        ReflectionSchema.SchemaAddEnums(b, evec)
        b.Finish(ReflectionSchema.SchemaEnd(b), file_identifier=b"BFBS")
        chain_bfbs = bytes(b.Output())
        self.assertIsNone(talos_schema.canonical_struct(chain_bfbs, "chain.o0"))
        self.assertIsNone(talos_schema.struct_hash(chain_bfbs, "chain.o0"))

    # REQ-CONTRACT-07
    def test_invalid_indices_basetypes_null_fields_and_enum_on_non_int(self):
        bt = ReflectionBaseType.BaseType
        cases = {
            "obj_neg_index": _single_field_bfbs(bt.Obj, index=-1),
            "obj_oob_index": _single_field_bfbs(bt.Obj, index=99),
            "enum_oob_index": _single_field_bfbs(bt.Int, index=0),
            "raw_basetype_neg": _single_field_bfbs(-128),
            "raw_basetype_high": _single_field_bfbs(127),
            "raw_element_high": _single_field_bfbs(bt.Array, element=127,
                                                   fixed_length=1),
            "string_field": _single_field_bfbs(bt.String),
            "zero_length_array": _single_field_bfbs(bt.Array, element=bt.Int,
                                                    fixed_length=0),
            "offset_past_end": _single_field_bfbs(bt.Long, offset=1, bytesize=8),
            "array_past_end": _single_field_bfbs(bt.Array, element=bt.Long,
                                                 fixed_length=2, bytesize=8),
            "zero_bytesize": _single_field_bfbs(bt.Int, bytesize=0),
            "missing_type": _single_field_bfbs(bt.Int, omit_type=True),
            "missing_fields": _single_field_bfbs(bt.Int, omit_fields=True),
            "bool_with_enum": _single_field_bfbs(bt.Bool, index=0,
                                                 with_enum=True),
            "float_with_enum": _single_field_bfbs(bt.Float, index=0,
                                                  with_enum=True),
            "double_with_enum": _single_field_bfbs(bt.Double, index=0,
                                                   with_enum=True),
            "float_array_with_enum": _single_field_bfbs(
                bt.Array, element=bt.Float, index=0, fixed_length=2,
                with_enum=True),
        }
        # Sanity: valid integer enum passes.
        valid_enum = _single_field_bfbs(bt.UByte, index=0, with_enum=True)
        self.assertIsNotNone(talos_schema.struct_hash(valid_enum, "bad.Struct"))

        for label, bfbs in cases.items():
            with self.subTest(label=label):
                self.assertIsNone(talos_schema.canonical_struct(bfbs, "bad.Struct"))
                self.assertIsNone(talos_schema.struct_hash(bfbs, "bad.Struct"))


if __name__ == "__main__":
    unittest.main()
