"""The reflection-driven struct.Struct message codec: byte-for-byte against
flatc C++ (messages_golden) and flatc Python (flatbuffers.Builder)."""

import math
import os
import random
import struct
import subprocess
import sys
import unittest

import flatbuffers
from python.runfiles import runfiles
from reflection import BaseType as ReflectionBaseType
from reflection import Field as ReflectionField
from reflection import Object as ReflectionObject
from reflection import Schema as ReflectionSchema
from reflection import Type as ReflectionType

from control.PositionState import PositionState
from control.PositionTarget import PositionTarget
from control.VelocityState import VelocityState
from control.VelocityTarget import VelocityTarget
from demo.Goal import Goal
from demo.Report import Report
from demo.Status import Status
from geometry.OdometrySample import OdometrySample
from geometry.Pose2d import Pose2d
from geometry.PoseEstimate import PoseEstimate
from geometry.PoseObservation import PoseObservation
from geometry.Rotation2d import Rotation2d
from geometry.SwerveModulePosition import SwerveModulePosition
from geometry.Translation2d import Translation2d
from geometry.Twist2d import Twist2d
from schema_test.Arrays import Arrays
from schema_test.Inner import Inner
from schema_test.Level import Level
from schema_test.Mixed import Mixed
from schema_test.Mode import Mode
from talos._messages import Message, MessageType, message_type

TYPES = (Goal, Status, Report, Translation2d, Rotation2d, Pose2d, Twist2d,
         SwerveModulePosition, OdometrySample, PoseObservation, PoseEstimate,
         VelocityTarget, VelocityState, PositionTarget, PositionState)

_INT_BITS = {"b": (8, True), "B": (8, False), "h": (16, True),
             "H": (16, False), "i": (32, True), "I": (32, False),
             "q": (64, True), "Q": (64, False)}


def zero_fields(cls):
    """Every field of cls at zero: encode takes no omitted field."""
    mtype = message_type(cls)
    out = {}
    pos = 0
    for field in mtype.fields:
        count = mtype._arrays.get(field)
        zero = False if mtype._slot_codes[pos] == "?" else 0
        out[field] = (zero,) * count if count is not None else zero
        pos += count or 1
    return out


# The values messages_golden.cc encodes with flatc C++, by its labels.
GOLDEN_VALUES = {
    "inner": (Inner, {"mode": Mode.HIGH, "offset_m": -1.5}),
    "mixed_zero": (Mixed, zero_fields(Mixed)),
    "mixed_top": (Mixed, {
        "flag": True, "position_m": 1234.5678, "level": Level.TOP,
        "sample_count": 4000000000, "inner_mode": Mode.LOW,
        "inner_offset_m": 0.25, "speed_frac": -0.75}),
    "mixed_bottom": (Mixed, {
        "flag": False, "position_m": -0.5, "level": Level.BOTTOM,
        "sample_count": 7, "inner_mode": Mode.OFF, "inner_offset_m": -8.0,
        "speed_frac": 1.0}),
    "arrays": (Arrays, {
        "samples_m": (1.5, -2.25, 1e9), "modes": (Mode.HIGH, Mode.LOW),
        "inners_0_mode": Mode.LOW, "inners_0_offset_m": 3.5,
        "inners_1_mode": Mode.HIGH, "inners_1_offset_m": -0.125,
        "channel_id": (1, 2, 3, 254, 255)}),
    "arrays_zero": (Arrays, zero_fields(Arrays)),
}


def golden_bytes():
    """{label: flatc C++ bytes} from messages_golden."""
    path = runfiles.Create().Rlocation(os.environ["TALOS_MESSAGES_GOLDEN"])
    out = subprocess.run([path], check=True, capture_output=True, text=True)
    return {label: bytes.fromhex(data)
            for label, data in (line.split() for line in out.stdout.splitlines())}


def random_value(rng, code):
    if code == "?":
        return rng.random() < 0.5
    if code == "d":
        return rng.uniform(-1e6, 1e6)
    if code == "f":  # exactly representable, so it reads back equal
        return struct.unpack("<f", struct.pack("<f", rng.uniform(-1e3, 1e3)))[0]
    bits, signed = _INT_BITS[code]
    low = -(1 << (bits - 1)) if signed else 0
    return rng.randint(low, low + (1 << bits) - 1)


def random_fields(rng, mtype):
    """Random values for a type without arrays (one packed value a field)."""
    return {field: random_value(rng, code)
            for field, code in zip(mtype.fields, mtype._slot_codes)}


def builder_bytes(cls, mtype, fields):
    """What flatc Python's Create<Struct> writes for `fields`, through
    flatbuffers.Builder, independent of talos."""
    create = getattr(sys.modules[cls.__module__], "Create" + cls.__name__)
    builder = flatbuffers.Builder(0)
    create(builder, *(fields.get(f, 0) for f in mtype.fields))
    return bytes(builder.Bytes[builder.Head():])


class CppBytesTest(unittest.TestCase):
    """Nested structs, fixed arrays, enums of two widths and mixed
    alignment, against the bytes flatc C++ structs hold."""

    @classmethod
    def setUpClass(cls):
        cls.golden = golden_bytes()


    def test_encoding_matches_cpp_byte_for_byte(self):
        for label, (cls, fields) in GOLDEN_VALUES.items():
            self.assertEqual(message_type(cls).encode(fields),
                             self.golden[label], label)

    def test_decoding_cpp_bytes_reads_the_values(self):
        for label, (cls, fields) in GOLDEN_VALUES.items():
            mtype = message_type(cls)
            message = mtype.decode(self.golden[label])
            for field in mtype.fields:
                expected = fields.get(field)
                if expected is None:
                    count = mtype._arrays.get(field)
                    expected = 0 if count is None else (0,) * count
                self.assertEqual(getattr(message, field), expected,
                                 (label, field))


class StructCodecTest(unittest.TestCase):
    def test_every_type_has_its_schema_layout(self):
        for cls in TYPES:
            mtype = message_type(cls)
            self.assertEqual(mtype.size, cls.SizeOf(), cls.__name__)
            self.assertEqual(mtype.schema_name, cls.TALOS_SCHEMA_NAME)
            self.assertEqual(mtype.schema_hash, cls.TALOS_SCHEMA_HASH)
            builder = flatbuffers.Builder(0)
            getattr(sys.modules[cls.__module__], "Create" + cls.__name__)(
                builder, *(0 for _ in mtype.fields))
            self.assertEqual(mtype.alignment, builder.minalign, cls.__name__)
        # Nested structs and fixed arrays, as flatc C++ lays them out.
        for cls, size, alignment in ((Inner, 8, 4), (Mixed, 40, 8),
                                     (Arrays, 56, 8)):
            mtype = message_type(cls)
            self.assertEqual((mtype.size, mtype.alignment), (size, alignment),
                             cls.__name__)

    def test_random_values_encode_as_flatc_python_and_decode_back(self):
        rng = random.Random(1234)
        for cls in TYPES:
            mtype = message_type(cls)
            for _ in range(200):
                fields = random_fields(rng, mtype)
                data = mtype.encode(fields)
                self.assertEqual(data, builder_bytes(cls, mtype, fields),
                                 (cls.__name__, fields))
                message = mtype.decode(data)
                for field in mtype.fields:
                    self.assertEqual(getattr(message, field), fields[field],
                                     (cls.__name__, field))
                    self.assertIs(type(getattr(message, field)),
                                  type(fields[field]), (cls.__name__, field))

    def test_nested_struct_fields_flatten(self):
        mtype = message_type(OdometrySample)
        self.assertEqual(mtype.fields, (
            "sample_time_ns", "pose_x_m", "pose_y_m", "pose_yaw_rad",
            "velocity_vx_mps", "velocity_vy_mps", "velocity_omega_radps"))
        data = mtype.encode(dict(zero_fields(OdometrySample),
                                 sample_time_ns=42, pose_x_m=1.25,
                                 velocity_omega_radps=0.25))
        sample = mtype.decode(data)
        self.assertEqual(sample.sample_time_ns, 42)
        self.assertEqual(sample.pose_x_m, 1.25)
        self.assertEqual(sample.pose_y_m, 0.0)
        self.assertEqual(sample.velocity_omega_radps, 0.25)
        raw = OdometrySample()
        raw.Init(data, 0)
        self.assertEqual(raw.Pose(Pose2d()).XM(), 1.25)

    def test_unknown_message_attribute_is_rejected(self):
        goal = message_type(Goal).decode(
            message_type(Goal).encode(zero_fields(Goal)))
        with self.assertRaises(AttributeError):
            _ = goal.height

    def test_bad_fields_and_values_raise_type_error(self):
        goal, arrays, mixed = (message_type(Goal), message_type(Arrays),
                               message_type(Mixed))
        for mtype, fields in (
                (goal, {"height": 1.0}),
                (goal, {"tick_count": 1 << 40}),
                (goal, {"height_rot": "up"}),
                (arrays, {"inners": ()}),
                (arrays, {"samples_m": (1.0, 2.0)}),
                (arrays, {"samples_m": 1.0}),
                (arrays, {"channel_id": (1, 2, 3, 4, 256)}),
                (mixed, {"level": 40000}),
                (mixed, {"speed_frac": 1e300})):
            with self.subTest(fields=fields):
                with self.assertRaises(TypeError):
                    mtype.encode(dict(zero_fields(mtype.cls), **fields))

    def test_omitted_fields_raise_type_error_naming_them(self):
        # C++ constructs a struct from every field; a Python send that left
        # one at zero would actuate a target with it at zero.
        for cls, omitted in ((Goal, ("enabled",)),
                             (Arrays, ("samples_m", "inners_1_mode")),
                             (OdometrySample, ("pose_yaw_rad",))):
            fields = zero_fields(cls)
            for field in omitted:
                del fields[field]
            with self.subTest(cls=cls.__name__):
                with self.assertRaises(TypeError) as raised:
                    message_type(cls).encode(fields)
                for field in omitted:
                    self.assertIn(field, str(raised.exception))

    def test_a_class_without_a_schema_raises_type_error(self):
        class Handwritten:
            @classmethod
            def SizeOf(cls):  # noqa: N802 (flatc's API)
                return 8

        with self.assertRaises(TypeError):
            MessageType(Handwritten)

    # REQ-CONTRACT-14: bool fields accept only bool (never coercing "false",
    # NaN, None, or integers), numeric fields reject bools, and fixed arrays
    # require tuple or list (rejecting unordered sets, dicts, strings, bytes).
    def test_strict_bool_numeric_and_array_type_validation(self):
        goal = message_type(Goal)
        mixed = message_type(Mixed)
        arrays = message_type(Arrays)
        estimate = message_type(PoseEstimate)

        for bad_bool in ("false", "true", "", math.nan, None, 0, 1, 0.0, []):
            with self.subTest(bad_bool=bad_bool):
                with self.assertRaises(TypeError):
                    goal.encode(dict(zero_fields(Goal), enabled=bad_bool))
                with self.assertRaises(TypeError):
                    mixed.encode(dict(zero_fields(Mixed), flag=bad_bool))
                with self.assertRaises(TypeError):
                    estimate.encode(
                        dict(zero_fields(PoseEstimate), valid=bad_bool))

        for mtype, field in (
                (goal, "height_rot"),
                (goal, "tick_count"),
                (mixed, "level"),
                (mixed, "sample_count"),
                (mixed, "speed_frac")):
            for bad_num in (True, False):
                with self.subTest(field=field, bad_num=bad_num):
                    with self.assertRaises(TypeError):
                        mtype.encode(
                            dict(zero_fields(mtype.cls), **{field: bad_num}))

        # Booleans inside numeric fixed arrays are also rejected.
        with self.assertRaises(TypeError):
            arrays.encode(dict(zero_fields(Arrays),
                               samples_m=(True, 0.0, 0.0)))
        with self.assertRaises(TypeError):
            arrays.encode(dict(zero_fields(Arrays),
                               channel_id=[False, 0, 0, 0, 0]))

        # Fixed arrays reject sets, dicts, strings, bytes, and generators.
        for bad_array in (
                {1.0, 2.0, 3.0},
                {1.0: 0, 2.0: 0, 3.0: 0},
                "abc",
                b"abc",
                (float(i) for i in range(3))):
            with self.subTest(bad_array=bad_array):
                with self.assertRaises(TypeError):
                    arrays.encode(
                        dict(zero_fields(Arrays), samples_m=bad_array))
        with self.assertRaises(TypeError):
            arrays.encode(dict(zero_fields(Arrays), channel_id="abcde"))
        with self.assertRaises(TypeError):
            arrays.encode(dict(zero_fields(Arrays), channel_id=b"abcde"))

        # Both tuple and list encode identically.
        as_tuple = arrays.encode(
            dict(zero_fields(Arrays), samples_m=(1.5, -2.25, 4.0)))
        as_list = arrays.encode(
            dict(zero_fields(Arrays), samples_m=[1.5, -2.25, 4.0]))
        self.assertEqual(as_tuple, as_list)

    # REQ-CONTRACT-14: Message.__init__ rejects buffers whose byte length does
    # not match mtype.size instead of deferring a struct.error or ignoring tail.
    def test_message_rejects_wrong_buffer_length(self):
        goal_type = message_type(Goal)
        valid = goal_type.encode(zero_fields(Goal))
        for bad_data in (b"", valid[:-1], valid + b"\0"):
            with self.subTest(length=len(bad_data)):
                with self.assertRaises(ValueError):
                    Message(goal_type, bad_data)
                with self.assertRaises(ValueError):
                    goal_type.decode(bad_data)

    # REQ-CONTRACT-14: MessageType rejects schemas whose flattened field names
    # collide rather than silently overwriting earlier fields in self.index.
    def test_duplicate_flattened_field_names_raise_type_error(self):
        b = flatbuffers.Builder(512)
        inner_name = b.CreateString("dup.Inner")
        outer_name = b.CreateString("dup.Outer")
        decl = b.CreateString("//dup/s.fbs")
        b_name = b.CreateString("b")
        a_name = b.CreateString("a")
        ab_name = b.CreateString("a_b")

        # Inner { b: double } (8 bytes)
        ReflectionType.TypeStart(b)
        ReflectionType.TypeAddBaseType(b, ReflectionBaseType.BaseType.Double)
        ReflectionType.TypeAddIndex(b, -1)
        t_double1 = ReflectionType.TypeEnd(b)

        ReflectionField.FieldStart(b)
        ReflectionField.FieldAddName(b, b_name)
        ReflectionField.FieldAddType(b, t_double1)
        ReflectionField.FieldAddOffset(b, 0)
        f_b = ReflectionField.FieldEnd(b)

        ReflectionObject.ObjectStartFieldsVector(b, 1)
        b.PrependUOffsetTRelative(f_b)
        inner_fields = b.EndVector()

        ReflectionObject.ObjectStart(b)
        ReflectionObject.ObjectAddName(b, inner_name)
        ReflectionObject.ObjectAddFields(b, inner_fields)
        ReflectionObject.ObjectAddIsStruct(b, True)
        ReflectionObject.ObjectAddMinalign(b, 8)
        ReflectionObject.ObjectAddBytesize(b, 8)
        ReflectionObject.ObjectAddDeclarationFile(b, decl)
        inner_obj = ReflectionObject.ObjectEnd(b)

        # Outer { a: Inner @0; a_b: double @8; } (16 bytes) -> both flatten to "a_b"
        ReflectionType.TypeStart(b)
        ReflectionType.TypeAddBaseType(b, ReflectionBaseType.BaseType.Obj)
        ReflectionType.TypeAddIndex(b, 0)
        t_inner = ReflectionType.TypeEnd(b)

        ReflectionField.FieldStart(b)
        ReflectionField.FieldAddName(b, a_name)
        ReflectionField.FieldAddType(b, t_inner)
        ReflectionField.FieldAddOffset(b, 0)
        f_a = ReflectionField.FieldEnd(b)

        ReflectionType.TypeStart(b)
        ReflectionType.TypeAddBaseType(b, ReflectionBaseType.BaseType.Double)
        ReflectionType.TypeAddIndex(b, -1)
        t_double2 = ReflectionType.TypeEnd(b)

        ReflectionField.FieldStart(b)
        ReflectionField.FieldAddName(b, ab_name)
        ReflectionField.FieldAddType(b, t_double2)
        ReflectionField.FieldAddOffset(b, 8)
        f_ab = ReflectionField.FieldEnd(b)

        ReflectionObject.ObjectStartFieldsVector(b, 2)
        b.PrependUOffsetTRelative(f_ab)
        b.PrependUOffsetTRelative(f_a)
        outer_fields = b.EndVector()

        ReflectionObject.ObjectStart(b)
        ReflectionObject.ObjectAddName(b, outer_name)
        ReflectionObject.ObjectAddFields(b, outer_fields)
        ReflectionObject.ObjectAddIsStruct(b, True)
        ReflectionObject.ObjectAddMinalign(b, 8)
        ReflectionObject.ObjectAddBytesize(b, 16)
        ReflectionObject.ObjectAddDeclarationFile(b, decl)
        outer_obj = ReflectionObject.ObjectEnd(b)

        ReflectionSchema.SchemaStartObjectsVector(b, 2)
        b.PrependUOffsetTRelative(outer_obj)
        b.PrependUOffsetTRelative(inner_obj)
        objs = b.EndVector()

        ReflectionSchema.SchemaStartEnumsVector(b, 0)
        enums = b.EndVector()

        ReflectionSchema.SchemaStart(b)
        ReflectionSchema.SchemaAddObjects(b, objs)
        ReflectionSchema.SchemaAddEnums(b, enums)
        b.Finish(ReflectionSchema.SchemaEnd(b), file_identifier=b"BFBS")

        class DupOuter:
            TALOS_SCHEMA_NAME = "dup.Outer"
            TALOS_SCHEMA_HASH = 1
            TALOS_SCHEMA_BFBS = bytes(b.Output())

        with self.assertRaisesRegex(TypeError, "a_b"):
            MessageType(DupOuter)


if __name__ == "__main__":
    unittest.main()
