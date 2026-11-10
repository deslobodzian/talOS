"""The reflection-driven struct.Struct message codec: byte-for-byte against
flatc C++ (messages_golden) and flatc Python (flatbuffers.Builder)."""

import os
import random
import struct
import subprocess
import sys
import unittest

import flatbuffers
from python.runfiles import runfiles

from control.PositionState import PositionState
from control.PositionTarget import PositionTarget
from control.VelocityState import VelocityState
from control.VelocityTarget import VelocityTarget
from demo.Goal import Goal
from demo.Report import Report
from demo.Status import Status
from geometry.OdometrySample import OdometrySample
from geometry.Pose2d import Pose2d
from geometry.Rotation2d import Rotation2d
from geometry.Translation2d import Translation2d
from geometry.Twist2d import Twist2d
from schema_test.Arrays import Arrays
from schema_test.Inner import Inner
from schema_test.Level import Level
from schema_test.Mixed import Mixed
from schema_test.Mode import Mode
from talos._messages import MessageType, message_type

TYPES = (Goal, Status, Report, Translation2d, Rotation2d, Pose2d, Twist2d,
         OdometrySample, VelocityTarget, VelocityState, PositionTarget,
         PositionState)

_INT_BITS = {"b": (8, True), "B": (8, False), "h": (16, True),
             "H": (16, False), "i": (32, True), "I": (32, False),
             "q": (64, True), "Q": (64, False)}


def zero_fields(cls):
    """Every field of cls at zero: encode takes no omitted field."""
    mtype = message_type(cls)
    return {field: (0,) * mtype._arrays[field] if field in mtype._arrays
            else 0 for field in mtype.fields}


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

    def test_layout_is_the_schema_layout(self):
        for cls, size, alignment in ((Inner, 8, 4), (Mixed, 40, 8),
                                     (Arrays, 56, 8)):
            mtype = message_type(cls)
            self.assertEqual((mtype.size, mtype.alignment), (size, alignment))
            self.assertEqual(mtype.size, cls.SizeOf())
        self.assertEqual(message_type(Mixed).fields, (
            "flag", "position_m", "level", "sample_count", "inner_mode",
            "inner_offset_m", "speed_frac"))
        self.assertEqual(message_type(Arrays).fields, (
            "samples_m", "modes", "inners_0_mode", "inners_0_offset_m",
            "inners_1_mode", "inners_1_offset_m", "channel_id"))

    def test_a_list_encodes_like_a_tuple(self):
        self.assertEqual(
            message_type(Arrays).encode(
                dict(GOLDEN_VALUES["arrays"][1], samples_m=[1.5, -2.25, 1e9])),
            self.golden["arrays"])


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

    def test_encoding_matches_the_builder_byte_for_byte(self):
        rng = random.Random(1234)
        for cls in TYPES:
            mtype = message_type(cls)
            zeros = zero_fields(cls)
            self.assertEqual(mtype.encode(zeros),
                             builder_bytes(cls, mtype, zeros), cls.__name__)
            for _ in range(200):
                fields = random_fields(rng, mtype)
                self.assertEqual(mtype.encode(fields),
                                 builder_bytes(cls, mtype, fields),
                                 (cls.__name__, fields))

    def test_decoding_round_trips(self):
        rng = random.Random(5678)
        for cls in TYPES:
            mtype = message_type(cls)
            for _ in range(200):
                fields = random_fields(rng, mtype)
                message = mtype.decode(mtype.encode(fields))
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


if __name__ == "__main__":
    unittest.main()
