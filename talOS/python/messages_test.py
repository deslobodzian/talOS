"""The struct.Struct message codec against the flatbuffers.Builder path."""

import random
import struct
import sys
import time
import unittest
from unittest import mock

import flatbuffers

from control.ChassisTarget import ChassisTarget
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
from talos import _messages
from talos._messages import Message, MessageType, message_type

TYPES = (Goal, Status, Report, Translation2d, Rotation2d, Pose2d, Twist2d,
         OdometrySample, VelocityTarget, VelocityState, PositionTarget,
         PositionState, ChassisTarget)

_INT_BITS = {"b": (8, True), "B": (8, False), "h": (16, True),
             "H": (16, False), "i": (32, True), "I": (32, False),
             "q": (64, True), "Q": (64, False)}


class Scaled:
    """A hand-written struct whose Create converts its arguments, which the
    layout recorder cannot follow: it must take the Builder path."""

    __slots__ = ["_tab"]

    @classmethod
    def SizeOf(cls):  # noqa: N802 (flatc's API)
        return 8

    def Init(self, buf, pos):  # noqa: N802
        self._tab = flatbuffers.table.Table(buf, pos)

    def Count(self):  # noqa: N802
        return self._tab.Get(flatbuffers.number_types.Int32Flags,
                             self._tab.Pos)

    def Ratio(self):  # noqa: N802
        return self._tab.Get(flatbuffers.number_types.Float32Flags,
                             self._tab.Pos + 4)


def CreateScaled(builder, count, ratio):  # noqa: N802
    builder.Prep(4, 8)
    builder.PrependFloat32(float(ratio))
    builder.PrependInt32(int(count))
    return builder.Offset()


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
    return {field: random_value(rng, code)
            for field, code in zip(mtype._packed_fields, mtype._packed_codes)}


def builder_bytes(cls, mtype, fields):
    """What flatbuffers.Builder writes for `fields`, independent of talos."""
    create = getattr(sys.modules[cls.__module__], "Create" + cls.__name__)
    builder = flatbuffers.Builder(0)
    create(builder, *(fields.get(f, 0) for f in mtype.fields))
    return bytes(builder.Bytes[builder.Head():])


def builder_type(cls):
    """cls's MessageType on the Builder and accessor path."""
    with mock.patch.object(_messages, "_layout", return_value=None):
        return MessageType(cls)


class StructCodecTest(unittest.TestCase):
    def test_every_type_takes_the_struct_path(self):
        for cls in TYPES:
            mtype = message_type(cls)
            self.assertIsNotNone(mtype.packer, cls.__name__)
            self.assertEqual(mtype.packer.size, cls.SizeOf(), cls.__name__)

    def test_encoding_matches_the_builder_byte_for_byte(self):
        rng = random.Random(1234)
        for cls in TYPES:
            mtype = message_type(cls)
            self.assertEqual(mtype.encode({}),
                             builder_bytes(cls, mtype, {}), cls.__name__)
            for _ in range(200):
                fields = random_fields(rng, mtype)
                self.assertEqual(mtype.encode(fields),
                                 builder_bytes(cls, mtype, fields),
                                 (cls.__name__, fields))
                # A subset defaults the rest to 0/False.
                subset = dict(list(fields.items())[::2])
                self.assertEqual(mtype.encode(subset),
                                 builder_bytes(cls, mtype, subset),
                                 (cls.__name__, subset))

    def test_decoding_round_trips_and_matches_the_accessors(self):
        rng = random.Random(5678)
        for cls in TYPES:
            mtype, slow = message_type(cls), builder_type(cls)
            for _ in range(200):
                fields = random_fields(rng, mtype)
                data = mtype.encode(fields)
                message, accessed = mtype.decode(data), slow.decode(data)
                for field in mtype.fields:
                    self.assertEqual(getattr(message, field), fields[field],
                                     (cls.__name__, field))
                    self.assertEqual(getattr(message, field),
                                     getattr(accessed, field),
                                     (cls.__name__, field))
                    self.assertIs(type(getattr(message, field)),
                                  type(getattr(accessed, field)))
                self.assertEqual(repr(message), repr(accessed))

    def test_nested_struct_fields_flatten(self):
        mtype = message_type(OdometrySample)
        self.assertEqual(mtype.fields, (
            "timestamp_ns", "pose_x_m", "pose_y_m", "pose_yaw_rot",
            "velocity_vx_mps", "velocity_vy_mps", "velocity_omega_radps"))
        data = mtype.encode({"timestamp_ns": 42, "pose_x_m": 1.25,
                             "velocity_omega_radps": 0.25})
        sample = mtype.decode(data)
        self.assertEqual(sample.timestamp_ns, 42)
        self.assertEqual(sample.pose_x_m, 1.25)
        self.assertEqual(sample.pose_y_m, 0.0)
        self.assertEqual(sample.velocity_omega_radps, 0.25)
        raw = OdometrySample()
        raw.Init(data, 0)
        self.assertEqual(raw.Pose(Pose2d()).XM(), 1.25)

    def test_message_views(self):
        mtype = message_type(Report)
        report = mtype.decode(mtype.encode({"phase": 5, "height_rot": 1.5}))
        self.assertIsInstance(report, Message)
        self.assertIs(type(report.phase), int)
        self.assertEqual(repr(report), "Report(phase=HOLDING, height_rot=1.5)")
        self.assertEqual(report, mtype.decode(report._bytes))
        goal = message_type(Goal).decode(message_type(Goal).encode({}))
        self.assertIs(goal.enabled, False)
        self.assertEqual(repr(goal),
                         "Goal(height_rot=0.0, enabled=False, count=0)")
        with self.assertRaisesRegex(AttributeError, "height_rot, enabled"):
            goal.height

    def test_bad_fields_raise_type_error(self):
        mtype = message_type(Goal)
        with self.assertRaisesRegex(TypeError, "Goal has no field height; "
                                    "it has height_rot, enabled, count"):
            mtype.encode({"height": 1.0})
        with self.assertRaisesRegex(TypeError, "field count"):
            mtype.encode({"count": 1 << 40})
        with self.assertRaisesRegex(TypeError, "field height_rot"):
            mtype.encode({"height_rot": "up"})

    def test_unrecordable_create_falls_back_to_the_builder(self):
        mtype = MessageType(Scaled)
        self.assertIsNone(mtype.packer)
        data = mtype.encode({"count": -3, "ratio": 0.5})
        builder = flatbuffers.Builder(0)
        CreateScaled(builder, -3, 0.5)
        self.assertEqual(data, bytes(builder.Bytes[builder.Head():]))
        message = mtype.decode(data)
        self.assertEqual((message.count, message.ratio), (-3, 0.5))
        self.assertEqual(repr(message), "Scaled(count=-3, ratio=0.5)")
        with self.assertRaisesRegex(TypeError, "Scaled has no field x"):
            mtype.encode({"x": 1})

    def test_builder_path_matches_for_generated_types(self):
        rng = random.Random(91011)
        for cls in TYPES:
            fast, slow = message_type(cls), builder_type(cls)
            self.assertIsNone(slow.packer)
            for _ in range(20):
                fields = random_fields(rng, fast)
                self.assertEqual(slow.encode(fields), fast.encode(fields))
                decoded = slow.decode(fast.encode(fields))
                for field in fast.fields:
                    self.assertEqual(getattr(decoded, field), fields[field])

    def test_struct_path_is_faster(self):
        fast, slow = message_type(OdometrySample), builder_type(OdometrySample)
        fields = random_fields(random.Random(7), fast)
        data = fast.encode(fields)

        def best(mtype, ops=10_000):
            runs = []
            for _ in range(3):
                start = time.perf_counter()
                for _ in range(ops):
                    message = mtype.decode(mtype.encode(fields))
                    message.pose_x_m
                    message.velocity_omega_radps
                runs.append(time.perf_counter() - start)
            return min(runs)

        self.assertEqual(slow.encode(fields), data)
        fast_s, slow_s = best(fast), best(slow)
        self.assertLess(fast_s * 2, slow_s,
                        f"struct {fast_s:.4f}s vs builder {slow_s:.4f}s")


if __name__ == "__main__":
    unittest.main()
