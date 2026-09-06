"""Payload codec for IPCMessage::TestMessage over the RTMS transport.

Field access goes through the ``flatc --python`` bindings checked in under
``IPCMessage/`` (regenerate verbatim with
``flatc --python -o talOS/ipc/python talOS/ipc/ipc_test_message.fbs``).
The wire bytes are the flatbuffer struct laid out inline, little-endian:
``id`` int32 @ 0, ``value`` float32 @ 4 — exactly ``builder.Prep(4, 8)``
followed by ``PrependFloat32``/``PrependInt32``, i.e. ``struct '<if'``.

C++ layout contract (asserted by peer.cc static_asserts, probed here):
sizeof == 8, alignof == 4.
"""

import struct

from IPCMessage.TestMessage import CreateTestMessage, TestMessage

import flatbuffers

MESSAGE_SIZE = TestMessage.SizeOf()
MESSAGE_ALIGNMENT = 4

_WIRE = struct.Struct("<if")


def encode(msg_id, value):
    """Encode (id, value) to the 8 payload bytes the C++ peer memcpys."""
    return _WIRE.pack(msg_id, value)


def decode(payload):
    """Decode 8 payload bytes via the generated accessors -> (id, value)."""
    if len(payload) != MESSAGE_SIZE:
        raise ValueError(
            "TestMessage payload must be %d bytes, got %d"
            % (MESSAGE_SIZE, len(payload))
        )
    msg = TestMessage()
    msg.Init(bytes(payload), 0)
    return (msg.Id(), msg.Value())


def encode_via_builder(msg_id, value):
    """Same bytes through the generated Create* helper (cross-check only)."""
    builder = flatbuffers.Builder(16)
    CreateTestMessage(builder, msg_id, value)
    head = builder.head
    return bytes(builder.Bytes[head:head + MESSAGE_SIZE])
