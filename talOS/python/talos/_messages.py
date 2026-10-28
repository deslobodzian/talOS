"""flatc struct classes as topic messages: snake_case views and encoding.

A flatc Python struct class has CamelCase accessors (RollerVelocityRps) and
a module-level Create<Struct>(builder, rollerVelocityRps, ...) whose
parameters list the fields in order. Both are derived from that signature,
so views and send(**fields) use the .fbs field names (roller_velocity_rps).
"""

import functools
import inspect
import re
import sys

import flatbuffers


class MessageType:
    """One flatc struct class: its size, fields and codec."""

    def __init__(self, cls):
        self.cls = cls
        self.size = cls.SizeOf()  # the slot alignment is always 8
        create = getattr(sys.modules[cls.__module__], "Create" + cls.__name__)
        params = list(inspect.signature(create).parameters)[1:]  # builder
        self.accessors = {}  # snake_case field -> flatc accessor
        for param in params:
            accessor = param[0].upper() + param[1:]
            if not callable(getattr(cls, accessor, None)):
                raise TypeError(f"{cls.__name__}: field {param} is a nested "
                                "struct or array, which talos does not map")
            self.accessors[re.sub(r"(?<=[a-z0-9])([A-Z])", r"_\1",
                                  param).lower()] = accessor
        self.fields = tuple(self.accessors)
        self._create = create

    def encode(self, fields):
        unknown = set(fields) - set(self.fields)
        if unknown:
            raise TypeError(f"{self.cls.__name__} has no field "
                            f"{', '.join(sorted(unknown))}; it has "
                            f"{', '.join(self.fields)}")
        builder = flatbuffers.Builder(self.size)
        self._create(builder, *(fields.get(f, 0) for f in self.fields))
        data = bytes(builder.Bytes[builder.Head():])
        assert len(data) == self.size, (self.cls.__name__, len(data))
        return data

    def decode(self, data):
        return Message(self, data)


@functools.lru_cache(maxsize=None)
def message_type(cls):
    return MessageType(cls)


class Message:
    """A received message: msg.roller_velocity_rps reads RollerVelocityRps()."""

    __slots__ = ("_type", "_bytes", "_struct")

    def __init__(self, mtype, data):
        self._type = mtype
        self._bytes = bytes(data)
        self._struct = mtype.cls()
        self._struct.Init(self._bytes, 0)

    def __getattr__(self, name):
        accessor = self._type.accessors.get(name)
        if accessor is None:
            raise AttributeError(f"{self._type.cls.__name__} has no field "
                                 f"{name!r}; it has "
                                 f"{', '.join(self._type.fields)}")
        return getattr(self._struct, accessor)()

    def __eq__(self, other):
        return (isinstance(other, Message) and other._type is self._type
                and other._bytes == self._bytes)

    def __repr__(self):
        fields = ", ".join(f"{f}={getattr(self, f)!r}"
                           for f in self._type.fields)
        return f"{self._type.cls.__name__}({fields})"
