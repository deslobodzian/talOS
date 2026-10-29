"""flatc struct classes as topic messages: snake_case views and encoding.

A flatc Python struct class has CamelCase accessors (RollerVelocityRps) and
a module-level Create<Struct>(builder, rollerVelocityRps, ...) whose
parameters list the fields in order. Both are derived from that signature,
so views and send(**fields) use the .fbs field names (roller_velocity_rps).

Enum-typed fields read as plain ints; repr names them. The generated .py
does not say which fields are enums, so that is read from the .pyi stub
flatc --python-typing writes beside it (talos_messages always passes it).
"""

import ast
import enum
import functools
import importlib
import inspect
import os
import re
import sys

import flatbuffers


def enum_names(cls):
    """{value: name} for an enum.Enum, or for a flatc enum class (int
    attributes); the first name wins for an aliased value."""
    if isinstance(cls, type) and issubclass(cls, enum.Enum):
        return {member: member.name for member in cls}
    names = {}
    for name, value in vars(cls).items():
        if (not name.startswith("_") and isinstance(value, int)
                and not isinstance(value, bool)):
            names.setdefault(value, name)
    return names


def _literal_enum(annotation, imports):
    """The enum class of a `typing.Literal[E.A, E.B]` annotation, or None."""
    if not (isinstance(annotation, ast.Subscript)
            and ast.unparse(annotation.value) in ("typing.Literal", "Literal")):
        return None
    members = annotation.slice
    first = members.elts[0] if isinstance(members, ast.Tuple) else members
    if not isinstance(first, ast.Attribute):
        return None
    head, _, rest = ast.unparse(first.value).partition(".")
    if head not in imports:
        return None
    module, name = imports[head]
    try:
        found = getattr(importlib.import_module(module), name)
        for part in filter(None, rest.split(".")):
            found = getattr(found, part)
    except (ImportError, AttributeError):
        return None
    return found if isinstance(found, type) else None


def _stub_enums(cls):
    """{accessor: enum class} for cls's enum-typed fields, from its .pyi
    stub; {} when there is none."""
    source = getattr(sys.modules[cls.__module__], "__file__", None)
    if not source:
        return {}
    stub = os.path.splitext(source)[0] + ".pyi"
    try:
        with open(stub, encoding="utf-8") as f:
            tree = ast.parse(f.read(), stub)
    except (OSError, SyntaxError):
        return {}
    imports = {}  # local name -> (module, name)
    for node in tree.body:
        if isinstance(node, ast.ImportFrom) and node.module:
            for alias in node.names:
                imports[alias.asname or alias.name] = (node.module, alias.name)
    enums = {}
    for node in tree.body:
        if not (isinstance(node, ast.ClassDef) and node.name == cls.__name__):
            continue
        for method in node.body:
            if isinstance(method, ast.FunctionDef) and method.returns:
                found = _literal_enum(method.returns, imports)
                if found is not None:
                    enums[method.name] = found
    return enums


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
        stub_enums = _stub_enums(cls)
        self.enum_names = {  # field -> {value: name}, enum-typed fields only
            field: enum_names(stub_enums[accessor])
            for field, accessor in self.accessors.items()
            if accessor in stub_enums}
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
        def shown(field):
            value = getattr(self, field)
            names = self._type.enum_names.get(field, {})
            return names[value] if value in names else repr(value)

        fields = ", ".join(f"{f}={shown(f)}" for f in self._type.fields)
        return f"{self._type.cls.__name__}({fields})"
