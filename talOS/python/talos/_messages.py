"""flatc struct classes as topic messages: snake_case views and encoding.

A flatc Python struct class has CamelCase accessors (RollerVelocityRps) and
a module-level Create<Struct>(builder, rollerVelocityRps, ...) whose
parameters list the fields in order. Both are derived from that signature,
so views and send(**fields) use the .fbs field names (roller_velocity_rps).

A field that is itself a struct flattens in Create (pose_x, pose_y, ...) and
reads through a sub-struct accessor (Pose(obj)). Those map through the
sub-struct's class, resolved from the accessor's annotation in the .pyi stub
flatc --python-typing writes beside it (talos_messages always passes it), so
send(pose_x=...) and msg.pose_x both work, at any depth.

Enum-typed fields read as plain ints; repr names them. The generated .py
does not say which fields are enums, so that is read from the .pyi stub.

Encoding and decoding run through one struct.Struct per type, whose layout
is read off Create<Struct> itself: it runs once against a builder that only
records what it would write (see _layout). A Create that the recorder cannot
follow keeps the flatbuffers.Builder and accessor path for that type.
"""


import ast
import enum
import functools
import importlib
import inspect
import os
import re
import struct
import sys

import flatbuffers

# Builder.Prepend<Kind> -> struct format character; little-endian, like the
# flatbuffers wire format.
_FORMATS = {
    "Bool": "?", "Byte": "B", "Uint8": "B", "Int8": "b", "Uint16": "H",
    "Int16": "h", "Uint32": "I", "Int32": "i", "Uint64": "Q", "Int64": "q",
    "Float32": "f", "Float64": "d",
}


class _Recorder:
    """Stands in for flatbuffers.Builder during one Create<Struct> call and
    records the bytes it would write. Builder writes back to front, so
    `written` lists ("pad", n) and (format, argument) last field first;
    `offset` is the byte count so far, which is what Prep aligns against."""

    def __init__(self):
        self.offset = 0
        self.written = []

    def Prep(self, size, additional_bytes):  # noqa: N802 (Builder's API)
        self.Pad(-(self.offset + additional_bytes) % size)

    def Pad(self, n):  # noqa: N802
        if n > 0:
            self.written.append(("pad", n))
            self.offset += n

    def Offset(self):  # noqa: N802
        return self.offset

    def __getattr__(self, name):
        kind = name[len("Prepend"):] if name.startswith("Prepend") else None
        if kind not in _FORMATS:
            raise AttributeError(name)
        fmt = _FORMATS[kind]

        def prepend(value):
            size = struct.calcsize(fmt)
            self.Prep(size, 0)
            self.written.append((fmt, value))
            self.offset += size

        return prepend


def _layout(create, param_count, size):
    """(struct.Struct, ((Create parameter index, format character), ...) per
    unpacked value) for the struct Create writes, or None when Create does
    anything but align, pad and prepend each parameter once, or the sizes
    disagree."""
    params = [object() for _ in range(param_count)]
    recorder = _Recorder()
    try:
        create(recorder, *params)
    except Exception:  # noqa: BLE001 (any surprise means: use the Builder)
        return None
    position = {id(param): i for i, param in enumerate(params)}
    codes, values = ["<"], []
    for fmt, value in reversed(recorder.written):
        if fmt == "pad":
            codes.append(f"{value}x")
            continue
        index = position.get(id(value))
        if index is None or value is not params[index]:
            return None
        codes.append(fmt)
        values.append((index, fmt))
    if sorted(index for index, _ in values) != list(range(param_count)):
        return None
    try:
        packer = struct.Struct("".join(codes))
    except struct.error:
        return None
    if packer.size != size or recorder.offset != size:
        return None
    return packer, tuple(values)


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


def _snake(name):
    return re.sub(r"(?<=[a-z0-9])([A-Z])", r"_\1", name).lower()


def _stub_model(cls):
    """(module AST, {local name: (module, name)}) for cls's .pyi stub."""
    source = getattr(sys.modules[cls.__module__], "__file__", None)
    if not source:
        return None
    try:
        with open(os.path.splitext(source)[0] + ".pyi", encoding="utf-8") as f:
            tree = ast.parse(f.read())
    except (OSError, SyntaxError):
        return None
    imports = {}  # local name -> (module, name)
    for node in tree.body:
        if isinstance(node, ast.ImportFrom) and node.module:
            for alias in node.names:
                imports[alias.asname or alias.name] = (node.module, alias.name)
    return tree, imports


def _stub_enums(cls):
    """{accessor: enum class} for cls's enum-typed fields, from its .pyi
    stub; {} when there is none."""
    parsed = _stub_model(cls)
    if parsed is None:
        return {}
    tree, imports = parsed
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


def _stub_structs(cls):
    """{accessor: struct class} for cls's nested-struct accessors, from the
    accessor annotations in its .pyi stub; {} when there is none."""
    parsed = _stub_model(cls)
    if parsed is None:
        return {}
    tree, imports = parsed
    structs = {}
    for node in tree.body:
        if not (isinstance(node, ast.ClassDef) and node.name == cls.__name__):
            continue
        for method in node.body:
            if not isinstance(method, ast.FunctionDef):
                continue
            args = method.args.args
            if len(args) != 2 or not isinstance(args[1].annotation, ast.Name):
                continue
            head = args[1].annotation.id
            if head not in imports:
                continue
            module, name = imports[head]
            try:
                found = getattr(importlib.import_module(module), name)
            except (ImportError, AttributeError):
                continue
            if (isinstance(found, type) and hasattr(found, "SizeOf")
                    and hasattr(sys.modules.get(found.__module__),
                                "Create" + found.__name__)):
                structs[method.name] = found
    return structs



class MessageType:
    """One flatc struct class: its size, fields and codec."""

    def __init__(self, cls):
        self.cls = cls
        self.size = cls.SizeOf()  # the slot alignment is always 8
        create = getattr(sys.modules[cls.__module__], "Create" + cls.__name__)
        params = list(inspect.signature(create).parameters)[1:]  # builder
        stubs = _stub_structs(cls)
        self.accessors = {}  # snake_case field -> flatc accessor
        self.structs = {}  # flattened field -> (accessor, sub type, sub field)
        fields = []  # Create order: flattened names are Create's parameters
        for param in params:
            field = _snake(param)
            fields.append(field)
            accessor = param[0].upper() + param[1:]
            if callable(getattr(cls, accessor, None)):
                self.accessors[field] = accessor
                continue
            # A nested struct flattens (pose_x, ...): the head names its
            # accessor, whose stub annotation names the sub-struct class.
            head, sep, rest = param.partition("_")
            base = head[0].upper() + head[1:] if head else ""
            sub_cls = stubs.get(base)
            sub = message_type(sub_cls) if sub_cls is not None else None
            sub_field = _snake(rest) if sub is not None else None
            if not sep or sub is None or sub_field not in sub.fields:
                raise TypeError(f"{cls.__name__}: field {param} is a nested "
                                "struct or array, which talos does not map")
            self.structs[field] = (base, sub, sub_field)
        self.fields = tuple(fields)
        self._field_set = frozenset(fields)
        stub_enums = _stub_enums(cls)
        self.enum_names = {  # field -> {value: name}, enum-typed fields only
            field: enum_names(stub_enums[accessor])
            for field, accessor in self.accessors.items()
            if accessor in stub_enums}
        self._create = create
        layout = _layout(create, len(params), self.size)
        if layout is None:  # Builder and accessors, in Create order
            self.packer = None
            self.index = {field: i for i, field in enumerate(self.fields)}
            self.unpack = self._read_all
        else:  # one struct.Struct, in memory order
            self.packer, values = layout
            self._packed_fields = tuple(self.fields[i] for i, _ in values)
            self._packed_codes = tuple(code for _, code in values)
            self.index = {f: i for i, f in enumerate(self._packed_fields)}
            self.unpack = self.packer.unpack_from

    def _read(self, obj, field):
        """field's value from a flatc struct object (recurses into structs)."""
        accessor = self.accessors.get(field)
        if accessor is not None:
            return getattr(obj, accessor)()
        base, sub, sub_field = self.structs[field]
        sub_obj = sub.cls()
        getattr(obj, base)(sub_obj)
        return sub._read(sub_obj, sub_field)

    def _read_all(self, data):
        """Every field of `data` in Create order, through flatc accessors."""
        obj = self.cls()
        obj.Init(data, 0)
        return tuple(self._read(obj, field) for field in self.fields)

    def encode(self, fields):
        if not self._field_set.issuperset(fields):
            unknown = set(fields) - self._field_set
            raise TypeError(f"{self.cls.__name__} has no field "
                            f"{', '.join(sorted(unknown))}; it has "
                            f"{', '.join(self.fields)}")
        if self.packer is None:
            return self._build(fields)
        get = fields.get
        values = [get(field, 0) for field in self._packed_fields]
        try:
            return self.packer.pack(*values)
        except struct.error:
            raise TypeError(self._bad_value(values)) from None

    def _bad_value(self, values):
        """Names the field and value the packer rejected."""
        for field, code, value in zip(self._packed_fields, self._packed_codes,
                                      values):
            try:
                struct.pack("<" + code, value)
            except struct.error as error:
                return (f"{self.cls.__name__}: bad value {value!r} for "
                        f"field {field}: {error}")
        return f"{self.cls.__name__}: bad field value"

    def _build(self, fields):
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
    """A received message: msg.roller_velocity_rps reads that field. The
    fields are unpacked together on the first read and kept."""

    __slots__ = ("_type", "_bytes", "_values")

    def __init__(self, mtype, data):
        self._type = mtype
        self._bytes = bytes(data)
        self._values = None

    def __getattr__(self, name):
        mtype = self._type
        index = mtype.index.get(name)
        if index is None:
            raise AttributeError(f"{mtype.cls.__name__} has no field "
                                 f"{name!r}; it has "
                                 f"{', '.join(mtype.fields)}")
        values = self._values
        if values is None:
            values = self._values = mtype.unpack(self._bytes)
        return values[index]

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
