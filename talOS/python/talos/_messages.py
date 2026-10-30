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
        stub_enums = _stub_enums(cls)
        self.enum_names = {  # field -> {value: name}, enum-typed fields only
            field: enum_names(stub_enums[accessor])
            for field, accessor in self.accessors.items()
            if accessor in stub_enums}
        self._create = create

    def _read(self, struct, field):
        """field's value from a raw struct object (recurses into structs)."""
        accessor = self.accessors.get(field)
        if accessor is not None:
            return getattr(struct, accessor)()
        base, sub, sub_field = self.structs[field]
        sub_struct = sub.cls()
        getattr(struct, base)(sub_struct)
        return sub._read(sub_struct, sub_field)


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
        if name not in self._type.fields:
            raise AttributeError(f"{self._type.cls.__name__} has no field "
                                 f"{name!r}; it has "
                                 f"{', '.join(self._type.fields)}")
        return self._type._read(self._struct, name)

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
