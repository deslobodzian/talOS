"""Schema hashes of talos messages, from FlatBuffers binary schemas.

talos_messages compiles each schema to a .bfbs (a reflection.Schema) and
gives every generated struct class three attributes:

    from control.VelocityTarget import VelocityTarget
    VelocityTarget.TALOS_SCHEMA_NAME  # "control.VelocityTarget"
    VelocityTarget.TALOS_SCHEMA_HASH  # struct_hash(BFBS, NAME)
    VelocityTarget.TALOS_SCHEMA_BFBS  # the schema file's .bfbs bytes

struct_hash reads .bfbs bytes with the reflection classes flatc generates
from upstream reflection.fbs (`import reflection`) and computes the hash
talOS/schema/schema_hash.h defines: the same value C++'s
talos::schema::struct_hash and talos::Descriptor<M>::kHash give.
"""

from reflection.BaseType import BaseType
from reflection.Schema import Schema

# reflection::EnumNameBaseType: flatc spells BaseType.None as None_.
_BASE_NAMES = {value: name.rstrip("_") for name, value in vars(BaseType).items()
               if not name.startswith("__")}

# flatbuffers/hash.h FnvTraits<uint64_t>.
_FNV_PRIME = 0x00000100000001B3
_FNV_OFFSET_BASIS = 0xCBF29CE484222645
_MASK = (1 << 64) - 1


def _fnv1a_64(data):
    value = _FNV_OFFSET_BASIS
    for byte in data:
        value = ((value ^ byte) * _FNV_PRIME) & _MASK
    return value


def _text(raw):
    return raw.decode("utf-8")


def _type(schema, base, index, out):
    if base == BaseType.Obj:
        _struct(schema, schema.Objects(index), out)
    elif index < 0:
        out.append(_BASE_NAMES[base])
    else:
        enum = schema.Enums(index)
        out.append(f"enum {_text(enum.Name())}:{_BASE_NAMES[base]} {{")
        for i in range(enum.ValuesLength()):
            value = enum.Values(i)
            out.append(f"{_text(value.Name())}={value.Value()},")
        out.append("}")


def _struct(schema, obj, out):
    out.append(f"struct {_text(obj.Name())} {obj.Bytesize()} "
               f"{obj.Minalign()} {{")
    for i in range(obj.FieldsLength()):
        field = obj.Fields(i)
        kind = field.Type()
        out.append(f"{_text(field.Name())}@{field.Offset()}:")
        if kind.BaseType() == BaseType.Array:
            out.append("[")
            _type(schema, kind.Element(), kind.Index(), out)
            out.append(f"*{kind.FixedLength()}]")
        else:
            _type(schema, kind.BaseType(), kind.Index(), out)
        out.append(";")
    out.append("}")


def canonical_struct(bfbs, name):
    """The canonical text of struct `name` in .bfbs bytes (schema_hash.h),
    or None when the bytes are not a reflection.Schema or hold no struct of
    that name."""
    data = bytes(bfbs)
    if not Schema.SchemaBufferHasIdentifier(data, 0):
        return None
    schema = Schema.GetRootAs(data, 0)
    wanted = name.encode("utf-8")
    for i in range(schema.ObjectsLength()):
        obj = schema.Objects(i)
        if obj.Name() == wanted:
            if not obj.IsStruct():
                return None
            out = []
            _struct(schema, obj, out)
            return "".join(out)
    return None


def struct_hash(bfbs, name):
    """The schema hash of struct `name` ("control.VelocityTarget") in .bfbs
    bytes, or None when canonical_struct is None."""
    text = canonical_struct(bfbs, name)
    return None if text is None else _fnv1a_64(text.encode("utf-8"))
