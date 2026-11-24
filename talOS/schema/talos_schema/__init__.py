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
find_struct resolves a named struct inside a .bfbs reflection schema and is
shared with talos._messages.
"""

import struct

from reflection.BaseType import BaseType
from reflection.Schema import Schema


class SchemaError(ValueError):
    """A reflection .bfbs buffer is invalid or lacks the requested struct."""


# reflection::EnumNameBaseType: flatc spells BaseType.None as None_.
_BASE_NAMES = {value: name.rstrip("_") for name, value in vars(BaseType).items()
               if not name.startswith("__")}

# Scalar byte sizes matching flatbuffers::GetTypeSize (UType..Double).
_SCALAR_SIZES = {
    BaseType.UType: 1,
    BaseType.Bool: 1,
    BaseType.Byte: 1,
    BaseType.UByte: 1,
    BaseType.Short: 2,
    BaseType.UShort: 2,
    BaseType.Int: 4,
    BaseType.UInt: 4,
    BaseType.Long: 8,
    BaseType.ULong: 8,
    BaseType.Float: 4,
    BaseType.Double: 8,
}

# Integer scalar base types that may name a reflection enum (verify.h).
_ENUM_BASE_TYPES = frozenset({
    BaseType.UType,
    BaseType.Byte,
    BaseType.UByte,
    BaseType.Short,
    BaseType.UShort,
    BaseType.Int,
    BaseType.UInt,
    BaseType.Long,
    BaseType.ULong,
})

# Parity limits with talOS/schema/schema_hash.cc and flatbuffers::Verifier.
_MAX_DEPTH = 32
_MAX_STRUCT_VISITS = 4096
_MAX_VERIFY_TABLES = 1000000

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
    if raw is None:
        raise SchemaError("missing reflection string")
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError as error:
        raise SchemaError(f"invalid UTF-8 string: {error}") from None


class _Verifier:
    """Structural FlatBuffers verifier for reflection.Schema buffers, matching
    talos::schema::VerifySchema (bounds, alignment, table/vector/string
    integrity, and raw BaseType byte range)."""

    __slots__ = ("_data", "_size", "_tables")

    def __init__(self, data):
        self._data = data
        self._size = len(data)
        self._tables = 0

    def _check_range(self, pos, length, align=1):
        if (pos < 0 or length < 0 or (pos & (align - 1)) != 0
                or length > self._size or pos > self._size - length):
            raise SchemaError("out-of-bounds or misaligned reflection buffer")

    def _read_u32(self, pos):
        self._check_range(pos, 4, 4)
        return struct.unpack_from("<I", self._data, pos)[0]

    def _read_i32(self, pos):
        self._check_range(pos, 4, 4)
        return struct.unpack_from("<i", self._data, pos)[0]

    def _read_u16(self, pos):
        self._check_range(pos, 2, 2)
        return struct.unpack_from("<H", self._data, pos)[0]

    def _read_i8(self, pos):
        self._check_range(pos, 1, 1)
        return struct.unpack_from("<b", self._data, pos)[0]

    def _verify_offset(self, pos):
        rel = self._read_u32(pos)
        if rel == 0 or rel >= 0x80000000:
            raise SchemaError("invalid reflection table or vector offset")
        target = pos + rel
        if target >= self._size:
            raise SchemaError("reflection offset points past end of buffer")
        return target

    def _table_start(self, table_pos):
        self._tables += 1
        if self._tables > _MAX_VERIFY_TABLES:
            raise SchemaError("reflection schema exceeds table verification cap")
        soff = self._read_i32(table_pos)
        vtable = table_pos - soff
        vsize = self._read_u16(vtable)
        if vsize < 4 or (vsize & 1) != 0:
            raise SchemaError("invalid reflection vtable size")
        self._check_range(vtable, vsize, 2)
        return vtable, vsize

    def _field_pos(self, table_pos, vtable, vsize, voff):
        if voff + 2 > vsize:
            return None
        off = self._read_u16(vtable + voff)
        if off == 0:
            return None
        return table_pos + off

    def _verify_scalar(self, table_pos, vtable, vsize, voff, size, align):
        fp = self._field_pos(table_pos, vtable, vsize, voff)
        if fp is not None:
            self._check_range(fp, size, align)
        return fp

    def _verify_string(self, table_pos, vtable, vsize, voff):
        fp = self._field_pos(table_pos, vtable, vsize, voff)
        if fp is None:
            return
        spos = self._verify_offset(fp)
        length = self._read_u32(spos)
        # String needs length bytes plus trailing NUL terminator.
        if length >= self._size or spos + 4 + length >= self._size:
            raise SchemaError("reflection string exceeds buffer bounds")
        if self._data[spos + 4 + length] != 0:
            raise SchemaError("reflection string is not NUL-terminated")

    def _verify_table_vector(self, table_pos, vtable, vsize, voff, verify_elem):
        fp = self._field_pos(table_pos, vtable, vsize, voff)
        if fp is None:
            return
        vpos = self._verify_offset(fp)
        length = self._read_u32(vpos)
        max_elems = (self._size - (vpos + 4)) // 4
        if length > max_elems:
            raise SchemaError("reflection vector exceeds buffer bounds")
        for i in range(length):
            verify_elem(self._verify_offset(vpos + 4 + i * 4))

    def _verify_type(self, table_pos):
        vtable, vsize = self._table_start(table_pos)
        for voff in (4, 6):  # base_type, element (int8)
            fp = self._verify_scalar(table_pos, vtable, vsize, voff, 1, 1)
            if fp is not None:
                raw = self._read_i8(fp)
                if raw < BaseType.None_ or raw > BaseType.MaxBaseType:
                    raise SchemaError(f"out-of-range BaseType byte {raw}")
        self._verify_scalar(table_pos, vtable, vsize, 8, 4, 4)   # index
        self._verify_scalar(table_pos, vtable, vsize, 10, 2, 2)  # fixed_length

    def _verify_field(self, table_pos):
        vtable, vsize = self._table_start(table_pos)
        self._verify_string(table_pos, vtable, vsize, 4)  # name
        tfp = self._field_pos(table_pos, vtable, vsize, 6)  # type
        if tfp is not None:
            self._verify_type(self._verify_offset(tfp))
        self._verify_scalar(table_pos, vtable, vsize, 8, 2, 2)   # id
        self._verify_scalar(table_pos, vtable, vsize, 10, 2, 2)  # offset

    def _verify_object(self, table_pos):
        vtable, vsize = self._table_start(table_pos)
        self._verify_string(table_pos, vtable, vsize, 4)  # name
        self._verify_table_vector(table_pos, vtable, vsize, 6,
                                  self._verify_field)  # fields
        self._verify_scalar(table_pos, vtable, vsize, 8, 1, 1)   # is_struct
        self._verify_scalar(table_pos, vtable, vsize, 10, 4, 4)  # minalign
        self._verify_scalar(table_pos, vtable, vsize, 12, 4, 4)  # bytesize

    def _verify_enum_val(self, table_pos):
        vtable, vsize = self._table_start(table_pos)
        self._verify_string(table_pos, vtable, vsize, 4)  # name
        self._verify_scalar(table_pos, vtable, vsize, 6, 8, 8)  # value
        ufp = self._field_pos(table_pos, vtable, vsize, 10)  # union_type
        if ufp is not None:
            self._verify_type(self._verify_offset(ufp))

    def _verify_enum(self, table_pos):
        vtable, vsize = self._table_start(table_pos)
        self._verify_string(table_pos, vtable, vsize, 4)  # name
        self._verify_table_vector(table_pos, vtable, vsize, 6,
                                  self._verify_enum_val)  # values
        self._verify_scalar(table_pos, vtable, vsize, 8, 1, 1)  # is_union
        utfp = self._field_pos(table_pos, vtable, vsize, 10)  # underlying_type
        if utfp is not None:
            self._verify_type(self._verify_offset(utfp))

    def _verify_rpc_call(self, table_pos):
        vtable, vsize = self._table_start(table_pos)
        self._verify_string(table_pos, vtable, vsize, 4)  # name
        for voff in (6, 8):  # request, response
            ofp = self._field_pos(table_pos, vtable, vsize, voff)
            if ofp is not None:
                self._verify_object(self._verify_offset(ofp))

    def _verify_service(self, table_pos):
        vtable, vsize = self._table_start(table_pos)
        self._verify_string(table_pos, vtable, vsize, 4)  # name
        self._verify_table_vector(table_pos, vtable, vsize, 6,
                                  self._verify_rpc_call)  # calls

    def verify_schema(self):
        if self._size < 8 or self._data[4:8] != b"BFBS":
            raise SchemaError("not a reflection schema")
        root = self._verify_offset(0)
        vtable, vsize = self._table_start(root)
        self._verify_table_vector(root, vtable, vsize, 4,
                                  self._verify_object)  # objects
        self._verify_table_vector(root, vtable, vsize, 6,
                                  self._verify_enum)  # enums
        self._verify_string(root, vtable, vsize, 8)   # file_ident
        self._verify_string(root, vtable, vsize, 10)  # file_ext
        rfp = self._field_pos(root, vtable, vsize, 12)  # root_table
        if rfp is not None:
            self._verify_object(self._verify_offset(rfp))
        self._verify_table_vector(root, vtable, vsize, 14,
                                  self._verify_service)  # services


def _value_size(schema, base, index):
    if base == BaseType.Obj:
        if (schema.ObjectsIsNone() or index < 0
                or index >= schema.ObjectsLength()):
            raise SchemaError(f"invalid object index {index}")
        obj = schema.Objects(index)
        if obj is None or not obj.IsStruct() or obj.Bytesize() <= 0:
            raise SchemaError(f"object {index} is not a positive-sized struct")
        return obj.Bytesize()
    size = _SCALAR_SIZES.get(base)
    if size is None:
        raise SchemaError(f"unsupported base type {base}")
    if index >= 0:
        if (base not in _ENUM_BASE_TYPES or schema.EnumsIsNone()
                or index >= schema.EnumsLength()):
            raise SchemaError(
                f"invalid enum index {index} for base type {base}")
    return size


def _type(schema, base, index, out, depth, open_indices, visits):
    _value_size(schema, base, index)
    if base == BaseType.Obj:
        if index in open_indices:
            raise SchemaError(f"cyclic struct reference at index {index}")
        open_indices.add(index)
        try:
            _struct(schema, schema.Objects(index), out, depth, open_indices,
                    visits)
        finally:
            open_indices.remove(index)
        return
    if index < 0:
        out.append(_BASE_NAMES[base])
        return
    enum = schema.Enums(index)
    if enum is None or enum.Name() is None or enum.ValuesIsNone():
        raise SchemaError(f"invalid enum at index {index}")
    out.append(f"enum {_text(enum.Name())}:{_BASE_NAMES[base]} {{")
    for i in range(enum.ValuesLength()):
        value = enum.Values(i)
        if value is None or value.Name() is None:
            raise SchemaError(f"invalid enum value at index {i}")
        out.append(f"{_text(value.Name())}={value.Value()},")
    out.append("}")


def _struct(schema, obj, out, depth, open_indices, visits):
    if (depth >= _MAX_DEPTH or visits[0] >= _MAX_STRUCT_VISITS
            or obj is None or not obj.IsStruct() or obj.Bytesize() <= 0
            or obj.Name() is None or obj.FieldsIsNone()):
        raise SchemaError("invalid or too-deep struct schema")
    visits[0] += 1
    bytesize = obj.Bytesize()
    out.append(f"struct {_text(obj.Name())} {bytesize} "
               f"{obj.Minalign()} {{")
    for i in range(obj.FieldsLength()):
        field = obj.Fields(i)
        if field is None or field.Type() is None or field.Name() is None:
            raise SchemaError(f"invalid field at index {i}")
        kind = field.Type()
        is_array = kind.BaseType() == BaseType.Array
        elem_base = kind.Element() if is_array else kind.BaseType()
        size = _value_size(schema, elem_base, kind.Index())
        count = kind.FixedLength() if is_array else 1
        offset = field.Offset()
        if (count == 0 or offset < 0 or offset > bytesize
                or size > (bytesize - offset) // count):
            raise SchemaError(
                f"field {_text(field.Name())} exceeds struct bounds")
        out.append(f"{_text(field.Name())}@{offset}:")
        if is_array:
            out.append("[")
            _type(schema, kind.Element(), kind.Index(), out, depth + 1,
                  open_indices, visits)
            out.append(f"*{kind.FixedLength()}]")
        else:
            _type(schema, kind.BaseType(), kind.Index(), out, depth + 1,
                  open_indices, visits)
        out.append(";")
    out.append("}")


def _find_struct_with_index(bfbs, name):
    if not isinstance(bfbs, (bytes, bytearray, memoryview)):
        raise SchemaError("not a reflection schema")
    data = bytes(bfbs)
    _Verifier(data).verify_schema()
    schema = Schema.GetRootAs(data, 0)
    if not isinstance(name, str):
        raise SchemaError("struct name must be a string")
    wanted = name.encode("utf-8")
    if schema.ObjectsIsNone():
        raise SchemaError(f"has no struct {name}")
    for i in range(schema.ObjectsLength()):
        obj = schema.Objects(i)
        if obj is not None and obj.Name() == wanted:
            if not obj.IsStruct():
                raise SchemaError(f"{name} is a table, not a struct")
            if obj.Bytesize() <= 0:
                raise SchemaError(f"{name} has non-positive bytesize")
            return schema, obj, i
    raise SchemaError(f"has no struct {name}")


def find_struct(bfbs, name):
    """Returns `(reflection.Schema, reflection.Object)` for struct `name` in
    .bfbs bytes, or raises SchemaError naming the reason when the bytes are not
    a reflection.Schema, name a table, or hold no struct of that name."""
    schema, obj, _ = _find_struct_with_index(bfbs, name)
    return schema, obj


def canonical_struct(bfbs, name):
    """The canonical text of struct `name` in .bfbs bytes (schema_hash.h),
    or None when the bytes are not a reflection.Schema or hold no struct of
    that name."""
    try:
        schema, obj, obj_index = _find_struct_with_index(bfbs, name)
        out = []
        _struct(schema, obj, out, 0, {obj_index}, [0])
    except SchemaError:
        return None
    return "".join(out)


def struct_hash(bfbs, name):
    """The schema hash of struct `name` ("control.VelocityTarget") in .bfbs
    bytes, or None when canonical_struct is None."""
    text = canonical_struct(bfbs, name)
    return None if text is None else _fnv1a_64(text.encode("utf-8"))
