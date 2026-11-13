# Schema

FlatBuffers schema identity, unit linting, and reflection decoding.

- **Descriptors:** `talos_messages` emits a binary reflection schema (`.bfbs`)
  per `.fbs` file, verifies numeric field unit suffixes
  ([methodology](../../docs/METHODOLOGY.md)),
  and generates `talos::Descriptor<M>` (`kName`, `kHash`, `kBfbs`;
  `descriptor.h`) and `TALOS_SCHEMA_NAME`/`HASH`/`BFBS` Python attributes.
- **Identity:** `schema::Identity` (`identity.h`) bundles schema name, 64-bit
  hash, and optional `.bfbs` reflection bytes. `schema::Of<M>()` obtains
  identity for any described message.
- **Hashing:** `struct_hash` (`schema_hash.h`, `talos_schema.struct_hash`)
  computes an FNV-1a 64 hash of canonical field definitions, types, offsets,
  and layout from reflection data.
- **Decoding:** `//talOS/schema:decode` (`decode.h`, `DecodeToText`) parses
  raw binary payloads into named fields using reflection schema bytes without
  compile-time types.
- **Field paths:** `//talOS/schema:field_path` (`field_path.h`) resolves a
  slash-separated path (`inner/offset_m`) through nested structs of a
  reflection schema to the leaf scalar's offset, type, size and enum
  (`ResolveField`, with `ScalarPaths` listing every path), encodes a bool,
  number or enum label into a message at it with range checks
  (`EncodeField`, `EncodeNumber`), and reads it back per tick without
  allocating (`ReadDouble`, `ReadInt64`, `ReadBool`). The teleop node encodes
  its TOML bindings with it; the telemetry node reads its channels with it.
