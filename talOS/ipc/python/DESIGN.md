# Python IPC client design

## Wrapper rule: languages bind, never reimplement

The C++ loop, the RTMS shared-memory layout, and the describe JSON envelope
are the ground truth. Python must not duplicate any of them.

* `node_api.py` is the only Python door into the transport. It is a `ctypes`
  binding over the exact `extern "C"` ABI in `talOS/node_api/node_api.h`
  (14 exports: the 10 transport/describe functions plus
  `talos_node_register`, `talos_node_publish`, `talos_node_heartbeat`,
  `talos_node_close`). It loads
  `bazel-bin/talOS/node_api/libtalos_node.so` (override with `TALOS_NODE_LIB`,
  or via test runfiles).
* `rtms.py` keeps the historic public names (`Publisher`, `Subscriber`,
  `RTMSQueue` with `register_reader`/`read_next`, `shm_object_name`,
  `align_up`, policy strings, frozen-layout constants) but every live byte
  flows through a `talos_*` call. No `mmap`/`struct` slot arithmetic survives
  on the live path; the `OFF_*` constants are documentation values only.
* `describe_json(name, target, sources)` takes
  `(kind_int, topic, message_bytes, external, optional)` rows, maps kinds
  1..4 and the external/optional flag bits onto `TalosSource`, and returns
  the JSON string from `talos_describe_emit`. Python never hand-formats the
  envelope, so a schema change lands in C++ once. This signature is a
  cross-track contract: do not drift.
* `Node(name, target, session_id, flags, sources)` claims a registry slot
  via `talos_node_register`, publishes the same rows via
  `talos_node_publish`, and `heartbeat(dispatches)` samples once per tick —
  the same begin/heartbeat/dispatch/end bracketing as the C++ Reporter, so
  Studio's live graph sees Python nodes through the identical reader path.
  Unavailable registry degrades to heartbeat no-ops (stderr once), never an
  abort. `FLAG_SIMULATION`/`FLAG_REPLAY` mirror the Reporter bits.

## Additive-only ABI policy

Mirrored from `node_api.h`: never renumber, remove, or re-signature an
existing export; never change `TalosSource` layout or the meaning of a
`TalosSourceKind`/status code; only ADD functions, kinds, or flags.
`talos_abi_version()` is bumped on every addition, and `node_api.py`
refuses a library older than `TALOS_NODE_ABI_VERSION`.

## Live-path API deltas (vs the pre-wrapper client)

* `write` pads short payloads with zeros (the old zero-filled slot did this
  implicitly); oversized payloads still raise `ValueError`. The strict
  exact-size check lives in `node_api`; the padding lives in `rtms`.
* `write` returns `None`: the sequence counter is C++-owned and has no ABI
  export. Sequences are still reported by `read_next`.
* Per-call `overflow_policy` other than `OVERWRITE_OLDEST` raises
  `ValueError`: the C library bakes that policy and per-call selection has
  no ABI. `read_mode` `SEQUENCE` is one poll; `LATEST` drains to newest.
* `writer_sequence` raises `NotImplementedError` (no ABI export).
* `slots` is validated (power of two >= 2) but layout is C++-owned; the
  library always uses `MAX_SLOTS`.
* `_ShmSegment` keeps only `unlink` (test cleanup); mapping is C++-owned.

## Test-only in-memory path

The committed round-trip test builds queues via `RTMSQueue.__new__` with a
bytearray `_seg` to exercise protocol formulas without shared memory. When
`_seg` is present, `rtms.RTMSQueue` serves calls from a counter/dict
emulation (`_init_header`, `_select_sequence`, `_fake_read`, ...). It holds
no shm struct math and is never used by real queues.

## Owner commands

```sh
bazel build //talOS/node_api:libtalos_node.so
bazel test //talOS/ipc/python/...
bazel test //talOS/node_api/...
```
