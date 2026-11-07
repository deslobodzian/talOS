# Real Time Messaging System (RTMS)

This will be a single producer, mutlti consumer architecture pub-sub system.

Utilizing a Ring Buffer in shared memory for each topic. Goal is to be lock free.

## Nameing scheme

All topics will be broadcasted under `"/dev/rtms/topic_name"`

## Messages

All messages will use FlatBuffers for serialization.
FlatBuffers **must** be structs! Tables are not allowed so the slot size is
known at compile time.

Every publication also stores an `int64_t` monotonic timestamp in its slot,
separate from the message schema. `write(bytes)` samples the host clock used by
the event Poller (`CLOCK_MONOTONIC_RAW` on macOS, `CLOCK_MONOTONIC` elsewhere).
`write(bytes, publish_time_ns)` preserves the supplied nanoseconds for simulation
and replay. `read_next()` returns that timestamp as `MessageInfo::publish_time_ns`
alongside the sequence and dropped-message count. The payload and timestamp are
copied before the same slot-validity check; an overwritten candidate exposes
neither its timestamp nor its metadata as an accepted message.

## Schema identity

The segment's creator records its schema name and 64-bit schema hash in the
header (`talOS/schema/identity.h`). Every subsequent attach checks both; a
mismatch fails, naming both types even if message sizes match.

## Segment lifetime

Under the launcher, a session owns every declared topic: the launcher creates
each segment (`session_owner = true`) before spawning any nodes and unlinks it
when the session ends. Nodes only attach, so a node that restarts or exits
cleanly leaves its topics in place without disrupting subscribers.

Without the launcher (tests, CLI tools), the first queue to open a name creates
the segment and the last one to close unlinks it. Dead readers are reclaimed.

Publishers (`RTMSOptions::publisher`) atomically claim the header's writer pid
when opening. A second live publisher, including another publisher in the same
process, is refused with the topic and both pids. A dead publisher's claim is
reclaimed on open, allowing crash restarts without disconnecting live readers.
Closing the publisher releases its claim.
Low-level queues opened without `publisher` claim ownership on their first
write, so omitting that option cannot bypass the single-writer guard.

Each topic has eight reader slots (`MAX_READERS`), including observer
subscriptions. Dead readers are reclaimed before registration fails. The graph
lint rejects declared reader counts above this limit before launch; standalone
`ipc::Subscriber` construction throws with the topic and limit if exhausted.

## Memory

Each ring buffer is stored with a corresponding header in shared memory:

`[rtms_header][data...]`

### RTMS Header

`RTMSHeader` (`talOS/rtms/rtms.h`) defines the state of the shared memory:
- `slots`: Number of slots in the ring buffer (power of 2).
- `message_bytes`: Size of the message struct in bytes.
- `message_alignment`: Message struct alignment.
- `schema_hash` and `schema_name`: Identity of the message schema.
- `lifetime`: Atomic bitflag and refcount tracking session ownership and attached queues.
- `writer`: `Writer` tracking sequence and owner pid.
- `readers`: Array of `Reader` structs tracking reader sequences and pids.

### Ring slots (segment version 5)

For payload size `M` and message alignment `A`, each slot is:

`[M payload bytes][padding to 8-byte boundary][int64 publish_time_ns][tail padding]`

- Header size: 832 bytes (`sizeof(RTMSHeader)`, compile-time asserted).
- Slot alignment: `max(A, alignof(int64_t))` (at least 8 bytes).
- Timestamp offset: `align_up(M, 8)`.
- Slot stride: `align_up(align_up(M, 8) + 8, max(A, 8))`.
- Data offset: `align_up(sizeof(RTMSHeader), max(A, 8))`.
- Segment bytes: `data_offset + slots * slot_stride` (before OS page rounding).

An 8-byte payload aligned to 4 or 8 takes a 16-byte slot; a 16-byte payload
aligned to 8 takes 24 bytes; aligned to 16 it takes 32 bytes. These sizes are
compile-time asserted in `rtms.h`. Version 4 segments lack publication timestamps
and cannot be attached by version 5 readers.
