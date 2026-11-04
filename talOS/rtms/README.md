# Real Time Messaging System (RTMS)

This will be a single producer, mutlti consumer architecture pub-sub system.

Utilizing a Ring Buffer in shared memory for each topic. Goal is to be lock free.

## Nameing scheme

All topics will be broadcasted under `"/dev/rtms/topic_name"`

## Messages

All messages will use FlatBuffers for serialization.
FlatBuffers **must** be structs! Tables are not allowed so the slot size is
known at compile time.

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
