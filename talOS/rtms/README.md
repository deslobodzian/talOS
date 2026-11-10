# Real Time Messaging System (RTMS)

Single-producer, multi-consumer shared-memory ring buffers, one per topic.
Topic naming and message conventions live in
[NAMING.md](../NAMING.md) and [ARCHITECTURE.md](../../ARCHITECTURE.md).

Every publication also stores an `int64_t` monotonic timestamp in its slot,
separate from the message schema. `write(bytes)` samples the host clock used by
the event Poller (`CLOCK_MONOTONIC_RAW` on macOS, `CLOCK_MONOTONIC` elsewhere).
`write(bytes, publish_time_ns)` preserves the supplied nanoseconds for simulation
and replay. `read_next()` returns that timestamp as `MessageInfo::publish_time_ns`
alongside the sequence and dropped-message count. The payload and timestamp are
copied before the same slot-validity check; an overwritten candidate exposes
neither its timestamp nor its metadata as an accepted message. A reader the
writer laps skips to the oldest message still intact and counts the skipped ones
as dropped, whatever overflow policy the reader assumed.

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
If that last queue's process dies after marking the segment closing but before
unlinking it, the next queue to open the name waits briefly for the unlink and
then, when no live process holds the segment, unlinks it itself.

Publishers (`RTMSOptions::publisher`) atomically claim the header's writer when
opening. A second live publisher, including another publisher in the same
process, is refused with the topic and both pids. A dead publisher's claim is
reclaimed on open, allowing crash restarts without disconnecting live readers.
Closing the publisher releases its claim.
Low-level queues opened without `publisher` claim ownership on their first
write, so omitting that option cannot bypass the single-writer guard.

A writer or reader claim records its process as the pid plus the process start
time (`/proc/<pid>/stat` on Linux, `sysctl(KERN_PROC_PID)` on macOS). The claim
is dead once no process with that pid runs, the pid's process is a zombie, or
the pid now belongs to a process that started at another time, so a pid reused
after a crash does not keep the crashed process's claims.

Each topic has eight reader slots (`MAX_READERS`), including observer
subscriptions. Dead readers are reclaimed before registration fails. The graph
lint rejects declared reader counts above this limit before launch; standalone
`ipc::Subscriber` construction throws with the topic and limit if exhausted.

## Memory layout

`RTMSHeader` (`rtms.h`) records slot geometry, schema identity, lifetime state,
writer ownership/sequence and reader positions. Publication timestamps are
stored separately from message payloads. The exact segment version, header
size, alignment and slot-stride formulas live in the
[data-flow reference](../../docs/DATA_FLOW.md#main-processor-shared-memory-slots).
