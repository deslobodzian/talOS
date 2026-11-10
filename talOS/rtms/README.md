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
then, when no live process holds the segment, unlinks it itself. A segment
whose last holder was killed otherwise stays and is reused; every attach,
publisher or reader, refuses one of another version, layout or schema, which a
launcher session replaces or `shm_unlink` removes.

Publishers (`RTMSOptions::publisher`) atomically claim the header's writer when
opening. A second live publisher, including another publisher in the same
process, is refused with the topic and both pids. A dead publisher's claim is
reclaimed on open, allowing crash restarts without disconnecting live readers.
Closing the publisher releases its claim.
Low-level queues opened without `publisher` claim ownership on their first
write, so omitting that option cannot bypass the single-writer guard.

A writer or reader claim records its process as the pid plus the low 20 bits
of the process start time (`/proc/<pid>/stat` clock ticks on Linux, wrapping
every 2.9 hours; `sysctl(KERN_PROC_PID)` milliseconds on macOS, every 17.5
minutes). The claim is dead once no process with that pid runs, the pid's
process is a zombie, or the pid now belongs to a process that started at
another time, so a pid reused after a crash does not keep the crashed
process's claims; only a reuse a whole number of wraps later, to the tick or
millisecond, about one in a million, would.

A claim also records its pid namespace (Linux, the inode of
`/proc/self/ns/pid`): a process in another pid namespace sharing `/dev/shm` (a
container run with `--ipc=host`) cannot look the pid up, so to it the claim
counts as live and stays until the segment is unlinked. The kernel numbers
that inode from the allocator that also numbers every other namespace and
every dynamic `/proc` entry (per-IRQ files, each network namespace's
`/proc/net`), lowest free first, so it grows with the host's `/proc` rather
than its pid namespaces; the 20-bit code is exact up to about a million such
entries. Past that the code is opaque, and so is a process's own whenever its
`/proc` is not its own pid namespace's (`NSpid` in `/proc/self/status`). A
process whose own code is opaque reclaims no claim for its whole session: a
crashed publisher's restart is refused and dead readers' slots stay held. The
check is `:process_liveness` (`process_liveness.h`), which the live node
registry also uses for its rows.

Each topic has eight reader slots (`MAX_READERS`), including observer
subscriptions. Dead readers are reclaimed before registration fails. The graph
lint rejects declared reader counts above this limit before launch; standalone
`ipc::Subscriber` construction throws with the topic and limit if exhausted.

A queue keeps the state word it claimed each reader slot with. Once the slot
holds another word -- another process judged this one dead and freed the slot,
which a process may since have claimed -- `read_next()` reports `INACTIVE`
rather than share a claimant's cursor, and `release_reader()` leaves the slot
alone. The [realtime event loop](../events/README.md) then
claims a slot again. All claims of one process carry the same word, so a slot
freed and claimed again by another queue of the same process before this one
reads goes unnoticed.

## Memory layout

`RTMSHeader` (`rtms.h`) records slot geometry, schema identity, lifetime state,
writer ownership/sequence and reader positions. Publication timestamps are
stored separately from message payloads. The exact segment version, header
size, alignment and slot-stride formulas live in the
[data-flow reference](../../docs/DATA_FLOW.md#main-processor-shared-memory-slots).
