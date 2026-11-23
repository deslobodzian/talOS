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
copied as uniform 8-byte relaxed atomic words (`std::atomic_ref<std::uint64_t>`
for the payload up to `publish_time_offset_`, zero-padding short writes and any
sub-8-byte tail, and `std::atomic_ref<std::int64_t>` for the timestamp) before
the same slot-validity check; an overwritten candidate exposes neither its
timestamp nor its metadata as an accepted message. Paired with the writer's
release fence before overwriting a slot and the reader's acquire fence before
checking `writer.sequence` and `reliable_sequence`, a concurrent overwrite of a
slot being copied is a well-defined seqlock free of C++ data races, so
`//talOS/rtms:ring_lap_test` runs under ThreadSanitizer on one mapping
([TESTING.md §2](../../docs/TESTING.md#where-a-behaviour-is-tested)). A reader the
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
That last queue marks the segment closing and records its own process as the
one that will unlink it, in one step. Among processes of one build (the
segment's format version does not tell builds apart here), a queue that opens
the name meanwhile waits for that unlink while the process lives, however long
it is descheduled, and never unlinks the segment in its place (it gives up
after about a second, a bound for a stopped process). If the process dies
before unlinking, or its unlink fails and it gives the record up, a later open
of the name, when no live process holds the segment, records itself in its
place and unlinks it; an open that recorded itself and then failed to unlink
fails rather than try again. A recorded process in another pid namespace, or
one checked by a process whose own namespace cannot be compared, counts as
running ([process liveness](process_liveness.h)): while it lives it unlinks the
segment as usual, but if it died or is stuck, such an open, a session's
included (below), waits and then fails until the name is unlinked. A segment
whose last holder was killed otherwise stays and is reused; every attach,
publisher or reader, refuses one of another version, layout or schema, which a
launcher session replaces or `shm_unlink` removes.

A session replaces the segment a name holds when it creates its own. A previous
session's segment, one that is not of this build's format (too small, no magic,
another version), or a name it cannot open or map, it unlinks. A standalone
segment it unlinks only as the one process recorded to unlink it, so that no
standalone queue's close or reclaim can unlink the name after the session's
segment is created: while the recorded process lives the session waits for it
to unlink the segment, as an open does; otherwise (queues still attached, or a
recorded process that died or gave the record up) it records itself and unlinks
the segment, then looks at the name again until it is free. It stops after 200
attempts to claim, or when its unlink fails; if the name then still holds a
segment, the launch fails with `RTMS: another process created <topic> while the
session was creating it`. Queues still attached keep using the unlinked
segment, and their close unlinks nothing. A session owner killed between
recording itself and unlinking, or whose unlink failed, leaves such a segment
closing with those queues still attached: while they hold its writer or a
reader slot, a standalone open of the name fails with the "stayed closing"
error, until they exit or the next session replaces it. A recorded process
still alive after the wait (about a second) fails the launch with `RTMS: a
session cannot replace <topic>: its claimant, pid <pid>, is still unlinking
it`, followed by the remedy: stop that pid or wait for it to exit, then start
the session again. When the session cannot check the recorded process (it is in
another pid namespace, or the session's `/proc` is not its own pid
namespace's), it counts as running and its pid may name another process: the
error then says `pid <pid>, which this process cannot check (another pid
namespace, or /proc is not this one's), counts as still unlinking it`, and the
remedy is to `shm_unlink` the name once that process is gone, then start the
session again.

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

Each topic has 32 reader slots (`MAX_READERS`), including observer
subscriptions. Dead readers are reclaimed before registration fails. The graph
lint rejects declared reader counts above this limit before launch; standalone
`ipc::Subscriber` construction and a realtime loop's watcher registration
throw with the topic and limit if exhausted. Every reader—prompt (woken) or
polled/observer—holds one of those 32 slots; RTMS does not implement a
separate process-local cursor-only path for never-woken readers while 32 slots
per topic hold real robot graphs comfortably.

A queue keeps the state word it claimed each reader slot with. Once the slot
holds another word -- another process judged this one dead and freed the slot,
which a process may since have claimed -- `read_next()` reports `INACTIVE`
rather than share a claimant's cursor, and `release_reader()` leaves the slot
alone. The [realtime event loop](../events/README.md) then
claims a slot again. All claims of one process carry the same word, so a slot
freed and claimed again by another queue of the same process before this one
reads goes unnoticed.

**Late join.** `register_reader()` starts a reader at the writer's next
publication, in both read modes: nothing the ring held when the reader
registered is ever handed to it, not even the newest message under
`ReadMode::LATEST`, and a slot claimed again after being freed starts there
too. Robot safety relies on this (REQ-SAFE-31): a node that starts or
restarts within `command_timeout_us` never acts on a target, Driver Station
packet or hardware state its previous run was sent
([ARCHITECTURE.md](../../ARCHITECTURE.md#targets)), so no topic kind starts a
reader at the retained latest message: a late reader waits for the next
publication. `LateJoin` in `test.cc` holds the rule.

## Wake channel

A reader need not poll a topic: it can sleep on a `talos::WakeReceiver`
(`wake.h`, `//talOS/rtms:wake`) until the topic's writer publishes. A
notification is only a hint to go read; what arrived, and whether anything
did, is still decided by the sequence numbers, so a spurious wake costs one
empty read and nothing else.

A receiver is a non-blocking, close-on-exec `AF_UNIX` datagram socket bound to
an address derived only from its random nonzero 64-bit id: on Linux the
abstract name `\0talos-wake-<16 hex digits>` (no filesystem entry), on macOS
`/tmp/talos-wake-<16 hex digits>` (`sun_path` holds 104 bytes, too few for
`TEST_TMPDIR`), unlinked when the receiver is destroyed. A receiver killed
before its destructor runs leaves its file; as a dead reader's slot is freed
when the next reader claims one, every new macOS receiver first removes each
`/tmp/talos-wake-*` socket file at least 10 s old (inode change time,
`kWakeFileReclaimAgeS`) whose connect is refused (`ReclaimDeadWakeFiles`).
The age matters because macOS's `bind()` creates the file before it attaches
the socket, and a connect in between is refused; a bound receiver accepts
every connect until its destructor unlinks the file. A receiver also checks,
after its bind, that its file is still there, and binds afresh with a new id
if it is not, so a receiver never lives without a file. A new receiver whose
address is taken draws another id. Its fd is readable while
notifications are pending;
`drain()` consumes them. A `talos::WakeSender` keeps one socket connected to
the last id it notified and sends one byte, never blocking or raising
`SIGPIPE`. `send()` reports a `WakeOutcome`: `DELIVERED` (also when a full
receiver queue means a wake is already pending), `RECEIVER_GONE` (a refused
or missing address), or `FAILED` with the errno (no descriptor left for the
socket, a permission refused, an unexpected error: the hint is lost though
the receiver may be there). `notify()` is `send()` as a bool, false only for
`RECEIVER_GONE`.

The protocol has two header words: `Reader::wake_id`, the receiver a reader
slot's holder sleeps on (`set_reader_wake()`), and `Writer::waiters`, bit *i*
set while slot *i* waits for the next publication.

- **Reader.** On waking: `drain()`, then read every topic sharing the receiver
  until `EMPTY`, then `arm_wake()` each; sleep only if every `arm_wake()`
  returned `true`. `arm_wake()` sets the slot's bit (a sequentially
  consistent read-modify-write, skipped while the bit is still set) and then
  loads the writer sequence (sequentially consistent), returning `false` when
  there is something to read or the slot is no longer held.
- **Writer.** After the publication's sequence store, itself sequentially
  consistent, it loads `waiters`. Zero, the common case, ends the write: no
  exchange and no system call. Otherwise it exchanges the word for 0 and
  notifies each set slot's `wake_id`, so one sleep of a reader costs the
  writer at most one send however many publications follow.
  `wakes_sent()` counts the sends of one queue object, and `wake_failures()`
  those of them that were `FAILED`; the first failure of a queue is also
  printed once on stderr with the topic and the error, and the count reaches
  the writing node's registry row
  ([wake loss](../introspection/README.md#wake-loss)). The reader it missed
  is drained by its loop's backstop
  ([talOS/events](../events/README.md#the-three-loops)). Each reader slot
  holds one connected socket in every process that writes the topic, so a
  topic costs a writer up to `MAX_READERS` (32) descriptors.
- **Why no wake is lost.** The reader stores its bit before loading the
  sequence; the writer stores the sequence before loading the bits. Under
  sequential consistency at least one sees the other: the reader finds the
  publication and does not sleep, or the writer finds the bit and wakes it.
- A `DROP_NEWEST` write refused with `BUFFER_FULL` publishes nothing and wakes
  nobody.

**Publish cost.** A publication no reader is armed for costs the same
whatever the table's size: one load of `waiters`. Otherwise the writer pays
one `send()` per armed reader; walking the table's 32 bits to find them is
below the timer's resolution. `//talOS/rtms:wake_fanout_bench` times
`write()` with each armed reader in its own process asleep on its receiver
(`bazel run -c opt //talOS/rtms:wake_fanout_bench`); on a Ryzen 9 7950X3D
limited to 8 CPUs, Linux 7.2, no realtime priority, load average 7 to 8 from
other builds, median over three runs of 3000 publications:

| Readers armed (slots held) | 0 (0 or 32) | 1 (1) | 8 (8) | 8 (32) | 32 (32) |
|---|---:|---:|---:|---:|---:|
| `write()` median | 0.04 µs | 1.1 µs | 7.2–7.5 µs | 7.4–7.9 µs | 33.5–33.9 µs |
| `write()` p99 | 0.04–0.08 µs | 1.8–2.0 µs | 9–11 µs | 12–14 µs | 55–58 µs |

The bench's walk rows (slots armed with no receiver: the exchange and the
walk, no send) take 0.04 to 0.05 µs with 8 or 32 of 32 slots armed. In
another run under heavier load the same host paid 5.6 to 5.9 µs per armed
reader (182 µs median for 32), so a send's cost depends on the host's state
more than on the table; even then 32 armed readers take under 4% of a 5 ms
tick. macOS is unmeasured. Only prompt watchers arm
([talOS/events](../events/README.md#the-three-loops)).

**Crash window.** A writer killed after exchanging `waiters` but before
sending leaves a reader asleep beside an unread message. Whichever writer next
claims the topic (a publisher opening it, replacing a dead owner, or a queue's
first write) therefore notifies every slot with a nonzero `wake_id`, armed or
not.

**Slot reuse.** `register_reader()` and `release_reader()` clear the slot's
`wake_id` and bit, so no writer notifies a receiver its holder may have closed
or wakes the slot's next holder for the previous one's arm. Reclaiming a dead
reader clears only the dead holder's `wake_id` (compare-and-swap); it leaves
the bit, which a new claimant may already have set, and a set bit with no
receiver costs a writer one exchange and no send.

## Memory layout

`RTMSHeader` (`rtms.h`) records slot geometry, schema identity, lifetime state,
writer ownership/sequence, reader positions and the wake channel's words. Publication timestamps are
stored separately from message payloads. The exact segment version, header
size, alignment and slot-stride formulas live in the
[data-flow reference](../../docs/DATA_FLOW.md#main-processor-shared-memory-slots).
