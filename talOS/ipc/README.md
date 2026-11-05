# IPC

Typed single-producer / multi-consumer pub/sub on top of RTMS
(`//talOS/rtms`: lock-free shared-memory ring buffer per topic,
broadcast under `/dev/rtms/<topic_name>`).

## Contract

- `ipc::Publisher<Message>` / `ipc::Subscriber<Message>`
  (`publisher.h`, `subscriber.h`) are thin typed front-ends over
  `RTMSQueue`. One topic maps to one ring buffer sized
  `sizeof(Message)` x `MAX_SLOTS`.
- `Message` must be a trivially copyable flatbuffer **struct**
  (enforced by `concepts.h`: `NotDerivedFromFlatbufferTable` plus
  `static_assert(is_trivially_copyable)`) with a schema descriptor
  (`talos::Described<Message>`: a `talos_messages` struct, or a framework
  layout such as `hardware::Packet` or `TargetEnvelope<M>`). Tables are
  rejected so the slot size is known at compile time. The segment records
  the creator's schema name and hash; an endpoint of another schema fails
  to attach, naming both types (talOS/rtms/rtms.h).
- Default policy is `OVERWRITE_OLDEST` with `SEQUENCE` reads: a
  subscriber that stops reading gets lapped instead of stalling the
  publisher. `Subscriber::read_next()` returns
  `Received<Message>{message, sequence, dropped}` so consumers can
  detect laps and replay deterministically; `read()` is the plain
  latest-value shortcut.
- The publisher owns the topic layout outside a launcher session: it
  passes `RTMSOptions::publisher`, so it reclaims a segment of another
  build or schema that no live process holds. A live publisher exclusively
  owns the topic; another publisher throws with the topic and both pids.
  Each `Subscriber` registers one of eight shared-memory reader slots and
  throws with the topic and limit if they are exhausted. It is move-only
  (copying would double-release the slot), and releases it on destruction;
  a slot whose process died is reclaimed. Segment lifetime (launcher-owned,
  or the last holder unlinks) is RTMS's: see `talOS/rtms/rtms.h`.
- Test schemas live next to their users (e.g. `ipc_test_message.fbs`,
  built with `talos_messages`).

## Who uses it

- `//talOS/bridge` (`talOS/bridge/node.h`) — node graph transport.
- `//talOS/events` — realtime event loop publishing.
- `//talOS/driver_station:send_joystick` and
  `//2026-robot/main_processor/drivetrain:send_target` publish;
  `drivetrain:monitor` subscribes.
- Unit coverage in `:ipc_test`.
