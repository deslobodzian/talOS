# IPC

Typed single-producer / multi-consumer pub/sub on top of
[RTMS](../rtms/README.md).

## Contract

- `ipc::Publisher<Message>` / `ipc::Subscriber<Message>`
  (`publisher.h`, `subscriber.h`) are thin typed front-ends over
  `RTMSQueue`, passing `sizeof(Message)`, `alignof(Message)` and `MAX_SLOTS`.
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
  detect laps; `read()` discards that metadata but uses the same read mode
  (sequence delivery by default), not a latest-value shortcut.
- The publisher owns the topic layout outside a launcher session: it
  passes `RTMSOptions::publisher`, so it reclaims a segment of another
  build or schema that no live process holds. A live publisher exclusively
  owns the topic; another publisher throws with the topic and both pids.
  Each move-only `Subscriber` owns and releases a reader slot; construction
  throws if reader slots are exhausted. Transport ownership, reader capacity
  and segment lifetime are documented in [RTMS](../rtms/README.md).
- Test schemas live next to their users (e.g. `ipc_test_message.fbs`,
  built with `talos_messages`).

- Unit coverage in `:ipc_test`.
