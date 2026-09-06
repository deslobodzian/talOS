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
  `static_assert(is_trivially_copyable)`). Tables are rejected so the
  slot size is known at compile time.
- Default policy is `OVERWRITE_OLDEST` with `SEQUENCE` reads: a
  subscriber that stops reading gets lapped instead of stalling the
  publisher. `Subscriber::read_next()` returns
  `Received<Message>{message, sequence, dropped}` so consumers can
  detect laps and replay deterministically; `read()` is the plain
  latest-value shortcut.
- The publisher owns the topic layout: it passes
  `reclaim_mismatched_segment = true`, so it clears a segment left
  behind by an older binary. Each `Subscriber` registers one
  shared-memory reader slot, is move-only (copying would double-release
  the slot), and releases it on destruction.
- Flatbuffer `.fbs` schemas live next to their users (e.g.
  `ipc_test_message.fbs` for tests).

## Who uses it

- `//talOS/bridge` (`talOS/bridge/node.h`) — node graph transport.
- `//talOS/events` — realtime event loop publishing.
- `//2026-robot/main_processor/...` — `driver_station/send_joystick`,
  `drivetrain/send_target` publish; `drivetrain/monitor` subscribes.
- Local demos: `:pub_process` / `:sub_process` binaries, exercised by
  `tools/test_ipc.sh`; unit coverage in `:ipc_test`.

## Fold decision

`//talOS/ipc` stays a separate package: it is the *typed* layer,
`//talOS/rtms` the untyped transport. If the two ever merge, the
`Publisher`/`Subscriber` templates move into the `rtms` package and
this directory goes away. No fold is scheduled — this note replaces
the old "will likely move to RTMS" stub.
