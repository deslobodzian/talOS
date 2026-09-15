#pragma once

// talOS node_api: shared C++ ground-truth library for non-C++ nodes.
//
// A non-C++ node (Python today, others later) must speak exactly the same
// transport and describe envelope as a C++ node. The C++ loop, RTMS layout
// and describe JSON are the ground truth; this header is the only stable
// door into them. Python must pass raw source arrays and let
// talos_describe_emit build the envelope via
// introspect::DescribeManifest/DescribeToJson -- it must never hand-format
// the JSON itself, so a schema change lands here once instead of in every
// client.
//
// ABI STABILITY RULE (additive-only):
//   - Never renumber, remove, or re-signature an existing exported symbol.
//   - Never change the layout of TalosSource or the meaning of an existing
//     TalosSourceKind / status code.
//   - Only ADD new functions, new kinds, or new flags; old callers linked
//     against an earlier libtalos_node.so must keep working against a newer
//     one. talos_abi_version() is bumped whenever anything is added, and a
//     caller must refuse to run when the runtime version is older than the
//     version it was built against (never the reverse).
//   - No exceptions cross this boundary: every entry point catches all
//     exceptions internally and reports failure via its return value, with
//     details in talos_last_error().

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Bumped on every additive ABI change. Starts at 1.
#define TALOS_NODE_ABI_VERSION 6u

// Opaque handles. NULL means "open failed; see talos_last_error()".
typedef struct TalosPublisher TalosPublisher;
typedef struct TalosSubscriber TalosSubscriber;
// Owns one registry slot (released on close). NULL means the registry was
// unavailable; the caller keeps running degraded, like the C++ Reporter.
typedef struct TalosNode TalosNode;
// Parsed robot configuration.
typedef struct TalosConfig TalosConfig;
// Event loop instance (realtime, simulated, or replay).
typedef struct TalosLoop TalosLoop;

// Context passed to dispatch callbacks.
typedef struct TalosDispatchContext {
  uint16_t kind;
  uint16_t source_id;
  uint64_t dispatch_index;
  int64_t event_time_ns;
  int64_t now_ns;
  uint64_t sequence;
  uint64_t dropped;
} TalosDispatchContext;

// Callback signature for timer and watcher event dispatches.
typedef void (*TalosDispatchFn)(const TalosDispatchContext* context,
                                const void* data, uint32_t size,
                                void* user_data);

// Source kinds. Values mirror talos::event::SourceKind on purpose
// (TIMER=1, WATCHER=2, FETCHER=3, SENDER=4) so the mapping is mechanical.
typedef enum TalosSourceKind {
  TALOS_SOURCE_TIMER = 1,
  TALOS_SOURCE_WATCHER = 2,
  TALOS_SOURCE_FETCHER = 3,
  TALOS_SOURCE_SENDER = 4
} TalosSourceKind;

// One describe source, as the caller declares it. `topic` is a NUL-terminated
// topic path (or timer label for TIMER). `flags` uses the naming-protocol
// bits (kSourceFlagExternal = 1<<0, kSourceFlagOptional = 1<<1); 0 is fine.
typedef struct TalosSource {
  int32_t kind;
  const char* topic;
  uint32_t message_bytes;
  uint32_t flags;
} TalosSource;

// Status codes shared by the int-returning entry points.
enum {
  TALOS_OK = 0,         // Success (publish ok, describe written).
  TALOS_EMPTY = 1,      // poll_next: caught up, nothing new.
  TALOS_ERR = -1,       // Generic failure; see talos_last_error().
  TALOS_ERR_ARG = -2,   // Bad argument (NULL, empty topic, bad size/align...).
  TALOS_ERR_SMALL = -3  // Describe buffer too small; out_written has need.
};

// ABI version this library was built against.
uint32_t talos_abi_version(void);

// Monotonic nanoseconds (CLOCK_MONOTONIC). Never throws.
int64_t talos_monotonic_ns(void);

// Last error on this thread, NUL-terminated; valid until the next node_api
// call on this thread. Never NULL, never throws.
const char* talos_last_error(void);

// Open a topic for publishing raw fixed-size messages. message_bytes and
// alignment must match on both ends (they define the shm layout).
// Returns NULL on failure.
TalosPublisher* talos_topic_open_publisher(const char* topic,
                                           uint32_t message_bytes,
                                           uint32_t alignment);

// Copy `size` bytes into the topic. size must equal the message_bytes given
// at open. Returns TALOS_OK or a negative TALOS_ERR_*.
int32_t talos_publish(TalosPublisher* publisher, const void* data,
                      uint32_t size);

// Close a publisher opened above. NULL is a no-op. Never throws.
void talos_publisher_close(TalosPublisher* publisher);

// Open a topic for subscribing. Same layout contract as the publisher.
// Returns NULL on failure.
TalosSubscriber* talos_topic_open_subscriber(const char* topic,
                                             uint32_t message_bytes,
                                             uint32_t alignment);

// Copy the next message into out_data (capacity out_size, must be >=
// message_bytes). On success writes *out_sequence/*out_dropped (may be NULL).
// Returns TALOS_OK (message copied), TALOS_EMPTY (caught up), or negative.
int32_t talos_poll_next(TalosSubscriber* subscriber, void* out_data,
                        uint32_t out_size, uint64_t* out_sequence,
                        uint64_t* out_dropped);

// Close a subscriber opened above. NULL is a no-op. Never throws.
void talos_subscriber_close(TalosSubscriber* subscriber);

// Build the --describe envelope for (node_name, target, sources) using
// introspect::DescribeManifest/DescribeToJson. Writes NUL-terminated JSON
// into out_json (capacity out_capacity) and sets *out_written to bytes
// written excluding the NUL. With out_json==NULL or capacity too small,
// sets *out_written to bytes needed (excluding NUL) and returns
// TALOS_ERR_SMALL. Returns TALOS_OK or a negative TALOS_ERR_*.
int32_t talos_describe_emit(const char* node_name, const char* target,
                            const TalosSource* sources, uint32_t num_sources,
                            char* out_json, uint32_t out_capacity,
                            uint32_t* out_written);

// Claim a registry slot for (name, target, session_id, flags) so Studio's
// live graph sees this node. flags mirrors the C++ Reporter bits
// (bit 0 simulation, bit 1 replay). Publishes nothing yet; call
// talos_node_publish once with the same sources --describe reports.
// Returns NULL (with last_error set) when the registry is unavailable --
// keep running degraded, do not abort the node.
TalosNode* talos_node_register(const char* name, const char* target,
                               uint64_t session_id, uint32_t flags);

// Publish the node's source manifest into its slot. Same sources array shape
// as talos_describe_emit. Call once, after registration.
int32_t talos_node_publish(TalosNode* node, const TalosSource* sources,
                           uint32_t num_sources);

// One refresh sample: heartbeat, dispatch count and (zeroed) source counters
// move together behind the registry seqlock. Call once per loop tick with a
// monotonically increasing dispatches count.
int32_t talos_node_heartbeat(TalosNode* node, uint64_t dispatches);

// Release the slot. NULL is a no-op. Never throws.
void talos_node_close(TalosNode* node);

// Frozen RTMS shared-memory geometry, probed from the ground-truth structs
// with sizeof/offsetof (never copied by hand). Added in ABI 3 so wrapper
// languages read the layout instead of restating it.
typedef struct TalosRtmsLayout {
  uint32_t header_size;     // sizeof(RTMSHeader)
  uint32_t off_writer_seq;  // writer.sequence offset within the segment
  uint32_t off_readers;     // readers[] offset within the segment
  uint32_t reader_stride;   // sizeof one Reader slot
} TalosRtmsLayout;

// Fill *out with the frozen geometry. Returns bytes written (nonzero) or 0
// when out is NULL. Never throws.
uint32_t talos_rtms_layout(TalosRtmsLayout* out);

// State-gating predicates over scalar fields, added in ABI 4 so wrapper
// languages bind the shared spelling in talOS/hardware/gate.h instead of
// re-spelling the checks. Pure functions: they never fail, never touch
// shared memory, and never throw. Each returns 1 (true) or 0 (false).

// Whether the `next` sample should replace the last accepted one. `prev_*`
// are ignored when have_prev is 0. Mirrors hardware::AcceptState: every
// sample is dropped when it carries another config id (zero accepts any,
// for standalone and simulation runs); beyond that, the first sample is
// accepted and later ones must not be stale or duplicate samples from the
// same boot.
int32_t talos_accept_state(uint64_t config_id, int32_t have_prev,
                           uint64_t prev_boot_id, uint64_t prev_sample_time_us,
                           uint64_t next_config_id, uint64_t next_boot_id,
                           uint64_t next_sample_time_us);

// Whether the gateway is ready to be commanded: configured and enabled,
// with no hardware fault latched. Mirrors hardware::GatewayReady over the
// flags word.
int32_t talos_gateway_ready(uint32_t flags);

// Whether a latched target may drive, over integer nanoseconds. Mirrors
// hardware::TargetFresh: enabled, issued no later than now, and both the
// target and the last accepted state younger than the timeout. Strict on
// both timeouts (`<`, not `<=`): a target exactly one timeout old is
// already gone.
int32_t talos_target_fresh_ns(int64_t now_ns, int64_t issued_ns,
                              int32_t enabled, int64_t state_rx_ns,
                              uint64_t timeout_us);

// Load and parse robot configuration via talos::config::ParseRobotConfig.
// With simulation != 0, sets hardware.commissioned = true. Returns NULL on
// error (see talos_last_error()).
TalosConfig* talos_config_load(const char* config_path, int32_t simulation);

// Compute configuration ID over config->robot.hardware. Returns 0 on NULL.
uint64_t talos_config_get_id(const TalosConfig* config);

// Gateway period in microseconds. Returns 0 on NULL.
uint32_t talos_config_get_period_us(const TalosConfig* config);

// Gateway command timeout in microseconds. Returns 0 on NULL.
uint32_t talos_config_get_command_timeout_us(const TalosConfig* config);

// Search subsystem's devices in robot.GetDevices(subsystem) matching device_name
// across motors, sensors, digital_inputs, digital_outputs, analog_inputs,
// encoders, pwm_outputs. Returns logical ID (uint32_t) if found, or 0 if not
// found (with talos_last_error set).
uint32_t talos_config_get_device_id(const TalosConfig* config,
                                    const char* subsystem,
                                    const char* device_name);

// Close a configuration handle opened above. NULL is a no-op. Never throws.
void talos_config_close(TalosConfig* config);

// --- Native Event Loop Bridge (ABI v6) ---

// Create a realtime event loop. If log_path is non-NULL and non-empty, records
// dispatches via LogWriter with stream_name and session_id. Otherwise uses
// NullRecorder. Returns NULL on failure (see talos_last_error()).
TalosLoop* talos_loop_create_realtime(const char* log_path,
                                     const char* stream_name,
                                     uint64_t session_id);

// Create a simulated event loop on a virtual clock. If log_path is non-NULL
// and non-empty, records dispatches via LogWriter with stream_name. Otherwise
// uses NullRecorder. Returns NULL on failure (see talos_last_error()).
TalosLoop* talos_loop_create_simulated(const char* log_path,
                                      const char* stream_name);

// Create a replay event loop from a recorded dispatch log at replay_path.
// Returns NULL on failure (see talos_last_error()).
TalosLoop* talos_loop_create_replay(const char* replay_path);

// Register a timer on the loop. If period_ns > 0 or offset_ns > 0, the timer
// is armed at loop.start_time() + offset_ns with period_ns. Returns timer id
// or 0xFFFF on failure.
uint16_t talos_loop_register_timer(TalosLoop* loop, const char* name,
                                   uint64_t period_ns, uint64_t offset_ns,
                                   TalosDispatchFn fn, void* user_data);

// Register a message watcher on the given topic. Returns watcher source id or
// 0xFFFF on failure.
uint16_t talos_loop_register_watcher(TalosLoop* loop, const char* topic,
                                     uint32_t message_bytes,
                                     uint32_t alignment, TalosDispatchFn fn,
                                     void* user_data);

// Register a message sender on the given topic. Returns sender source id or
// 0xFFFF on failure.
uint16_t talos_loop_register_sender(TalosLoop* loop, const char* topic,
                                    uint32_t message_bytes,
                                    uint32_t alignment);

// Arm an existing timer with the given period and offset from current time.
// Returns TALOS_OK or negative on error.
int32_t talos_loop_arm_timer(TalosLoop* loop, uint16_t timer_id,
                             uint64_t period_ns, uint64_t offset_ns);

// Disarm an existing timer. Returns TALOS_OK or negative on error.
int32_t talos_loop_disarm_timer(TalosLoop* loop, uint16_t timer_id);

// Send a message on a registered sender source. Returns TALOS_OK on success
// or negative on error / transport rejection.
int32_t talos_loop_send(TalosLoop* loop, uint16_t sender_id, const void* data,
                        uint32_t size);

// Monotonic now in nanoseconds as observed by the loop.
int64_t talos_loop_now_ns(const TalosLoop* loop);

// Run the loop until talos_loop_exit() is called (or until end of replay log).
// Returns TALOS_OK or negative on error.
int32_t talos_loop_run(TalosLoop* loop);

// Run the loop for duration_ns nanoseconds. Returns TALOS_OK or negative.
int32_t talos_loop_run_for(TalosLoop* loop, int64_t duration_ns);

// Request the loop to exit. Safe from a handler or another thread.
void talos_loop_exit(TalosLoop* loop);

// Whether replay execution diverged from the recorded log (1 = true, 0 = false).
int32_t talos_loop_diverged(const TalosLoop* loop);

// Whether replay execution reached the clean exit record (1 = true, 0 = false).
int32_t talos_loop_reached_exit(const TalosLoop* loop);

// Cumulative dispatch count executed by the loop.
uint64_t talos_loop_dispatch_count(const TalosLoop* loop);

// Inject a message into a simulated channel from outside the loop.
// Returns TALOS_OK or negative on error.
int32_t talos_loop_sim_inject(TalosLoop* loop, const char* topic,
                              const void* data, uint32_t size);

// Attach an introspection reporter to a realtime loop. Returns TALOS_OK or
// negative on error.
int32_t talos_loop_attach_reporter(TalosLoop* loop, const char* name,
                                   const char* target, uint64_t session_id,
                                   uint32_t flags);

// Destroy and release the loop instance. NULL is a no-op.
void talos_loop_destroy(TalosLoop* loop);

#ifdef __cplusplus
}  // extern "C"
#endif
