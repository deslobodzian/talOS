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
#define TALOS_NODE_ABI_VERSION 1u

// Opaque handles. NULL means "open failed; see talos_last_error()".
typedef struct TalosPublisher TalosPublisher;
typedef struct TalosSubscriber TalosSubscriber;

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
  TALOS_OK = 0,        // Success (publish ok, describe written).
  TALOS_EMPTY = 1,     // poll_next: caught up, nothing new.
  TALOS_ERR = -1,      // Generic failure; see talos_last_error().
  TALOS_ERR_ARG = -2,  // Bad argument (NULL, empty topic, bad size/align...).
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

#ifdef __cplusplus
}  // extern "C"
#endif
