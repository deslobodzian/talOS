#include "talOS/node_api/node_api.h"

#include <time.h>

#include <cstring>
#include <exception>
#include <optional>
#include <string>
#include <utility>

#include "talOS/events/manifest.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"
#include "talOS/ipc/publisher.h"
#include "talOS/rtms/rtms.h"

namespace {
thread_local std::string g_last_error;

void SetError(const char* what) {
  g_last_error = (what != nullptr) ? what : "unknown error";
}
void SetErrorStr(const std::string& what) { g_last_error = what; }
void ClearError() { g_last_error.clear(); }

bool ValidLayout(uint32_t message_bytes, uint32_t alignment) {
  if (message_bytes == 0 || alignment == 0) return false;
  // Power of two alignment, as RTMS aligns slots by it.
  return (alignment & (alignment - 1)) == 0;
}

// Shared by describe_emit and node_publish: the manifest is built from raw
// caller arrays in exactly one place, so --describe and the registry row can
// never disagree about what a node owns.
bool BuildManifest(const TalosSource* sources, uint32_t num_sources,
                   talos::event::Manifest& manifest) {
  manifest.clear();
  manifest.reserve(num_sources);
  for (uint32_t i = 0; i < num_sources; ++i) {
    if (sources[i].topic == nullptr) {
      SetError("source topic must be non-NULL");
      return false;
    }
    talos::event::Registration reg{};
    reg.id = static_cast<uint16_t>(i);
    switch (sources[i].kind) {
      case TALOS_SOURCE_TIMER:
        reg.kind = talos::event::SourceKind::TIMER;
        break;
      case TALOS_SOURCE_WATCHER:
        reg.kind = talos::event::SourceKind::WATCHER;
        break;
      case TALOS_SOURCE_FETCHER:
        reg.kind = talos::event::SourceKind::FETCHER;
        break;
      case TALOS_SOURCE_SENDER:
        reg.kind = talos::event::SourceKind::SENDER;
        break;
      default:
        SetError("unknown source kind (want 1..4)");
        return false;
    }
    reg.name = sources[i].topic;
    reg.message_bytes = sources[i].message_bytes;
    manifest.push_back(std::move(reg));
  }
  return true;
}

void BuildAttributes(const TalosSource* sources, uint32_t num_sources,
                     std::vector<talos::introspect::EndpointAttribute>& attrs) {
  attrs.clear();
  attrs.reserve(num_sources);
  for (uint32_t i = 0; i < num_sources; ++i) {
    attrs.push_back({std::string_view(sources[i].topic), sources[i].flags});
  }
}

}  // namespace

struct TalosPublisher {
  std::optional<ipc::RawPublisher> inner;
  std::size_t message_bytes = 0;
};

struct TalosSubscriber {
  std::optional<RTMSQueue> queue;
  std::optional<std::size_t> reader_id;
  std::size_t message_bytes = 0;
};

uint32_t talos_abi_version(void) { return TALOS_NODE_ABI_VERSION; }

int64_t talos_monotonic_ns(void) {
  struct timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

const char* talos_last_error(void) { return g_last_error.c_str(); }

TalosPublisher* talos_topic_open_publisher(const char* topic,
                                           uint32_t message_bytes,
                                           uint32_t alignment) {
  ClearError();
  try {
    if (topic == nullptr || topic[0] == '\0') {
      SetError("topic must be a non-empty string");
      return nullptr;
    }
    if (!ValidLayout(message_bytes, alignment)) {
      SetError("message_bytes and alignment must be non-zero; alignment pow2");
      return nullptr;
    }
    auto* out = new TalosPublisher{};
    out->message_bytes = message_bytes;
    out->inner.emplace(std::string_view(topic), message_bytes, alignment);
    return out;
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown open-publisher failure");
  }
  return nullptr;
}

int32_t talos_publish(TalosPublisher* publisher, const void* data,
                      uint32_t size) {
  ClearError();
  try {
    if (publisher == nullptr || data == nullptr) {
      SetError("publisher and data must be non-NULL");
      return TALOS_ERR_ARG;
    }
    if (size != publisher->message_bytes) {
      SetError("payload size must equal message_bytes given at open");
      return TALOS_ERR_ARG;
    }
    RTMSMessage msg{size, data};
    const WriteStatus status = publisher->inner->write(msg);
    if (status == WriteResult::SUCCESS) return TALOS_OK;
    if (status == WriteResult::BUFFER_FULL) {
      SetError("ring buffer full (DROP_NEWEST policy)");
      return TALOS_ERR;
    }
    SetError("write rejected (size mismatch)");
    return TALOS_ERR;
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown publish failure");
  }
  return TALOS_ERR;
}

void talos_publisher_close(TalosPublisher* publisher) {
  try {
    delete publisher;
  } catch (...) {
  }
}

TalosSubscriber* talos_topic_open_subscriber(const char* topic,
                                             uint32_t message_bytes,
                                             uint32_t alignment) {
  ClearError();
  try {
    if (topic == nullptr || topic[0] == '\0') {
      SetError("topic must be a non-empty string");
      return nullptr;
    }
    if (!ValidLayout(message_bytes, alignment)) {
      SetError("message_bytes and alignment must be non-zero; alignment pow2");
      return nullptr;
    }
    auto* out = new TalosSubscriber{};
    out->message_bytes = message_bytes;
    out->queue.emplace(std::string_view(topic), message_bytes, alignment,
                       MAX_SLOTS, RTMSOptions{});
    out->reader_id = out->queue->register_reader();
    if (!out->reader_id) {
      SetError("no free reader slots (max 8)");
      delete out;
      return nullptr;
    }
    return out;
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown open-subscriber failure");
  }
  return nullptr;
}

int32_t talos_poll_next(TalosSubscriber* subscriber, void* out_data,
                        uint32_t out_size, uint64_t* out_sequence,
                        uint64_t* out_dropped) {
  ClearError();
  try {
    if (subscriber == nullptr || out_data == nullptr) {
      SetError("subscriber and out_data must be non-NULL");
      return TALOS_ERR_ARG;
    }
    if (out_size < subscriber->message_bytes) {
      SetError("out buffer smaller than message_bytes");
      return TALOS_ERR_ARG;
    }
    if (!subscriber->reader_id) {
      SetError("subscriber has no reader slot");
      return TALOS_ERR;
    }
    MessageInfo info{};
    const ReadResult result = subscriber->queue->read_next(
        subscriber->reader_id.value(),
        std::span<std::byte>{static_cast<std::byte*>(out_data),
                             subscriber->message_bytes},
        info);
    switch (result) {
      case ReadResult::OK:
        if (out_sequence != nullptr) *out_sequence = info.sequence;
        if (out_dropped != nullptr) *out_dropped = info.dropped;
        return TALOS_OK;
      case ReadResult::EMPTY:
        return TALOS_EMPTY;
      case ReadResult::INACTIVE:
        SetError("reader slot inactive");
        return TALOS_ERR;
      case ReadResult::INVALID:
        SetError("invalid reader or buffer");
        return TALOS_ERR_ARG;
      case ReadResult::TORN:
        SetError("reader lapped beyond recovery");
        return TALOS_ERR;
    }
    SetError("unknown read result");
    return TALOS_ERR;
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown poll failure");
  }
  return TALOS_ERR;
}

void talos_subscriber_close(TalosSubscriber* subscriber) {
  try {
    if (subscriber == nullptr) return;
    if (subscriber->queue && subscriber->reader_id) {
      subscriber->queue->release_reader(subscriber->reader_id.value());
    }
    delete subscriber;
  } catch (...) {
  }
}

int32_t talos_describe_emit(const char* node_name, const char* target,
                            const TalosSource* sources, uint32_t num_sources,
                            char* out_json, uint32_t out_capacity,
                            uint32_t* out_written) {
  ClearError();
  try {
    if (node_name == nullptr || target == nullptr) {
      SetError("node_name and target must be non-NULL");
      return TALOS_ERR_ARG;
    }
    if (num_sources > 0 && sources == nullptr) {
      SetError("sources must be non-NULL when num_sources > 0");
      return TALOS_ERR_ARG;
    }
    talos::event::Manifest manifest;
    if (!BuildManifest(sources, num_sources, manifest)) return TALOS_ERR_ARG;
    std::vector<talos::introspect::EndpointAttribute> attrs;
    BuildAttributes(sources, num_sources, attrs);
    talos::introspect::Description desc = talos::introspect::DescribeManifest(
        node_name, target, manifest, attrs);
    const std::string json = talos::introspect::DescribeToJson(desc);
    const uint32_t need = static_cast<uint32_t>(json.size());
    if (out_written != nullptr) *out_written = need;
    if (out_json == nullptr || out_capacity < need + 1) {
      if (out_json != nullptr && out_capacity > 0) out_json[0] = '\0';
      return TALOS_ERR_SMALL;
    }
    std::memcpy(out_json, json.data(), need);
    out_json[need] = '\0';
    return TALOS_OK;
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown describe failure");
  }
  return TALOS_ERR;
}

struct TalosNode {
  std::optional<talos::introspect::NodeRegistration> registration;
};

TalosNode* talos_node_register(const char* name, const char* target,
                              uint64_t session_id, uint32_t flags) {
  ClearError();
  try {
    if (name == nullptr || name[0] == '\0' || target == nullptr ||
        target[0] == '\0') {
      SetError("name and target must be non-empty strings");
      return nullptr;
    }
    auto* out = new TalosNode{};
    try {
      out->registration.emplace(
          talos::introspect::NodeRegistration::Identity{
              .name = name, .target = target, .session_id = session_id,
              .flags = flags});
    } catch (...) {
      delete out;
      throw;
    }
    return out;
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown register failure");
  }
  return nullptr;
}

int32_t talos_node_publish(TalosNode* node, const TalosSource* sources,
                          uint32_t num_sources) {
  ClearError();
  try {
    if (node == nullptr || !node->registration) {
      SetError("node must be a registered handle");
      return TALOS_ERR_ARG;
    }
    if (num_sources > 0 && sources == nullptr) {
      SetError("sources must be non-NULL when num_sources > 0");
      return TALOS_ERR_ARG;
    }
    talos::event::Manifest manifest;
    if (!BuildManifest(sources, num_sources, manifest)) return TALOS_ERR_ARG;
    std::vector<talos::introspect::EndpointAttribute> attrs;
    BuildAttributes(sources, num_sources, attrs);
    node->registration->publish(manifest, attrs);
    return TALOS_OK;
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown publish failure");
  }
  return TALOS_ERR;
}

int32_t talos_node_heartbeat(TalosNode* node, uint64_t dispatches) {
  ClearError();
  try {
    if (node == nullptr || !node->registration) {
      SetError("node must be a registered handle");
      return TALOS_ERR_ARG;
    }
    // One refresh sample, mirroring Reporter::sample: heartbeat, dispatch
    // count and source counters move together behind the seqlock.
    node->registration->begin_sample();
    node->registration->heartbeat();
    node->registration->set_dispatch_count(dispatches);
    node->registration->end_sample();
    return TALOS_OK;
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown heartbeat failure");
  }
  return TALOS_ERR;
}

void talos_node_close(TalosNode* node) {
  try {
    delete node;
  } catch (...) {
  }
}
