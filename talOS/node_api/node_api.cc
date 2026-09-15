#include "talOS/node_api/node_api.h"

#include <time.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/events/callback.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/manifest.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/recorder.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/events/time.h"
#include "talOS/hardware/gate.h"
#include "talOS/hardware/messages.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"
#include "talOS/introspection/reporter.h"
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

uint32_t talos_rtms_layout(TalosRtmsLayout* out) {
  if (out == nullptr) return 0;
  out->header_size = static_cast<uint32_t>(sizeof(RTMSHeader));
  out->off_writer_seq = static_cast<uint32_t>(offsetof(RTMSHeader, writer) +
                                              offsetof(Writer, sequence));
  out->off_readers = static_cast<uint32_t>(offsetof(RTMSHeader, readers));
  out->reader_stride = static_cast<uint32_t>(sizeof(Reader));
  return static_cast<uint32_t>(sizeof(TalosRtmsLayout));
}

int64_t talos_monotonic_ns(void) {
  struct timespec ts {};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int32_t talos_accept_state(uint64_t config_id, int32_t have_prev,
                           uint64_t prev_boot_id, uint64_t prev_sample_time_us,
                           uint64_t next_config_id, uint64_t next_boot_id,
                           uint64_t next_sample_time_us) {
  // Pure scalar-to-struct translation over hardware::AcceptState: no
  // allocation, no shared memory, nothing that can throw.
  talos::hardware::State prev{};
  prev.boot_id = prev_boot_id;
  prev.sample_time_us = prev_sample_time_us;
  talos::hardware::State next{};
  next.config_id = next_config_id;
  next.boot_id = next_boot_id;
  next.sample_time_us = next_sample_time_us;
  return talos::hardware::AcceptState(have_prev != 0, prev, config_id, next)
             ? 1
             : 0;
}

int32_t talos_gateway_ready(uint32_t flags) {
  talos::hardware::State state{};
  state.flags = flags;
  return talos::hardware::GatewayReady(state) ? 1 : 0;
}

int32_t talos_target_fresh_ns(int64_t now_ns, int64_t issued_ns,
                              int32_t enabled, int64_t state_rx_ns,
                              uint64_t timeout_us) {
  return talos::hardware::TargetFresh(
             talos::event::MonotonicTime::from_nanos(now_ns),
             talos::event::MonotonicTime::from_nanos(issued_ns), enabled != 0,
             talos::event::MonotonicTime::from_nanos(state_rx_ns),
             std::chrono::microseconds{static_cast<int64_t>(timeout_us)})
             ? 1
             : 0;
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
    if (!publisher->inner) {
      SetError("publisher has no transport");
      return TALOS_ERR;
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
    SetError("unknown publisher-close failure");
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
    if (!subscriber->queue) {
      SetError("subscriber has no transport");
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
    SetError("unknown subscriber-close failure");
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
    talos::introspect::Description desc =
        talos::introspect::DescribeManifest(node_name, target, manifest, attrs);
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
      out->registration.emplace(talos::introspect::NodeRegistration::Identity{
          .name = name,
          .target = target,
          .session_id = session_id,
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
    SetError("unknown node-close failure");
  }
}

struct TalosConfig {
  talos::config::RobotConfig robot;
};

TalosConfig* talos_config_load(const char* config_path, int32_t simulation) {
  ClearError();
  try {
    if (config_path == nullptr || config_path[0] == '\0') {
      SetError("config_path must be a non-empty string");
      return nullptr;
    }
    auto* out = new TalosConfig{};
    try {
      out->robot = talos::config::ParseRobotConfig(config_path);
      if (simulation != 0) {
        out->robot.hardware.commissioned = true;
      }
      return out;
    } catch (...) {
      delete out;
      throw;
    }
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown config load failure");
  }
  return nullptr;
}

uint64_t talos_config_get_id(const TalosConfig* config) {
  ClearError();
  try {
    if (config == nullptr) {
      SetError("config must be non-NULL");
      return 0;
    }
    return talos::hardware::ConfigurationId(config->robot.hardware);
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown config get_id failure");
  }
  return 0;
}

uint32_t talos_config_get_period_us(const TalosConfig* config) {
  ClearError();
  try {
    if (config == nullptr) {
      SetError("config must be non-NULL");
      return 0;
    }
    return config->robot.hardware.period_us;
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown config get_period_us failure");
  }
  return 0;
}

uint32_t talos_config_get_command_timeout_us(const TalosConfig* config) {
  ClearError();
  try {
    if (config == nullptr) {
      SetError("config must be non-NULL");
      return 0;
    }
    return config->robot.hardware.command_timeout_us;
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown config get_command_timeout_us failure");
  }
  return 0;
}

uint32_t talos_config_get_device_id(const TalosConfig* config,
                                    const char* subsystem,
                                    const char* device_name) {
  ClearError();
  try {
    if (config == nullptr) {
      SetError("config must be non-NULL");
      return 0;
    }
    if (subsystem == nullptr || subsystem[0] == '\0') {
      SetError("subsystem must be a non-empty string");
      return 0;
    }
    if (device_name == nullptr || device_name[0] == '\0') {
      SetError("device_name must be a non-empty string");
      return 0;
    }
    const talos::hardware::Devices* devs = config->robot.GetDevices(subsystem);
    if (devs == nullptr) {
      SetErrorStr("subsystem not found: " + std::string(subsystem));
      return 0;
    }
    const std::string name(device_name);
    auto match_device = [&](const std::vector<uint16_t>& dev_ids,
                            const auto& list) -> uint32_t {
      for (uint16_t id : dev_ids) {
        for (const auto& item : list) {
          if (item.id == id && item.name == name) {
            return id;
          }
        }
      }
      return 0;
    };
    if (uint32_t id =
            match_device(devs->motors, config->robot.hardware.motors)) {
      return id;
    }
    if (uint32_t id =
            match_device(devs->sensors, config->robot.hardware.sensors)) {
      return id;
    }
    if (uint32_t id = match_device(devs->digital_inputs,
                                   config->robot.hardware.digital_inputs)) {
      return id;
    }
    if (uint32_t id = match_device(devs->digital_outputs,
                                   config->robot.hardware.digital_outputs)) {
      return id;
    }
    if (uint32_t id = match_device(devs->analog_inputs,
                                   config->robot.hardware.analog_inputs)) {
      return id;
    }
    if (uint32_t id = match_device(devs->encoders,
                                   config->robot.hardware.encoders)) {
      return id;
    }
    if (uint32_t id = match_device(devs->pwm_outputs,
                                   config->robot.hardware.pwm_outputs)) {
      return id;
    }
    SetErrorStr("device '" + name + "' not found in subsystem '" +
                std::string(subsystem) + "'");
    return 0;
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown config get_device_id failure");
  }
  return 0;
}

void talos_config_close(TalosConfig* config) {
  try {
    delete config;
  } catch (...) {
    SetError("unknown config-close failure");
  }
}

// --- Event Loop Bridge (ABI v6) ---

struct CallbackSlot {
  TalosDispatchFn fn{nullptr};
  void* user_data{nullptr};
};

static void DispatchThunk(void* object, const talos::event::Context& context,
                          std::span<const std::byte> payload) {
  auto* slot = static_cast<CallbackSlot*>(object);
  if (slot == nullptr || slot->fn == nullptr) {
    return;
  }
  TalosDispatchContext c_ctx{
      .kind = static_cast<uint16_t>(context.kind),
      .source_id = context.source_id,
      .dispatch_index = context.dispatch_index,
      .event_time_ns = context.event_time.nanos(),
      .now_ns = context.now.nanos(),
      .sequence = context.sequence,
      .dropped = context.dropped,
  };
  try {
    slot->fn(&c_ctx, payload.data(), static_cast<uint32_t>(payload.size()),
             slot->user_data);
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown dispatch callback exception");
  }
}

struct TalosLoop {
 public:
  virtual ~TalosLoop() = default;

  virtual uint16_t register_timer(const char* name, uint64_t period_ns,
                                  uint64_t offset_ns, TalosDispatchFn fn,
                                  void* user_data) = 0;
  virtual uint16_t register_watcher(const char* topic, uint32_t message_bytes,
                                    uint32_t alignment, TalosDispatchFn fn,
                                    void* user_data) = 0;
  virtual uint16_t register_sender(const char* topic, uint32_t message_bytes,
                                   uint32_t alignment) = 0;
  virtual int32_t arm_timer(uint16_t timer_id, uint64_t period_ns,
                            uint64_t offset_ns) = 0;
  virtual int32_t disarm_timer(uint16_t timer_id) = 0;
  virtual int32_t send(uint16_t sender_id, const void* data, uint32_t size) = 0;
  virtual int64_t now_ns() const = 0;
  virtual int32_t run() = 0;
  virtual int32_t run_for(int64_t duration_ns) = 0;
  virtual void exit() = 0;
  virtual int32_t diverged() const { return 0; }
  virtual int32_t reached_exit() const { return 0; }
  virtual uint64_t dispatch_count() const = 0;
  virtual int32_t sim_inject(const char* topic, const void* data,
                             uint32_t size) {
    SetError("sim_inject is only supported on simulated loop");
    return TALOS_ERR;
  }
  virtual int32_t attach_reporter(const char* name, const char* target,
                                  uint64_t session_id, uint32_t flags) {
    return TALOS_OK;
  }
};

template <typename LoopType>
class TalosLoopBase : public TalosLoop {
 public:
  virtual LoopType& loop() = 0;
  virtual const LoopType& loop() const = 0;

  uint16_t register_timer(const char* name, uint64_t period_ns,
                          uint64_t offset_ns, TalosDispatchFn fn,
                          void* user_data) override {
    if (name == nullptr || name[0] == '\0') {
      SetError("timer name must be non-empty");
      return 0xFFFF;
    }
    if (fn == nullptr) {
      SetError("timer callback fn must be non-NULL");
      return 0xFFFF;
    }
    auto slot = std::make_unique<CallbackSlot>();
    slot->fn = fn;
    slot->user_data = user_data;
    CallbackSlot* slot_ptr = slot.get();
    callback_slots_.push_back(std::move(slot));

    talos::event::Thunk thunk{slot_ptr, &DispatchThunk};
    const uint16_t id = loop().register_timer(name, thunk);
    if (period_ns > 0 || offset_ns > 0) {
      const talos::event::MonotonicTime deadline =
          loop().start_time() + std::chrono::nanoseconds{offset_ns};
      const talos::event::Duration period{std::chrono::nanoseconds{period_ns}};
      loop().arm_timer(id, deadline, period);
    }
    return id;
  }

  uint16_t register_watcher(const char* topic, uint32_t message_bytes,
                            uint32_t alignment, TalosDispatchFn fn,
                            void* user_data) override {
    if (topic == nullptr || topic[0] == '\0') {
      SetError("watcher topic must be non-empty");
      return 0xFFFF;
    }
    if (!ValidLayout(message_bytes, alignment)) {
      SetError("message_bytes and alignment must be non-zero; alignment pow2");
      return 0xFFFF;
    }
    if (fn == nullptr) {
      SetError("watcher callback fn must be non-NULL");
      return 0xFFFF;
    }
    auto slot = std::make_unique<CallbackSlot>();
    slot->fn = fn;
    slot->user_data = user_data;
    CallbackSlot* slot_ptr = slot.get();
    callback_slots_.push_back(std::move(slot));

    talos::event::Thunk thunk{slot_ptr, &DispatchThunk};
    return loop().register_watcher(topic, message_bytes, alignment, thunk);
  }

  uint16_t register_sender(const char* topic, uint32_t message_bytes,
                           uint32_t alignment) override {
    if (topic == nullptr || topic[0] == '\0') {
      SetError("sender topic must be non-empty");
      return 0xFFFF;
    }
    if (!ValidLayout(message_bytes, alignment)) {
      SetError("message_bytes and alignment must be non-zero; alignment pow2");
      return 0xFFFF;
    }
    return loop().register_sender(topic, message_bytes, alignment);
  }

  int32_t arm_timer(uint16_t timer_id, uint64_t period_ns,
                    uint64_t offset_ns) override {
    const talos::event::MonotonicTime deadline =
        loop().monotonic_now() + std::chrono::nanoseconds{offset_ns};
    const talos::event::Duration period{std::chrono::nanoseconds{period_ns}};
    loop().arm_timer(timer_id, deadline, period);
    return TALOS_OK;
  }

  int32_t disarm_timer(uint16_t timer_id) override {
    loop().disarm_timer(timer_id);
    return TALOS_OK;
  }

  int32_t send(uint16_t sender_id, const void* data, uint32_t size) override {
    if (data == nullptr && size > 0) {
      SetError("send data must be non-NULL when size > 0");
      return TALOS_ERR_ARG;
    }
    const bool ok = loop().send_message(
        sender_id,
        std::span<const std::byte>{reinterpret_cast<const std::byte*>(data),
                                   size});
    return ok ? TALOS_OK : TALOS_ERR;
  }

  int64_t now_ns() const override {
    return loop().monotonic_now().nanos();
  }

  void exit() override {
    loop().exit();
  }

  uint64_t dispatch_count() const override {
    return loop().dispatch_count();
  }

 protected:
  std::vector<std::unique_ptr<CallbackSlot>> callback_slots_;
};

template <talos::event::RecorderPolicy Recorder>
class TalosRealtimeLoopImpl
    : public TalosLoopBase<talos::event::RealtimeEventLoop<Recorder>> {
 public:
  using LoopType = talos::event::RealtimeEventLoop<Recorder>;

  TalosRealtimeLoopImpl() : loop_{} {}
  explicit TalosRealtimeLoopImpl(Recorder recorder)
      : loop_{std::move(recorder)} {}

  LoopType& loop() override { return loop_; }
  const LoopType& loop() const override { return loop_; }

  int32_t run() override {
    loop_.run();
    return TALOS_OK;
  }

  int32_t run_for(int64_t duration_ns) override {
    loop_.run_for(
        talos::event::Duration{std::chrono::nanoseconds{duration_ns}});
    return TALOS_OK;
  }

  int32_t attach_reporter(const char* name, const char* target,
                          uint64_t session_id, uint32_t flags) override {
    if (name == nullptr || name[0] == '\0') {
      SetError("name must be a non-empty string");
      return TALOS_ERR_ARG;
    }
    if (target == nullptr || target[0] == '\0') {
      SetError("target must be a non-empty string");
      return TALOS_ERR_ARG;
    }
    talos::introspect::Reporter::Options options;
    options.name = name;
    options.target = target;
    options.session_id = session_id;
    options.simulation = (flags & (1 << 0)) != 0;
    options.replay = (flags & (1 << 1)) != 0;
    reporter_.emplace(loop_, std::move(options));
    return TALOS_OK;
  }

 private:
  LoopType loop_;
  std::optional<talos::introspect::Reporter> reporter_;
};

template <talos::event::RecorderPolicy Recorder>
class TalosSimulatedLoopImpl
    : public TalosLoopBase<talos::event::SimulatedEventLoop<Recorder>> {
 public:
  using LoopType = talos::event::SimulatedEventLoop<Recorder>;

  TalosSimulatedLoopImpl() : environment_{}, loop_{environment_} {}
  explicit TalosSimulatedLoopImpl(Recorder recorder)
      : environment_{}, loop_{environment_, std::move(recorder)} {}

  LoopType& loop() override { return loop_; }
  const LoopType& loop() const override { return loop_; }

  int32_t run() override {
    loop_.run_until(talos::event::MonotonicTime::max());
    return TALOS_OK;
  }

  int32_t run_for(int64_t duration_ns) override {
    loop_.run_for(
        talos::event::Duration{std::chrono::nanoseconds{duration_ns}});
    return TALOS_OK;
  }

  int32_t sim_inject(const char* topic, const void* data,
                     uint32_t size) override {
    if (topic == nullptr || topic[0] == '\0') {
      SetError("topic must be a non-empty string");
      return TALOS_ERR_ARG;
    }
    if (data == nullptr && size > 0) {
      SetError("data must be non-NULL when size > 0");
      return TALOS_ERR_ARG;
    }
    environment_.channel(topic, size).publish(
        std::span<const std::byte>{reinterpret_cast<const std::byte*>(data),
                                   size});
    return TALOS_OK;
  }

  ~TalosSimulatedLoopImpl() override {
    loop_.finish();
  }

 private:
  talos::event::SimulationEnvironment environment_;
  LoopType loop_;
};

class TalosReplayLoopImpl
    : public TalosLoopBase<
          talos::event::ReplayEventLoop<talos::event::NullRecorder>> {
 public:
  using LoopType = talos::event::ReplayEventLoop<talos::event::NullRecorder>;

  explicit TalosReplayLoopImpl(const char* replay_path)
      : reader_{replay_path},
        loop_{reader_, LoopType::Options{.throw_on_divergence = false}} {}

  LoopType& loop() override { return loop_; }
  const LoopType& loop() const override { return loop_; }

  int32_t run() override {
    loop_.run();
    if (loop_.diverged() && !loop_.divergences().empty()) {
      SetErrorStr(loop_.divergences().front().detail);
    }
    return TALOS_OK;
  }

  int32_t run_for(int64_t /*duration_ns*/) override {
    SetError("run_for is not supported on replay loop");
    return TALOS_ERR;
  }

  int32_t diverged() const override {
    return loop_.diverged() ? 1 : 0;
  }

  int32_t reached_exit() const override {
    return loop_.reached_exit() ? 1 : 0;
  }

 private:
  talos::event::log::LogReader reader_;
  LoopType loop_;
};

TalosLoop* talos_loop_create_realtime(const char* log_path,
                                     const char* stream_name,
                                     uint64_t session_id) {
  ClearError();
  try {
    if (log_path != nullptr && log_path[0] != '\0') {
      talos::event::log::LogWriter writer(
          log_path, stream_name != nullptr ? stream_name : "",
          talos::event::log::LogWriter::Options{}, session_id);
      return new TalosRealtimeLoopImpl<talos::event::log::LogWriter>(
          std::move(writer));
    } else {
      return new TalosRealtimeLoopImpl<talos::event::NullRecorder>();
    }
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown create realtime loop failure");
  }
  return nullptr;
}

TalosLoop* talos_loop_create_simulated(const char* log_path,
                                      const char* stream_name) {
  ClearError();
  try {
    if (log_path != nullptr && log_path[0] != '\0') {
      talos::event::log::LogWriter writer(
          log_path, stream_name != nullptr ? stream_name : "",
          talos::event::log::LogWriter::Options{});
      return new TalosSimulatedLoopImpl<talos::event::log::LogWriter>(
          std::move(writer));
    } else {
      return new TalosSimulatedLoopImpl<talos::event::NullRecorder>();
    }
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown create simulated loop failure");
  }
  return nullptr;
}

TalosLoop* talos_loop_create_replay(const char* replay_path) {
  ClearError();
  try {
    if (replay_path == nullptr || replay_path[0] == '\0') {
      SetError("replay_path must be a non-empty string");
      return nullptr;
    }
    return new TalosReplayLoopImpl(replay_path);
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown create replay loop failure");
  }
  return nullptr;
}

uint16_t talos_loop_register_timer(TalosLoop* loop, const char* name,
                                   uint64_t period_ns, uint64_t offset_ns,
                                   TalosDispatchFn fn, void* user_data) {
  ClearError();
  try {
    if (loop == nullptr) {
      SetError("loop must be non-NULL");
      return 0xFFFF;
    }
    return loop->register_timer(name, period_ns, offset_ns, fn, user_data);
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown register_timer failure");
  }
  return 0xFFFF;
}

uint16_t talos_loop_register_watcher(TalosLoop* loop, const char* topic,
                                     uint32_t message_bytes,
                                     uint32_t alignment, TalosDispatchFn fn,
                                     void* user_data) {
  ClearError();
  try {
    if (loop == nullptr) {
      SetError("loop must be non-NULL");
      return 0xFFFF;
    }
    return loop->register_watcher(topic, message_bytes, alignment, fn,
                                  user_data);
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown register_watcher failure");
  }
  return 0xFFFF;
}

uint16_t talos_loop_register_sender(TalosLoop* loop, const char* topic,
                                    uint32_t message_bytes,
                                    uint32_t alignment) {
  ClearError();
  try {
    if (loop == nullptr) {
      SetError("loop must be non-NULL");
      return 0xFFFF;
    }
    return loop->register_sender(topic, message_bytes, alignment);
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown register_sender failure");
  }
  return 0xFFFF;
}

int32_t talos_loop_arm_timer(TalosLoop* loop, uint16_t timer_id,
                             uint64_t period_ns, uint64_t offset_ns) {
  ClearError();
  try {
    if (loop == nullptr) {
      SetError("loop must be non-NULL");
      return TALOS_ERR_ARG;
    }
    return loop->arm_timer(timer_id, period_ns, offset_ns);
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown arm_timer failure");
  }
  return TALOS_ERR;
}

int32_t talos_loop_disarm_timer(TalosLoop* loop, uint16_t timer_id) {
  ClearError();
  try {
    if (loop == nullptr) {
      SetError("loop must be non-NULL");
      return TALOS_ERR_ARG;
    }
    return loop->disarm_timer(timer_id);
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown disarm_timer failure");
  }
  return TALOS_ERR;
}

int32_t talos_loop_send(TalosLoop* loop, uint16_t sender_id, const void* data,
                        uint32_t size) {
  ClearError();
  try {
    if (loop == nullptr) {
      SetError("loop must be non-NULL");
      return TALOS_ERR_ARG;
    }
    return loop->send(sender_id, data, size);
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown send failure");
  }
  return TALOS_ERR;
}

int64_t talos_loop_now_ns(const TalosLoop* loop) {
  if (loop == nullptr) return 0;
  try {
    return loop->now_ns();
  } catch (...) {
    return 0;
  }
}

int32_t talos_loop_run(TalosLoop* loop) {
  ClearError();
  try {
    if (loop == nullptr) {
      SetError("loop must be non-NULL");
      return TALOS_ERR_ARG;
    }
    return loop->run();
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown loop run failure");
  }
  return TALOS_ERR;
}

int32_t talos_loop_run_for(TalosLoop* loop, int64_t duration_ns) {
  ClearError();
  try {
    if (loop == nullptr) {
      SetError("loop must be non-NULL");
      return TALOS_ERR_ARG;
    }
    if (duration_ns < 0) {
      SetError("duration_ns must not be negative");
      return TALOS_ERR_ARG;
    }
    return loop->run_for(duration_ns);
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown loop run_for failure");
  }
  return TALOS_ERR;
}

void talos_loop_exit(TalosLoop* loop) {
  if (loop == nullptr) return;
  try {
    loop->exit();
  } catch (...) {
    SetError("unknown loop exit failure");
  }
}

int32_t talos_loop_diverged(const TalosLoop* loop) {
  if (loop == nullptr) return 0;
  try {
    return loop->diverged();
  } catch (...) {
    return 0;
  }
}

int32_t talos_loop_reached_exit(const TalosLoop* loop) {
  if (loop == nullptr) return 0;
  try {
    return loop->reached_exit();
  } catch (...) {
    return 0;
  }
}

uint64_t talos_loop_dispatch_count(const TalosLoop* loop) {
  if (loop == nullptr) return 0;
  try {
    return loop->dispatch_count();
  } catch (...) {
    return 0;
  }
}

int32_t talos_loop_sim_inject(TalosLoop* loop, const char* topic,
                              const void* data, uint32_t size) {
  ClearError();
  try {
    if (loop == nullptr) {
      SetError("loop must be non-NULL");
      return TALOS_ERR_ARG;
    }
    return loop->sim_inject(topic, data, size);
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown sim_inject failure");
  }
  return TALOS_ERR;
}

int32_t talos_loop_attach_reporter(TalosLoop* loop, const char* name,
                                   const char* target, uint64_t session_id,
                                   uint32_t flags) {
  ClearError();
  try {
    if (loop == nullptr) {
      SetError("loop must be non-NULL");
      return TALOS_ERR_ARG;
    }
    return loop->attach_reporter(name, target, session_id, flags);
  } catch (const std::exception& e) {
    SetErrorStr(e.what());
  } catch (...) {
    SetError("unknown attach_reporter failure");
  }
  return TALOS_ERR;
}

void talos_loop_destroy(TalosLoop* loop) {
  try {
    delete loop;
  } catch (...) {
    SetError("unknown loop-destroy failure");
  }
}
