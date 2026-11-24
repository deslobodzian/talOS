// Prints the sizes docs/DATA_FLOW.md states, one `name value` line each,
// taken from the headers that define and assert them, so the numbers come
// from the compiler rather than from reading C++ text.
// tools/check_data_flow_sizes.py compares them with the doc's
// `<!-- size: name -->` markers; //tools:check_data_flow_sizes_test runs both.
//
// A name printed here must be marked somewhere in the doc, and a marker must
// name something printed here: adding a line here without a marker, or a
// marker without a line, fails the check.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>

#include "bridge/bridge_status_generated.h"
#include "geometry/geometry_generated.h"
#include "sim/sim_generated.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/log/format.h"
#include "talOS/hardware/config.h"
#include "talOS/hardware/config_wire.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/hardware/sim_rings.h"
#include "talOS/introspection/registry.h"
#include "talOS/protocol/frame.h"
#include "talOS/protocol/types.h"
#include "talOS/rtms/rtms.h"
#include "talOS/telemetry/wire.h"

namespace {

namespace hw = talos::hardware;
namespace ds = talos::driver_station;
namespace tlog = talos::event::log;
namespace reg = talos::introspect;
namespace tw = talos::telemetry_wire;
using talos::protocol::kFrameHeaderSize;
using talos::protocol::kMaxPayloadSize;

// Every user topic's slot is 8-byte aligned (ARCHITECTURE.md "Topics and
// messages"), byte topics included.
constexpr std::size_t kByteTopicAlignment = 8;

void Print(std::string_view name, std::uint64_t value) {
  std::printf("%.*s %llu\n", static_cast<int>(name.size()), name.data(),
              static_cast<unsigned long long>(value));
}

template <typename M>
void PrintMessage(std::string_view message_name, std::string_view slot_name) {
  Print(message_name, sizeof(M));
  Print(slot_name, rtms_slot_stride(sizeof(M), alignof(M)));
}

std::uint64_t EmptyDriverStationBytes() {
  std::uint8_t buffer[ds::kMaxEncodedSize]{};
  return ds::Encode(ds::DriverStationData{}, std::span<std::uint8_t>{buffer});
}

}  // namespace

int main() {
  // UDP frame.
  Print("protocol.version", talos::protocol::kProtocolVersion);
  Print("frame.header_bytes", kFrameHeaderSize);
  Print("frame.max_payload_bytes", kMaxPayloadSize);
  Print("frame.max_bytes", talos::protocol::kMaxFrameSize);

  // Device caps.
  Print("caps.motors", hw::kMaxMotors);
  Print("caps.sensors", hw::kMaxSensors);
  Print("caps.digital_inputs", hw::kMaxDigitalInputs);
  Print("caps.digital_outputs", hw::kMaxDigitalOutputs);
  Print("caps.analog_inputs", hw::kMaxAnalogInputs);
  Print("caps.encoders", hw::kMaxEncoders);
  Print("caps.pwm_outputs", hw::kMaxPwmOutputs);
  Print("caps.can_buses", hw::kMaxCanBuses);

  // State, type 20.
  const std::size_t state_suffix =
      hw::kHealthSampleSize + hw::kGatewayCountersSize;
  Print("state.max_payload_bytes", hw::kMaxStateSize);
  Print("state.max_frame_bytes", kFrameHeaderSize + hw::kMaxStateSize);
  Print("state.headroom_bytes", kMaxPayloadSize - hw::kMaxStateSize);
  Print("state.prefix_bytes", hw::kStateBaseSize - state_suffix);
  Print("state.suffix_bytes", state_suffix);
  Print("state.health_bytes", hw::kHealthSampleSize);
  Print("state.counters_bytes", hw::kGatewayCountersSize);
  Print("state.can_bus_bytes", hw::kCanBusSampleSize);
  Print("state.gateway_build_bytes", hw::kGatewayBuildSize);
  Print("state.motor_bytes", hw::kMotorSampleSize);
  Print("state.sensor_bytes", hw::kSensorSampleSize);
  Print("state.digital_input_bytes", hw::kDigitalInputSampleSize);
  Print("state.digital_output_bytes", hw::kDigitalOutputSampleSize);
  Print("state.analog_input_bytes", hw::kAnalogInputSampleSize);
  Print("state.encoder_bytes", hw::kEncoderSampleSize);
  Print("state.pwm_output_bytes", hw::kPwmSampleSize);

  // Command, type 21.
  Print("command.max_payload_bytes", hw::kMaxCommandSize);
  Print("command.max_frame_bytes", kFrameHeaderSize + hw::kMaxCommandSize);
  Print("command.headroom_bytes", kMaxPayloadSize - hw::kMaxCommandSize);
  Print("command.prefix_bytes", hw::kCommandBaseSize);
  Print("command.motor_bytes", hw::kMotorRequestSize);
  Print("command.digital_output_bytes", hw::kDigitalOutputRequestSize);
  Print("command.pwm_output_bytes", hw::kPwmRequestSize);

  // Config chunk (22) and ConfigAck (23).
  Print("config.chunk_header_bytes", sizeof(hw::ConfigChunkHeader));
  Print("config.chunk_data_bytes", hw::kMaxConfigChunkData);
  Print("config.max_serialized_bytes", hw::kMaxSerializedConfigSize);
  Print("config.max_chunk_count", hw::kMaxConfigChunkCount);
  Print("config_ack.payload_bytes", sizeof(hw::ConfigAckPayload));
  Print("config_ack.frame_bytes",
        kFrameHeaderSize + sizeof(hw::ConfigAckPayload));
  Print("config_ack.headroom_bytes",
        kMaxPayloadSize - sizeof(hw::ConfigAckPayload));

  // DriverStation, type 24.
  Print("ds.max_payload_bytes", ds::kMaxEncodedSize);
  Print("ds.max_frame_bytes", kFrameHeaderSize + ds::kMaxEncodedSize);
  Print("ds.headroom_bytes", kMaxPayloadSize - ds::kMaxEncodedSize);
  Print("ds.fixed_bytes", ds::kFixedEncodedSize);
  Print("ds.joystick_max_bytes", ds::kMaxJoystickEncodedSize);
  Print("ds.empty_bytes", EmptyDriverStationBytes());
  Print("ds.fms_string_bytes", ds::kMaxGameMessageLength - 1);
  Print("ds.axes", ds::kMaxAxes);
  Print("ds.povs", ds::kMaxPovs);
  Print("ds.name_bytes", ds::kMaxNameLength - 1);

  // Main-processor shared-memory slots.
  Print("rtms.version", kRtmsVersion);
  Print("rtms.header_bytes", sizeof(RTMSHeader));
  Print("rtms.max_readers", MAX_READERS);
  Print("rtms.reader_slot_bytes", sizeof(Reader));
  PrintMessage<hw::Packet>("packet.bytes", "packet.slot_bytes");
  PrintMessage<bridge::BridgeStatus>("bridge_status.bytes",
                                     "bridge_status.slot_bytes");
  Print("bridge_status.can_bus_bytes", sizeof(bridge::CanBusStatus));
  PrintMessage<geometry::PoseObservation>("pose_observation.bytes",
                                          "pose_observation.slot_bytes");
  PrintMessage<geometry::PoseEstimate>("pose_estimate.bytes",
                                       "pose_estimate.slot_bytes");
  PrintMessage<geometry::OdometrySample>("odometry_sample.bytes",
                                         "odometry_sample.slot_bytes");
  PrintMessage<sim::MatchState>("match_state.bytes", "match_state.slot_bytes");
  PrintMessage<sim::GamePieces>("game_pieces.bytes", "game_pieces.slot_bytes");
  Print("game_pieces.slot_count", talos::hardware::sim::kPiecesSlots);
  Print("game_pieces.ring_bytes",
        rtms_slot_stride(sizeof(sim::GamePieces), alignof(sim::GamePieces)) *
            talos::hardware::sim::kPiecesSlots);
  const std::size_t telemetry_slot =
      rtms_slot_stride(tw::slot_bytes, kByteTopicAlignment);
  Print("telemetry.slot_stride_bytes", telemetry_slot);
  Print("telemetry.slot_count", tw::slot_count);
  Print("telemetry.ring_bytes", telemetry_slot * tw::slot_count);

  // Live node registry.
  Print("registry.header_bytes", offsetof(reg::RegistryHeader, nodes));
  Print("registry.rows", reg::kMaxNodes);
  Print("registry.row_bytes", sizeof(reg::NodeRecord));
  Print("registry.row_alignment", alignof(reg::NodeRecord));
  Print("registry.total_bytes", sizeof(reg::RegistryHeader));
  Print("registry.sources_per_row", reg::kMaxSourcesPerNode);
  Print("registry.source_record_bytes", sizeof(reg::SourceRecord));
  Print("registry.selections_per_row", reg::kMaxSelectionsPerNode);
  Print("registry.selection_record_bytes", sizeof(reg::SelectionRecord));

  // Dispatch logs.
  Print("log.version", tlog::FORMAT_VERSION);
  Print("log.oldest_readable_version", tlog::OLDEST_READABLE_VERSION);
  Print("log.file_header_bytes", sizeof(tlog::FileHeader));
  Print("log.manifest_entry_bytes", sizeof(tlog::ManifestEntry));
  Print("log.record_header_bytes", sizeof(tlog::RecordHeader));
  Print("log.gap_record_bytes", sizeof(tlog::GapRecord));
  Print("log.gap_record_total_bytes", tlog::GAP_RECORD_BYTES);

  // Telemetry pages and the Studio bridge's burst.
  Print("telemetry.value_page_bytes", tw::kMaxValueFrameBytes);
  Print("telemetry.sample_bytes", tw::kSampleBytes);
  Print("telemetry.dictionary_page_bytes", tw::kMaxDictionaryPageBytes);
  Print("telemetry.max_channels", tw::kMaxChannels);
  Print(
      "telemetry.keyframe_pages",
      (tw::kMaxChannels + tw::kMinSamplesPerPage - 1) / tw::kMinSamplesPerPage);
  Print("telemetry.keyframe_bytes", tw::kMaxUpdateBytes);
  Print("telemetry.client_burst_bytes", tw::kClientBurstBytes);
  Print("telemetry.dictionary_string_bytes", tw::kMaxDictionaryStringBytes);
  Print("telemetry.enum_labels", tw::kMaxEnumLabels);
  Print("telemetry.drive_summary_bytes", tw::kDriveSummaryMaxBytes);
  return 0;
}
