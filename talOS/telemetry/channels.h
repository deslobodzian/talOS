#pragma once

// The telemetry channel model: which channels a topic's message holds, what
// they are called, and how each one's value is read from the message's
// bytes. The telemetry node builds its dictionary from it, and a reader of
// recorded messages names what it decodes with it, so a channel has one name
// live and recorded.
//
//   name     `<topic without its leading slash>/<field path>`: nested structs
//            as path segments, array elements as `<field>/<index>`
//            ("demo/status/inners/1/offset_m")
//   kind     from the field's type (KindOf)
//   unit     from the field name's suffix (talOS/schema/unit_suffix.h);
//            none for a bool or an enum
//   labels   an enum's value names, indexed by value (EnumLabels)
//   value    the raw 64-bit sample a telemetry page carries (ReadChannel)
//
// Flattening allocates and throws (construction time); ReadChannel and
// ReadDriverStation neither allocate nor throw (per tick).

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "flatbuffers/reflection.h"
#include "talOS/schema/field_path.h"
#include "talOS/telemetry/frames.h"
#include "talOS/telemetry/telemetry_generated.h"
#include "talOS/telemetry/wire.h"

namespace talos::telemetry {

// One scalar of a topic's message, as the dictionary names it.
struct Channel {
  std::string name;
  std::string unit;
  std::vector<std::string> labels;  // non-empty: enum channel
  ChannelKind kind{Talos::Telemetry::ChannelKind_Double};
  // Where the scalar is in the message and its reflected type. None for a
  // channel ReadChannel does not read (DriverStationChannels).
  std::size_t offset{0};
  reflection::BaseType base{reflection::BaseType::None};
};

// The channel kind a reflected scalar travels as: integers keep their
// width's exact value, floats become doubles.
inline ChannelKind KindOf(reflection::BaseType base) {
  using reflection::BaseType;
  switch (base) {
    case BaseType::Bool:
      return Talos::Telemetry::ChannelKind_Bool;
    case BaseType::Byte:
    case BaseType::Short:
    case BaseType::Int:
    case BaseType::Long:
      return Talos::Telemetry::ChannelKind_Int;
    case BaseType::UByte:
    case BaseType::UType:
    case BaseType::UShort:
    case BaseType::UInt:
    case BaseType::ULong:
      return Talos::Telemetry::ChannelKind_UInt;
    default:
      return Talos::Telemetry::ChannelKind_Double;
  }
}

// The raw sample of the reflected scalar at `data`, or false for a type
// telemetry does not carry. A NaN or infinite float travels as itself: a
// reading, not an absence.
inline bool ReadScalarRaw(reflection::BaseType base, const std::byte* data,
                          std::uint64_t& raw) {
  if (schema::ScalarSize(base) == 0) return false;
  switch (KindOf(base)) {
    case Talos::Telemetry::ChannelKind_Bool:
      raw = schema::ReadBool(base, data) ? 1 : 0;
      return true;
    case Talos::Telemetry::ChannelKind_Int:
    case Talos::Telemetry::ChannelKind_UInt:
      raw = static_cast<std::uint64_t>(schema::ReadInt64(base, data));
      return true;
    default:
      raw = RawFromDouble(schema::ReadDouble(base, data));
      return true;
  }
}

// `channel`'s raw sample in `message`, a whole message of its topic. False
// when the channel has no scalar to read or its scalar ends past `message`.
inline bool ReadChannel(const Channel& channel,
                        std::span<const std::byte> message,
                        std::uint64_t& raw) {
  const std::size_t size = schema::ScalarSize(channel.base);
  if (size == 0 || channel.offset > message.size() ||
      message.size() - channel.offset < size) {
    return false;
  }
  return ReadScalarRaw(channel.base, message.data() + channel.offset, raw);
}

// The value names of enum `enum_index` of `schema`, indexed by value. Empty
// for an enum whose largest value is negative or not below kMaxEnumLabels
// (wide bit flags, sparse codes), and for an index that names no enum.
std::vector<std::string> EnumLabels(const reflection::Schema& schema,
                                    std::int32_t enum_index);

// Bounds on one topic's flattening, so a damaged or crafted schema (a
// recorded log's) is refused rather than read out of bounds or expanded
// without end. Real messages are far inside them: the deepest nests one
// struct in another (drivetrain.DrivetrainState's modules), the largest
// (bridge.BridgeStatus) has under 50 channels, and a topic cannot have more
// channels than a whole dictionary holds (kMaxChannels).
inline constexpr std::size_t kMaxNestingDepth = 16;
inline constexpr std::size_t kMaxTopicChannels = telemetry_wire::kMaxChannels;
// Struct instances visited, arrays of structs counted per element: bounds a
// walk of structs that hold no scalar.
inline constexpr std::size_t kMaxTopicStructs = 4096;

// What a caller keeps of a topic. The review server takes the defaults: a
// recorded schema past the cap is refused. Live telemetry keeps only the
// channels its config does not exclude, and truncates rather than throws,
// so its own dictionary cap (node.h) names the topic to exclude.
struct FlattenOptions {
  // When set, a channel it returns false for (by name) is neither returned
  // nor counted against max_channels.
  std::function<bool(std::string_view)> keep;
  std::size_t max_channels{kMaxTopicChannels};
  // Past max_channels kept channels: return the first max_channels and go on
  // counting (FlattenTopic's `kept`) instead of throwing.
  bool truncate{false};
};

// Every scalar of struct `schema_name` in `bfbs` (a reflection.Schema) that
// `options` keeps, in field id order with nested structs and arrays
// expanded in place, named under `topic`. Throws std::invalid_argument
// naming the topic when `bfbs` does not verify or lacks the struct, or when
// a field names an object the schema does not hold, a struct contains
// itself, structs nest deeper than kMaxNestingDepth, the walk passes
// kMaxTopicStructs struct instances, or (unless options.truncate) the kept
// channels pass options.max_channels. `kept`, when given, is set to how many
// channels `options` kept, those past a truncating flattening's cap
// included.
std::vector<Channel> FlattenTopic(std::string_view topic,
                                  std::string_view schema_name,
                                  std::string_view bfbs,
                                  const FlattenOptions& options = {},
                                  std::size_t* kept = nullptr);

// The Driver Station topic's channels, decoded from its packet rather than
// read from a struct: connected, enabled, autonomous, teleop, test, estop,
// fms_attached, alliance (labels unknown, red, blue) and match_time_s, all
// under "hw/state/driver_station/".
inline constexpr std::size_t kDriverStationChannelCount = 9;
std::vector<Channel> DriverStationChannels();

// The raw sample of each DriverStationChannels() channel, in that order, from
// `packet`, the bytes of one /hw/state/driver_station message (a
// hardware::Packet). False when it is too short or does not decode.
bool ReadDriverStation(
    std::span<const std::byte> packet,
    std::array<std::uint64_t, kDriverStationChannelCount>& raw);

}  // namespace talos::telemetry
