#pragma once

// Telemetry page encoding, shared by the producer (node.h), the Studio
// bridge's per-client updates (studio/bridge) and the mock producer, so every
// TLM3 page is built and bounded one way. See README.md for the protocol.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "talOS/telemetry/telemetry_generated.h"
#include "talOS/telemetry/wire.h"

namespace talos::telemetry {

using Talos::Telemetry::ChannelKind;

// One dictionary entry, in wire order.
struct ChannelDef {
  std::string name;
  std::string unit;
  std::vector<std::string> labels;  // non-empty: enum channel
  ChannelKind kind{Talos::Telemetry::ChannelKind_Double};
};

// FNV-1a 32 over names, units, kinds and labels in dictionary order, with
// NUL separators between fields and 0xFF between channels.
inline std::uint32_t DictionaryId(std::span<const ChannelDef> dictionary) {
  std::uint32_t hash = 2166136261u;
  auto mix = [&hash](std::string_view text) {
    for (const char c : text) {
      hash ^= static_cast<std::uint8_t>(c);
      hash *= 16777619u;
    }
  };
  auto separator = [&hash](std::uint8_t byte) {
    hash ^= byte;
    hash *= 16777619u;
  };
  for (const ChannelDef& channel : dictionary) {
    mix(channel.name);
    separator(0);
    mix(channel.unit);
    separator(0);
    separator(static_cast<std::uint8_t>(channel.kind));
    separator(0);
    for (const std::string& label : channel.labels) {
      mix(label);
      separator(0);
    }
    separator(0xff);
  }
  return hash;
}

// A sample travels as its raw 64-bit pattern, read per the channel's kind.
inline std::uint64_t RawFromDouble(double value) {
  std::uint64_t raw = 0;
  std::memcpy(&raw, &value, sizeof raw);
  return raw;
}
inline double DoubleFromRaw(std::uint64_t raw) {
  double value = 0;
  std::memcpy(&value, &raw, sizeof value);
  return value;
}

// The fields every page of one tick (or one client update) shares.
struct PageHeader {
  std::uint64_t stream_id{0};
  std::uint64_t sequence_id{0};
  std::uint64_t timestamp_ns{0};
  std::uint32_t dictionary_id{0};
  std::uint32_t channel_count{0};
};

// Which part of the encoded samples and absent channels one value page holds.
struct PageSpan {
  std::size_t first_sample{0};
  std::size_t sample_count{0};
  std::size_t first_absent{0};
  std::size_t absent_count{0};
};

namespace detail {

inline constexpr std::size_t kSampleBytes = telemetry_wire::kSampleBytes;
// Per absent channel: a uint32 channel index.
inline constexpr std::size_t kAbsentBytes = 4;
// Alignment padding beyond what the measured page had: before the uint64
// values vector, and at the end of the size-prefixed buffer.
inline constexpr std::size_t kPageSlackBytes = 16;

inline void FinishValuePage(flatbuffers::FlatBufferBuilder& builder,
                            const PageHeader& header,
                            const Talos::Telemetry::Pose& chassis,
                            bool keyframe, std::uint16_t page,
                            std::uint16_t page_count,
                            std::span<const std::uint32_t> channels,
                            std::span<const std::uint64_t> values,
                            std::span<const std::uint32_t> absent) {
  builder.Clear();
  const auto channels_offset =
      channels.empty() ? 0
                       : builder.CreateVector(channels.data(), channels.size());
  const auto values_offset =
      values.empty() ? 0 : builder.CreateVector(values.data(), values.size());
  const auto absent_offset =
      absent.empty() ? 0 : builder.CreateVector(absent.data(), absent.size());
  Talos::Telemetry::FrameBuilder frame{builder};
  frame.add_stream_id(header.stream_id);
  frame.add_sequence_id(header.sequence_id);
  frame.add_timestamp_ns(header.timestamp_ns);
  frame.add_dictionary_id(header.dictionary_id);
  frame.add_chassis(&chassis);
  frame.add_page(page);
  frame.add_page_count(page_count);
  frame.add_channel_count(header.channel_count);
  frame.add_keyframe(keyframe);
  if (!channels.empty()) frame.add_channels(channels_offset);
  if (!values.empty()) frame.add_values(values_offset);
  if (!absent.empty()) frame.add_absent(absent_offset);
  Talos::Telemetry::FinishSizePrefixedFrameBuffer(builder, frame.Finish());
}

// Bytes of a value page besides its samples and absent channels: every
// scalar at its widest and all three vectors present (their lengths, offsets
// and vtable entries), measured with one entry each and those entries'
// bytes taken off, plus alignment slack. At most
// telemetry_wire::kValuePageOverheadBound (frames_test).
inline std::size_t ValuePageOverhead() {
  static const std::size_t overhead = [] {
    flatbuffers::FlatBufferBuilder builder{256};
    const PageHeader header{~0ull, ~0ull, ~0ull, ~0u, ~0u};
    const Talos::Telemetry::Pose chassis{1, 1, 1, 1, 1, 1};
    const std::uint32_t channel = ~0u;
    const std::uint64_t value = ~0ull;
    FinishValuePage(builder, header, chassis, true, 0xffff, 0xffff,
                    {&channel, 1}, {&value, 1}, {&channel, 1});
    return static_cast<std::size_t>(builder.GetSize()) - kSampleBytes -
           kAbsentBytes + kPageSlackBytes;
  }();
  return overhead;
}

// Upper bound on one ChannelInfo's bytes in a dictionary page: its offset in
// the vector, the table and its vtable, and each string with its length,
// terminator and padding.
inline std::size_t DictionaryEntryBytes(const ChannelDef& channel) {
  std::size_t bytes = 48 + channel.name.size() + 8 + channel.unit.size() + 8;
  if (!channel.labels.empty()) bytes += 4 + 4 * channel.labels.size();
  for (const std::string& label : channel.labels) bytes += label.size() + 8;
  return bytes;
}

inline void FinishDictionaryPage(flatbuffers::FlatBufferBuilder& builder,
                                 const PageHeader& header, std::uint16_t page,
                                 std::uint16_t page_count,
                                 std::uint32_t first_channel,
                                 std::span<const ChannelDef> entries) {
  builder.Clear();
  std::vector<flatbuffers::Offset<Talos::Telemetry::ChannelInfo>> offsets;
  offsets.reserve(entries.size());
  std::vector<flatbuffers::Offset<flatbuffers::String>> labels;
  for (const ChannelDef& channel : entries) {
    labels.clear();
    for (const std::string& label : channel.labels) {
      labels.push_back(builder.CreateString(label));
    }
    const auto name = builder.CreateString(channel.name);
    const auto unit =
        channel.unit.empty() ? 0 : builder.CreateString(channel.unit);
    const auto label_vector =
        channel.labels.empty() ? 0 : builder.CreateVector(labels);
    offsets.push_back(Talos::Telemetry::CreateChannelInfo(
        builder, name, unit, label_vector, channel.kind));
  }
  const auto dictionary = builder.CreateVector(offsets);
  Talos::Telemetry::FrameBuilder frame{builder};
  frame.add_stream_id(header.stream_id);
  frame.add_sequence_id(header.sequence_id);
  frame.add_timestamp_ns(header.timestamp_ns);
  frame.add_dictionary_id(header.dictionary_id);
  frame.add_page(page);
  frame.add_page_count(page_count);
  frame.add_channel_count(header.channel_count);
  frame.add_first_channel(first_channel);
  frame.add_dictionary(dictionary);
  Talos::Telemetry::FinishSizePrefixedFrameBuffer(builder, frame.Finish());
}

inline std::size_t DictionaryPageOverhead() {
  static const std::size_t overhead = [] {
    flatbuffers::FlatBufferBuilder builder{256};
    const PageHeader header{~0ull, ~0ull, ~0ull, ~0u, ~0u};
    FinishDictionaryPage(builder, header, 0xffff, 0xffff, ~0u, {});
    return static_cast<std::size_t>(builder.GetSize()) + kPageSlackBytes;
  }();
  return overhead;
}

inline std::span<const std::uint8_t> Bytes(
    const flatbuffers::FlatBufferBuilder& builder) {
  return {builder.GetBufferPointer(), builder.GetSize()};
}

}  // namespace detail

// Splits `channels`/`values` (parallel: values[i] is channels[i]'s raw
// sample) and `absent` into value pages of at most kMaxValueFrameBytes,
// samples first, and calls emit(page bytes, PageSpan) for each in order.
// Nothing to carry still makes one page: an update that says "no change"
// (or, on a keyframe, "every channel absent"). Never throws, since the
// Studio bridge encodes on its serving path: false means the update could
// not be encoded (mismatched spans, over 65535 pages, or a page over its
// bound, which frames_test shows cannot happen); pages already emitted then
// form an incomplete update, which a receiver discards.
template <class Emit>
[[nodiscard]] bool EncodeValuePages(flatbuffers::FlatBufferBuilder& builder,
                                    const PageHeader& header,
                                    const Talos::Telemetry::Pose& chassis,
                                    bool keyframe,
                                    std::span<const std::uint32_t> channels,
                                    std::span<const std::uint64_t> values,
                                    std::span<const std::uint32_t> absent,
                                    Emit&& emit) {
  if (channels.size() != values.size()) return false;
  const std::size_t room =
      telemetry_wire::kMaxValueFrameBytes - detail::ValuePageOverhead();
  // The same partition twice: once to count pages, once to emit them.
  const auto partition = [&](auto&& visit) {
    std::size_t sample = 0;
    std::size_t gone = 0;
    std::size_t page = 0;
    do {
      PageSpan span{.first_sample = sample, .first_absent = gone};
      span.sample_count =
          std::min(channels.size() - sample, room / detail::kSampleBytes);
      const std::size_t left = room - span.sample_count * detail::kSampleBytes;
      span.absent_count =
          std::min(absent.size() - gone, left / detail::kAbsentBytes);
      sample += span.sample_count;
      gone += span.absent_count;
      visit(page++, span);
    } while (sample < channels.size() || gone < absent.size());
    return page;
  };
  const std::size_t page_count = partition([](std::size_t, const PageSpan&) {});
  if (page_count > 0xffff) return false;
  bool fits = true;
  partition([&](std::size_t page, const PageSpan& span) {
    if (!fits) return;
    detail::FinishValuePage(
        builder, header, chassis, keyframe, static_cast<std::uint16_t>(page),
        static_cast<std::uint16_t>(page_count),
        channels.subspan(span.first_sample, span.sample_count),
        values.subspan(span.first_sample, span.sample_count),
        absent.subspan(span.first_absent, span.absent_count));
    fits = builder.GetSize() <= telemetry_wire::kMaxValueFrameBytes;
    if (fits) emit(detail::Bytes(builder), span);
  });
  return fits;
}

// Splits the dictionary into pages of at most one slot, whole channels per
// page, and calls emit(page bytes) for each in order. Throws, naming the
// channel, when one channel's entry cannot fit a page alone.
template <class Emit>
void EncodeDictionaryPages(flatbuffers::FlatBufferBuilder& builder,
                           const PageHeader& header,
                           std::span<const ChannelDef> dictionary,
                           Emit&& emit) {
  const std::size_t room =
      telemetry_wire::kMaxDictionaryPageBytes - detail::DictionaryPageOverhead();
  for (const ChannelDef& channel : dictionary) {
    if (detail::DictionaryEntryBytes(channel) > room) {
      throw std::runtime_error(
          "telemetry: channel '" + channel.name + "' needs about " +
          std::to_string(detail::DictionaryEntryBytes(channel)) +
          " B of dictionary, over the " + std::to_string(room) +
          " B one dictionary page holds (see talOS/telemetry/README.md)");
    }
  }
  const auto partition = [&](auto&& visit) {
    std::size_t first = 0;
    std::size_t page = 0;
    do {
      std::size_t count = 0;
      std::size_t bytes = 0;
      while (first + count < dictionary.size() &&
             bytes + detail::DictionaryEntryBytes(dictionary[first + count]) <=
                 room) {
        bytes += detail::DictionaryEntryBytes(dictionary[first + count]);
        ++count;
      }
      visit(page++, first, count);
      first += count;
    } while (first < dictionary.size());
    return page;
  };
  const std::size_t page_count =
      partition([](std::size_t, std::size_t, std::size_t) {});
  if (page_count > 0xffff) {
    throw std::length_error("telemetry: dictionary needs over 65535 pages");
  }
  partition([&](std::size_t page, std::size_t first, std::size_t count) {
    detail::FinishDictionaryPage(builder, header,
                                 static_cast<std::uint16_t>(page),
                                 static_cast<std::uint16_t>(page_count),
                                 static_cast<std::uint32_t>(first),
                                 dictionary.subspan(first, count));
    if (builder.GetSize() > telemetry_wire::kMaxDictionaryPageBytes) {
      throw std::logic_error("telemetry: dictionary page over its byte bound");
    }
    emit(detail::Bytes(builder));
  });
}

// The pages a dictionary takes on the wire: every channel present (the
// worst case for value pages, whose size depends only on what they carry).
struct PageBudget {
  std::size_t value_pages{0};
  std::size_t value_bytes{0};
  std::size_t largest_value_page{0};
  std::size_t dictionary_pages{0};
  std::size_t dictionary_bytes{0};
};

inline PageBudget MeasurePages(std::span<const ChannelDef> dictionary) {
  PageBudget budget;
  flatbuffers::FlatBufferBuilder builder{1024};
  const PageHeader header{~0ull, ~0ull, ~0ull, ~0u,
                          static_cast<std::uint32_t>(dictionary.size())};
  std::vector<std::uint32_t> channels(dictionary.size());
  for (std::size_t i = 0; i < channels.size(); ++i) {
    channels[i] = static_cast<std::uint32_t>(i);
  }
  const std::vector<std::uint64_t> values(dictionary.size(), ~0ull);
  const Talos::Telemetry::Pose chassis{1, 1, 1, 1, 1, 1};
  if (!EncodeValuePages(
          builder, header, chassis, true, channels, values, {},
          [&](std::span<const std::uint8_t> page, const PageSpan&) {
            ++budget.value_pages;
            budget.value_bytes += page.size();
            budget.largest_value_page =
                std::max(budget.largest_value_page, page.size());
          })) {
    throw std::logic_error("telemetry: a value page outgrew its bound");
  }
  EncodeDictionaryPages(builder, header, dictionary,
                        [&](std::span<const std::uint8_t> page) {
                          ++budget.dictionary_pages;
                          budget.dictionary_bytes += page.size();
                        });
  return budget;
}

}  // namespace talos::telemetry
