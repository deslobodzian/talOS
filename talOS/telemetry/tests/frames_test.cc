#include "talOS/telemetry/frames.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "talOS/telemetry/telemetry_generated.h"
#include "talOS/telemetry/wire.h"

namespace talos::telemetry {
namespace {

// The measured overhead stays under the bound wire.h sizes the channel cap
// with, so the cap's static_assert holds for real pages.
TEST(ValuePages, OverheadIsWithinTheBoundTheChannelCapAssumes) {
  EXPECT_LE(detail::ValuePageOverhead(),
            telemetry_wire::kValuePageOverheadBound);
}

// Every mix of samples and absent channels across the page boundaries (a
// keyframe or a change-only update, widest scalars) encodes, every page is
// within the cap, and the pages carry each sample and absent channel once.
TEST(ValuePages, EveryMixAtThePageBoundariesFitsThePageCap) {
  flatbuffers::FlatBufferBuilder builder{2048};
  const PageHeader header{~0ull, ~0ull, ~0ull, ~0u, ~0u};
  const Talos::Telemetry::Pose chassis{1, 1, 1, 1, 1, 1};
  std::vector<std::uint32_t> channels(260, ~0u);
  std::vector<std::uint64_t> values(260, ~0ull);
  std::vector<std::uint32_t> absent(600, ~0u);
  std::size_t largest = 0;
  for (const bool keyframe : {false, true}) {
    for (std::size_t samples = 0; samples <= channels.size(); ++samples) {
      for (std::size_t gone = 0; gone <= absent.size(); gone += 7) {
        std::size_t carried = 0;
        std::size_t carried_absent = 0;
        std::size_t pages = 0;
        const bool encoded = EncodeValuePages(
            builder, header, chassis, keyframe,
            std::span{channels}.first(samples),
            std::span{values}.first(samples), std::span{absent}.first(gone),
            [&](std::span<const std::uint8_t> page, const PageSpan& span) {
              ++pages;
              largest = std::max(largest, page.size());
              EXPECT_LE(page.size(), telemetry_wire::kMaxValueFrameBytes);
              EXPECT_EQ(span.first_sample, carried);
              EXPECT_EQ(span.first_absent, carried_absent);
              carried += span.sample_count;
              carried_absent += span.absent_count;
            });
        ASSERT_TRUE(encoded) << samples << " samples, " << gone << " absent";
        EXPECT_EQ(carried, samples);
        EXPECT_EQ(carried_absent, gone);
        EXPECT_GE(pages, 1u);
      }
    }
  }
  // The pages are filled to within the alignment slack: the boundary was
  // reached, not avoided.
  EXPECT_GE(largest,
            telemetry_wire::kMaxValueFrameBytes - 2 * detail::kPageSlackBytes);
}

// The dictionary pager fills a page by DictionaryEntryBytes, so a bound below
// a real entry would build a page past its slot. Every field set (an alert
// level, a unit, labels), at every name length's padding, stays within it.
TEST(DictionaryPages, EntryBoundHoldsWithEveryFieldSet) {
  flatbuffers::FlatBufferBuilder builder{4096};
  DictionaryScratch scratch;
  const PageHeader header{~0ull, ~0ull, ~0ull, ~0u, ~0u};
  for (std::size_t name_bytes = 1; name_bytes <= 8; ++name_bytes) {
    for (const std::size_t label_count : {0u, 1u, 3u}) {
      for (const auto alert : {Talos::Telemetry::AlertLevel_None,
                               Talos::Telemetry::AlertLevel_Error}) {
        for (const std::size_t count : {1u, 20u}) {
          std::vector<ChannelDef> entries;
          std::size_t bound = detail::DictionaryPageOverhead();
          for (std::size_t i = 0; i < count; ++i) {
            ChannelDef channel;
            channel.name = std::string(name_bytes, 'a') + std::to_string(i);
            channel.unit = i % 2 == 0 ? "m/s" : "";
            channel.labels.assign(label_count, std::string(name_bytes, 'l'));
            channel.kind = Talos::Telemetry::ChannelKind_UInt;
            channel.alert = alert;
            bound += detail::DictionaryEntryBytes(channel);
            entries.push_back(std::move(channel));
          }
          detail::FinishDictionaryPage(builder, scratch, header, 0xffff, 0xffff,
                                       ~0u, entries);
          EXPECT_LE(builder.GetSize(), bound)
              << count << " entries, " << name_bytes << " B names, "
              << label_count << " labels, alert " << static_cast<int>(alert);
          const auto* frame = Talos::Telemetry::GetSizePrefixedFrame(
              builder.GetBufferPointer());
          ASSERT_EQ(frame->dictionary()->size(), entries.size());
          EXPECT_EQ(frame->dictionary()->Get(0)->alert(), alert);
        }
      }
    }
  }
}

// REQ-CONTRACT-28: EncodeValuePages rejects mismatched channels/values without
// emitting any page, and emits exactly one valid empty page when nothing is
// carried.
TEST(ValuePages, ErrorAndEmptyPaths) {
  flatbuffers::FlatBufferBuilder builder{2048};
  const PageHeader header{7, 11, 1000, 42, 4};
  const Talos::Telemetry::Pose chassis{0, 0, 0, 0, 0, 0};

  // Mismatched channels vs values lengths: returns false and emits nothing.
  {
    const std::vector<std::uint32_t> channels{0, 1};
    const std::vector<std::uint64_t> values{10};
    std::size_t emitted = 0;
    EXPECT_FALSE(EncodeValuePages(
        builder, header, chassis, true, channels, values, {},
        [&](std::span<const std::uint8_t>, const PageSpan&) { ++emitted; }));
    EXPECT_EQ(emitted, 0u);
  }

  // Empty update still emits one valid page (page 0 of 1).
  {
    std::size_t emitted = 0;
    ASSERT_TRUE(EncodeValuePages(
        builder, header, chassis, false, {}, {}, {},
        [&](std::span<const std::uint8_t> page, const PageSpan& span) {
          ++emitted;
          EXPECT_LE(page.size(), telemetry_wire::kMaxValueFrameBytes);
          EXPECT_EQ(span.sample_count, 0u);
          EXPECT_EQ(span.absent_count, 0u);
          const auto* frame =
              Talos::Telemetry::GetSizePrefixedFrame(page.data());
          ASSERT_NE(frame, nullptr);
          EXPECT_EQ(frame->page(), 0u);
          EXPECT_EQ(frame->page_count(), 1u);
          EXPECT_EQ(frame->channels(), nullptr);
          EXPECT_EQ(frame->absent(), nullptr);
        }));
    EXPECT_EQ(emitted, 1u);
  }
}

// REQ-CONTRACT-28: EncodeDictionaryPages handles empty dictionaries (1 empty
// page), exact single-entry room boundary (accepted on 1 page), 1-byte over
// single-entry room (throws std::runtime_error naming the channel), and
// multi-page splits with exact first_channel/page/page_count invariants.
TEST(DictionaryPages, NearLimitAndErrorPaths) {
  flatbuffers::FlatBufferBuilder builder{4096};
  DictionaryScratch scratch;
  const PageHeader header{9, 0, 2000, 99, 0};

  // Empty dictionary emits one empty page.
  {
    std::size_t pages = 0;
    EncodeDictionaryPages(
        builder, scratch, header, {}, [&](std::span<const std::uint8_t> page) {
          ++pages;
          EXPECT_LE(page.size(), telemetry_wire::kMaxDictionaryPageBytes);
          const auto* frame =
              Talos::Telemetry::GetSizePrefixedFrame(page.data());
          ASSERT_NE(frame, nullptr);
          EXPECT_EQ(frame->page(), 0u);
          EXPECT_EQ(frame->page_count(), 1u);
          EXPECT_EQ(frame->first_channel(), 0u);
          ASSERT_NE(frame->dictionary(), nullptr);
          EXPECT_EQ(frame->dictionary()->size(), 0u);
        });
    EXPECT_EQ(pages, 1u);
  }

  // Construct one channel whose DictionaryEntryBytes equals `room` exactly, and
  // another 1 byte past `room`.
  const std::size_t room = telemetry_wire::kMaxDictionaryPageBytes -
                           detail::DictionaryPageOverhead();
  ChannelDef at_limit;
  at_limit.name = "demo/state/exact";
  at_limit.kind = Talos::Telemetry::ChannelKind_UInt;
  // Base size with 0 labels is 48 + name.size() + 8 + 0 + 8.
  // Adding labels adds 4 + 4*N + sum(label.size() + 8).
  const std::size_t base_with_vec = detail::DictionaryEntryBytes(at_limit) + 4;
  ASSERT_LT(base_with_vec, room);
  const std::size_t remaining = room - base_with_vec;
  // Each 100-byte label costs 4 + 100 + 8 = 112 bytes; adjust the last label
  // so DictionaryEntryBytes(at_limit) == room exactly.
  const std::size_t full_labels = remaining / 112;
  const std::size_t rem = remaining % 112;
  // Ensure remainder >= 12 (minimum cost of a 0-byte label: 4 + 0 + 8 = 12) by
  // borrowing one full label when rem < 12.
  const std::size_t count_100 = rem < 12 ? full_labels - 1 : full_labels;
  const std::size_t last_len = (rem < 12 ? rem + 112 : rem) - 12;
  at_limit.labels.assign(count_100, std::string(100, 'a'));
  at_limit.labels.emplace_back(last_len, 'b');
  ASSERT_EQ(detail::DictionaryEntryBytes(at_limit), room);

  {
    std::size_t pages = 0;
    EncodeDictionaryPages(builder, scratch, header, std::span{&at_limit, 1},
                          [&](std::span<const std::uint8_t> page) {
                            ++pages;
                            EXPECT_LE(page.size(),
                                      telemetry_wire::kMaxDictionaryPageBytes);
                          });
    EXPECT_EQ(pages, 1u);
  }

  // Two at_limit channels must split cleanly into 2 pages with exact
  // first_channel and page_count.
  {
    const std::vector<ChannelDef> two{at_limit, at_limit};
    std::vector<std::uint32_t> firsts;
    std::vector<std::uint16_t> indices;
    EncodeDictionaryPages(
        builder, scratch, header, two, [&](std::span<const std::uint8_t> page) {
          EXPECT_LE(page.size(), telemetry_wire::kMaxDictionaryPageBytes);
          const auto* frame =
              Talos::Telemetry::GetSizePrefixedFrame(page.data());
          ASSERT_NE(frame, nullptr);
          EXPECT_EQ(frame->page_count(), 2u);
          indices.push_back(frame->page());
          firsts.push_back(frame->first_channel());
          ASSERT_NE(frame->dictionary(), nullptr);
          EXPECT_EQ(frame->dictionary()->size(), 1u);
        });
    EXPECT_EQ(indices, (std::vector<std::uint16_t>{0, 1}));
    EXPECT_EQ(firsts, (std::vector<std::uint32_t>{0, 1}));
  }

  // 1 byte past `room` throws std::runtime_error naming the channel.
  ChannelDef over_limit = at_limit;
  over_limit.name = "demo/state/too_big";
  over_limit.labels.back().push_back('z');
  ASSERT_GT(detail::DictionaryEntryBytes(over_limit), room);
  try {
    EncodeDictionaryPages(builder, scratch, header, std::span{&over_limit, 1},
                          [](std::span<const std::uint8_t>) {});
    ADD_FAILURE() << "encoded channel past single dictionary page room";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string{error.what()}.find("'demo/state/too_big'"),
              std::string::npos)
        << error.what();
  }
}

}  // namespace
}  // namespace talos::telemetry
