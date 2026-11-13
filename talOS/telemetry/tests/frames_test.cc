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

}  // namespace
}  // namespace talos::telemetry
