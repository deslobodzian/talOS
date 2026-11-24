#include "talOS/telemetry/config_validation.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection_generated.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/manifest.h"
#include "talOS/introspection/describe.h"
#include "talOS/telemetry/wire.h"

namespace talos::telemetry {
namespace {

std::string MakeDoubleStructSchema(std::string_view object_name,
                                   std::size_t count, std::size_t name_bytes) {
  flatbuffers::FlatBufferBuilder builder;
  std::vector<flatbuffers::Offset<reflection::Field>> fields;
  fields.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    std::string name = std::to_string(i);
    if (name.size() < name_bytes) name.resize(name_bytes, 'x');
    const auto field_name = builder.CreateString(name);
    reflection::TypeBuilder type_builder{builder};
    type_builder.add_base_type(reflection::BaseType::Double);
    type_builder.add_index(-1);
    const auto type = type_builder.Finish();
    reflection::FieldBuilder field_builder{builder};
    field_builder.add_name(field_name);
    field_builder.add_type(type);
    field_builder.add_id(static_cast<std::uint16_t>(i));
    field_builder.add_offset(static_cast<std::uint16_t>(i * sizeof(double)));
    fields.push_back(field_builder.Finish());
  }
  const auto obj_name = builder.CreateString(object_name);
  const auto field_vector = builder.CreateVector(fields);
  reflection::ObjectBuilder object_builder{builder};
  object_builder.add_name(obj_name);
  object_builder.add_fields(field_vector);
  object_builder.add_is_struct(true);
  object_builder.add_minalign(8);
  object_builder.add_bytesize(
      static_cast<std::int32_t>(count * sizeof(double)));
  const auto object = object_builder.Finish();
  const auto objects = builder.CreateVector(
      std::vector<flatbuffers::Offset<reflection::Object>>{object});
  const auto enums = builder.CreateVector(
      std::vector<flatbuffers::Offset<reflection::Enum>>{});
  reflection::SchemaBuilder schema_builder{builder};
  schema_builder.add_objects(objects);
  schema_builder.add_enums(enums);
  reflection::FinishSchemaBuffer(builder, schema_builder.Finish());
  return {reinterpret_cast<const char*>(builder.GetBufferPointer()),
          builder.GetSize()};
}

// REQ-OPS-22: BuildTelemetryDictionary accumulates excess channels beyond the
// per-topic flattening cap into `uncounted`, bucketed by 2-segment owner/role,
// so CheckDictionaryChannelCap reports the true count without splitting
// 3-segment topics across separate buckets.
TEST(ConfigValidationTest, BuildDictionaryAccumulatesUncountedByOwnerRole) {
  const std::string bfbs = MakeDoubleStructSchema("big.Sub", 2500, 4);
  introspect::naming::SourceShape source;
  source.kind = event::SourceKind::SENDER;
  source.name = "/big/state/sub";
  source.message_bytes = 2500 * sizeof(double);
  source.alignment = 8;
  source.schema = "big.Sub";
  source.schema_hash = 42;
  source.bfbs = bfbs;

  introspect::Description node;
  node.name = "big";
  node.target = "//big:node";
  node.sources = {source};

  std::map<std::string, std::size_t> uncounted{{"stale/bucket", 999}};
  const std::vector<ChannelDef> dict =
      BuildTelemetryDictionary({node}, {"*"}, {}, &uncounted);
  // 1 (pose/valid) + (kMaxChannels + 1) built channels.
  ASSERT_EQ(dict.size(), telemetry_wire::kMaxChannels + 2);
  // Stale entries were cleared and excess 2500 - 2049 = 451 bucketed under
  // "big/state".
  ASSERT_EQ(uncounted.size(), 1u);
  EXPECT_EQ(uncounted.at("big/state"),
            2500u - (telemetry_wire::kMaxChannels + 1));

  try {
    CheckDictionaryChannelCap(dict, uncounted);
    ADD_FAILURE() << "accepted dictionary over kMaxChannels";
  } catch (const std::runtime_error& error) {
    const std::string_view msg = error.what();
    EXPECT_NE(msg.find("2501 channels"), std::string_view::npos) << msg;
    EXPECT_NE(msg.find("big/state (2500)"), std::string_view::npos) << msg;
    EXPECT_EQ(msg.find("big/state/sub"), std::string_view::npos) << msg;
  }
}

// REQ-CONTRACT-28, REQ-OPS-22: CheckDictionaryPageBudget admits dictionaries
// whose dictionary + value pages fit within `telemetry_wire::slot_count` and
// refuses dictionaries whose resend tick would exceed the 64-slot ring.
TEST(ConfigValidationTest, PageBudgetRefusesCombinedPagesOverSlotCount) {
  std::vector<ChannelDef> fits;
  for (std::size_t i = 0; i < 64; ++i) {
    fits.push_back(ChannelDef{
        .name = "demo/state/c" + std::to_string(i),
        .kind = Talos::Telemetry::ChannelKind_Double,
    });
  }
  const PageBudget ok = CheckDictionaryPageBudget(fits);
  EXPECT_LE(ok.dictionary_pages + ok.value_pages, telemetry_wire::slot_count);

  // 64 channels each carrying ~12 KiB of enum labels -> 64 dictionary pages +
  // 1 value page = 65 > 64 slots.
  std::vector<ChannelDef> over;
  const std::vector<std::string> big_labels(80, std::string(140, 'l'));
  for (std::size_t i = 0; i < 64; ++i) {
    over.push_back(ChannelDef{
        .name = "demo/state/enum_" + std::to_string(i),
        .labels = big_labels,
        .kind = Talos::Telemetry::ChannelKind_UInt,
    });
  }
  try {
    static_cast<void>(CheckDictionaryPageBudget(over));
    ADD_FAILURE() << "accepted dictionary exceeding 64 combined pages";
  } catch (const std::runtime_error& error) {
    const std::string_view msg = error.what();
    EXPECT_NE(msg.find("65 pages"), std::string_view::npos) << msg;
    EXPECT_NE(msg.find("64 dictionary + 1 value"), std::string_view::npos)
        << msg;
    EXPECT_NE(msg.find("64-slot"), std::string_view::npos) << msg;
  }
}

// REQ-OPS-38, REQ-CONTRACT-10: ParseLabelSpecs invokes `on_own_string_list`
// with `"labels"` when `labels = []` so LintParameterKeys registers `"labels"`
// as read.
TEST(ConfigValidationTest, EmptyLabelsRegistersOwnStringListRead) {
  const auto cfg = config::ParseRobotConfigString(R"(
[robot]
period_us = 10000
[subsystems.telemetry]
node = "//talOS/telemetry:node"
labels = []
)");
  std::vector<std::string> own_strings;
  std::vector<std::string> own_lists;
  const std::vector<LabelSpec> specs = ParseLabelSpecs(
      cfg, "telemetry",
      LabelParamHooks{
          .on_own_string =
              [&](std::string_view key) { own_strings.emplace_back(key); },
          .on_own_string_list =
              [&](std::string_view key) { own_lists.emplace_back(key); },
      });
  EXPECT_TRUE(specs.empty());
  EXPECT_EQ(own_lists, (std::vector<std::string>{"labels"}));
  EXPECT_EQ(own_strings,
            (std::vector<std::string>{"labels[0].channel", "labels[0].from"}));
}

// REQ-OPS-22: ChannelSourceBucket normalizes leading slashes and collapses
// 3+ segment topic/channel names to `owner/role`.
TEST(ConfigValidationTest, ChannelSourceBucketNormalizesTopicsAndChannels) {
  EXPECT_EQ(ChannelSourceBucket("/budget/state"), "budget/state");
  EXPECT_EQ(ChannelSourceBucket("/budget/state/sub"), "budget/state");
  EXPECT_EQ(ChannelSourceBucket("budget/state/sub/field_m"), "budget/state");
  EXPECT_EQ(ChannelSourceBucket("pose/valid"), "pose/valid");
}

}  // namespace
}  // namespace talos::telemetry
