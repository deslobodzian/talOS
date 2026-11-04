#include "talOS/schema/decode.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

#include "schema_test/shapes_generated.h"

namespace talos::schema {
namespace {

template <class T>
std::span<const std::byte> BytesOf(const T& value) {
  return std::as_bytes(std::span<const T, 1>{&value, 1});
}

TEST(Decode, NamesEveryFieldInDeclarationOrder) {
  const schema_test::Mixed mixed{
      true, 1.5, schema_test::Level::TOP, 7,
      schema_test::Inner{schema_test::Mode::LOW, 0.25f}, 0.5f};
  EXPECT_EQ(DecodeToText(Of<schema_test::Mixed>(), BytesOf(mixed)),
            "{flag: true, position_m: 1.5, level: TOP, sample_count: 7, "
            "inner: {mode: LOW, offset_m: 0.25}, speed_frac: 0.5}");
}

TEST(Decode, AnEnumValueWithNoNamePrintsItsNumber) {
  schema_test::Inner inner{static_cast<schema_test::Mode>(9), 1.0f};
  EXPECT_EQ(DecodeToText(Of<schema_test::Inner>(), BytesOf(inner)),
            "{mode: 9, offset_m: 1}");
}

TEST(Decode, FixedArraysPrintEachElement) {
  std::vector<std::byte> bytes(sizeof(schema_test::Arrays));
  const double samples[3] = {1.0, 2.0, 3.0};
  std::memcpy(bytes.data(), samples, sizeof samples);
  const std::string text = DecodeToText(Of<schema_test::Arrays>(), bytes);
  EXPECT_NE(text.find("samples_m: [1, 2, 3]"), std::string::npos) << text;
  EXPECT_NE(text.find("modes: [OFF, OFF]"), std::string::npos) << text;
  EXPECT_NE(text.find("inners: [{mode: OFF, offset_m: 0}, {mode: OFF, "
                      "offset_m: 0}]"),
            std::string::npos)
      << text;
  EXPECT_NE(text.find("channel_id: [0, 0, 0, 0, 0]"), std::string::npos)
      << text;
}

TEST(Decode, ATargetEnvelopeDecodesItsHeaderThenTheMessage) {
  const schema_test::Inner inner{schema_test::Mode::HIGH, 2.0f};
  std::vector<std::byte> bytes(kEnvelopeHeaderBytes + sizeof inner);
  const std::uint64_t since = 1200;
  std::memcpy(bytes.data(), &since, sizeof since);
  bytes[kEnvelopeActiveOffset] = std::byte{1};
  std::memcpy(bytes.data() + kEnvelopeHeaderBytes, &inner, sizeof inner);
  EXPECT_EQ(DecodeToText(Envelope(Of<schema_test::Inner>()), bytes),
            "{mode_since_us: 1200, active: true, message: {mode: HIGH, "
            "offset_m: 2}}");
}

TEST(Decode, ALayoutWithoutSchemaBytesPrintsHex) {
  std::vector<std::byte> bytes(100);
  bytes[0] = std::byte{0xab};
  bytes[2] = std::byte{0x01};
  EXPECT_EQ(DecodeToText(Bytes(), bytes), "bytes=100 hex=ab0001");

  std::vector<std::byte> long_bytes(kMaxHexBytes + 10, std::byte{0xff});
  const std::string text = DecodeToText(Bytes(), long_bytes);
  EXPECT_EQ(text.size(),
            std::string{"bytes=74 hex="}.size() + 2 * kMaxHexBytes +
                std::string{"...(+10)"}.size())
      << text;
}

TEST(Decode, AWrongSizePayloadSaysSoInsteadOfReadingPastIt) {
  const std::vector<std::byte> bytes(3);
  EXPECT_EQ(DecodeToText(Of<schema_test::Inner>(), bytes),
            "<3 bytes, schema_test.Inner is 8>");
}

}  // namespace
}  // namespace talos::schema
