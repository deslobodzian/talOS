// talOS/launcher/wpilog.h against hand-built bytes of the WPILib DataLog 1.0
// layout: what AdvantageScope and WPILib's DataLogReader expect to parse.

#include "talOS/launcher/wpilog.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes Of(const std::string& text) { return {text.begin(), text.end()}; }

Bytes Cat(std::initializer_list<Bytes> parts) {
  Bytes out;
  for (const Bytes& part : parts) {
    out.insert(out.end(), part.begin(), part.end());
  }
  return out;
}

Bytes HeaderNoExtra() {
  return {'W', 'P', 'I', 'L', 'O', 'G', 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
}

// Guards: the file header is "WPILOG", version 0x0100 little-endian, and a
// length-prefixed extra header.
TEST(WpilogWriterTest, FileHeaderCarriesVersionAndExtraHeader) {
  std::ostringstream out;
  talos::wpilog::Writer writer{out, "ab"};
  ASSERT_TRUE(writer.ok());
  EXPECT_EQ(
      Of(out.str()),
      Cat({Of("WPILOG"), {0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 'a', 'b'}}));
}

// Guards: Start/SetMetadata/Finish control records in entry 0, entry ids
// from 1, and the minimal 1-byte widths of a small record.
TEST(WpilogWriterTest, ControlRecordsMatchTheSpecLayout) {
  std::ostringstream out;
  talos::wpilog::Writer writer{out, ""};
  EXPECT_EQ(writer.start("a", "double", "m", 0), 1u);
  EXPECT_EQ(writer.start("bc", "int64", "", 5), 2u);
  writer.set_metadata(1, "xy", 7);
  writer.finish(2, 9);
  ASSERT_TRUE(writer.ok());
  const Bytes start_a = Cat({
      // header byte, entry 0, payload size 25, timestamp 0
      {0x00, 0x00, 25, 0x00},
      // Start, entry 1
      {0x00, 0x01, 0x00, 0x00, 0x00},
      {0x01, 0x00, 0x00, 0x00, 'a'},
      Cat({{0x06, 0x00, 0x00, 0x00}, Of("double")}),
      {0x01, 0x00, 0x00, 0x00, 'm'},
  });
  const Bytes start_bc = Cat({
      {0x00, 0x00, 24, 0x05},
      {0x00, 0x02, 0x00, 0x00, 0x00},
      {0x02, 0x00, 0x00, 0x00, 'b', 'c'},
      Cat({{0x05, 0x00, 0x00, 0x00}, Of("int64")}),
      {0x00, 0x00, 0x00, 0x00},
  });
  const Bytes set_metadata = {0x00, 0x00, 11,   0x07, 0x02, 0x01, 0x00, 0x00,
                              0x00, 0x02, 0x00, 0x00, 0x00, 'x',  'y'};
  const Bytes finish = {0x00, 0x00, 5, 0x09, 0x01, 0x02, 0x00, 0x00, 0x00};
  EXPECT_EQ(Of(out.str()),
            Cat({HeaderNoExtra(), start_a, start_bc, set_metadata, finish}));
}

// Guards: a data record's payload encodings, little-endian on any host:
// double, float, int64 (two's complement) and boolean.
TEST(WpilogWriterTest, ScalarPayloadsAreLittleEndian) {
  Bytes payload;
  talos::wpilog::AppendDouble(payload, 1.5);
  EXPECT_EQ(payload, (Bytes{0, 0, 0, 0, 0, 0, 0xF8, 0x3F}));
  payload.clear();
  talos::wpilog::AppendFloat(payload, -2.0f);
  EXPECT_EQ(payload, (Bytes{0x00, 0x00, 0x00, 0xC0}));
  payload.clear();
  talos::wpilog::AppendInt64(payload, -2);
  EXPECT_EQ(payload, (Bytes{0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}));
  payload.clear();
  talos::wpilog::AppendBoolean(payload, true);
  talos::wpilog::AppendBoolean(payload, false);
  EXPECT_EQ(payload, (Bytes{0x01, 0x00}));

  std::ostringstream out;
  talos::wpilog::Writer writer{out, ""};
  const Bytes value = {0, 0, 0, 0, 0, 0, 0xF8, 0x3F};
  writer.append(1, value, 1000);
  ASSERT_TRUE(writer.ok());
  // Timestamp 1000 = 0x03E8 takes two bytes: header bits 4-6 = 1.
  EXPECT_EQ(Of(out.str()),
            Cat({HeaderNoExtra(), {0x10, 0x01, 0x08, 0xE8, 0x03}, value}));
}

// Guards: the header byte's three widths (entry, size, timestamp minus one)
// and the variable-length fields at their widest.
TEST(WpilogWriterTest, RecordHeaderWidthsGrowWithValues) {
  Bytes out;
  const Bytes payload(300, 0xAB);
  talos::wpilog::AppendRecord(out, 0x012345, 0x0102030405, payload);
  // entry 3 bytes (2), size 2 bytes (1 << 2), timestamp 5 bytes (4 << 4).
  EXPECT_EQ(
      out,
      Cat({{0x46, 0x45, 0x23, 0x01, 0x2C, 0x01, 0x05, 0x04, 0x03, 0x02, 0x01},
           payload}));

  out.clear();
  talos::wpilog::AppendRecord(out, 0xFFFFFFFFu, 0xFFFFFFFFFFFFFFFFu, {});
  EXPECT_EQ(out, (Bytes{0x73, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF,
                        0xFF, 0xFF, 0xFF, 0xFF, 0xFF}));
}

}  // namespace
