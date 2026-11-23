#pragma once

#include <string>
#include <string_view>

namespace talos::telemetry {

// Appends `value` to `out` as a quoted JSON string: `"` and `\` are
// backslash-escaped, newline, carriage return and tab are written as \n, \r
// and \t, every other byte below 0x20 as \u00XX in lowercase hex, and every
// byte from 0x20 up (0x7f and non-ASCII included) as is. The dashboard
// layout (/drive/layout.json) and the Studio bridge's JSON both write their
// strings through it. talOS/vision/recording/recording.cc keeps its own copy
// with the same escaping rules because talOS/vision does not depend on
// talOS/telemetry.
inline void AppendEscaped(std::string& out, std::string_view value) {
  out += '"';
  for (const char c : value) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          static constexpr char kHex[] = "0123456789abcdef";
          const auto byte = static_cast<unsigned char>(c);
          out += "\\u00";
          out += kHex[byte >> 4];
          out += kHex[byte & 0xF];
        } else {
          out += c;
        }
    }
  }
  out += '"';
}

}  // namespace talos::telemetry
