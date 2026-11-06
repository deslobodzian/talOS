// Minimal dependency-free JSON parser for small configuration files
// (field layouts, sim camera tables). Supports the full JSON value model
// (null/bool/number/string/array/object) with a bounded nesting depth so a
// hostile file cannot blow the call stack. This intentionally duplicates no
// third-party dependency: the vision stack has no JSON library, and these
// files are kilobytes.
//
// Numbers use strtod; precision matches what WPILib tooling writes (17
// significant digits round-trip through double).

#pragma once

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace talos::vision::field_sim {

struct JsonValue {
  enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };

  Type type = Type::kNull;
  bool boolean = false;
  double number = 0.0;
  std::string str;
  std::vector<JsonValue> arr;
  std::vector<std::string> keys;
  std::vector<JsonValue> values;

  const JsonValue* Find(std::string_view key) const {
    if (type != Type::kObject) return nullptr;
    for (size_t i = 0; i < keys.size(); ++i) {
      if (keys[i] == key) return &values[i];
    }
    return nullptr;
  }

  double AsDouble(double fallback) const {
    return type == Type::kNumber ? number : fallback;
  }

  int AsInt(int fallback) const {
    if (type != Type::kNumber) return fallback;
    return static_cast<int>(number);
  }

  std::string AsString(const std::string& fallback) const {
    return type == Type::kString ? str : fallback;
  }

  bool AsBool(bool fallback) const {
    return type == Type::kBool ? boolean : fallback;
  }
};

namespace json_detail {

inline constexpr int kMaxDepth = 64;

class Parser {
 public:
  explicit Parser(std::string_view text) : text_(text) {}

  bool Run(JsonValue* out, std::string* error) {
    SkipWs();
    if (!ParseValue(out, 0, error)) return false;
    SkipWs();
    if (pos_ != text_.size()) {
      Fail(error, "trailing characters after JSON document");
      return false;
    }
    return true;
  }

 private:
  void SkipWs() {
    while (pos_ < text_.size()) {
      const char c = text_[pos_];
      if (c != ' ' && c != '\t' && c != '\n' && c != '\r') break;
      ++pos_;
    }
  }

  void Fail(std::string* error, const char* what) {
    if (error != nullptr) {
      *error = std::string("json parse error at byte ") + std::to_string(pos_) +
               ": " + what;
    }
  }

  bool Consume(char c) {
    if (pos_ < text_.size() && text_[pos_] == c) {
      ++pos_;
      return true;
    }
    return false;
  }

  bool ParseValue(JsonValue* out, int depth, std::string* error) {
    if (depth > kMaxDepth) {
      Fail(error, "nesting too deep");
      return false;
    }
    if (pos_ >= text_.size()) {
      Fail(error, "unexpected end of input");
      return false;
    }
    const char c = text_[pos_];
    if (c == '{') return ParseObject(out, depth, error);
    if (c == '[') return ParseArray(out, depth, error);
    if (c == '"') {
      out->type = JsonValue::Type::kString;
      return ParseString(&out->str, error);
    }
    if (c == 't' || c == 'f' || c == 'n') return ParseLiteral(out, error);
    if (c == '-' || (c >= '0' && c <= '9')) return ParseNumber(out, error);
    Fail(error, "unexpected character");
    return false;
  }

  bool ParseObject(JsonValue* out, int depth, std::string* error) {
    ++pos_;  // {
    out->type = JsonValue::Type::kObject;
    SkipWs();
    if (Consume('}')) return true;
    while (true) {
      SkipWs();
      if (pos_ >= text_.size() || text_[pos_] != '"') {
        Fail(error, "expected string key");
        return false;
      }
      std::string key;
      if (!ParseString(&key, error)) return false;
      SkipWs();
      if (!Consume(':')) {
        Fail(error, "expected ':' after key");
        return false;
      }
      SkipWs();
      JsonValue value;
      if (!ParseValue(&value, depth + 1, error)) return false;
      out->keys.push_back(std::move(key));
      out->values.push_back(std::move(value));
      SkipWs();
      if (Consume('}')) return true;
      if (!Consume(',')) {
        Fail(error, "expected ',' or '}'");
        return false;
      }
    }
  }

  bool ParseArray(JsonValue* out, int depth, std::string* error) {
    ++pos_;  // [
    out->type = JsonValue::Type::kArray;
    SkipWs();
    if (Consume(']')) return true;
    while (true) {
      SkipWs();
      JsonValue value;
      if (!ParseValue(&value, depth + 1, error)) return false;
      out->arr.push_back(std::move(value));
      SkipWs();
      if (Consume(']')) return true;
      if (!Consume(',')) {
        Fail(error, "expected ',' or ']'");
        return false;
      }
    }
  }

  bool ParseString(std::string* out, std::string* error) {
    ++pos_;  // opening quote
    while (pos_ < text_.size()) {
      const char c = text_[pos_++];
      if (c == '"') return true;
      if (c == '\\') {
        if (pos_ >= text_.size()) break;
        const char e = text_[pos_++];
        switch (e) {
          case '"':
            out->push_back('"');
            break;
          case '\\':
            out->push_back('\\');
            break;
          case '/':
            out->push_back('/');
            break;
          case 'b':
            out->push_back('\b');
            break;
          case 'f':
            out->push_back('\f');
            break;
          case 'n':
            out->push_back('\n');
            break;
          case 'r':
            out->push_back('\r');
            break;
          case 't':
            out->push_back('\t');
            break;
          case 'u': {
            if (pos_ + 4 > text_.size()) {
              Fail(error, "truncated unicode escape");
              return false;
            }
            unsigned code = 0;
            for (int i = 0; i < 4; ++i) {
              const char h = text_[pos_++];
              code <<= 4;
              if (h >= '0' && h <= '9')
                code |= static_cast<unsigned>(h - '0');
              else if (h >= 'a' && h <= 'f')
                code |= static_cast<unsigned>(h - 'a' + 10);
              else if (h >= 'A' && h <= 'F')
                code |= static_cast<unsigned>(h - 'A' + 10);
              else {
                Fail(error, "invalid unicode escape");
                return false;
              }
            }
            // Encode as UTF-8 (no surrogate-pair combining: layout files
            // are ASCII; anything exotic degrades to U+FFFD).
            if (code < 0x80) {
              out->push_back(static_cast<char>(code));
            } else if (code < 0x800) {
              out->push_back(static_cast<char>(0xC0 | (code >> 6)));
              out->push_back(static_cast<char>(0x80 | (code & 0x3F)));
            } else {
              out->push_back(static_cast<char>(0xE0 | (code >> 12)));
              out->push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
              out->push_back(static_cast<char>(0x80 | (code & 0x3F)));
            }
            break;
          }
          default:
            Fail(error, "invalid escape");
            return false;
        }
        continue;
      }
      if (static_cast<unsigned char>(c) < 0x20) {
        Fail(error, "unescaped control character");
        return false;
      }
      out->push_back(c);
    }
    Fail(error, "unterminated string");
    return false;
  }

  bool ParseLiteral(JsonValue* out, std::string* error) {
    if (text_.substr(pos_, 4) == "true") {
      pos_ += 4;
      out->type = JsonValue::Type::kBool;
      out->boolean = true;
      return true;
    }
    if (text_.substr(pos_, 5) == "false") {
      pos_ += 5;
      out->type = JsonValue::Type::kBool;
      out->boolean = false;
      return true;
    }
    if (text_.substr(pos_, 4) == "null") {
      pos_ += 4;
      out->type = JsonValue::Type::kNull;
      return true;
    }
    Fail(error, "invalid literal");
    return false;
  }

  bool ParseNumber(JsonValue* out, std::string* error) {
    // Scan the token first and convert a copy: strtod reads until the first
    // invalid byte, which would over-read when a number ends at the buffer.
    const size_t start = pos_;
    while (pos_ < text_.size()) {
      const char c = text_[pos_];
      const bool digit = (c >= '0' && c <= '9') || c == '+' || c == '-' ||
                         c == '.' || c == 'e' || c == 'E';
      if (!digit) break;
      ++pos_;
    }
    const std::string token(text_.substr(start, pos_ - start));
    char* end = nullptr;
    const double v = std::strtod(token.c_str(), &end);
    if (end == token.c_str()) {
      pos_ = start;
      Fail(error, "invalid number");
      return false;
    }
    // strtod may consume fewer bytes than the scan (e.g. "1e"); rewind the
    // slack so the next token starts cleanly (and errors stay loud).
    pos_ = start + static_cast<size_t>(end - token.c_str());
    out->type = JsonValue::Type::kNumber;
    out->number = v;
    return true;
  }

  std::string_view text_;
  size_t pos_ = 0;
};

}  // namespace json_detail

// Parses a whole JSON document. Returns false with a human-readable error.
inline bool ParseJson(std::string_view text, JsonValue* out,
                      std::string* error = nullptr) {
  if (out == nullptr) return false;
  *out = JsonValue{};
  json_detail::Parser parser(text);
  return parser.Run(out, error);
}

}  // namespace talos::vision::field_sim
