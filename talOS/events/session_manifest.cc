#include "talOS/events/session_manifest.h"

#include <charconv>
#include <fstream>
#include <iterator>
#include <system_error>

namespace talos::event::session_manifest {

namespace {

constexpr int kMaxDepth = 32;

// One pass over a manifest's text: reads the members Fields holds and skips
// every other value, checking only that it is JSON. Not a general reader: it
// keeps nothing it was not asked for.
class Scanner {
 public:
  explicit Scanner(std::string_view text) : text_{text} {}

  Fields Manifest() {
    Fields fields;
    SkipSpace();
    if (Peek() == '{') {
      Object([&](std::string_view key) {
        if (key == kSessionId) {
          fields.session_id = Unsigned();
        } else if (key == kHardwareProtocolVersion) {
          fields.hardware_protocol_version = Unsigned();
        } else if (key == kConfigPath) {
          if (Peek() == '"') {
            fields.config_path = String();
          } else {
            fields.config_path.reset();
            Skip(1);
          }
        } else if (key == kSimulation) {
          fields.simulation = OptionalBool();
        } else if (key == kStartWallNs) {
          fields.start_wall_ns = Unsigned();
        } else if (key == kEndWallNs) {
          fields.end_wall_ns = Unsigned();
        } else if (key == kNodes) {
          fields.nodes = Nodes();
        } else {
          Skip(1);
        }
      });
    } else {
      Skip(0);
    }
    SkipSpace();
    if (at_ != text_.size()) Fail("text after its value");
    return fields;
  }

 private:
  [[noreturn]] void Fail(std::string_view what) const {
    throw ManifestError("at byte " + std::to_string(at_) + ": " +
                        std::string{what});
  }

  char Peek() const { return at_ < text_.size() ? text_[at_] : '\0'; }

  void SkipSpace() {
    while (at_ < text_.size() && (text_[at_] == ' ' || text_[at_] == '\t' ||
                                  text_[at_] == '\n' || text_[at_] == '\r')) {
      ++at_;
    }
  }

  void Expect(char c) {
    if (Peek() != c) Fail(std::string{"expected '"} + c + "'");
    ++at_;
  }

  // An object at the cursor; `member(key)` reads each member's value, the
  // cursor at its first character.
  template <class Member>
  void Object(Member member) {
    Expect('{');
    SkipSpace();
    if (Peek() == '}') {
      ++at_;
      return;
    }
    while (true) {
      SkipSpace();
      const std::string key = String();
      SkipSpace();
      Expect(':');
      SkipSpace();
      member(key);
      SkipSpace();
      if (Peek() == '}') {
        ++at_;
        return;
      }
      Expect(',');
    }
  }

  // An array at the cursor; `element()` reads each element, the cursor at
  // its first character.
  template <class Element>
  void Array(Element element) {
    Expect('[');
    SkipSpace();
    if (Peek() == ']') {
      ++at_;
      return;
    }
    while (true) {
      SkipSpace();
      element();
      SkipSpace();
      if (Peek() == ']') {
        ++at_;
        return;
      }
      Expect(',');
    }
  }

  // Any value, inside `depth` containers.
  void Skip(int depth) {
    if (depth > kMaxDepth) Fail("nesting deeper than 32");
    switch (Peek()) {
      case '{':
        Object([&](std::string_view) { Skip(depth + 1); });
        return;
      case '[':
        Array([&] { Skip(depth + 1); });
        return;
      case '"':
        String();
        return;
      case 't':
        Literal("true");
        return;
      case 'f':
        Literal("false");
        return;
      case 'n':
        Literal("null");
        return;
      default:
        Number();
    }
  }

  void Literal(std::string_view word) {
    if (text_.substr(at_, word.size()) != word) Fail("not a JSON value");
    at_ += word.size();
  }

  // A number's text: -?digits[.digits][(e|E)[+|-]digits].
  std::string_view Number() {
    const std::size_t start = at_;
    const auto digits = [&] {
      const std::size_t first = at_;
      while (Peek() >= '0' && Peek() <= '9') ++at_;
      if (at_ == first) Fail("not a JSON value");
    };
    if (Peek() == '-') ++at_;
    digits();
    if (Peek() == '.') {
      ++at_;
      digits();
    }
    if (Peek() == 'e' || Peek() == 'E') {
      ++at_;
      if (Peek() == '+' || Peek() == '-') ++at_;
      digits();
    }
    return text_.substr(start, at_ - start);
  }

  // A value that is an unsigned decimal integer fitting in 64 bits, or
  // nullopt for any other value (skipped).
  std::optional<std::uint64_t> Unsigned() {
    if (Peek() != '-' && (Peek() < '0' || Peek() > '9')) {
      Skip(1);
      return std::nullopt;
    }
    const std::string_view number = Number();
    std::uint64_t value = 0;
    const auto [end, error] =
        std::from_chars(number.data(), number.data() + number.size(), value);
    if (error != std::errc{} || end != number.data() + number.size()) {
      return std::nullopt;
    }
    return value;
  }

  // A JSON boolean, or nullopt for any other value (skipped).
  std::optional<bool> OptionalBool() {
    if (Peek() == 't') {
      Literal("true");
      return true;
    }
    if (Peek() == 'f') {
      Literal("false");
      return false;
    }
    Skip(1);
    return std::nullopt;
  }

  bool Bool(std::string_view key) {
    if (Peek() == 't') {
      Literal("true");
      return true;
    }
    if (Peek() == 'f') {
      Literal("false");
      return false;
    }
    Fail("a member's " + std::string{key} + " is not true or false");
  }

  unsigned Hex4() {
    unsigned value = 0;
    for (int i = 0; i < 4; ++i, ++at_) {
      const char c = Peek();
      value <<= 4U;
      if (c >= '0' && c <= '9') {
        value |= static_cast<unsigned>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        value |= static_cast<unsigned>(c - 'a' + 10);
      } else if (c >= 'A' && c <= 'F') {
        value |= static_cast<unsigned>(c - 'A' + 10);
      } else {
        Fail("a \\u escape without four hex digits");
      }
    }
    return value;
  }

  static void AppendUtf8(std::string& out, unsigned code) {
    // Each continuation byte: 10 and the code's next six bits.
    const auto continuation = [&](unsigned shift) {
      out += static_cast<char>(0x80U | ((code >> shift) & 0x3FU));
    };
    if (code < 0x80U) {
      out += static_cast<char>(code);
    } else if (code < 0x800U) {
      out += static_cast<char>(0xC0U | (code >> 6U));
      continuation(0U);
    } else if (code < 0x10000U) {
      out += static_cast<char>(0xE0U | (code >> 12U));
      continuation(6U);
      continuation(0U);
    } else {
      out += static_cast<char>(0xF0U | (code >> 18U));
      continuation(12U);
      continuation(6U);
      continuation(0U);
    }
  }

  // A string at the cursor, its escapes decoded.
  std::string String() {
    Expect('"');
    std::string value;
    while (true) {
      if (at_ >= text_.size()) Fail("a string without its closing quote");
      const char c = text_[at_++];
      if (c == '"') return value;
      if (static_cast<unsigned char>(c) < 0x20) {
        Fail("a control character in a string");
      }
      if (c != '\\') {
        value += c;
        continue;
      }
      if (at_ >= text_.size()) Fail("a string without its closing quote");
      const char escape = text_[at_++];
      switch (escape) {
        case '"':
        case '\\':
        case '/':
          value += escape;
          break;
        case 'b':
          value += '\b';
          break;
        case 'f':
          value += '\f';
          break;
        case 'n':
          value += '\n';
          break;
        case 'r':
          value += '\r';
          break;
        case 't':
          value += '\t';
          break;
        case 'u': {
          unsigned code = Hex4();
          if (code >= 0xD800 && code < 0xDC00 &&
              text_.substr(at_, 2) == "\\u") {
            at_ += 2;
            const unsigned low = Hex4();
            if (low < 0xDC00 || low > 0xDFFF) Fail("an unpaired surrogate");
            code = 0x10000U + ((code - 0xD800U) << 10U) + (low - 0xDC00U);
          } else if (code >= 0xD800 && code <= 0xDFFF) {
            Fail("an unpaired surrogate");
          }
          AppendUtf8(value, code);
          break;
        }
        default:
          --at_;
          Fail("an unknown escape");
      }
    }
  }

  std::vector<Member> Nodes() {
    if (Peek() != '[') Fail("its nodes is not an array");
    std::vector<Member> nodes;
    Array([&] {
      if (Peek() != '{') Fail("a member of nodes is not an object");
      Member member;
      Object([&](std::string_view key) {
        if (key == kLogGap) {
          member.log_gap = Bool(key);
        } else if (key == kLogCapped) {
          member.log_capped = Bool(key);
        } else if (key == kRecorderFailed) {
          member.recorder_failed = Bool(key);
        } else if (key == kLogPaths) {
          member.log_paths = StringArray();
        } else if (key == kGapLogs) {
          member.gap_logs = StringArray();
        } else if (key == kRecorderFailedLogs) {
          member.recorder_failed_logs = StringArray();
        } else {
          Skip(3);
        }
      });
      nodes.push_back(std::move(member));
    });
    return nodes;
  }

  // An array of strings, or nullopt for any other value (skipped).
  std::optional<std::vector<std::string>> StringArray() {
    if (Peek() != '[') {
      Skip(3);
      return std::nullopt;
    }
    std::vector<std::string> paths;
    bool strings = true;
    Array([&] {
      if (Peek() == '"') {
        paths.push_back(String());
      } else {
        strings = false;
        Skip(4);
      }
    });
    if (!strings) return std::nullopt;
    return paths;
  }

  std::string_view text_;
  std::size_t at_{0};
};

}  // namespace

Fields Parse(std::string_view text) { return Scanner{text}.Manifest(); }

std::optional<std::string> ReadText(const std::filesystem::path& directory) {
  std::ifstream stream{directory / kFileName, std::ios::binary};
  if (!stream) return std::nullopt;
  return std::string{std::istreambuf_iterator<char>{stream}, {}};
}

}  // namespace talos::event::session_manifest
