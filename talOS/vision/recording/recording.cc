#include "talOS/vision/recording/recording.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace talos::vision::recording {
namespace {

// --- JSON writing ----------------------------------------------------------

void AppendEscaped(std::string& out, std::string_view value) {
  out += '"';
  for (const unsigned char c : value) {
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
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof buf, "\\u%04x", c);
          out += buf;
        } else {
          out += static_cast<char>(c);
        }
    }
  }
  out += '"';
}

void AppendDouble(std::string& out, double value) {
  if (!std::isfinite(value)) {
    value = 0.0;
  }
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.17g", value);
  out += buf;
}

void AppendInt(std::string& out, std::int64_t value) {
  out += std::to_string(value);
}

// --- JSON reading ----------------------------------------------------------

struct JsonMember;

struct Json {
  enum class Type { kNull, kBool, kInt, kDouble, kString, kArray, kObject };
  Type type = Type::kNull;
  bool boolean = false;
  std::int64_t integer = 0;
  double number = 0.0;
  std::string str;
  std::vector<Json> array;
  std::vector<JsonMember> object;

  const Json* Find(std::string_view key) const;
  bool IsNull() const { return type == Type::kNull; }
  std::int64_t AsInt(std::int64_t fallback = 0) const {
    if (type == Type::kInt) {
      return integer;
    }
    if (type == Type::kDouble) {
      return static_cast<std::int64_t>(number);
    }
    return fallback;
  }
  double AsDouble(double fallback = 0.0) const {
    if (type == Type::kDouble) {
      return number;
    }
    if (type == Type::kInt) {
      return static_cast<double>(integer);
    }
    return fallback;
  }
  bool AsBool(bool fallback = false) const {
    return type == Type::kBool ? boolean : fallback;
  }
};

struct JsonMember {
  std::string key;
  Json value;
};

const Json* Json::Find(std::string_view key) const {
  for (const auto& member : object) {
    if (member.key == key) {
      return &member.value;
    }
  }
  return nullptr;
}

class Parser {
 public:
  explicit Parser(std::string_view text) : text_(text) {}

  bool Run(Json* out, std::string* error) {
    SkipWs();
    if (!ParseValue(out, error)) {
      return false;
    }
    SkipWs();
    if (pos_ != text_.size()) {
      return Fail(error, "trailing characters");
    }
    return true;
  }

 private:
  bool Fail(std::string* error, const char* what) {
    if (error != nullptr) {
      *error = what;
    }
    return false;
  }
  void SkipWs() {
    while (pos_ < text_.size()) {
      const char c = text_[pos_];
      if (c != ' ' && c != '\t' && c != '\n' && c != '\r') {
        break;
      }
      ++pos_;
    }
  }
  bool ParseValue(Json* out, std::string* error) {
    if (pos_ >= text_.size()) {
      return Fail(error, "unexpected end of input");
    }
    const char c = text_[pos_];
    if (c == '{') {
      return ParseObject(out, error);
    }
    if (c == '[') {
      return ParseArray(out, error);
    }
    if (c == '"') {
      out->type = Json::Type::kString;
      return ParseString(&out->str, error);
    }
    if (c == 't' || c == 'f' || c == 'n') {
      return ParseLiteral(out, error);
    }
    if (c == '-' || (c >= '0' && c <= '9')) {
      return ParseNumber(out, error);
    }
    return Fail(error, "unexpected character");
  }
  bool ParseObject(Json* out, std::string* error) {
    ++pos_;  // {
    out->type = Json::Type::kObject;
    SkipWs();
    if (pos_ < text_.size() && text_[pos_] == '}') {
      ++pos_;
      return true;
    }
    for (;;) {
      SkipWs();
      std::string key;
      if (!ParseString(&key, error)) {
        return false;
      }
      SkipWs();
      if (pos_ >= text_.size() || text_[pos_] != ':') {
        return Fail(error, "expected ':'");
      }
      ++pos_;
      SkipWs();
      Json value;
      if (!ParseValue(&value, error)) {
        return false;
      }
      out->object.push_back({std::move(key), std::move(value)});
      SkipWs();
      if (pos_ < text_.size() && text_[pos_] == ',') {
        ++pos_;
        continue;
      }
      if (pos_ < text_.size() && text_[pos_] == '}') {
        ++pos_;
        return true;
      }
      return Fail(error, "expected ',' or '}'");
    }
  }
  bool ParseArray(Json* out, std::string* error) {
    ++pos_;  // [
    out->type = Json::Type::kArray;
    SkipWs();
    if (pos_ < text_.size() && text_[pos_] == ']') {
      ++pos_;
      return true;
    }
    for (;;) {
      SkipWs();
      Json value;
      if (!ParseValue(&value, error)) {
        return false;
      }
      out->array.push_back(std::move(value));
      SkipWs();
      if (pos_ < text_.size() && text_[pos_] == ',') {
        ++pos_;
        continue;
      }
      if (pos_ < text_.size() && text_[pos_] == ']') {
        ++pos_;
        return true;
      }
      return Fail(error, "expected ',' or ']'");
    }
  }
  bool ParseString(std::string* out, std::string* error) {
    ++pos_;  // opening quote
    while (pos_ < text_.size()) {
      const char c = text_[pos_++];
      if (c == '"') {
        return true;
      }
      if (c == '\\') {
        if (pos_ >= text_.size()) {
          break;
        }
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
              return Fail(error, "truncated \\u");
            }
            unsigned code = 0;
            for (int i = 0; i < 4; ++i) {
              const char h = text_[pos_++];
              code <<= 4;
              if (h >= '0' && h <= '9') {
                code |= h - '0';
              } else if (h >= 'a' && h <= 'f') {
                code |= h - 'a' + 10;
              } else if (h >= 'A' && h <= 'F') {
                code |= h - 'A' + 10;
              } else {
                return Fail(error, "invalid \\u escape");
              }
            }
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
            return Fail(error, "invalid escape");
        }
        continue;
      }
      if (static_cast<unsigned char>(c) < 0x20) {
        return Fail(error, "unescaped control character");
      }
      out->push_back(c);
    }
    return Fail(error, "unterminated string");
  }
  bool ParseLiteral(Json* out, std::string* error) {
    if (text_.substr(pos_, 4) == "true") {
      pos_ += 4;
      out->type = Json::Type::kBool;
      out->boolean = true;
      return true;
    }
    if (text_.substr(pos_, 5) == "false") {
      pos_ += 5;
      out->type = Json::Type::kBool;
      return true;
    }
    if (text_.substr(pos_, 4) == "null") {
      pos_ += 4;
      out->type = Json::Type::kNull;
      return true;
    }
    return Fail(error, "invalid literal");
  }
  bool ParseNumber(Json* out, std::string* error) {
    const std::size_t start = pos_;
    bool is_int = true;
    while (pos_ < text_.size()) {
      const char c = text_[pos_];
      if (c == '.' || c == 'e' || c == 'E') {
        is_int = false;
      }
      if ((c >= '0' && c <= '9') || c == '+' || c == '-' || c == '.' ||
          c == 'e' || c == 'E') {
        ++pos_;
      } else {
        break;
      }
    }
    const std::string token(text_.substr(start, pos_ - start));
    if (is_int) {
      char* end = nullptr;
      const std::int64_t v = std::strtoll(token.c_str(), &end, 10);
      if (end == token.c_str() || *end != '\0') {
        return Fail(error, "invalid integer");
      }
      out->type = Json::Type::kInt;
      out->integer = static_cast<std::int64_t>(v);
    } else {
      char* end = nullptr;
      const double v = std::strtod(token.c_str(), &end);
      if (end == token.c_str() || *end != '\0') {
        return Fail(error, "invalid number");
      }
      out->type = Json::Type::kDouble;
      out->number = v;
    }
    return true;
  }

  std::string_view text_;
  std::size_t pos_ = 0;
};

bool ReadExtrinsics(const Json& o, Extrinsics* out) {
  out->x_m = o.AsDouble();
  const auto get = [&](const char* k) {
    const Json* v = o.Find(k);
    return v != nullptr ? v->AsDouble() : 0.0;
  };
  out->x_m = get("x_m");
  out->y_m = get("y_m");
  out->z_m = get("z_m");
  out->roll_rad = get("roll_rad");
  out->pitch_rad = get("pitch_rad");
  out->yaw_rad = get("yaw_rad");
  return true;
}

bool ReadIntrinsics(const Json& o, CameraIntrinsics* out) {
  const auto get = [&](const char* k) {
    const Json* v = o.Find(k);
    return v != nullptr ? v->AsDouble() : 0.0;
  };
  out->fx = get("fx");
  out->fy = get("fy");
  out->cx = get("cx");
  out->cy = get("cy");
  out->k1 = get("k1");
  out->k2 = get("k2");
  out->k3 = get("k3");
  out->p1 = get("p1");
  out->p2 = get("p2");
  return true;
}

}  // namespace

std::string RecordStem(std::uint64_t capture_ns, std::string_view reason) {
  std::string out(20, '0');
  std::string digits = std::to_string(capture_ns);
  const std::size_t keep = digits.size() > 20 ? 20 : digits.size();
  std::memcpy(out.data() + (20 - keep), digits.data(), keep);
  out += '_';
  out += reason;
  return out;
}

std::string SerializeRecord(const CameraContext& context,
                            const FrameRecord& record) {
  std::string out;
  out += "{\"format_version\":";
  AppendInt(out, kFormatVersion);
  out += ",\"camera\":";
  AppendEscaped(out, context.camera);
  out += ",\"capture_ns\":";
  AppendInt(out, static_cast<std::int64_t>(record.capture_ns));
  out += ",\"reason\":";
  AppendEscaped(out, record.reason);
  out += ",\"width\":";
  AppendInt(out, context.width);
  out += ",\"height\":";
  AppendInt(out, context.height);
  out += ",\"intrinsics\":{\"fx\":";
  AppendDouble(out, context.intrinsics.fx);
  out += ",\"fy\":";
  AppendDouble(out, context.intrinsics.fy);
  out += ",\"cx\":";
  AppendDouble(out, context.intrinsics.cx);
  out += ",\"cy\":";
  AppendDouble(out, context.intrinsics.cy);
  out += ",\"k1\":";
  AppendDouble(out, context.intrinsics.k1);
  out += ",\"k2\":";
  AppendDouble(out, context.intrinsics.k2);
  out += ",\"k3\":";
  AppendDouble(out, context.intrinsics.k3);
  out += ",\"p1\":";
  AppendDouble(out, context.intrinsics.p1);
  out += ",\"p2\":";
  AppendDouble(out, context.intrinsics.p2);
  out += "},\"mounting\":{\"x_m\":";
  AppendDouble(out, context.mounting.x_m);
  out += ",\"y_m\":";
  AppendDouble(out, context.mounting.y_m);
  out += ",\"z_m\":";
  AppendDouble(out, context.mounting.z_m);
  out += ",\"roll_rad\":";
  AppendDouble(out, context.mounting.roll_rad);
  out += ",\"pitch_rad\":";
  AppendDouble(out, context.mounting.pitch_rad);
  out += ",\"yaw_rad\":";
  AppendDouble(out, context.mounting.yaw_rad);
  out += "},\"detector\":{\"backend\":";
  AppendEscaped(out, context.detector.backend);
  out += ",\"decimate\":";
  AppendDouble(out, context.detector.decimate);
  out += ",\"threads\":";
  AppendInt(out, context.detector.threads);
  out += ",\"sigma\":";
  AppendDouble(out, context.detector.sigma);
  out += ",\"refine_edges\":";
  out += context.detector.refine_edges ? "true" : "false";
  out += ",\"hamming_threshold\":";
  AppendInt(out, context.detector.hamming_threshold);
  out += ",\"min_decision_margin\":";
  AppendDouble(out, context.detector.min_decision_margin);
  out += "},\"detections\":[";
  for (std::size_t i = 0; i < record.detections.size(); ++i) {
    const auto& d = record.detections[i];
    if (i != 0) {
      out += ',';
    }
    out += "{\"tag_id\":";
    AppendInt(out, d.tag_id);
    out += ",\"hamming\":";
    AppendInt(out, d.hamming);
    out += ",\"decision_margin\":";
    AppendDouble(out, d.decision_margin);
    out += ",\"corners_px\":[";
    for (int c = 0; c < 8; ++c) {
      if (c != 0) {
        out += ',';
      }
      AppendDouble(out, d.corners_px[c]);
    }
    out += "],\"valid\":";
    out += d.valid ? "true" : "false";
    out += ",\"reprojection_error_px\":";
    AppendDouble(out, d.reprojection_error_px);
    out += ",\"ambiguity\":";
    AppendDouble(out, d.ambiguity);
    out += ",\"reject_reason\":";
    if (d.reject_reason.has_value()) {
      AppendEscaped(out, *d.reject_reason);
    } else {
      out += "null";
    }
    out += '}';
  }
  out += "],\"observation\":{\"valid\":";
  out += record.observation.valid ? "true" : "false";
  out += ",\"x_m\":";
  AppendDouble(out, record.observation.x_m);
  out += ",\"y_m\":";
  AppendDouble(out, record.observation.y_m);
  out += ",\"yaw_rad\":";
  AppendDouble(out, record.observation.yaw_rad);
  out += ",\"x_std_m\":";
  AppendDouble(out, record.observation.x_std_m);
  out += ",\"y_std_m\":";
  AppendDouble(out, record.observation.y_std_m);
  out += ",\"yaw_std_rad\":";
  AppendDouble(out, record.observation.yaw_std_rad);
  out += ",\"tag_count\":";
  AppendInt(out, record.observation.tag_count);
  out += "},\"truth\":";
  if (record.truth.has_value()) {
    out += "{\"x_m\":";
    AppendDouble(out, record.truth->x_m);
    out += ",\"y_m\":";
    AppendDouble(out, record.truth->y_m);
    out += ",\"yaw_rad\":";
    AppendDouble(out, record.truth->yaw_rad);
    out += '}';
  } else {
    out += "null";
  }
  out += '}';
  return out;
}

bool ParseRecord(std::string_view json, CameraContext* context,
                 FrameRecord* record, std::string* error) {
  if (context == nullptr || record == nullptr) {
    if (error != nullptr) {
      *error = "null output";
    }
    return false;
  }
  *context = CameraContext{};
  *record = FrameRecord{};
  Json root;
  Parser parser{json};
  if (!parser.Run(&root, error)) {
    return false;
  }
  if (root.type != Json::Type::kObject) {
    if (error != nullptr) {
      *error = "top level must be an object";
    }
    return false;
  }
  const Json* camera = root.Find("camera");
  if (camera != nullptr && camera->type == Json::Type::kString) {
    context->camera = camera->str;
  }
  context->width = static_cast<int>(
      root.Find("width") != nullptr ? root.Find("width")->AsInt() : 0);
  context->height = static_cast<int>(
      root.Find("height") != nullptr ? root.Find("height")->AsInt() : 0);
  record->capture_ns = static_cast<std::uint64_t>(
      root.Find("capture_ns") != nullptr ? root.Find("capture_ns")->AsInt()
                                         : 0);
  const Json* reason = root.Find("reason");
  if (reason != nullptr && reason->type == Json::Type::kString) {
    record->reason = reason->str;
  }
  if (const Json* i = root.Find("intrinsics"); i != nullptr) {
    ReadIntrinsics(*i, &context->intrinsics);
  }
  if (const Json* m = root.Find("mounting"); m != nullptr) {
    ReadExtrinsics(*m, &context->mounting);
  }
  if (const Json* det = root.Find("detector"); det != nullptr) {
    if (const Json* v = det->Find("backend");
        v != nullptr && v->type == Json::Type::kString) {
      context->detector.backend = v->str;
    }
    context->detector.decimate = det->Find("decimate") != nullptr
                                     ? det->Find("decimate")->AsDouble()
                                     : 1.0;
    context->detector.threads = static_cast<int>(
        det->Find("threads") != nullptr ? det->Find("threads")->AsInt() : 1);
    context->detector.sigma =
        det->Find("sigma") != nullptr ? det->Find("sigma")->AsDouble() : 0.0;
    context->detector.refine_edges = det->Find("refine_edges") != nullptr
                                         ? det->Find("refine_edges")->AsBool()
                                         : true;
    context->detector.hamming_threshold =
        static_cast<int>(det->Find("hamming_threshold") != nullptr
                             ? det->Find("hamming_threshold")->AsInt()
                             : 1);
    context->detector.min_decision_margin =
        det->Find("min_decision_margin") != nullptr
            ? det->Find("min_decision_margin")->AsDouble()
            : 30.0;
  }
  if (const Json* dets = root.Find("detections");
      dets != nullptr && dets->type == Json::Type::kArray) {
    record->detections.reserve(dets->array.size());
    for (const auto& d : dets->array) {
      DetectionRecord dr;
      dr.tag_id = static_cast<int>(d.AsInt());
      if (const Json* v = d.Find("tag_id"); v != nullptr) {
        dr.tag_id = static_cast<int>(v->AsInt());
      }
      if (const Json* v = d.Find("hamming"); v != nullptr) {
        dr.hamming = static_cast<int>(v->AsInt());
      }
      if (const Json* v = d.Find("decision_margin"); v != nullptr) {
        dr.decision_margin = v->AsDouble();
      }
      if (const Json* v = d.Find("valid"); v != nullptr) {
        dr.valid = v->AsBool();
      }
      if (const Json* v = d.Find("reprojection_error_px"); v != nullptr) {
        dr.reprojection_error_px = v->AsDouble();
      }
      if (const Json* v = d.Find("ambiguity"); v != nullptr) {
        dr.ambiguity = v->AsDouble();
      }
      if (const Json* v = d.Find("reject_reason");
          v != nullptr && v->type == Json::Type::kString) {
        dr.reject_reason = v->str;
      }
      if (const Json* v = d.Find("corners_px");
          v != nullptr && v->type == Json::Type::kArray) {
        for (std::size_t c = 0; c < 8 && c < v->array.size(); ++c) {
          dr.corners_px[c] = static_cast<float>(v->array[c].AsDouble());
        }
      }
      record->detections.push_back(std::move(dr));
    }
  }
  if (const Json* obs = root.Find("observation"); obs != nullptr) {
    record->observation.valid =
        obs->Find("valid") != nullptr ? obs->Find("valid")->AsBool() : false;
    record->observation.x_m =
        obs->Find("x_m") != nullptr ? obs->Find("x_m")->AsDouble() : 0.0;
    record->observation.y_m =
        obs->Find("y_m") != nullptr ? obs->Find("y_m")->AsDouble() : 0.0;
    record->observation.yaw_rad = obs->Find("yaw_rad") != nullptr
                                      ? obs->Find("yaw_rad")->AsDouble()
                                      : 0.0;
    record->observation.x_std_m = obs->Find("x_std_m") != nullptr
                                      ? obs->Find("x_std_m")->AsDouble()
                                      : 0.0;
    record->observation.y_std_m = obs->Find("y_std_m") != nullptr
                                      ? obs->Find("y_std_m")->AsDouble()
                                      : 0.0;
    record->observation.yaw_std_rad = obs->Find("yaw_std_rad") != nullptr
                                          ? obs->Find("yaw_std_rad")->AsDouble()
                                          : 0.0;
    record->observation.tag_count = static_cast<int>(
        obs->Find("tag_count") != nullptr ? obs->Find("tag_count")->AsInt()
                                          : 0);
  }
  if (const Json* truth = root.Find("truth");
      truth != nullptr && !truth->IsNull()) {
    TruthRecord t;
    t.x_m =
        truth->Find("x_m") != nullptr ? truth->Find("x_m")->AsDouble() : 0.0;
    t.y_m =
        truth->Find("y_m") != nullptr ? truth->Find("y_m")->AsDouble() : 0.0;
    t.yaw_rad = truth->Find("yaw_rad") != nullptr
                    ? truth->Find("yaw_rad")->AsDouble()
                    : 0.0;
    record->truth = t;
  }
  return true;
}

}  // namespace talos::vision::recording
