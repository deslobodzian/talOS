// Field-layout JSON loader implementation. See the header for the schema
// contract and frame conventions.

#include "talOS/vision/field_sim/field_layout_loader.h"

#include <cmath>
#include <string>
#include <string_view>
#include <utility>

#include "talOS/vision/field_sim/json_parser.h"

namespace talos::vision::field_sim {
namespace {

const JsonValue* RequiredObject(const JsonValue& parent, std::string_view key,
                                const char* what, std::string* error) {
  const JsonValue* v = parent.Find(key);
  if (v == nullptr || v->type != JsonValue::Type::kObject) {
    if (error != nullptr) {
      *error = std::string("field layout: missing object '") +
               std::string(what) + "'";
    }
    return nullptr;
  }
  return v;
}

bool TagPose(const JsonValue& tag, talos::geometry::SE3<double>* out,
             std::string* error) {
  const JsonValue* pose = RequiredObject(tag, "pose", "pose", error);
  if (pose == nullptr) return false;
  const JsonValue* t =
      RequiredObject(*pose, "translation", "translation", error);
  if (t == nullptr) return false;
  const JsonValue* r = RequiredObject(*pose, "rotation", "rotation", error);
  if (r == nullptr) return false;
  const JsonValue* q = RequiredObject(*r, "quaternion", "quaternion", error);
  if (q == nullptr) return false;

  double tx = 0.0, ty = 0.0, tz = 0.0;
  for (const auto& [k, v] : {std::pair{"x", &tx}, {"y", &ty}, {"z", &tz}}) {
    const JsonValue* c = t->Find(k);
    if (c == nullptr || c->type != JsonValue::Type::kNumber) {
      if (error != nullptr) {
        *error =
            std::string("field layout: tag translation missing '") + k + "'";
      }
      return false;
    }
    *v = c->number;
  }
  const Eigen::Vector3d translation(tx, ty, tz);
  double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
  for (const auto& [k, v] :
       {std::pair{"W", &w}, {"X", &x}, {"Y", &y}, {"Z", &z}}) {
    const JsonValue* c = q->Find(k);
    if (c == nullptr || c->type != JsonValue::Type::kNumber) {
      if (error != nullptr) {
        *error =
            std::string("field layout: tag quaternion missing '") + k + "'";
      }
      return false;
    }
    *v = c->number;
  }
  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (!(norm > 1e-9)) {
    if (error != nullptr) *error = "field layout: zero-norm tag quaternion";
    return false;
  }
  const Eigen::Quaterniond quat(w / norm, x / norm, y / norm, z / norm);
  *out = talos::geometry::SE3<double>(
      talos::geometry::SO3<double>(quat.toRotationMatrix()), translation);
  return true;
}

}  // namespace

bool LoadFieldLayoutFromJson(std::string_view text, double tag_size_m,
                             FieldLayoutData* out, std::string* error) {
  if (out == nullptr) return false;
  *out = FieldLayoutData{};
  if (!(tag_size_m > 0.0)) {
    if (error != nullptr) *error = "field layout: tag size must be positive";
    return false;
  }

  JsonValue root;
  if (!ParseJson(text, &root, error)) return false;
  if (root.type != JsonValue::Type::kObject) {
    if (error != nullptr) *error = "field layout: root must be an object";
    return false;
  }

  // Accept both schema generations; prefer the 2026+ key when both exist.
  const JsonValue* tags = root.Find("field-tags");
  if (tags == nullptr) tags = root.Find("tags");
  if (tags == nullptr || tags->type != JsonValue::Type::kArray ||
      tags->arr.empty()) {
    if (error != nullptr) {
      *error = "field layout: missing non-empty 'field-tags' (or 'tags') array";
    }
    return false;
  }

  const JsonValue* dims = root.Find("field-dimensions");
  if (dims == nullptr) dims = root.Find("field");
  if (dims == nullptr || dims->type != JsonValue::Type::kObject) {
    if (error != nullptr) {
      *error = "field layout: missing 'field-dimensions' (or 'field') object";
    }
    return false;
  }
  const JsonValue* length = dims->Find("length");
  const JsonValue* width = dims->Find("width");
  if (length == nullptr || width == nullptr ||
      length->type != JsonValue::Type::kNumber ||
      width->type != JsonValue::Type::kNumber || !(length->number > 0.0) ||
      !(width->number > 0.0)) {
    if (error != nullptr) {
      *error = "field layout: field dimensions need positive length/width";
    }
    return false;
  }
  out->dims.length_m = length->number;
  out->dims.width_m = width->number;
  if (const JsonValue* name = root.Find("name")) out->name = name->AsString("");

  out->layout = FieldLayout(tag_size_m);
  for (const JsonValue& tag : tags->arr) {
    if (tag.type != JsonValue::Type::kObject) {
      if (error != nullptr) *error = "field layout: tag entry must be object";
      return false;
    }
    const JsonValue* id = tag.Find("ID");
    if (id == nullptr || id->type != JsonValue::Type::kNumber) {
      if (error != nullptr) *error = "field layout: tag missing numeric ID";
      return false;
    }
    talos::geometry::SE3<double> pose;
    if (!TagPose(tag, &pose, error)) return false;
    out->layout.SetTagPose(static_cast<int>(id->number), pose);
  }
  return true;
}

}  // namespace talos::vision::field_sim
