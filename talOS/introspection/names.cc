#include "talOS/introspection/names.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "flatbuffers/reflection.h"
#include "talOS/schema/verify.h"

namespace talos::introspect::naming::detail {

namespace {

constexpr std::size_t kMaxAlertWalkDepth = 64;

class AlertFieldWalk {
 public:
  AlertFieldWalk(const reflection::Schema& schema,
                 std::vector<AlertPrefixedField>& out)
      : schema_{schema},
        out_{out},
        seen_(schema.objects() != nullptr ? schema.objects()->size() : 0) {}

  void Struct(std::size_t root_index) {
    struct Frame {
      std::size_t index;
      std::size_t field_pos;
      std::string prefix;
      std::size_t depth;
    };
    if (root_index >= seen_.size() || seen_[root_index]) return;
    seen_[root_index] = true;
    std::vector<Frame> stack;
    stack.push_back({root_index, 0, {}, 0});
    while (!stack.empty()) {
      Frame& frame = stack.back();
      const reflection::Object* object = schema_.objects()->Get(
          static_cast<flatbuffers::uoffset_t>(frame.index));
      if (object == nullptr || object->fields() == nullptr ||
          frame.field_pos >= object->fields()->size()) {
        stack.pop_back();
        continue;
      }
      const reflection::Field* field = object->fields()->Get(
          static_cast<flatbuffers::uoffset_t>(frame.field_pos++));
      if (field == nullptr || field->name() == nullptr ||
          field->type() == nullptr) {
        continue;
      }
      const std::string name = field->name()->str();
      const std::string path =
          frame.prefix.empty() ? name : frame.prefix + "." + name;
      const reflection::Type& type = *field->type();
      const bool nested = type.base_type() == reflection::BaseType::Obj ||
                          (type.base_type() == reflection::BaseType::Array &&
                           type.element() == reflection::BaseType::Obj);
      if (name.starts_with(kAlertPrefix)) {
        out_.push_back(
            {path, name, type.base_type() == reflection::BaseType::Bool});
      }
      if (nested && type.index() >= 0 && frame.depth + 1 < kMaxAlertWalkDepth) {
        const auto next_idx = static_cast<std::size_t>(type.index());
        if (next_idx < seen_.size() && !seen_[next_idx]) {
          seen_[next_idx] = true;
          stack.push_back({next_idx, 0, path, frame.depth + 1});
        }
      }
    }
  }

 private:
  const reflection::Schema& schema_;
  std::vector<AlertPrefixedField>& out_;
  std::vector<bool> seen_;  // by object index: walked already
};

}  // namespace

AlertFieldsResult AlertPrefixedFields(std::string_view bfbs,
                                      std::string_view root) {
  AlertFieldsResult result;
  // Uses header-only //talOS/schema:verify rather than :field_path: this
  // library is part of libtalos_node.so, and a compiled library in that
  // closure is left out of every talos_cc_shared_node's own link.
  const reflection::Schema* schema = schema::VerifySchema(bfbs);
  if (schema == nullptr || schema->objects() == nullptr) {
    result.error = "reflection schema (bfbs) failed verification";
    return result;
  }
  if (root.empty()) {
    result.error = "reflection schema (bfbs) has no root struct name";
    return result;
  }
  const auto* objects = schema->objects();
  for (flatbuffers::uoffset_t i = 0; i < objects->size(); ++i) {
    const auto* obj = objects->Get(i);
    if (obj != nullptr && obj->name() != nullptr &&
        obj->name()->string_view() == root) {
      AlertFieldWalk{*schema, result.fields}.Struct(i);
      return result;
    }
  }
  result.error = "reflection schema (bfbs) does not contain struct '" +
                 std::string{root} + "'";
  return result;
}

}  // namespace talos::introspect::naming::detail
