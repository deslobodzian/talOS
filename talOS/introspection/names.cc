#include "talOS/introspection/names.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "flatbuffers/reflection.h"
#include "flatbuffers/verifier.h"

namespace talos::introspect::naming::detail {

namespace {

class AlertFieldWalk {
 public:
  AlertFieldWalk(const reflection::Schema& schema,
                 std::vector<AlertPrefixedField>& out)
      : schema_{schema}, out_{out}, seen_(schema.objects()->size()) {}

  void Struct(std::size_t index, const std::string& prefix) {
    if (index >= seen_.size() || seen_[index]) return;
    seen_[index] = true;
    const reflection::Object& object =
        *schema_.objects()->Get(static_cast<flatbuffers::uoffset_t>(index));
    for (const reflection::Field* field : *object.fields()) {
      const std::string name = field->name()->str();
      const std::string path = prefix.empty() ? name : prefix + "." + name;
      const reflection::Type& type = *field->type();
      const bool nested = type.base_type() == reflection::BaseType::Obj ||
                          (type.base_type() == reflection::BaseType::Array &&
                           type.element() == reflection::BaseType::Obj);
      if (name.starts_with(kAlertPrefix)) {
        out_.push_back(
            {path, name, type.base_type() == reflection::BaseType::Bool});
      }
      if (nested && type.index() >= 0) {
        Struct(static_cast<std::size_t>(type.index()), path);
      }
    }
  }

 private:
  const reflection::Schema& schema_;
  std::vector<AlertPrefixedField>& out_;
  std::vector<bool> seen_;  // by object index: walked already
};

}  // namespace

std::vector<AlertPrefixedField> AlertPrefixedFields(std::string_view bfbs,
                                                    std::string_view root) {
  std::vector<AlertPrefixedField> out;
  // Verified here, not through //talOS/schema:field_path: this library is
  // part of libtalos_node.so, and a library in that closure is left out of
  // every talos_cc_shared_node's own link.
  flatbuffers::Verifier verifier{
      reinterpret_cast<const std::uint8_t*>(bfbs.data()), bfbs.size()};
  if (!reflection::VerifySchemaBuffer(verifier)) return out;
  const reflection::Schema* schema = reflection::GetSchema(bfbs.data());
  const auto* objects = schema->objects();
  for (flatbuffers::uoffset_t i = 0; i < objects->size(); ++i) {
    if (objects->Get(i)->name()->string_view() == root) {
      AlertFieldWalk{*schema, out}.Struct(i, "");
      break;
    }
  }
  return out;
}

}  // namespace talos::introspect::naming::detail
