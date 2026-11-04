// talos_messages' schema step: reads one schema's .bfbs and
//
//   descriptor_gen lint <bfbs> <file>
//       fails when a numeric field of a struct declared in <file> has no
//       unit suffix (talOS/schema/unit_suffix.h, docs/METHODOLOGY.md §6);
//   descriptor_gen cc <bfbs> <file> <out.h> <label>
//       writes talos::Descriptor<M> for each struct declared in <file>,
//       forward-declaring M, so flatc's <file>_generated.h includes it
//       (--cpp-include) and every user of a struct sees its descriptor;
//   descriptor_gen py <bfbs> <file> <python tree>
//       appends TALOS_SCHEMA_NAME/HASH/BFBS to each such struct's flatc
//       Python class, in <tree>/<namespace path>/<Struct>.py.
//
// <file> is the schema's declaration file as flatc --bfbs-filenames records
// it ("//<include prefix>/<name>.fbs"); structs of included files belong to
// their own package's descriptors.

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "flatbuffers/reflection_generated.h"
#include "flatbuffers/verifier.h"
#include "talOS/schema/schema_hash.h"
#include "talOS/schema/unit_suffix.h"

namespace {

struct Input {
  std::vector<std::uint8_t> bytes;
  const reflection::Schema* schema{nullptr};
  std::vector<const reflection::Object*> structs;  // declared in the file
};

bool load(const char* bfbs_path, std::string_view file, Input& in) {
  std::ifstream stream(bfbs_path, std::ios::binary);
  if (!stream) {
    std::fprintf(stderr, "descriptor_gen: cannot read %s\n", bfbs_path);
    return false;
  }
  in.bytes.assign(std::istreambuf_iterator<char>(stream),
                  std::istreambuf_iterator<char>());
  flatbuffers::Verifier verifier(in.bytes.data(), in.bytes.size());
  if (!reflection::VerifySchemaBuffer(verifier)) {
    std::fprintf(stderr, "descriptor_gen: %s is not a reflection.Schema\n",
                 bfbs_path);
    return false;
  }
  in.schema = reflection::GetSchema(in.bytes.data());
  for (const reflection::Object* object : *in.schema->objects()) {
    if (object->is_struct() && object->declaration_file() != nullptr &&
        object->declaration_file()->string_view() == file) {
      in.structs.push_back(object);
    }
  }
  return true;
}

bool is_scalar_number(reflection::BaseType base) {
  return base >= reflection::BaseType::Byte &&
         base <= reflection::BaseType::Double;
}

int lint(const Input& in, std::string_view file) {
  int violations = 0;
  for (const reflection::Object* object : in.structs) {
    for (const reflection::Field* field : *object->fields()) {
      const reflection::Type& type = *field->type();
      const reflection::BaseType base =
          type.base_type() == reflection::BaseType::Array ? type.element()
                                                          : type.base_type();
      // Bools, enums (a scalar with an enum index) and structs carry no unit.
      if (!is_scalar_number(base) || type.index() >= 0) continue;
      if (talos::schema::has_unit_suffix(field->name()->string_view())) {
        continue;
      }
      ++violations;
      std::fprintf(
          stderr, "talos_messages: %.*s: %s.%s (%s) has no unit suffix\n",
          static_cast<int>(file.size()), file.data(), object->name()->c_str(),
          field->name()->c_str(), reflection::EnumNameBaseType(base));
    }
  }
  if (violations == 0) return 0;
  std::string suffixes;
  for (std::string_view suffix : talos::schema::kUnitSuffixes) {
    suffixes += ' ';
    suffixes += suffix;
  }
  std::fprintf(stderr,
               "talos_messages: end every numeric field with a unit suffix "
               "(docs/METHODOLOGY.md §6):%s\n",
               suffixes.c_str());
  return 1;
}

std::string replace_all(std::string_view text, std::string_view from,
                        std::string_view to) {
  std::string out;
  std::size_t start = 0;
  for (std::size_t at; (at = text.find(from, start)) != text.npos;) {
    out.append(text.substr(start, at - start)).append(to);
    start = at + from.size();
  }
  return out.append(text.substr(start));
}

std::string hex64(std::uint64_t value) {
  char text[19];
  std::snprintf(text, sizeof(text), "0x%016llx",
                static_cast<unsigned long long>(value));
  return text;
}

bool write(const char* path, const std::string& text, bool append) {
  std::ofstream out(path, append ? std::ios::app : std::ios::trunc);
  out << text;
  out.close();
  if (!out) {
    std::fprintf(stderr, "descriptor_gen: cannot write %s\n", path);
    return false;
  }
  return true;
}

int write_cc(const Input& in, std::string_view file, const char* out_path,
             std::string_view label) {
  // One array per schema file, named injectively after it (every other
  // character becomes _<hex>), so no two files' arrays share a name in a
  // translation unit and no name holds a reserved "__".
  std::string array = "bfbs";
  for (char c : file) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      array += c;
    } else {
      char escaped[4];
      std::snprintf(escaped, sizeof(escaped), "_%02x",
                    static_cast<unsigned char>(c));
      array += escaped;
    }
  }
  std::ostringstream h;
  h << "// Schema descriptors of " << file << " (" << label
    << "), generated by talos_messages.\n"
       "#pragma once\n\n"
       "#include <cstdint>\n#include <span>\n#include <string_view>\n\n"
       "#include \"talOS/schema/descriptor.h\"\n\n";
  // Forward declarations: flatc's generated header includes this one before
  // it declares the structs (--cpp-include), and a specialization needs only
  // the name.
  for (const reflection::Object* object : in.structs) {
    const std::string_view name = object->name()->string_view();
    const std::size_t dot = name.rfind('.');
    const std::string_view ns =
        dot == std::string_view::npos ? std::string_view{} : name.substr(0, dot);
    const std::string_view leaf =
        dot == std::string_view::npos ? name : name.substr(dot + 1);
    if (ns.empty()) {
      h << "struct " << leaf << ";\n";
    } else {
      h << "namespace " << replace_all(ns, ".", "::") << " {\nstruct " << leaf
        << ";\n}\n";
    }
  }
  h << "\nnamespace talos::schema_detail {\n"
       "alignas(8) inline constexpr std::uint8_t "
    << array << "[] = {";
  for (std::size_t i = 0; i < in.bytes.size(); ++i) {
    h << (i % 12 == 0 ? "\n   " : "") << ' '
      << static_cast<unsigned>(in.bytes[i]) << ',';
  }
  h << "\n};\n}  // namespace talos::schema_detail\n\nnamespace talos {\n";
  for (const reflection::Object* object : in.structs) {
    const std::string_view name = object->name()->string_view();
    h << "\ntemplate <>\nstruct Descriptor<::" << replace_all(name, ".", "::")
      << "> {\n  static constexpr std::string_view kName = \"" << name
      << "\";\n  static constexpr std::uint64_t kHash = "
      << hex64(talos::schema::struct_hash(*in.schema, *object))
      << "ULL;\n  static constexpr std::span<const std::uint8_t> kBfbs{\n"
         "      schema_detail::"
      << array << "};\n};\n";
  }
  h << "\n}  // namespace talos\n";
  return write(out_path, h.str(), false) ? 0 : 1;
}

int write_py(const Input& in, std::string_view tree) {
  std::ostringstream bytes;
  for (std::size_t i = 0; i < in.bytes.size(); ++i) {
    if (i % 16 == 0) bytes << (i == 0 ? "" : "\"\n") << "    b\"";
    char escaped[5];
    std::snprintf(escaped, sizeof(escaped), "\\x%02x", in.bytes[i]);
    bytes << escaped;
  }
  bytes << "\"\n";
  for (const reflection::Object* object : in.structs) {
    const std::string name = object->name()->str();
    const std::size_t dot = name.rfind('.');
    const std::string cls = name.substr(dot == name.npos ? 0 : dot + 1);
    const std::string module =
        std::string(tree) + "/" + replace_all(name, ".", "/") + ".py";
    if (!std::ifstream(module).good()) {
      std::fprintf(stderr, "descriptor_gen: flatc wrote no %s\n",
                   module.c_str());
      return 1;
    }
    std::ostringstream py;
    py << "\n\n# Schema descriptor (talOS/schema/descriptor.h), appended by "
          "talos_messages.\n"
       << cls << ".TALOS_SCHEMA_NAME = \"" << name << "\"\n"
       << cls << ".TALOS_SCHEMA_HASH = "
       << hex64(talos::schema::struct_hash(*in.schema, *object)) << "\n"
       << cls << ".TALOS_SCHEMA_BFBS = (\n"
       << bytes.str() << ")\n";
    if (!write(module.c_str(), py.str(), true)) return 1;
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string_view> args(argv + 1, argv + argc);
  const auto usage = [] {
    std::fprintf(stderr,
                 "usage: descriptor_gen lint <bfbs> <file>\n"
                 "       descriptor_gen cc <bfbs> <file> <out.h> <label>\n"
                 "       descriptor_gen py <bfbs> <file> <python tree>\n");
    return 2;
  };
  if (args.size() < 3) return usage();
  const std::string_view mode = args[0];
  const std::string_view file = args[2];
  Input in;
  if (!load(argv[2], file, in)) return 1;
  if (mode == "lint" && args.size() == 3) return lint(in, file);
  if (mode == "cc" && args.size() == 5) {
    return write_cc(in, file, argv[4], args[4]);
  }
  if (mode == "py" && args.size() == 4) return write_py(in, args[3]);
  return usage();
}
