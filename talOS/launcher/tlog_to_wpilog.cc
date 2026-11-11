// tlog_to_wpilog: talOS dispatch logs (.tlog) as one WPILib DataLog
// (.wpilog), so a session opens in AdvantageScope. Offline only.
//
//   tlog_to_wpilog <log-file-or-session-dir> [more ...] -o <out.wpilog>
//
// The logs are read and merged with MergeLogs (log_merge.h, as log_dump
// reads them), and each payload is walked with flatbuffers/reflection.h
// from the schema the log carries, so no recording build is needed. The
// mapping, the clock and the caveats are in talOS/launcher/README.md,
// "Converting a session to WPILOG".

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <bit>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "flatbuffers/reflection.h"
#include "flatbuffers/verifier.h"
#include "talOS/events/context.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/launcher/log_merge.h"
#include "talOS/launcher/wpilog.h"
#include "talOS/schema/identity.h"
#include "talOS/schema/unit_suffix.h"

namespace {

using reflection::BaseType;
using talos::event::EventKind;
using talos::launcher::MergedRecord;

// Per-reader dropped-message counters are entries of this owner, named
// /talos_log/dropped/<node> followed by the topic.
constexpr std::string_view kCounterOwner = "/talos_log";
// SendStatus::OK (talOS/events/event_loop_base.h), a SEND record's aux.
constexpr std::uint32_t kSendOk = 0;
// Guards against a hostile schema: real messages are far smaller. Every
// struct instance the walk visits counts against kMaxStructs, so a schema
// whose structs fan out to each other cannot make the walk exponential.
constexpr int kMaxDepth = 16;
constexpr std::size_t kMaxLeaves = 4096;
constexpr std::size_t kMaxStructs = 4096;

// One scalar or fixed array of scalars in a message, and its entry.
struct Leaf {
  std::string suffix;  // "pose/x_m", "wheels/0/speed_mps", "samples_m"
  BaseType base{BaseType::None};
  std::size_t offset{0};
  bool array{false};
  std::size_t count{0};  // elements, when `array`
  std::string unit;
  // Filled by BuildPlan: "<topic>/<suffix>", the WPILOG type, metadata.
  std::string name;
  std::string type;
  std::string metadata;
};

// A geometry.Pose2d, also written as AdvantageScope's [x, y, heading_rad].
struct PoseLeaf {
  std::string suffix;
  std::size_t x_offset{0};
  std::size_t y_offset{0};
  std::size_t yaw_offset{0};
  std::string name;  // "<topic>/<suffix>"
};

// How one schema's payload becomes entries; or why it cannot.
struct Plan {
  bool ok{false};
  std::string why;
  std::string schema;
  std::size_t bytes{0};
  std::vector<Leaf> leaves;
  std::vector<PoseLeaf> poses;
  std::size_t structs{0};  // struct instances visited, against kMaxStructs
};

// A Type's base type (or element type, with reflection::Type::VT_ELEMENT),
// read as its stored byte: a hostile schema's value outside BaseType is
// None, never held in the enum.
BaseType BaseOf(const reflection::Type& type,
                flatbuffers::voffset_t field = reflection::Type::VT_BASE_TYPE) {
  // Type privately is a Table, as every generated table (reflection.h
  // reads fields the same way).
  const auto raw =
      reinterpret_cast<const flatbuffers::Table*>(&type)->GetField<std::int8_t>(
          field, 0);
  if (raw < 0 || raw >= static_cast<std::int8_t>(BaseType::MaxBaseType)) {
    return BaseType::None;
  }
  return static_cast<BaseType>(raw);
}

bool IsScalar(BaseType base) {
  return base >= BaseType::UType && base <= BaseType::Double;
}

std::string Join(std::string_view path, std::string_view name) {
  if (path.empty()) return std::string{name};
  std::string out{path};
  out += '/';
  out += name;
  return out;
}

bool Refuse(Plan& plan, std::string_view field, std::string_view why) {
  plan.why = "field ";
  plan.why += field;
  plan.why += ' ';
  plan.why += why;
  return false;
}

// The pose mapping table has one row: geometry.Pose2d (talOS/msgs) with its
// three double fields, at struct path P, adds the double[] entry P (or
// "pose" when the Pose2d is the whole message).
void AddPose(const reflection::Object& object, std::size_t offset,
             const std::string& path, Plan& plan) {
  if (object.name()->string_view() != "geometry.Pose2d") return;
  std::optional<std::size_t> x;
  std::optional<std::size_t> y;
  std::optional<std::size_t> yaw;
  for (const reflection::Field* field : *object.fields()) {
    if (BaseOf(*field->type()) != BaseType::Double) continue;
    const std::string_view name = field->name()->string_view();
    const std::size_t at = offset + field->offset();
    if (name == "x_m") x = at;
    if (name == "y_m") y = at;
    if (name == "yaw_rad") yaw = at;
  }
  if (x && y && yaw) {
    plan.poses.push_back(
        {path.empty() ? std::string{"pose"} : path, *x, *y, *yaw, {}});
  }
}

bool AddStruct(const reflection::Schema& schema,
               const reflection::Object& object, std::size_t offset,
               const std::string& path, int depth, Plan& plan) {
  if (depth > kMaxDepth) {
    plan.why = "structs nest more than " + std::to_string(kMaxDepth) + " deep";
    return false;
  }
  if (++plan.structs > kMaxStructs) {
    plan.why = "it expands to more than " + std::to_string(kMaxStructs) +
               " struct instances";
    return false;
  }
  const auto bytes = static_cast<std::size_t>(object.bytesize());
  // reflection lists fields by name; keep the declared order.
  std::vector<const reflection::Field*> fields(object.fields()->begin(),
                                               object.fields()->end());
  std::sort(fields.begin(), fields.end(),
            [](const reflection::Field* a, const reflection::Field* b) {
              return a->id() < b->id();
            });
  for (const reflection::Field* field : fields) {
    const reflection::Type& type = *field->type();
    const std::string_view name = field->name()->string_view();
    const bool array = BaseOf(type) == BaseType::Array;
    const BaseType base =
        array ? BaseOf(type, reflection::Type::VT_ELEMENT) : BaseOf(type);
    const std::size_t count = array ? type.fixed_length() : 1;
    const reflection::Object* nested = nullptr;
    std::size_t element_bytes = 0;
    if (base == BaseType::Obj) {
      if (type.index() < 0 ||
          static_cast<std::size_t>(type.index()) >= schema.objects()->size()) {
        return Refuse(plan, name, "names no object");
      }
      nested = schema.objects()->Get(
          static_cast<flatbuffers::uoffset_t>(type.index()));
      if (!nested->is_struct() || nested->bytesize() <= 0) {
        return Refuse(plan, name, "is not a struct");
      }
      element_bytes = static_cast<std::size_t>(nested->bytesize());
    } else if (IsScalar(base)) {
      element_bytes = flatbuffers::GetTypeSize(base);
    } else {
      return Refuse(plan, name, "is not a scalar, enum or struct");
    }
    if (field->offset() + count * element_bytes > bytes) {
      return Refuse(plan, name, "lies outside its struct");
    }
    const std::size_t at = offset + field->offset();
    const std::string field_path = Join(path, name);
    if (nested == nullptr) {
      if (plan.leaves.size() >= kMaxLeaves) {
        return Refuse(plan, name, "is past the field limit");
      }
      plan.leaves.push_back({field_path,
                             base,
                             at,
                             array,
                             array ? count : 0,
                             std::string{talos::schema::unit_for_field(name)},
                             {},
                             {},
                             {}});
      continue;
    }
    if (!array) {
      if (!AddStruct(schema, *nested, at, field_path, depth + 1, plan)) {
        return false;
      }
      continue;
    }
    for (std::size_t k = 0; k < count; ++k) {
      if (!AddStruct(schema, *nested, at + k * element_bytes,
                     Join(field_path, std::to_string(k)), depth + 1, plan)) {
        return false;
      }
    }
  }
  AddPose(object, offset, path, plan);
  return true;
}

// A topic's Plan from the schema identity its log carries; entry names and
// metadata are filled in by BuildPlan below.
Plan WalkSchema(const talos::schema::Identity& identity) {
  Plan plan;
  plan.schema = identity.name;
  if (identity.bfbs.empty()) {
    plan.why =
        (identity.name.empty() ? std::string{"no schema"} : identity.name) +
        " carries no FlatBuffers schema";
    return plan;
  }
  const auto* bfbs =
      reinterpret_cast<const std::uint8_t*>(identity.bfbs.data());
  flatbuffers::Verifier verifier{bfbs, identity.bfbs.size()};
  if (!reflection::VerifySchemaBuffer(verifier)) {
    plan.why = "its schema bytes are not a reflection schema";
    return plan;
  }
  const reflection::Schema& schema = *reflection::GetSchema(bfbs);
  const std::string_view inner = talos::schema::EnvelopeInner(identity.name);
  const std::string name{inner.empty() ? std::string_view{identity.name}
                                       : inner};
  const reflection::Object* object =
      schema.objects()->LookupByKey(name.c_str());
  if (object == nullptr || !object->is_struct() || object->bytesize() <= 0) {
    plan.why = "its schema has no struct " + name;
    return plan;
  }
  std::size_t offset = 0;
  std::string path;
  if (!inner.empty()) {
    // talOS/devices/target.h's header, as DecodeToText names it.
    plan.leaves.push_back(
        {"mode_since_us", BaseType::ULong, 0, false, 0, "us", {}, {}, {}});
    plan.leaves.push_back({"active",
                           BaseType::Bool,
                           talos::schema::kEnvelopeActiveOffset,
                           false,
                           0,
                           "",
                           {},
                           {},
                           {}});
    offset = talos::schema::kEnvelopeHeaderBytes;
    path = "message";
  }
  plan.bytes = offset + static_cast<std::size_t>(object->bytesize());
  plan.ok = AddStruct(schema, *object, offset, path, 0, plan);
  return plan;
}

// `bytes` little-endian bytes at `at`, whatever the host's byte order.
std::uint64_t Load(const std::byte* at, std::size_t bytes) {
  std::uint64_t value = 0;
  for (std::size_t i = 0; i < bytes; ++i) {
    value |= static_cast<std::uint64_t>(at[i]) << (8U * i);
  }
  return value;
}

// An integer, enum or UType at `at`; a ulong keeps its bits (wraps above
// INT64_MAX).
std::int64_t LoadInteger(BaseType base, const std::byte* at) {
  switch (base) {
    case BaseType::Byte:
      return static_cast<std::int8_t>(static_cast<std::uint8_t>(Load(at, 1)));
    case BaseType::Short:
      return static_cast<std::int16_t>(static_cast<std::uint16_t>(Load(at, 2)));
    case BaseType::Int:
      return static_cast<std::int32_t>(static_cast<std::uint32_t>(Load(at, 4)));
    case BaseType::UShort:
      return static_cast<std::int64_t>(Load(at, 2));
    case BaseType::UInt:
      return static_cast<std::int64_t>(Load(at, 4));
    case BaseType::Long:
    case BaseType::ULong:
      return static_cast<std::int64_t>(Load(at, 8));
    default:  // UType, UByte
      return static_cast<std::int64_t>(Load(at, 1));
  }
}

double LoadDouble(const std::byte* at) {
  return std::bit_cast<double>(Load(at, 8));
}

void AppendElement(BaseType base, const std::byte* at,
                   std::vector<std::uint8_t>& out) {
  switch (base) {
    case BaseType::Bool:
      talos::wpilog::AppendBoolean(out, at[0] != std::byte{0});
      return;
    case BaseType::Float:
      talos::wpilog::AppendFloat(
          out, std::bit_cast<float>(static_cast<std::uint32_t>(Load(at, 4))));
      return;
    case BaseType::Double:
      talos::wpilog::AppendDouble(out, LoadDouble(at));
      return;
    default:
      talos::wpilog::AppendInt64(out, LoadInteger(base, at));
      return;
  }
}

std::string TypeOf(const Leaf& leaf) {
  std::string type = leaf.base == BaseType::Bool     ? "boolean"
                     : leaf.base == BaseType::Float  ? "float"
                     : leaf.base == BaseType::Double ? "double"
                                                     : "int64";
  if (leaf.array) type += "[]";
  return type;
}

std::string JsonString(std::string_view value) {
  std::string result{"\""};
  constexpr char kHex[] = "0123456789abcdef";
  for (const char c : value) {
    const auto byte = static_cast<unsigned char>(c);
    if (c == '"' || c == '\\') {
      result += '\\';
      result += c;
    } else if (byte < 0x20) {
      result += "\\u00";
      result += kHex[byte >> 4U];
      result += kHex[byte & 15U];
    } else {
      result += c;
    }
  }
  result += '"';
  return result;
}

// {"source":"talos.tlog","schema":...,"unit":...,"format":...}
std::string Metadata(std::string_view schema, std::string_view unit,
                     std::string_view format = {}) {
  std::string out = "{\"source\":\"talos.tlog\"";
  if (!schema.empty()) out += ",\"schema\":" + JsonString(schema);
  if (!unit.empty()) out += ",\"unit\":" + JsonString(unit);
  if (!format.empty()) out += ",\"format\":" + JsonString(format);
  out += '}';
  return out;
}

// `topic`'s plan for `identity`, with every entry's name, type and metadata.
Plan BuildPlan(std::string_view topic,
               const talos::schema::Identity& identity) {
  Plan plan = WalkSchema(identity);
  if (!plan.ok) return plan;
  for (Leaf& leaf : plan.leaves) {
    leaf.name = Join(topic, leaf.suffix);
    leaf.type = TypeOf(leaf);
    leaf.metadata = Metadata(plan.schema, leaf.unit);
  }
  for (PoseLeaf& pose : plan.poses) pose.name = Join(topic, pose.suffix);
  return plan;
}

// "dropped/<node><topic>", the counter's suffix below kCounterOwner.
std::string DroppedSuffix(std::string_view process, std::string_view topic) {
  std::string suffix{"dropped/"};
  suffix += process;
  suffix += topic;
  return suffix;
}

std::string SizeMismatch(std::size_t bytes, const Plan& plan) {
  std::string why = std::to_string(bytes);
  why += "-byte payload, ";
  why += plan.schema;
  why += " is ";
  why += std::to_string(plan.bytes);
  return why;
}

// Entries by name, started on first use, each owned by one topic.
class Entries {
 public:
  explicit Entries(talos::wpilog::Writer& writer) : writer_{writer} {}

  // The id of `owner`'s entry `name` (`owner/suffix`), starting it at
  // `timestamp_us` on first use. A name is its first owner's: another
  // topic whose field path lands on it (/arm/state field left.angle_rad and
  // /arm/state/left field angle_rad) gets `owner:suffix` instead, which no
  // other topic can produce (names hold no ':'). 0 (skip the value) when the
  // entry was started with another type.
  std::uint32_t get(const std::string& name, std::string_view owner,
                    std::string_view suffix, const std::string& type,
                    const std::string& metadata, std::uint64_t timestamp_us) {
    auto found = entries_.find(name);
    if (found != entries_.end() && found->second.owner != owner) {
      std::string renamed{owner};
      renamed += ':';
      renamed += suffix;
      if (renames_.insert(renamed).second) {
        std::fprintf(stderr,
                     "tlog_to_wpilog: warning: %s is %s's; %.*s's entry is "
                     "%s\n",
                     name.c_str(), found->second.owner.c_str(),
                     static_cast<int>(owner.size()), owner.data(),
                     renamed.c_str());
      }
      return start_or_find(renamed, owner, type, metadata, timestamp_us);
    }
    return start_or_find(name, owner, type, metadata, timestamp_us);
  }

  std::size_t size() const { return entries_.size(); }

 private:
  std::uint32_t start_or_find(const std::string& name, std::string_view owner,
                              const std::string& type,
                              const std::string& metadata,
                              std::uint64_t timestamp_us) {
    const auto found = entries_.find(name);
    if (found == entries_.end()) {
      const std::uint32_t id =
          writer_.start(name, type, metadata, timestamp_us);
      entries_.emplace(name, Entry{id, type, std::string{owner}});
      return id;
    }
    if (found->second.owner == owner && found->second.type == type) {
      return found->second.id;
    }
    if (conflicts_.insert(name).second) {
      std::fprintf(stderr,
                   "tlog_to_wpilog: warning: %s is %s, also logged as %s; "
                   "keeping the first\n",
                   name.c_str(), found->second.type.c_str(), type.c_str());
    }
    return 0;
  }

  struct Entry {
    std::uint32_t id;
    std::string type;
    std::string owner;
  };
  talos::wpilog::Writer& writer_;
  std::unordered_map<std::string, Entry> entries_;
  std::set<std::string> conflicts_;
  std::set<std::string> renames_;
};

struct Options {
  std::vector<std::string> inputs;
  std::string output;
};

bool ParseOptions(int argc, char** argv, Options& options, std::string& error) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--") {
      while (++i < argc) options.inputs.emplace_back(argv[i]);
      break;
    }
    if (arg == "-o" || arg == "--output") {
      if (++i == argc) {
        error = "missing value for " + arg;
        return false;
      }
      options.output = argv[i];
      continue;
    }
    if (arg.size() > 1 && arg.starts_with('-')) {
      error = "unknown flag: " + arg;
      return false;
    }
    options.inputs.push_back(arg);
  }
  if (options.output.empty()) {
    error = "an output file is required (-o <out.wpilog>)";
    return false;
  }
  if (options.inputs.empty()) {
    error = "a log file or session directory is required";
    return false;
  }
  return true;
}

// Microseconds of `ns` after `start_ns`; 0 before it. Unsigned arithmetic:
// a damaged log's times must not overflow.
std::uint64_t Micros(std::int64_t ns, std::int64_t start_ns) {
  if (ns <= start_ns) return 0;
  return (static_cast<std::uint64_t>(ns) -
          static_cast<std::uint64_t>(start_ns)) /
         1000;
}

int Fail(const std::string& message) {
  std::fprintf(stderr, "tlog_to_wpilog: error: %s\n", message.c_str());
  return 1;
}

// The input's log files: a directory is every *.tlog in it, a log file that
// log's kept segments too, oldest first (as log_dump reads them); each file
// once, by canonical path, however many inputs name it.
std::vector<std::string> InputFiles(const std::vector<std::string>& inputs) {
  std::vector<std::string> files;
  std::set<std::string> seen;
  const auto add = [&](const std::string& file) {
    std::error_code ec;
    const std::filesystem::path canonical =
        std::filesystem::weakly_canonical(file, ec);
    if (seen.insert(ec ? file : canonical.string()).second) {
      files.push_back(file);
    }
  };
  for (const auto& input : inputs) {
    std::error_code ec;
    if (std::filesystem::is_directory(input, ec)) {
      for (const auto& file : talos::event::log::FindLogFiles(input)) add(file);
      continue;
    }
    const auto found = talos::event::log::LogSegmentFiles(input);
    if (found.empty()) add(input);  // Reported when opened.
    for (const auto& file : found) add(file);
  }
  return files;
}

// One publication: its topic (an index into the topic table), RTMS
// sequence and publish time. A SEND and every reader's MESSAGE of it carry
// the same three.
using Publication = std::tuple<std::size_t, std::uint64_t, std::int64_t>;

struct Item {
  std::int64_t time_ns;
  std::size_t index;
  bool values;
};

// The records that become values or dropped counts, in publish-time order.
// Each publication's values come once: from its writer's SEND when that is
// logged (a refused send was never published), otherwise from the first
// reader's MESSAGE of it in merge order. So a reader's copy fills what the
// writer's log lacks (a capped log's deleted head, a cut tail) and every
// reader contributes what it received. Every MESSAGE with a dropped count
// is an item, for that reader's counter.
std::vector<Item> SelectItems(const std::vector<MergedRecord>& records) {
  std::map<std::string, std::size_t> topics;
  const auto key = [&](const MergedRecord& r) {
    const std::size_t topic =
        topics.try_emplace(r.source_name, topics.size()).first->second;
    return Publication{topic, r.header.sequence, r.header.publish_time_ns};
  };
  const auto is = [](const MergedRecord& r, EventKind kind) {
    return r.header.kind == static_cast<std::uint16_t>(kind);
  };
  std::set<Publication> sent;
  for (const auto& r : records) {
    if (r.source != nullptr && is(r, EventKind::SEND) &&
        r.header.aux == kSendOk) {
      sent.insert(key(r));
    }
  }
  std::set<Publication> taken;
  std::vector<Item> items;
  for (std::size_t i = 0; i < records.size(); ++i) {
    const MergedRecord& r = records[i];
    if (r.source == nullptr) continue;
    if (is(r, EventKind::SEND)) {
      if (r.header.aux == kSendOk && taken.insert(key(r)).second) {
        items.push_back({r.header.publish_time_ns, i, true});
      }
    } else if (is(r, EventKind::MESSAGE)) {
      const Publication publication = key(r);
      const bool values =
          !sent.contains(publication) && taken.insert(publication).second;
      if (values || r.header.aux > 0) {
        items.push_back({r.header.publish_time_ns, i, values});
      }
    }
  }
  // MergeLogs orders by event time; a WPILOG is stamped by publication.
  std::stable_sort(
      items.begin(), items.end(),
      [](const Item& a, const Item& b) { return a.time_ns < b.time_ns; });
  return items;
}

struct Totals {
  std::size_t entries{0};
  std::size_t values{0};
  std::map<std::string, std::string> skipped;  // topic -> why
};

// Writes `items` to `writer`; see README.md for the mapping.
Totals WriteItems(const std::vector<MergedRecord>& records,
                  const std::vector<Item>& items, std::int64_t start_ns,
                  talos::wpilog::Writer& writer) {
  Totals totals;
  Entries entries{writer};
  std::map<std::tuple<std::string, std::string, std::uint64_t>, Plan> plans;
  std::map<std::pair<std::string, std::string>, std::int64_t> dropped;
  std::vector<std::uint8_t> payload;
  const std::string pose_metadata =
      Metadata("geometry.Pose2d", {}, "[x_m, y_m, yaw_rad]");
  const std::string dropped_metadata =
      Metadata({}, {}, "messages this node's subscription lapped");
  const auto write = [&](const std::string& name, std::string_view owner,
                         std::string_view suffix, const std::string& type,
                         const std::string& metadata, std::uint64_t time_us) {
    const std::uint32_t id =
        entries.get(name, owner, suffix, type, metadata, time_us);
    if (id == 0) return;
    writer.append(id, payload, time_us);
    ++totals.values;
  };

  for (const Item& item : items) {
    const MergedRecord& r = records[item.index];
    const std::uint64_t time_us = Micros(item.time_ns, start_ns);
    const std::string& topic = r.source_name;
    if (r.header.kind == static_cast<std::uint16_t>(EventKind::MESSAGE) &&
        r.header.aux > 0) {
      std::int64_t& total = dropped[{r.process_name, topic}];
      total += r.header.aux;
      payload.clear();
      talos::wpilog::AppendInt64(payload, total);
      const std::string suffix = DroppedSuffix(r.process_name, topic);
      write(Join(kCounterOwner, suffix), kCounterOwner, suffix, "int64",
            dropped_metadata, time_us);
    }
    if (!item.values) continue;
    const talos::schema::Identity& identity = r.source->schema;
    auto plan_at = plans.find({topic, identity.name, identity.hash});
    if (plan_at == plans.end()) {
      plan_at = plans
                    .emplace(std::tuple{topic, identity.name, identity.hash},
                             BuildPlan(topic, identity))
                    .first;
    }
    const Plan& plan = plan_at->second;
    if (!plan.ok) {
      totals.skipped.try_emplace(topic, plan.why);
      continue;
    }
    if (r.payload.size() != plan.bytes) {
      totals.skipped.try_emplace(topic, SizeMismatch(r.payload.size(), plan));
      continue;
    }
    const std::byte* data = r.payload.data();
    for (const Leaf& leaf : plan.leaves) {
      payload.clear();
      const std::size_t element_bytes = flatbuffers::GetTypeSize(leaf.base);
      for (std::size_t k = 0; k < (leaf.array ? leaf.count : 1); ++k) {
        AppendElement(leaf.base, data + leaf.offset + k * element_bytes,
                      payload);
      }
      write(leaf.name, topic, leaf.suffix, leaf.type, leaf.metadata, time_us);
    }
    for (const PoseLeaf& pose : plan.poses) {
      payload.clear();
      talos::wpilog::AppendDouble(payload, LoadDouble(data + pose.x_offset));
      talos::wpilog::AppendDouble(payload, LoadDouble(data + pose.y_offset));
      talos::wpilog::AppendDouble(payload, LoadDouble(data + pose.yaw_offset));
      write(pose.name, topic, pose.suffix, "double[]", pose_metadata, time_us);
    }
  }
  totals.entries = entries.size();
  return totals;
}

// The output is written to a temporary file beside it (same directory, so
// same filesystem) and renamed over it only once complete, so a failed
// conversion leaves no partial file and any previous output unchanged.
// Created before anything is converted: an output that cannot be written
// fails first. The destructor removes the temporary file unless commit()
// renamed it into place.
class TempOutput {
 public:
  TempOutput() = default;
  TempOutput(const TempOutput&) = delete;
  TempOutput& operator=(const TempOutput&) = delete;
  ~TempOutput() {
    if (!path_.empty()) {
      std::error_code ignored;
      std::filesystem::remove(path_, ignored);
    }
  }

  // Creates the temporary file for `output`; false with `error` set when
  // `output` is something other than a regular file (a device, a
  // directory) or its directory cannot take a new file.
  bool create(const std::string& output, std::string& error) {
    std::error_code ec;
    const auto status = std::filesystem::status(output, ec);
    if (std::filesystem::exists(status) &&
        !std::filesystem::is_regular_file(status)) {
      error = "cannot write " + output + ": not a regular file";
      return false;
    }
    const std::filesystem::path target{output};
    const std::filesystem::path directory = target.has_parent_path()
                                                ? target.parent_path()
                                                : std::filesystem::path{"."};
    std::string pattern =
        (directory / ("." + target.filename().string() + ".tmp.XXXXXX"))
            .string();
    const int fd = ::mkstemp(pattern.data());
    if (fd < 0) {
      error = "cannot write " + output + ": " + std::strerror(errno);
      return false;
    }
    path_ = pattern;
    // mkstemp creates the file 0600; give it what a new file gets.
    const mode_t mask = ::umask(0);
    ::umask(mask);
    const int chmod_result =
        ::fchmod(fd, static_cast<mode_t>(0666U & ~static_cast<unsigned>(mask)));
    const int saved = errno;
    ::close(fd);
    if (chmod_result != 0) {
      error = "cannot write " + output + ": " + std::strerror(saved);
      return false;
    }
    return true;
  }

  const std::string& path() const { return path_; }

  // Renames the temporary file over `output`; false with `error` set.
  bool commit(const std::string& output, std::string& error) {
    std::error_code ec;
    std::filesystem::rename(path_, output, ec);
    if (ec) {
      error = "cannot write " + output + ": " + ec.message();
      return false;
    }
    path_.clear();
    return true;
  }

 private:
  std::string path_;
};

int Convert(const Options& options) {
  TempOutput temp;
  std::string error;
  if (!temp.create(options.output, error)) return Fail(error);

  const std::vector<std::string> files = InputFiles(options.inputs);
  if (files.empty()) return Fail("no .tlog files found");

  // Each file's header: its session and loop start. A file that cannot be
  // read fails here, named; one cut while it was being created holds no
  // records and is skipped (MergeLogs warns). Damaged records inside a log
  // are skipped by the reader, and MergeLogs names the log.
  std::int64_t start_ns = std::numeric_limits<std::int64_t>::max();
  std::uint64_t session_id = 0;
  for (const auto& file : files) {
    try {
      const talos::event::log::LogReader reader{file};
      start_ns = std::min(start_ns, reader.start_time().nanos());
      session_id = reader.session_id();
    } catch (const talos::event::log::IncompleteLogHeader&) {
      continue;
    } catch (const std::exception& e) {
      return Fail(file + ": " + e.what());
    }
  }
  if (start_ns == std::numeric_limits<std::int64_t>::max()) {
    return Fail("no readable .tlog files");
  }

  std::vector<MergedRecord> records;
  if (!talos::launcher::MergeLogs(files, records, error)) return Fail(error);
  const std::vector<Item> items = SelectItems(records);

  std::ofstream out{temp.path(), std::ios::binary | std::ios::trunc};
  if (!out) return Fail("cannot write " + options.output);
  Totals totals;
  try {
    talos::wpilog::Writer writer{
        out, "talos tlog_to_wpilog session_id=" + std::to_string(session_id)};
    totals = WriteItems(records, items, start_ns, writer);
    out.flush();
    const bool written = writer.ok() && out.good();
    out.close();
    if (!written || out.fail()) {
      return Fail("writing " + options.output + " failed");
    }
  } catch (const std::exception& e) {
    return Fail(options.output + ": " + e.what());
  }
  if (!temp.commit(options.output, error)) return Fail(error);
  for (const auto& [topic, why] : totals.skipped) {
    std::fprintf(stderr, "tlog_to_wpilog: skipped %s: %s\n", topic.c_str(),
                 why.c_str());
  }
  std::printf(
      "tlog_to_wpilog: %zu files, %zu records -> %zu entries, %zu "
      "values in %s\n",
      files.size(), records.size(), totals.entries, totals.values,
      options.output.c_str());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  std::string error;
  if (!ParseOptions(argc, argv, options, error)) {
    std::fprintf(stderr,
                 "tlog_to_wpilog: %s\nusage: %s <log-file-or-session-dir> "
                 "[more ...] -o <out.wpilog>\n",
                 error.c_str(), argv[0]);
    return 1;
  }
  try {
    return Convert(options);
  } catch (const std::exception& e) {
    return Fail(e.what());
  }
}
