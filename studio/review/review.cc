#include "studio/review/review.h"

#include <algorithm>
#include <bit>
#include <charconv>
#include <cstring>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <system_error>

#include "nlohmann/json.hpp"
#include "studio/bridge/system.h"
#include "studio/bridge/telemetry.h"
#include "talOS/launcher/session_retention.h"
#include "talOS/telemetry/frames.h"

namespace studio::review {
namespace {

namespace fs = std::filesystem;

Response Error(int status, std::string message) {
  return {status, "text/plain; charset=utf-8", std::move(message) + "\n"};
}

int Hex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool Decode(std::string_view text, std::string& out) {
  out.clear();
  for (std::size_t i = 0; i < text.size(); ++i) {
    const char c = text[i];
    if (c == '+') {
      out += ' ';
    } else if (c != '%') {
      out += c;
    } else {
      if (i + 2 >= text.size()) return false;
      const int high = Hex(text[i + 1]);
      const int low = Hex(text[i + 2]);
      if (high < 0 || low < 0) return false;
      out += static_cast<char>(high * 16 + low);
      i += 2;
    }
  }
  return true;
}

bool ParseInt(std::string_view text, std::int64_t& out) {
  const auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), out);
  return error == std::errc{} && end == text.data() + text.size() &&
         !text.empty();
}

// The one value of parameter `name`; false when it is absent, repeated or
// not an integer.
bool IntParameter(const std::vector<std::pair<std::string, std::string>>& query,
                  std::string_view name, std::int64_t& out) {
  int seen = 0;
  for (const auto& [key, value] : query) {
    if (key != name) continue;
    if (++seen > 1 || !ParseInt(value, out)) return false;
  }
  return seen == 1;
}

void AppendLe(std::string& out, const void* data, std::size_t bytes) {
  static_assert(std::endian::native == std::endian::little,
                "the series body is little-endian");
  out.append(static_cast<const char*>(data), bytes);
}

std::string_view ClassName(talos::launcher::MatchVerdict verdict) {
  switch (verdict) {
    case talos::launcher::MatchVerdict::kMatch:
      return "match";
    case talos::launcher::MatchVerdict::kUnknown:
      return "unknown";
    case talos::launcher::MatchVerdict::kOther:
      break;
  }
  return "other";
}

struct SessionEntry {
  std::string id;
  std::int64_t start_wall_ns{0};
  std::int64_t end_wall_ns{0};
  std::string_view verdict;
  std::vector<std::pair<std::string, std::uintmax_t>> files;
};

// A library session: a directory holding a manifest or an event log.
bool IsSession(const fs::path& dir) {
  std::error_code error;
  if (fs::is_regular_file(dir / "manifest.json", error)) return true;
  for (fs::directory_iterator it{dir, error};
       !error && it != fs::directory_iterator{}; it.increment(error)) {
    if (it->path().extension() == ".tlog") return true;
  }
  return false;
}

std::optional<SessionEntry> ReadEntry(const fs::path& dir) {
  SessionEntry entry;
  entry.id = dir.filename().string();
  if (!ValidSessionId(entry.id)) return std::nullopt;
  bool session = false;
  std::error_code error;
  for (fs::directory_iterator it{dir, error};
       !error && it != fs::directory_iterator{}; it.increment(error)) {
    std::error_code file_error;
    if (!it->is_regular_file(file_error)) continue;
    const std::string name = it->path().filename().string();
    session = session || name == "manifest.json" || name.ends_with(".tlog");
    const auto bytes = it->file_size(file_error);
    entry.files.emplace_back(name, file_error ? 0 : bytes);
  }
  if (!session) return std::nullopt;
  std::sort(entry.files.begin(), entry.files.end());
  std::ifstream stream{dir / "manifest.json"};
  if (stream) {
    const auto manifest = nlohmann::json::parse(
        std::string{std::istreambuf_iterator<char>{stream}, {}}, nullptr,
        /*allow_exceptions=*/false);
    if (manifest.is_object()) {
      for (const auto& [key, field] :
           {std::pair{"start_wall_ns", &entry.start_wall_ns},
            std::pair{"end_wall_ns", &entry.end_wall_ns}}) {
        const auto it = manifest.find(key);
        if (it != manifest.end() && it->is_number_integer()) {
          *field = it->get<std::int64_t>();
        }
      }
    }
  }
  return entry;
}

}  // namespace

bool ValidSessionId(std::string_view id) {
  if (id.empty() || id.size() > 255 || id.front() == '.') return false;
  return std::all_of(id.begin(), id.end(), [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
  });
}

bool ParseQuery(std::string_view query,
                std::vector<std::pair<std::string, std::string>>& out) {
  out.clear();
  while (!query.empty()) {
    const std::size_t amp = query.find('&');
    const std::string_view part = query.substr(0, amp);
    query = amp == std::string_view::npos ? std::string_view{}
                                          : query.substr(amp + 1);
    if (part.empty()) continue;
    const std::size_t eq = part.find('=');
    std::string key;
    std::string value;
    if (!Decode(part.substr(0, eq), key)) return false;
    if (eq != std::string_view::npos && !Decode(part.substr(eq + 1), value)) {
      return false;
    }
    out.emplace_back(std::move(key), std::move(value));
  }
  return true;
}

Review::Review(fs::path library, std::size_t cache_bytes)
    : library_(std::move(library)), cache_(cache_bytes) {}

Response Review::Get(std::string_view path, std::string_view query) {
  constexpr std::string_view kPrefix = "/review/";
  if (!path.starts_with(kPrefix)) return Error(404, "not found");
  path.remove_prefix(kPrefix.size());
  if (path == "sessions.json") return Sessions();
  const std::size_t slash = path.find('/');
  if (slash == std::string_view::npos) return Error(404, "not found");
  const std::string_view id = path.substr(0, slash);
  const std::string_view route = path.substr(slash + 1);
  if (route != "channels.json" && route != "bounds.json" &&
      route != "snapshot" && route != "series") {
    return Error(404, "not found");
  }
  Response error;
  const Session* session = Open(id, error);
  if (session == nullptr) return error;
  try {
    if (route == "channels.json") return Channels(*session);
    if (route == "bounds.json") return Bounds(*session);
    if (route == "snapshot") return Snapshot(*session, query);
    return Series(id, *session, query);
  } catch (const std::exception& e) {
    return Error(500, e.what());
  }
}

const Session* Review::Open(std::string_view id, Response& error) {
  if (!ValidSessionId(id)) {
    error = Error(400, "invalid session id");
    return nullptr;
  }
  if (const auto it = sessions_.find(id); it != sessions_.end()) {
    return it->second.get();
  }
  const fs::path dir = library_ / std::string{id};
  std::error_code ignored;
  if (!fs::is_directory(dir, ignored) || fs::is_symlink(dir, ignored) ||
      !IsSession(dir)) {
    error = Error(404, "no session " + std::string{id});
    return nullptr;
  }
  try {
    auto session = Session::Open(dir);
    const Session* opened = session.get();
    sessions_.emplace(std::string{id}, std::move(session));
    return opened;
  } catch (const std::exception& e) {
    // A log of another format version, another session's, or none.
    error = Error(422, e.what());
    return nullptr;
  }
}

Response Review::Sessions() {
  std::vector<SessionEntry> entries;
  std::error_code error;
  for (fs::directory_iterator it{library_, error};
       !error && it != fs::directory_iterator{}; it.increment(error)) {
    std::error_code type_error;
    if (!it->is_directory(type_error) || it->is_symlink(type_error)) continue;
    auto entry = ReadEntry(it->path());
    if (!entry) continue;
    // Classified read-only (the library is not ours to write, and may be
    // read-only), so remembered here until the session's files change size.
    std::uintmax_t bytes = 0;
    for (const auto& file : entry->files) bytes += file.second;
    auto& known = classes_[entry->id];
    if (known.first != bytes || known.second.empty()) {
      known = {bytes,
               ClassName(talos::launcher::InspectSession(
                             it->path(), talos::launcher::ClassCache::kReadOnly)
                             .verdict)};
    }
    entry->verdict = known.second;
    entries.push_back(std::move(*entry));
  }
  std::sort(entries.begin(), entries.end(),
            [](const SessionEntry& a, const SessionEntry& b) {
              return a.start_wall_ns != b.start_wall_ns
                         ? a.start_wall_ns > b.start_wall_ns
                         : a.id > b.id;
            });
  std::string out{"["};
  for (const SessionEntry& entry : entries) {
    if (out.size() > 1) out += ',';
    out += "{\"id\":";
    AppendEscaped(out, entry.id);
    out += ",\"start_wall_ns\":";
    AppendI64(out, entry.start_wall_ns);
    out += ",\"end_wall_ns\":";
    AppendI64(out, entry.end_wall_ns);
    out += ",\"class\":";
    AppendEscaped(out, entry.verdict);
    out += ",\"files\":[";
    for (std::size_t i = 0; i < entry.files.size(); ++i) {
      if (i != 0) out += ',';
      out += "{\"path\":";
      AppendEscaped(out, entry.files[i].first);
      out += ",\"bytes\":";
      out += std::to_string(entry.files[i].second);
      out += '}';
    }
    out += "]}";
  }
  out += ']';
  return {200, "application/json", std::move(out)};
}

Response Review::Channels(const Session& session) const {
  std::vector<talos::telemetry::ChannelDef> dictionary;
  for (const Topic& topic : session.topics()) {
    for (const auto& channel : topic.channels) {
      dictionary.push_back(
          {channel.name, channel.unit, channel.labels, channel.kind});
    }
  }
  std::string out{"{\"kind\":\"talos.telemetry.channels\",\"version\":"};
  out += std::to_string(kTelemetryVersion);
  out += ",\"stream_id\":";
  AppendU64(out, 0);
  out += ",\"dictionary_id\":";
  out += std::to_string(talos::telemetry::DictionaryId(dictionary));
  out += ",\"channels\":[";
  for (std::size_t i = 0; i < dictionary.size(); ++i) {
    if (i != 0) out += ',';
    const auto& channel = dictionary[i];
    out += "{\"name\":";
    AppendEscaped(out, channel.name);
    out += ",\"unit\":";
    AppendEscaped(out, channel.unit);
    out += ",\"kind\":";
    AppendEscaped(out, ChannelKindName(channel.kind));
    out += ",\"labels\":[";
    for (std::size_t j = 0; j < channel.labels.size(); ++j) {
      if (j != 0) out += ',';
      AppendEscaped(out, channel.labels[j]);
    }
    // No "alert" until naming::ParseAlertField exists (slice S3a).
    out += "]}";
  }
  out += "]}";
  return {200, "application/json", std::move(out)};
}

Response Review::Bounds(const Session& session) const {
  std::string out{"{\"start_ns\":"};
  AppendI64(out, session.start_ns());
  out += ",\"end_ns\":";
  AppendI64(out, session.end_ns());
  out += ",\"origin_wall_ns\":";
  AppendI64(out, session.origin_wall_ns());
  out += '}';
  return {200, "application/json", std::move(out)};
}

Response Review::Snapshot(const Session& session, std::string_view query) {
  std::vector<std::pair<std::string, std::string>> parameters;
  std::int64_t t_ns = 0;
  if (!ParseQuery(query, parameters) ||
      !IntParameter(parameters, "t_ns", t_ns)) {
    return Error(400, "snapshot needs one integer t_ns");
  }
  last_read_ = {};
  const auto samples = session.Snapshot(t_ns, &last_read_);
  std::string out{"{\"t_ns\":"};
  AppendI64(out, t_ns);
  out += ",\"channels\":{";
  bool first = true;
  for (const Sample& sample : samples) {
    const auto& channel = session.channel(sample.channel);
    if (!first) out += ',';
    first = false;
    AppendEscaped(out, channel.name);
    out += ":{\"value\":";
    if (sample.reading) {
      AppendSample(out, channel.kind, sample.raw);
    } else {
      out += "null";
    }
    out += ",\"sample_ns\":";
    AppendI64(out, sample.sample_ns);
    out += '}';
  }
  out += "}}";
  return {200, "application/json", std::move(out)};
}

Response Review::Series(std::string_view id, const Session& session,
                        std::string_view query) {
  std::vector<std::pair<std::string, std::string>> parameters;
  std::int64_t start_ns = 0;
  std::int64_t end_ns = 0;
  std::int64_t points = 0;
  if (!ParseQuery(query, parameters) ||
      !IntParameter(parameters, "start_ns", start_ns) ||
      !IntParameter(parameters, "end_ns", end_ns) ||
      !IntParameter(parameters, "points", points)) {
    return Error(400, "series needs one integer start_ns, end_ns and points");
  }
  if (end_ns <= start_ns || points < 1 || points > kMaxSeriesPoints) {
    return Error(400, "series needs end_ns > start_ns and points in 1.." +
                          std::to_string(kMaxSeriesPoints));
  }
  // One c=a,b: channel names never hold a comma, so the decoded value is
  // split on them whether each name or the whole list was encoded.
  std::vector<std::string> names;
  int lists = 0;
  for (const auto& [key, value] : parameters) {
    if (key != "c") continue;
    if (++lists > 1) return Error(400, "series takes one c=a,b list");
    std::size_t begin = 0;
    while (begin <= value.size()) {
      const std::size_t comma = value.find(',', begin);
      const std::size_t end = comma == std::string::npos ? value.size() : comma;
      if (end > begin) names.push_back(value.substr(begin, end - begin));
      begin = end + 1;
    }
  }
  if (names.empty() || names.size() > kMaxSeriesChannels) {
    return Error(400, "series needs 1.." + std::to_string(kMaxSeriesChannels) +
                          " channels (c=)");
  }
  std::vector<ChannelRef> refs;
  for (const std::string& name : names) {
    const auto ref = session.Find(name);
    if (!ref) return Error(404, "no channel " + name);
    refs.push_back(*ref);
  }
  const auto buckets = static_cast<std::uint32_t>(points);
  const auto key = [&](const std::string& name) {
    std::string out{id};
    out += '\0';
    out += name;
    out += '\0';
    out += std::to_string(start_ns) + ' ' + std::to_string(end_ns) + ' ' +
           std::to_string(buckets);
    return out;
  };
  std::vector<std::shared_ptr<const review::Series>> series(refs.size());
  std::vector<ChannelRef> missing;
  std::vector<std::size_t> missing_at;
  for (std::size_t i = 0; i < refs.size(); ++i) {
    series[i] = cache_.Get(key(names[i]));
    if (series[i] == nullptr) {
      missing.push_back(refs[i]);
      missing_at.push_back(i);
    }
  }
  last_read_ = {};
  if (!missing.empty()) {
    auto read =
        session.ReadSeries(missing, start_ns, end_ns, buckets, &last_read_);
    for (std::size_t j = 0; j < read.size(); ++j) {
      auto shared = std::make_shared<const review::Series>(std::move(read[j]));
      cache_.Put(key(names[missing_at[j]]), shared);
      series[missing_at[j]] = std::move(shared);
    }
  }
  std::string out;
  out.reserve(8 + series.size() * buckets * 36);
  const auto channels = static_cast<std::uint32_t>(series.size());
  AppendLe(out, &channels, sizeof channels);
  AppendLe(out, &buckets, sizeof buckets);
  for (const auto& s : series) {
    AppendLe(out, s->t_ns.data(), buckets * sizeof(double));
    AppendLe(out, s->min.data(), buckets * sizeof(double));
    AppendLe(out, s->max.data(), buckets * sizeof(double));
    AppendLe(out, s->last.data(), buckets * sizeof(double));
    AppendLe(out, s->count.data(), buckets * sizeof(std::uint32_t));
  }
  return {200, "application/octet-stream", std::move(out)};
}

}  // namespace studio::review
