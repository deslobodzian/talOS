#pragma once

/*
 * The driver dashboard's feed, `/drive` (REQ-OPS-35; the protocol a page is
 * written against is README.md "Driver dashboard feed"):
 *
 * - EgressPlan / BridgeEgress: the bridge's 160,000 B/s, with each drive
 *   client's budget reserved out of it before programmer clients share the
 *   rest, so no programmer client can defer a drive client's send.
 * - DriveSlots: the drive clients held, outside `max_clients`; a new one
 *   evicts the oldest.
 * - DriveMask: which dictionary channels a drive client is sent.
 * - BuildDriveSummary / DriveSummaryJson: the drive summary, what the
 *   `subsystems` widget needs that channels cannot give (a dead node raises
 *   no alert), capped at telemetry_wire::kDriveSummaryMaxBytes.
 * - DriveFeed: one drive client's telemetry (masked, at most `update_hz`
 *   updates a second) and summary (on change, at most every
 *   telemetry_wire::kDriveSummaryIntervalNs).
 *
 * Header-only, like :telemetry, so all of it is tested without a socket.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "studio/bridge/system.h"
#include "studio/bridge/telemetry.h"
#include "talOS/introspection/registry.h"
#include "talOS/telemetry/wire.h"

namespace studio {

// The egress rates, B/s
// (docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary). Without a
// dashboard layout nothing is reserved and programmer clients share the whole
// total, as before the drive feed existed.
struct EgressPlan {
  static constexpr double kTotalBytesPerSecond = EgressBudget::kBytesPerSecond;

  double programmer_bytes_per_s{kTotalBytesPerSecond};  // the shared bucket
  double programmer_client_bytes_per_s{kTotalBytesPerSecond};
  double drive_client_bytes_per_s{0};  // each drive client's own bucket
  std::size_t drive_clients{0};        // the drive slots; 0: no /drive

  // `drive_clients` x `drive_budget_bytes_per_s` is reserved (the layout's
  // `drive_clients` and `budget_bytes_per_s`; 0 and 0 without a layout).
  // Throws std::invalid_argument when the reservation leaves programmer
  // clients nothing (the layout parser already refuses over 80,000 B/s).
  static EgressPlan Of(std::size_t max_clients, std::uint32_t drive_clients,
                       std::uint32_t drive_budget_bytes_per_s) {
    const double reserved = static_cast<double>(drive_clients) *
                            static_cast<double>(drive_budget_bytes_per_s);
    if (max_clients == 0 || reserved >= kTotalBytesPerSecond) {
      throw std::invalid_argument(
          "studio_bridge: drive reservation leaves programmer clients no "
          "budget");
    }
    EgressPlan plan;
    plan.programmer_bytes_per_s = kTotalBytesPerSecond - reserved;
    plan.programmer_client_bytes_per_s =
        plan.programmer_bytes_per_s / static_cast<double>(max_clients);
    plan.drive_client_bytes_per_s = drive_budget_bytes_per_s;
    plan.drive_clients = drive_clients;
    return plan;
  }
};

// Every send's budget: a programmer client's is charged to its own bucket
// and to the shared one, a drive client's to its own bucket only. The shared
// bucket refills at the programmer share alone, so however much programmer
// clients spend, a drive client's sends wait only for its own bucket, and
// the rates add up to the total.
class BridgeEgress {
 public:
  explicit BridgeEgress(EgressPlan plan = {}) : plan_{plan} {}

  const EgressPlan& plan() const { return plan_; }

  // Whether `framed_bytes` fit now, spending nothing. When they do not,
  // `wait_ns` is how long until they will (EgressBudget::wait_ns of every
  // bucket the send is charged to, the largest; kNeverNs: never).
  bool Affordable(EgressBudget& own, bool drive, std::size_t framed_bytes,
                  std::int64_t now_ns, std::int64_t& wait_ns) {
    const double rate = drive ? plan_.drive_client_bytes_per_s
                              : plan_.programmer_client_bytes_per_s;
    const bool fits_own = own.available(framed_bytes, rate, now_ns);
    wait_ns = fits_own ? 0 : own.wait_ns(framed_bytes, rate);
    if (drive) return fits_own;
    const bool fits_shared =
        shared_.available(framed_bytes, plan_.programmer_bytes_per_s, now_ns);
    if (!fits_shared) {
      wait_ns = std::max(
          wait_ns, shared_.wait_ns(framed_bytes, plan_.programmer_bytes_per_s));
    }
    return fits_own && fits_shared;
  }

  // Charges a send Affordable allowed.
  void Spend(EgressBudget& own, bool drive, std::size_t framed_bytes) {
    own.spend(framed_bytes);
    if (!drive) shared_.spend(framed_bytes);
  }

 private:
  EgressPlan plan_;
  EgressBudget shared_;
};

// The close a drive client is sent when a newer one takes its slot: a
// WebSocket close code from the application range and its reason.
inline constexpr int kDriveEvictedCloseCode = 4000;
inline constexpr std::string_view kDriveEvictedReason =
    "replaced by a newer driver dashboard";

// The drive clients held, oldest first. A laptop reconnecting after a Wi-Fi
// drop always gets in: its new connection evicts the oldest, which may be
// its own dead one that the idle timeout has not closed yet.
template <class Client>
class DriveSlots {
 public:
  explicit DriveSlots(std::size_t capacity = 0) : capacity_{capacity} {}

  // Holds `client` as the newest. Returns the oldest when that leaves more
  // than the capacity held: it is no longer held, and the caller closes it.
  std::optional<Client> Admit(Client client) {
    clients_.push_back(client);
    if (clients_.size() <= capacity_) return std::nullopt;
    Client oldest = clients_.front();
    clients_.erase(clients_.begin());
    return oldest;
  }

  void Remove(Client client) {
    clients_.erase(std::remove(clients_.begin(), clients_.end(), client),
                   clients_.end());
  }

  const std::vector<Client>& clients() const { return clients_; }

 private:
  std::size_t capacity_;
  std::vector<Client> clients_;
};

// Which dictionary channels a drive client is sent: the layout's channels
// (telemetry::RequiredChannels) and, with an `alerts` widget
// (telemetry::CarriesAlertChannels), every alert channel of the dictionary.
// A named channel the dictionary lacks is simply never sent; the page shows
// it as absent.
class DriveMask {
 public:
  DriveMask() = default;
  DriveMask(std::vector<std::string> channels, bool alerts)
      : channels_{std::move(channels)}, alerts_{alerts} {
    std::sort(channels_.begin(), channels_.end());
  }

  // The mask for `state`'s dictionary, indexed like it: 1 for a channel the
  // client is sent. Recomputed only when the dictionary changes.
  const std::vector<std::uint8_t>& For(const TelemetryState& state) {
    if (!state.has_dictionary) {
      mask_.clear();
      known_ = false;
      return mask_;
    }
    if (known_ && stream_id_ == state.stream_id &&
        dictionary_id_ == state.dictionary_id &&
        mask_.size() == state.dictionary.size()) {
      return mask_;
    }
    mask_.assign(state.dictionary.size(), 0);
    for (std::size_t i = 0; i < state.dictionary.size(); ++i) {
      const TelemetryChannelInfo& channel = state.dictionary[i];
      const bool named =
          std::binary_search(channels_.begin(), channels_.end(), channel.name);
      const bool alert =
          alerts_ && channel.alert != Talos::Telemetry::AlertLevel_None;
      mask_[i] = named || alert ? 1 : 0;
    }
    known_ = true;
    stream_id_ = state.stream_id;
    dictionary_id_ = state.dictionary_id;
    return mask_;
  }

 private:
  std::vector<std::string> channels_;
  bool alerts_{false};
  bool known_{false};
  std::uint64_t stream_id_{0};
  std::uint32_t dictionary_id_{0};
  std::vector<std::uint8_t> mask_;
};

// One node of the drive summary.
struct DriveSummaryNode {
  std::string name;
  bool alive{false};
  bool critical{true};
  std::uint64_t restarts{0};
  bool quarantined{false};
  // The target lane source its Target last selected, while it is alive;
  // nullopt when it selected none or reads no target.
  std::optional<std::string> drives;
};

struct DriveSummary {
  std::string session;     // "" while no source names it
  bool supervised{false};  // restarts and quarantine come from a manifest
  std::vector<DriveSummaryNode> nodes;
};

// A node the session's config launches (the launcher's roster): its name and
// whether the launcher treats it as critical.
struct DriveRosterEntry {
  std::string name;
  bool critical{true};
};

// The drive summary of `roster`, in its order: `alive` and `drives` from
// the live registry's rows of that name, `restarts`, `quarantined` and the
// session from the launcher's manifest (`manifest_json`, empty when there
// is none). A manifest that does not parse is treated as absent, so the
// summary says `supervised: false` rather than zero restarts as a fact;
// the session then comes from the registry.
inline DriveSummary BuildDriveSummary(
    const std::vector<DriveRosterEntry>& roster,
    const std::vector<talos::introspect::NodeSnapshot>& registry,
    std::string_view manifest_json) {
  DriveSummary summary;
  nlohmann::json manifest;
  if (!manifest_json.empty()) {
    manifest = nlohmann::json::parse(manifest_json, nullptr, false);
  }
  summary.supervised = manifest.is_object();
  if (summary.supervised && manifest.contains("session_id")) {
    const auto& id = manifest["session_id"];
    if (id.is_number_unsigned()) {
      summary.session = std::to_string(id.get<std::uint64_t>());
    } else if (id.is_string()) {
      summary.session = id.get<std::string>();
    }
  }
  if (summary.session.empty()) {
    for (const auto& row : registry) {
      if (row.alive && row.session_id != 0) {
        summary.session = std::to_string(row.session_id);
        break;
      }
    }
  }
  summary.nodes.reserve(roster.size());
  for (const DriveRosterEntry& entry : roster) {
    DriveSummaryNode node;
    node.name = entry.name;
    node.critical = entry.critical;
    for (const auto& row : registry) {
      if (row.name != entry.name || !row.alive) continue;
      node.alive = true;
      for (const auto& selection : row.selections) {
        if (auto source = selection.selected_source()) {
          node.drives = std::move(*source);
          break;
        }
      }
      break;
    }
    if (summary.supervised && manifest.contains("nodes") &&
        manifest["nodes"].is_array()) {
      for (const auto& row : manifest["nodes"]) {
        if (!row.is_object() || !row.contains("name") ||
            !row["name"].is_string() || row["name"] != entry.name) {
          continue;
        }
        if (row.contains("restart_count") &&
            row["restart_count"].is_number_unsigned()) {
          node.restarts = row["restart_count"].get<std::uint64_t>();
        }
        if (row.contains("quarantined") && row["quarantined"].is_boolean()) {
          node.quarantined = row["quarantined"].get<bool>();
        }
        break;
      }
    }
    summary.nodes.push_back(std::move(node));
  }
  return summary;
}

namespace detail {

inline void AppendDriveNode(std::string& out, const DriveSummaryNode& node) {
  out += "{\"name\":";
  AppendEscaped(out, node.name);
  out += node.alive ? ",\"alive\":true" : ",\"alive\":false";
  out += node.critical ? ",\"critical\":true" : ",\"critical\":false";
  out += ",\"restarts\":";
  out += std::to_string(node.restarts);
  out += node.quarantined ? ",\"quarantined\":true" : ",\"quarantined\":false";
  out += ",\"drives\":";
  if (node.drives) {
    AppendEscaped(out, *node.drives);
  } else {
    out += "null";
  }
  out += '}';
}

}  // namespace detail

// The drive summary message (README.md "Driver dashboard feed"), at most
// `max_bytes`: when every node does not fit, the nodes at the end that do
// not are left out and counted in "omitted".
inline std::string DriveSummaryJson(
    const DriveSummary& summary,
    std::size_t max_bytes = talos::telemetry_wire::kDriveSummaryMaxBytes) {
  std::string head = "{\"kind\":\"talos.drive.summary\",\"session\":";
  AppendEscaped(head, summary.session);
  head += summary.supervised ? ",\"supervised\":true" : ",\"supervised\":false";
  head += ",\"nodes\":[";
  std::vector<std::string> nodes;
  nodes.reserve(summary.nodes.size());
  for (const DriveSummaryNode& node : summary.nodes) {
    std::string fragment;
    detail::AppendDriveNode(fragment, node);
    nodes.push_back(std::move(fragment));
  }
  const auto tail = [](std::size_t omitted) {
    return "],\"omitted\":" + std::to_string(omitted) + "}";
  };
  // The first `kept` nodes, separated by commas, then the tail counting
  // the rest: the most that fit.
  std::size_t kept = nodes.size();
  std::size_t body = 0;
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    body += nodes[i].size() + (i == 0 ? 0 : 1);
  }
  while (kept > 0 &&
         head.size() + body + tail(nodes.size() - kept).size() > max_bytes) {
    --kept;
    body -= nodes[kept].size() + (kept == 0 ? 0 : 1);
  }
  std::string out = std::move(head);
  out.reserve(out.size() + body + 32);
  for (std::size_t i = 0; i < kept; ++i) {
    if (i != 0) out += ',';
    out += nodes[i];
  }
  out += tail(nodes.size() - kept);
  return out;
}

// One drive client's summary: sent when it differs from what the client
// holds, at most once per kIntervalNs, and never past kMaxBytes.
class DriveSummaryFeed {
 public:
  static constexpr std::int64_t kIntervalNs =
      talos::telemetry_wire::kDriveSummaryIntervalNs;
  static constexpr std::size_t kMaxBytes =
      talos::telemetry_wire::kDriveSummaryMaxBytes;

  // `afford(framed_bytes)` and `send(text)` as in ClientFeed. A summary the
  // budget or the socket refuses waits and is retried; a newer one replaces
  // it.
  template <class Afford, class Send>
  void Pump(std::string_view summary, std::int64_t now_ns, Afford&& afford,
            Send&& send) {
    pending_ = !summary.empty() && summary.size() <= kMaxBytes &&
               (!sent_ || summary != held_);
    if (!pending_ || (sent_ && now_ns - sent_ns_ < kIntervalNs)) return;
    if (!afford(EgressBudget::framed(summary.size())) || !send(summary)) {
      return;
    }
    held_.assign(summary);
    sent_ = true;
    sent_ns_ = now_ns;
    pending_ = false;
  }

  // The end of the interval while a changed summary waits for it;
  // kNeverNs otherwise (one waiting only for budget is the caller's retry).
  std::int64_t NextDueNs() const {
    return pending_ && sent_ ? sent_ns_ + kIntervalNs : kNeverNs;
  }

 private:
  std::string held_;
  bool sent_{false};
  bool pending_{false};
  std::int64_t sent_ns_{0};
};

// One /drive client: the dictionary, then the masked telemetry at most
// `update_hz` updates a second, then the summary. It is sent no system graph.
// Until it holds a telemetry tick (no telemetry yet, or a new dictionary
// awaiting its keyframe), kHeartbeat goes whenever nothing else has for
// ClientFeed::kHeartbeatNs, so the page tells a bridge that is up but has no
// telemetry from a link that is down.
class DriveFeed {
 public:
  static constexpr std::string_view kHeartbeat =
      R"({"kind":"talos.drive.heartbeat"})";

  // The shortest gap between updates carrying a change, in whole
  // nanoseconds, rounded up so `update_hz` is never exceeded.
  static constexpr std::int64_t UpdateIntervalNs(std::uint32_t update_hz) {
    constexpr std::int64_t kSecondNs = 1'000'000'000;
    return update_hz == 0 ? kSecondNs : (kSecondNs + update_hz - 1) / update_hz;
  }

  void Configure(std::uint32_t update_hz) {
    telemetry_.LimitUpdates(UpdateIntervalNs(update_hz));
  }

  // `afford(framed_bytes)` as in ClientFeed; `send_text` sends a summary,
  // `send_binary` a telemetry page.
  template <class Afford, class SendText, class SendBinary>
  void Pump(std::string_view summary, const TelemetryState& state,
            const std::vector<std::uint8_t>& mask, std::int64_t now_ns,
            Afford&& afford, SendText&& send_text, SendBinary&& send_binary) {
    const auto sent = [this, now_ns](bool ok) {
      if (ok) {
        sent_ = true;
        sent_ns_ = now_ns;
      }
      return ok;
    };
    telemetry_.Pump(
        state, now_ns, afford,
        [&](std::string_view page) { return sent(send_binary(page)); }, &mask);
    summary_.Pump(summary, now_ns, afford,
                  [&](std::string_view text) { return sent(send_text(text)); });
    if (telemetry_.holds_tick() ||
        (sent_ && now_ns - sent_ns_ < ClientFeed::kHeartbeatNs)) {
      return;
    }
    if (afford(EgressBudget::framed(kHeartbeat.size()))) {
      sent(send_text(kHeartbeat));
    }
  }

  // The earliest instant Pump, with nothing new published, may send.
  std::int64_t NextDueNs() const {
    const std::int64_t heartbeat = telemetry_.holds_tick() || !sent_
                                       ? kNeverNs
                                       : sent_ns_ + ClientFeed::kHeartbeatNs;
    return std::min({telemetry_.NextDueNs(), summary_.NextDueNs(), heartbeat});
  }

  const ClientFeed& telemetry() const { return telemetry_; }

 private:
  ClientFeed telemetry_;
  DriveSummaryFeed summary_;
  bool sent_{false};  // anything, and when
  std::int64_t sent_ns_{0};
};

}  // namespace studio
