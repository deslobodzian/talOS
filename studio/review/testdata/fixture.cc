#include "studio/review/testdata/fixture.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "review_test/review_message_descriptor.h"
#include "review_test/review_message_generated.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/context.h"
#include "talOS/events/log/format.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/schema/identity.h"

namespace studio::review::testing {
namespace {

namespace ds = talos::driver_station;
using talos::event::Context;
using talos::event::EventKind;
using talos::event::MonotonicTime;
using talos::event::Registration;
using talos::event::SourceKind;
using talos::event::log::LogWriter;
using talos::event::log::LogWriterOptions;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::uint16_t kTick = 1;
constexpr std::uint16_t kShot = 2;
constexpr std::uint16_t kRequest = 3;
constexpr std::uint16_t kStatus = 4;
constexpr std::uint16_t kDriverStation = 3;

talos::schema::Identity ShotSchema() {
  using D = talos::Descriptor<review_test::Shot>;
  return {std::string{D::kName}, D::kHash,
          std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                      D::kBfbs.size()}};
}

talos::schema::Identity StatusSchema() {
  using D = talos::Descriptor<review_test::Status>;
  return {std::string{D::kName}, D::kHash,
          std::string{reinterpret_cast<const char*>(D::kBfbs.data()),
                      D::kBfbs.size()}};
}

talos::schema::Identity PacketSchema() {
  using D = talos::Descriptor<talos::hardware::Packet>;
  return {std::string{D::kName}, D::kHash, {}};
}

Registration Source(std::uint16_t id, SourceKind kind, std::string name,
                    std::uint32_t bytes, talos::schema::Identity schema) {
  Registration source;
  source.id = id;
  source.kind = kind;
  source.name = std::move(name);
  source.message_bytes = bytes;
  source.schema = std::move(schema);
  return source;
}

review_test::Shot ShotOf(int k) {
  return review_test::Shot{ShotPosition(k), static_cast<std::uint32_t>(k),
                           static_cast<review_test::Mode>(k % 3), k % 2 == 1,
                           ShotId(k)};
}

talos::hardware::Packet RequestOf(int k) {
  talos::hardware::Command command;
  command.config_id = 99;
  command.count = 1;
  command.motors[0] = {1, talos::hardware::Mode::kVoltage, 0, 0.5 * k, -0.25};
  talos::hardware::Packet packet{};
  packet.size =
      static_cast<std::uint32_t>(talos::hardware::Encode(command, packet.data));
  return packet;
}

talos::hardware::Packet DriverStationOf(int k) {
  ds::DriverStationData data;
  data.sample_time_us = static_cast<std::uint64_t>(k) * 100'000;
  data.mode_since_us = k < 50 ? 1 : 5'000'000;
  data.fms.flags = ds::kDsAttached | ds::kTeleop | (k >= 50 ? ds::kEnabled : 0);
  data.fms.alliance = ds::Alliance::kRed;
  data.fms.match_time_s = 150.0f - 0.1f * static_cast<float>(k);
  talos::hardware::Packet packet{};
  packet.size = static_cast<std::uint32_t>(ds::Encode(data, packet.data));
  return packet;
}

template <class T>
std::span<const std::byte> Bytes(const T& value) {
  return std::as_bytes(std::span{&value, 1});
}

void Check(const LogWriter& writer) {
  if (writer.failed()) throw std::runtime_error(writer.error());
}

// One shooter run: ticks first..last, each sending a shot and a request.
void WriteShooterRun(const std::string& path, int first, int last,
                     std::uint64_t max_bytes) {
  LogWriterOptions options{8192, 1, false};
  options.max_bytes = max_bytes;
  LogWriter writer{path, "shooter", options, kSessionId};
  writer.start({Source(kTick, SourceKind::TIMER, "update", 0, {}),
                Source(kShot, SourceKind::SENDER, "/shooter/state",
                       sizeof(review_test::Shot), ShotSchema()),
                Source(kRequest, SourceKind::SENDER, "/hw/request/shooter",
                       sizeof(talos::hardware::Packet), PacketSchema()),
                Source(kStatus, SourceKind::SENDER, "/shooter/status",
                       sizeof(review_test::Status), StatusSchema())},
               MonotonicTime::from_nanos(kStartNs + first * 20 * kMs));
  std::uint64_t index = 0;
  for (int k = first; k <= last; ++k) {
    Context tick{};
    tick.kind = EventKind::TIMER;
    tick.source_id = kTick;
    tick.dispatch_index = ++index;
    tick.event_time = MonotonicTime::from_nanos(kStartNs + k * 20 * kMs);
    tick.now = MonotonicTime::from_nanos(kStartNs + k * 20 * kMs + kMs / 2);
    tick.sequence = 1;
    writer.dispatch(tick, {});
    const review_test::Shot shot = ShotOf(k);
    writer.send(tick, kShot, Bytes(shot), 0, static_cast<std::uint64_t>(k));
    const talos::hardware::Packet request = RequestOf(k);
    writer.send(tick, kRequest, Bytes(request), 0,
                static_cast<std::uint64_t>(k));
    if (k == 0) {
      const review_test::Status status{kBootId};
      writer.send(tick, kStatus, Bytes(status), 0, 0);
    }
    if (k == kFailedShot) {
      // A send the transport failed (SendStatus 2): logged, never received.
      review_test::Shot failed = ShotOf(k);
      failed.mutate_position_m(-1.0);
      writer.send(tick, kShot, Bytes(failed), 2, 0);
    }
  }
  writer.flush();
  Check(writer);
}

void WriteObserver(const std::string& path) {
  struct Event {
    std::int64_t event_ns;
    std::uint16_t source;
    int k;
  };
  std::vector<Event> events;
  for (int k = 0; k <= 499; ++k) {
    if (k % 5 != 0 || (k > 299 && k < 350)) continue;
    events.push_back({kStartNs + k * 20 * kMs + kMs * 3 / 2, kShot, k});
  }
  for (int k = 0; k <= 100; ++k) {
    events.push_back({kStartNs + k * 100 * kMs + 2 * kMs, kDriverStation, k});
  }
  std::sort(events.begin(), events.end(), [](const Event& a, const Event& b) {
    return a.event_ns < b.event_ns;
  });
  LogWriter writer{path, "observer", LogWriterOptions{8192, 1, false},
                   kSessionId};
  writer.start(
      {Source(1, SourceKind::TIMER, "update", 0, {}),
       Source(kShot, SourceKind::WATCHER, "/shooter/state",
              sizeof(review_test::Shot), ShotSchema()),
       Source(kDriverStation, SourceKind::WATCHER, "/hw/state/driver_station",
              sizeof(talos::hardware::Packet), PacketSchema())},
      MonotonicTime::from_nanos(kStartNs + kObserverOriginNs));
  std::uint64_t index = 0;
  for (const Event& e : events) {
    Context message{};
    message.kind = EventKind::MESSAGE;
    message.dispatch_index = ++index;
    message.event_time = MonotonicTime::from_nanos(e.event_ns);
    message.now = message.event_time;
    message.sequence = static_cast<std::uint64_t>(e.k) + 1;
    if (e.source == kShot) {
      message.source_id = kShot;
      message.publish_time =
          MonotonicTime::from_nanos(kStartNs + e.k * 20 * kMs + kMs / 2);
      const review_test::Shot shot = ShotOf(e.k);
      writer.dispatch(message, Bytes(shot));
    } else {
      message.source_id = kDriverStation;
      message.publish_time =
          MonotonicTime::from_nanos(kStartNs + e.k * 100 * kMs);
      const talos::hardware::Packet packet = DriverStationOf(e.k);
      writer.dispatch(message, Bytes(packet));
    }
  }
  writer.flush();
  Check(writer);
}

// Flips a payload byte of the record of Driver Station packet `k`.
void DamagePacket(const std::string& path, int k) {
  std::uint64_t offset = 0;
  {
    talos::event::log::WindowedLogReader reader{path};
    talos::event::log::LogReader::Record record;
    while (reader.next(record)) {
      if (record.header.source_id == kDriverStation &&
          record.header.publish_time_ns == kStartNs + k * 100 * kMs) {
        offset = record.offset;
        break;
      }
    }
  }
  if (offset == 0) throw std::runtime_error("no packet to damage");
  std::fstream file{path, std::ios::in | std::ios::out | std::ios::binary};
  const auto at = static_cast<std::streamoff>(
      offset + sizeof(talos::event::log::RecordHeader) + 8);
  file.seekg(at);
  char byte = 0;
  file.read(&byte, 1);
  byte = static_cast<char>(byte ^ 0x5A);
  file.seekp(at);
  file.write(&byte, 1);
  if (!file) throw std::runtime_error("cannot damage " + path);
}

// Sets every log header's wall clock to kStartWallNs at kStartNs, in place
// of the time the fixture was written (the header's CRC does not cover it).
void FixWallClock(const std::string& dir) {
  namespace tlog = talos::event::log;
  for (const auto& entry : std::filesystem::directory_iterator{dir}) {
    if (entry.path().extension() != ".tlog") continue;
    std::fstream file{entry.path(),
                      std::ios::in | std::ios::out | std::ios::binary};
    tlog::FileHeader header{};
    file.read(reinterpret_cast<char*>(&header), sizeof header);
    header.start_wall_ns =
        kStartWallNs + (header.start_monotonic_ns - kStartNs);
    file.seekp(offsetof(tlog::FileHeader, start_wall_ns));
    file.write(reinterpret_cast<const char*>(&header.start_wall_ns),
               sizeof header.start_wall_ns);
    if (!file) {
      throw std::runtime_error("cannot set the wall clock of " +
                               entry.path().string());
    }
  }
}

}  // namespace

double ShotPosition(int k) {
  return k == kLastShot ? std::numeric_limits<double>::quiet_NaN()
                        : 0.25 * k + 0.125;
}

void WriteFixtureSession(const std::string& dir) {
  std::filesystem::create_directories(dir);
  WriteShooterRun(dir + "/shooter.tlog", 0, 299, 256u << 10);
  WriteShooterRun(dir + "/shooter.restart1.tlog", 350, 499, 0);
  WriteObserver(dir + "/observer.tlog");
  DamagePacket(dir + "/observer.tlog", kDamagedPacket);
  FixWallClock(dir);
  std::ofstream{dir + "/manifest.json"}
      << "{\n  \"session_id\": " << kSessionId
      << ",\n  \"simulation\": true,\n  \"start_wall_ns\": " << kStartWallNs
      << ",\n  \"end_wall_ns\": " << kStartWallNs + 10'000'000'000
      << ",\n  \"nodes\": []\n}\n";
  if (!std::filesystem::exists(
          talos::event::log::LogSegmentPath(dir + "/shooter.tlog", 0))) {
    throw std::runtime_error("the shooter's first run did not rotate");
  }
}

}  // namespace studio::review::testing
