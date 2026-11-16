// supervision_test's lost-wake fixture, run as `alpha`: it loses one wake
// notification on its byte topic, /alpha/telemetry (Node::publish_bytes),
// which its loop does not write. Its constructor holds a reader of that
// topic armed on a wake receiver of its own; its first tick publishes with
// the descriptor limit at the lowest free descriptor, so the writer cannot
// open the socket the notification needs, and raises the limit again at
// once. Under --describe it only declares the topic.

#include <fcntl.h>
#include <sys/resource.h>
#include <unistd.h>

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <stdexcept>

#include "talOS/node/node.h"
#include "talOS/rtms/rtms.h"
#include "talOS/rtms/wake.h"
#include "talOS/schema/identity.h"

namespace {

constexpr std::size_t kMessageBytes = 8;

class ByteWakeLoss {
 public:
  explicit ByteWakeLoss(talos::Node& node)
      : frames_{node.publish_bytes("/alpha/telemetry", kMessageBytes)} {
    if (node.describing()) return;
    reader_.emplace(
        "/alpha/telemetry", kMessageBytes, talos::kMessageAlignment,
        talos::schema::Bytes(), talos::BytePublishOptions{}.slots,
        RTMSOptions{.overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
                    .read_mode = ReadMode::SEQUENCE});
    reader_id_ = reader_->register_reader();
    if (!reader_id_) throw std::runtime_error("no reader slot");
    reader_->set_reader_wake(*reader_id_, receiver_.id());
    if (!reader_->arm_wake(*reader_id_))
      throw std::runtime_error("the reader could not arm");
  }

  void update() {
    if (sent_ || !reader_) return;
    sent_ = true;
    // The lowest free descriptor: with the soft limit there, no socket can
    // open until it is raised again.
    const int lowest = ::open("/dev/null", O_RDONLY | O_CLOEXEC);
    rlimit limit{};
    if (lowest < 0 || ::close(lowest) != 0 ||
        ::getrlimit(RLIMIT_NOFILE, &limit) != 0)
      throw std::runtime_error("cannot find the lowest free descriptor");
    const rlimit lowered{.rlim_cur = static_cast<rlim_t>(lowest),
                         .rlim_max = limit.rlim_max};
    const std::array<std::byte, kMessageBytes> frame{};
    const bool lowered_ok = ::setrlimit(RLIMIT_NOFILE, &lowered) == 0;
    frames_.send(frame);
    if (lowered_ok && ::setrlimit(RLIMIT_NOFILE, &limit) != 0)
      throw std::runtime_error("cannot raise the descriptor limit again");
  }

 private:
  talos::BytePublisher frames_;
  std::optional<RTMSQueue> reader_;
  std::optional<std::size_t> reader_id_;
  talos::WakeReceiver receiver_;
  bool sent_ = false;
};

}  // namespace

TALOS_NODE(ByteWakeLoss)
