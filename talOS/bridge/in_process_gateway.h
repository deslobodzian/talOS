#pragma once

// The gateway end of a HardwareNode's link, in process: the bridge's
// component tests (trust_test.cc) drive a node through it with no socket, on
// the synthetic clock Tick() takes. A frame handed to Receive() goes through
// HardwareNode::HandleGatewayFrame, the function Tick()'s receive loop calls
// for every datagram it reads, and every frame the node sends goes through
// SendToGateway() into sent(). Only the socket is left out, which the UDP
// cases in hardware_node_test.cc cover.
//
// HardwareNode's friend, in a testonly library private to talOS/bridge: no
// other package, and no production code, can reach it.

#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "talOS/bridge/node.h"
#include "talOS/hardware/messages.h"
#include "talOS/hardware/packet.h"
#include "talOS/protocol/frame.h"

namespace talos::hardware::bridge_test {

class InProcessGateway {
 public:
  // A frame the node sent, at its bridge-clock time (a UDP frame header's).
  struct Frame {
    protocol::FrameType type;
    uint64_t sent_us;
    std::vector<uint8_t> payload;
  };

  // Opens `node`'s topics (Open() without its socket) and takes every frame
  // it sends until this is destroyed. `node` must outlive this.
  explicit InProcessGateway(HardwareNode& node) : node_{node} {
    node_.OpenTopics();
    node_.frame_sink_ = {&InProcessGateway::Take, this};
  }
  ~InProcessGateway() { node_.frame_sink_ = {}; }
  InProcessGateway(const InProcessGateway&) = delete;
  InProcessGateway& operator=(const InProcessGateway&) = delete;
  InProcessGateway(InProcessGateway&&) = delete;
  InProcessGateway& operator=(InProcessGateway&&) = delete;

  // Hands the node one datagram of this protocol version, received at bridge
  // time `now_us`, as its receive loop does.
  void Receive(protocol::FrameType type, uint64_t now_us,
               std::span<const uint8_t> payload) {
    protocol::DecodedFrame frame{};
    frame.header.type = type;
    frame.header.monotonic_time_us = now_us;
    frame.header.payload_size = static_cast<uint32_t>(payload.size());
    frame.payload = payload.data();
    frame.payload_size = payload.size();
    node_.HandleGatewayFrame(frame, now_us);
  }

  // A State received at `now_us`, then the Tick(now_us) that takes it in
  // over UDP. False when the node did not count it received (it did not
  // decode).
  bool Deliver(const State& state, uint64_t now_us) {
    Packet packet{};
    packet.size = static_cast<uint32_t>(Encode(state, packet.data));
    const uint64_t before = node_.states_received();
    Receive(protocol::FrameType::kHardwareState, now_us,
            std::span<const uint8_t>{packet.data.data(), packet.size});
    node_.Tick(now_us);
    return node_.states_received() > before;
  }

  // Every frame sent since the last call, oldest first.
  std::vector<Frame> TakeSent() { return std::exchange(sent_, {}); }

 private:
  static void Take(void* context, protocol::FrameType type, uint64_t now_us,
                   std::span<const uint8_t> payload) {
    static_cast<InProcessGateway*>(context)->sent_.push_back(
        {type, now_us, {payload.begin(), payload.end()}});
  }

  HardwareNode& node_;
  std::vector<Frame> sent_;
};

}  // namespace talos::hardware::bridge_test
