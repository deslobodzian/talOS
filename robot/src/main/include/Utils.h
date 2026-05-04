#pragma once

#include "types.h"
#include "udp.h"

constexpr int kMaxSignals = 50;
class Payload {
public:
    explicit Payload(talos::protocol::RuntimeUdpPeer& peer) : peer_(peer), ptr_(nullptr)  {
        ptr_ = peer_.MutablePayloadBuffer(kMaxSignals);
    }
    void AddSignalToPayload(talos::protocol::SignalValue signal) {
    }
private:
    talos::protocol::RuntimeUdpPeer& peer_;
    uint8_t* ptr_ = nullptr;
};
