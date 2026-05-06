#pragma once

namespace talos::protocol {
struct TcpSocketOptions {
    bool non_blocking = true;
    bool no_delay = true;
};
} /* namespace protocol */
