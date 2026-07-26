#include "event_loop.h"
#include <sys/eventfd.h>

namespace talos::event {

EventLoop::EventLoop(int flags) :
    running_{false},
    event_fd_{
        ::eventfd(0,
        EFD_NONBLOCK | EFD_CLOEXEC
    )},
    epoll_{flags} {
}

} // namespace talos::event

