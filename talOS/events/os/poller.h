#pragma once

// Compile-time selection of the readiness backend. There is deliberately no
// common base class: KqueuePoller and EpollPoller are unrelated concrete
// types with an identical (but non-virtual) surface, so picking one is a
// type alias rather than a runtime dispatch. This keeps wait_until() free of
// vtable indirection on the 1kHz control-loop hot path.
#if defined(__APPLE__)
#include "talOS/events/os/kqueue_poller.h"

namespace talos::event {
using Poller = KqueuePoller;
}  // namespace talos::event

#elif defined(__linux__)
#include "talOS/events/os/epoll_poller.h"

namespace talos::event {
using Poller = EpollPoller;
}  // namespace talos::event

#else
#error "unsupported platform"
#endif
