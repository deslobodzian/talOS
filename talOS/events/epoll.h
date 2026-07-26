#pragma once

#include <sys/epoll.h>
#include <unistd.h>

#include <cerrno>
#include <system_error>

// In epoll event_data is part of epoll_event which looks like this:
// union epoll_data {
//   void     *ptr;
//   int       fd;
//   uint32_t  u32;
//   uint64_t  u64;
// };
//

#define EPOLL_EVENT_FD_NOT_DEFINED -1

struct EpollContext {
  void* handler_ptr;
  void (*dispatch_thunk)(void*, uint32_t);  // takes an events bitmask
};

template <typename Derived>
class EpollHandler {
 public:
  int fd() const { return fd_; };
  void register_loop(int epoll_fd, uint32_t events) {
    context_.handler_ptr = this;
    context_.dispatch_thunk = [](void* ptr, uint32_t events) {
      static_cast<Derived*>(static_cast<EpollHandler*>(ptr))->on_event(events);
      static_cast<EpollHandler*>(ptr)->drain_fd();
    };

    epoll_event event{};
    event.events = events;
    event.data.ptr = &context_;
    if (::epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd_, &event) == -1) {
      throw std::system_error(errno, std::generic_category(), "epoll_ctl ADD");
    }
  }

 protected:
  explicit EpollHandler(int fd) : fd_(fd) {}
  void set_fd(int fd) {
    if (fd_ == EPOLL_EVENT_FD_NOT_DEFINED) fd_ = fd;
  }
  ~EpollHandler() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  void drain_fd() {
    uint64_t buf;
    [[maybe_unused]] auto ssize = ::read(fd_, &buf, sizeof(uint64_t));
  }

  int fd_{};
  EpollContext context_{};
};

// Events is a uint32_t which is just a bitmask for possible events.
// From man pages epoll has a few types of events, the ones we will likely care
// about are: EPOLLIN: file descriptor is available for a read EPOLLOUT: file
// desriptor is available for a write EPOLLERR: error condition on the file
// descriptor Will likely not use (yet) EPOLLHUP: Hang up on file descriptor.
// Example pipe or stream socket closed its channel EPOLLPRI: exceptional
// condition is raised on the file descriptor EPOLLRDHUP
//
// Flags:
// EPOLLET: edge-triggered notification on file descriptor
// EPOLLONESHOT: one-shot notification on the file descriptor
// May not use (yet):
// EPOLLWAKEUP
// EPOLLEXCLUSIVE
//
class Epoll {
 public:
  explicit Epoll(int flags) : epoll_fd_(::epoll_create1(flags)) {
    if (epoll_fd_ < 0) {
      throw std::system_error(errno, std::generic_category(), "epoll_create1");
    }
  }

  ~Epoll() { ::close(epoll_fd_); }

  bool poll() {
    struct epoll_event event;
    int num_events = epoll_wait(epoll_fd_, &event, 1, -1);
    if (num_events == 0) {
      return false;
    }

    const EpollContext* event_context =
        static_cast<EpollContext*>(event.data.ptr);
    event_context->dispatch_thunk(event_context->handler_ptr, event.events);
    return true;
  }

  template <typename T>
  void register_event(EpollHandler<T>& event, uint32_t events) {
    event.register_loop(epoll_fd_, events);
  }

 private:
  int epoll_fd_{};
};
