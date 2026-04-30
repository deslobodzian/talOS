#pragma once

#include <memory>

#include <optional>

template <typename T>
class RingBuffer {
public:
    explicit RingBuffer(size_t size) :
     buffer_(std::make_unique<T[]>(size)),
        capacity_(size),
        head_(0),
        tail_(0),
        full_(false) {
    }

    std::optional<T> get() {
        if (empty()) {
            return std::nullopt;
        }

        T val = buffer_[tail_];
        full_ = false;
        tail_ = (tail_ + 1) % capacity_;
        return val;
    }

    bool empty() const { return (!full_ && (head_ == tail_)); }
    bool full() const { return full_; }
    size_t capacity() const { return capacity_; }

    size_t size() const {
        if (full_) {
            return capacity_;
        }
        if (head_ <= tail_) {
            return head_ - tail_;
        }

        return capacity_ + head_ - tail_;
    }


private:
    std::unique_ptr<T[]> buffer_;
    size_t capacity_;
    size_t head_;
    size_t tail_;
    bool full_;
};
