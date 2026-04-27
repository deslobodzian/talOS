#pragma once

#include "device.h"
#include <memory>

class RawBuffer {
public:
    RawBuffer(Device& device, BufferDesc desc) :
        device_(&device), desc_(desc) {
        allocation_ = device_->allocator().allocate(desc_);
        if (!allocation_) {
            throw std::runtime_error("buffer allocation failed");
        }

    }

    RawBuffer(const RawBuffer&) = delete; // Copy constructor
    RawBuffer& operator=(RawBuffer&) = delete; // Copy assignment

    RawBuffer(RawBuffer&&) noexcept = default; // Move constructor
    RawBuffer& operator=(RawBuffer&&) noexcept = default; // Move assignment

    Device& device() const { return *device_; }
    Allocation& allocation() { return *allocation_; }
    const Allocation& allocation() const { return *allocation_; }

    std::size_t size_bytes() const { return allocation_->size(); }
    MemoryLocation location() const { return allocation_->location(); }
    MemorySource source() const { return allocation_->source(); }

    void* data() const { return allocation_->data(); }

    void upload(const void* src, std::size_t bytes) {
        if (bytes > size_bytes()) {
            throw std::runtime_error("upload exceeds buffer size");
        }
        device_->allocator().copy_in(*allocation_, src, bytes);
    }

    void download(void* dst, std::size_t bytes) const {
        if (bytes > size_bytes()) {
            throw std::runtime_error("download exceeds buffer size");
        }
        device_->allocator().copy_out(*allocation_, dst, bytes);
    }

    void copy_from(const RawBuffer& src, std::size_t bytes) {
        if (&device_ != &src.device_) {
            throw std::runtime_error("cross-device copy not supported by RawBuffer");
        }
        if (bytes > size_bytes() || bytes > src.size_bytes()) {
            throw std::runtime_error("copy exceeds buffer size");
        }
        device_->allocator().copy_between(*allocation_, *src.allocation_, bytes);
    }

private:
    Device* device_;
    BufferDesc desc_;
    std::unique_ptr<Allocation> allocation_;
};

template <typename T>
class Buffer {
public:
    Buffer(Device& device,
           std::size_t count,
           MemoryLocation location = MemoryLocation::Device)
        : count_(count),
          raw_(device, BufferDesc{
              .size = count * sizeof(T),
              .location = location,
              .source = MemorySource::Allocate,
              .external_ptr = nullptr
          }) {}

    Buffer(Device& device,
           T* external_ptr,
           std::size_t count,
           MemoryLocation location = MemoryLocation::Host)
        : count_(count),
          raw_(device, BufferDesc{
              .size = count * sizeof(T),
              .location = location,
              .source = MemorySource::External,
              .external_ptr = external_ptr
          }) {}

    Buffer(const Buffer&) = delete;
    Buffer& operator=(const Buffer&) = delete;

    Buffer(Buffer&&) noexcept = default;
    Buffer& operator=(Buffer&&) noexcept = default;

    std::size_t count() const { return count_; }
    std::size_t size_bytes() const { return count_ * sizeof(T); }

    Device& device() const { return raw_.device(); }
    RawBuffer& raw() { return raw_; }
    const RawBuffer& raw() const { return raw_; }

    T* data() const {
        return static_cast<T*>(raw_.data());
    }

    void upload(const T* src, std::size_t count) {
        if (count > count_) {
            throw std::runtime_error("upload exceeds element count");
        }
        raw_.upload(src, count * sizeof(T));
    }

    void download(T* dst, std::size_t count) const {
        if (count > count_) {
            throw std::runtime_error("download exceeds element count");
        }
        raw_.download(dst, count * sizeof(T));
    }

private:
    std::size_t count_ = 0;
    RawBuffer raw_;
};
