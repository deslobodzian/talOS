#pragma once

#include <cstddef>
#include <cstring>
#include <memory>
#include <stdexcept>

#include "device.h"

class CpuAllocation : public Allocation {
 public:
  CpuAllocation(Device& device, void* ptr, std::size_t size,
                MemoryLocation location, MemorySource source)
      : device_(device),
        ptr_(ptr),
        size_(size),
        location_(location),
        source_(source) {}

  ~CpuAllocation() override {
    if (source_ == MemorySource::Allocate) {
      ::operator delete(ptr_);
    }
  }

  CpuAllocation(const CpuAllocation&) = delete;
  CpuAllocation& operator=(const CpuAllocation&) = delete;

  Device& device() const override { return device_; }

  void* map() override { return ptr_; }
  void unmap() override {}

  void* data() const override { return ptr_; }

  std::size_t size() const override { return size_; }
  MemoryLocation location() const override { return location_; }
  MemorySource source() const override { return source_; }

 private:
  Device& device_;
  void* ptr_ = nullptr;
  std::size_t size_ = 0;
  MemoryLocation location_ = MemoryLocation::Host;
  MemorySource source_ = MemorySource::Allocate;
};

class CPUAllocator : public Allocator {
 public:
  explicit CPUAllocator(Device& device) : device_(device) {
    if (device_.type() != DeviceType::CPU) {
      throw std::invalid_argument("CPUAllocator device must be a cpu!");
    }
  }

  ~CPUAllocator() override = default;

  std::unique_ptr<Allocation> allocate(const BufferDesc& desc) override {
    if (desc.size == 0) {
      throw std::runtime_error("cannot allocate zero-size buffer");
    }

    if (desc.source == MemorySource::External) {
      if (!desc.external_ptr) {
        throw std::runtime_error("external buffer requires external_ptr");
      }

      return std::make_unique<CpuAllocation>(device_, desc.external_ptr,
                                             desc.size, desc.location,
                                             MemorySource::External);
    }

    void* ptr = ::operator new(desc.size);

    return std::make_unique<CpuAllocation>(
        device_, ptr, desc.size, desc.location, MemorySource::Allocate);
  }

  void copy_in(Allocation& dst, const void* src, std::size_t bytes) override {
    if (bytes > dst.size()) {
      throw std::runtime_error("copy_in exceeds destination size");
    }

    void* dst_ptr = dst.map();
    if (!dst_ptr) {
      throw std::runtime_error("destination allocation is not mappable");
    }

    std::memcpy(dst_ptr, src, bytes);
    dst.unmap();
  }

  void copy_out(const Allocation& src, void* dst, std::size_t bytes) override {
    if (bytes > src.size()) {
      throw std::runtime_error("copy_out exceeds source size");
    }

    // For CPU allocation, data() is always valid.
    const void* src_ptr = src.data();
    if (!src_ptr) {
      throw std::runtime_error("source allocation is not readable");
    }

    std::memcpy(dst, src_ptr, bytes);
  }

  void copy_between(Allocation& dst, const Allocation& src,
                    std::size_t bytes) override {
    if (bytes > dst.size() || bytes > src.size()) {
      throw std::runtime_error("copy_between exceeds buffer size");
    }

    void* dst_ptr = dst.map();
    const void* src_ptr = src.data();

    if (!dst_ptr || !src_ptr) {
      throw std::runtime_error("allocation is not mappable");
    }

    std::memcpy(dst_ptr, src_ptr, bytes);
    dst.unmap();
  }

 private:
  Device& device_;
};

class CPUDevice : public Device {
 public:
  CPUDevice() : allocator_(*this) {}

  DeviceType type() const override { return DeviceType::CPU; }

  const char* name() const override { return "CPU"; }

  void synchronize() override {
    // CPU backend is synchronous.
  }

  Allocator& allocator() override { return allocator_; }

 private:
  CPUAllocator allocator_;
};
