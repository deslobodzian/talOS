#pragma once

#include <memory>

enum class DeviceType {
    CPU,
    METAL,
};

enum class MemoryLocation {
    Device,
    Host,
    Unified,
};

enum class MemorySource {
    Allocate,
    External
};

class Device;
class Allocator;

// BufferConfig is what the buffer looks like, who has access, etc
struct BufferDesc{
    std::size_t size = 0;
    MemoryLocation location = MemoryLocation::Device;
    MemorySource source = MemorySource::Allocate;
    void* external_ptr = nullptr;
};

class Allocation {
public:
    virtual ~Allocation() = default;

    virtual Device& device() const = 0;
    virtual void* map() = 0;
    virtual void unmap() = 0;
    virtual void* data() const = 0;
    virtual std::size_t size() const = 0;
    virtual MemoryLocation location() const = 0;
    virtual MemorySource source() const = 0;
};

// Allocator is incharge of memory creation and deletion
class Allocator {
public:
    virtual ~Allocator() = default;

    virtual std::unique_ptr<Allocation> allocate(const BufferDesc& desc) = 0;

    virtual void copy_in(Allocation& dst, const void* src, std::size_t bytes) = 0;
    virtual void copy_out(const Allocation& src, void* dst, std::size_t bytes) = 0;
    virtual void copy_between(Allocation& dst, const Allocation& src, std::size_t bytes) = 0;

};

// Device will typically represent the GPU of the application (CPU if not valid)
class Device {
public:
    virtual ~Device() = default;

    virtual DeviceType type() const = 0;
    virtual const char* name() const = 0;
    virtual void synchronize() = 0;
    virtual Allocator& allocator() = 0;
};
