#pragma once

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>

class SharedMemoryPtr {
public:
    SharedMemoryPtr(const char* name, std::size_t size) : shm_name_(name), size_(size) {
        attach_ptr(name, size);
    }

    ~SharedMemoryPtr() {
        reset();
    }

    SharedMemoryPtr(const SharedMemoryPtr&) = delete;
    SharedMemoryPtr& operator=(const SharedMemoryPtr&) = delete;


    SharedMemoryPtr(SharedMemoryPtr&& other) noexcept
        : shm_name_(std::move(other.shm_name_)),
        size_(std::exchange(other.size_, 0)),
        ptr_(std::exchange(other.ptr_, nullptr)) {
    }

    SharedMemoryPtr& operator=(SharedMemoryPtr&& other) noexcept {
        if (this != &other) {
            reset();
            shm_name_ = std::move(other.shm_name_);
            size_ = std::exchange(other.size_, 0);
            ptr_ = std::exchange(other.ptr_, nullptr);
        }
        return *this;
    }


    void* ptr() {
        return ptr_;
    }

private:

    static off_t to_off_t(std::size_t size) {
        constexpr auto max_off_t =
            static_cast<std::size_t>(std::numeric_limits<off_t>::max());
        if (size > max_off_t) {
           throw std::overflow_error("shared-memory size exceeds off_t range");
        }
        return static_cast<off_t>(size);

    }
    int attach_ptr(const char* shm_name, std::size_t size) {
        shm_unlink(shm_name);
        int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            std::perror("shm_open failed");
            return 1;
        }
        const off_t file_size = to_off_t(size_);

        if (ftruncate(shm_fd, file_size)) {
            std::perror("ftruncate failed");
            return 1;
        }

        void* shm_addr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (shm_addr == MAP_FAILED) {
            std::perror("mmap failed");
            return 1;
        }

        std::printf("Addr given: %p\n", shm_addr);

        // The Linux Programming Interface — Michael Kerrisk
        // Says we can close mmap after mapping.
        if (close(shm_fd) == -1) {
            std::perror("failed to close shm_fd");
            return -1;
        }
        ptr_ = shm_addr;

        std::printf("Addr given to ptr_: %p\n", ptr_);
        return 0;
    }

    void reset() noexcept {
        std::printf("Resetting ptr_: %p\n", ptr_);
        if (ptr_ != nullptr) {
            if (munmap(ptr_, size_) == -1) {
                std::perror("munmap failed");
            }
            ptr_ = nullptr;
        }
    }
    std::string shm_name_;
    std::size_t size_{0};
    void* ptr_ = nullptr;
};
