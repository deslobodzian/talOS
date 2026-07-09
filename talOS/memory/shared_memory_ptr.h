#pragma once

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <format>
#include <sys/stat.h>
#include <cstring>


enum class SharedMemoryMode {
    CREATE,
    ATTACH
};


class SharedMemoryPtr {
public:
    static SharedMemoryPtr create(std::string_view name, std::size_t size) {
        std::printf("Creating shared memory ptr SharedMemoryMode::CREATE\n");
        return{name, size, SharedMemoryMode::CREATE};
    }

    static SharedMemoryPtr attach(std::string_view name, std::size_t size) {
        std::printf("Creating shared memory ptr SharedMemoryMode::ATTACH\n");
        return{name, size, SharedMemoryMode::ATTACH};
    }

    ~SharedMemoryPtr() {
        reset();
        // If our create ptr dies remove shared memory
        if (mode_ == SharedMemoryMode::CREATE) {
            shm_unlink(shm_name_.c_str());
        }
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
    SharedMemoryPtr(std::string_view name, std::size_t size, SharedMemoryMode mode) :
        shm_name_(name),
        size_(size),
        mode_(mode) {
        if (map_ptr() < 0) {
            throw std::runtime_error("Failed to attach ptr!");
        }
    }

    static off_t to_off_t(std::size_t size) {
        constexpr auto max_off_t =
            static_cast<std::size_t>(std::numeric_limits<off_t>::max());
        if (size > max_off_t) {
           throw std::overflow_error("shared-memory size exceeds off_t range");
        }
        return static_cast<off_t>(size);

    }


    int map_ptr() {
        int shm_fd = -1;
        int oflag = -1;
        const off_t file_size = to_off_t(size_);

        switch (mode_) {
            case SharedMemoryMode::CREATE:
                shm_unlink(shm_name_.c_str());
                oflag = O_CREAT | O_RDWR | O_EXCL;
                shm_fd = shm_open(shm_name_.c_str(), oflag, 0666);
                if (shm_fd == -1) {
                    std::perror(std::format("shm_open failed on create for {}", shm_name_).c_str());
                    return -1;
                }

                if (ftruncate(shm_fd, file_size)) {
                    const int err = errno;
                    std::fprintf(stderr,
                        "ftruncate: fd=%d size=%zu errno=%d (%s)\n",
                         shm_fd, size_, err, std::strerror(err));
                    close(shm_fd);
                    return -1;
                }

                break;
            case SharedMemoryMode::ATTACH:
                oflag = O_RDWR;
                shm_fd = shm_open(shm_name_.c_str(), oflag, 0666);

                if (shm_fd == -1) {
                    std::perror(std::format("shm_open failed on attach for {}", shm_name_).c_str());
                    return -1;
                }

                struct stat st;
                if (fstat(shm_fd, &st) < 0) {
                    std::perror("fstat failed");
                    close(shm_fd);
                    return -1;
                }
                break;
            default:
                return -1;
                break;
        }


        void* shm_addr = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (shm_addr == MAP_FAILED) {
            std::perror("mmap failed");
            close(shm_fd);
            return -1;
        }

        // The Linux Programming Interface — Michael Kerrisk
        // Says we can close mmap after mapping.
        if (close(shm_fd) == -1) {
            std::perror("failed to close shm_fd");
            return -1;
        }

        std::printf("Addr given: %p\n", shm_addr);
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
    std::string shm_name_{"error"};
    std::size_t size_{0};
    SharedMemoryMode mode_;
    void* ptr_ = nullptr;
};
