#pragma once

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <string>

class SharedMemoryPtr {
public:
    SharedMemoryPtr(const char* name, off_t size) : shm_name_(name), size_(size) {
        attach_ptr(name, size);
    }
    ~SharedMemoryPtr() {
        std::printf("Deleting ptr_: %p\n", ptr_);
        if (munmap(ptr_, size_) == -1) {
            std::perror("munmap failed");

        }
        ptr_ = nullptr;

        std::printf("Deleted ptr_: %p\n", ptr_);
    }

private:
    int attach_ptr(const char* shm_name, off_t size) {
        shm_unlink(shm_name);
        int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
        if (shm_fd == -1) {
            std::perror("shm_open failed");
            return 1;
        }

        if (ftruncate(shm_fd, size)) {
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
    std::string shm_name_;
    off_t size_ = 0;
    void* ptr_ = nullptr;
};
