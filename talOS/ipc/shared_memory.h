#pragma once

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>

int open_shmmap(const char* shm_name, off_t size) {
    shm_unlink(shm_name);

    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::perror("shm_open failed");
        return 1;
    }

    if (ftruncate(shm_fd, size)) {
        std::perror("ftruncate failed");
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

    return 0;
}
