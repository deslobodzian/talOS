#pragma once

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
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
    SharedMemoryPtr(std::string_view name, std::size_t size) :
        shm_name_(name),
        size_(size) {
        if (map_ptr() < 0) {
            throw std::runtime_error("Failed to attach ptr!");
        }
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

    // The moved-from object is left as an ATTACH so that its destructor does
    // not unlink the shared memory the moved-to object now owns.
    SharedMemoryPtr(SharedMemoryPtr&& other) noexcept
        : shm_name_(std::move(other.shm_name_)),
        size_(std::exchange(other.size_, 0)),
        mode_(std::exchange(other.mode_, SharedMemoryMode::ATTACH)),
        ptr_(std::exchange(other.ptr_, nullptr)) {
    }

    SharedMemoryPtr& operator=(SharedMemoryPtr&& other) noexcept {
        if (this != &other) {
            reset();
            if (mode_ == SharedMemoryMode::CREATE) {
                shm_unlink(shm_name_.c_str());
            }
            shm_name_ = std::move(other.shm_name_);
            size_ = std::exchange(other.size_, 0);
            mode_ = std::exchange(other.mode_, SharedMemoryMode::ATTACH);
            ptr_ = std::exchange(other.ptr_, nullptr);
        }
        return *this;
    }


    void* ptr() {
        return ptr_;
    }

    const void* ptr() const {
        return ptr_;
    }

    SharedMemoryMode mode() const {
        return mode_;
    }

private:
 // Bounded so a real mismatch is still reported promptly: 200 x 250us.
 static constexpr int kAttachAttempts = 200;
 static constexpr useconds_t kAttachRetryUs = 250;

 static off_t to_off_t(std::size_t size) {
   constexpr auto max_off_t =
       static_cast<std::size_t>(std::numeric_limits<off_t>::max());
   if (size > max_off_t) {
     throw std::overflow_error("shared-memory size exceeds off_t range");
   }
   return static_cast<off_t>(size);
 }

    void create_or_attach_shm_map(
        int& shm_fd,
        SharedMemoryMode& shm_mode,
        const char* name,
        int oflag,
        mode_t mode) {
        shm_fd = shm_open(name, oflag, mode);
        if (shm_fd == -1) {
            if (errno == EEXIST) {
                shm_mode = SharedMemoryMode::ATTACH;
                create_or_attach_shm_map(shm_fd, shm_mode, name, O_RDWR, mode);
            }
        }
    }


    int map_ptr() {
        int shm_fd = -1;
        int oflag = -1;
        const off_t file_size = to_off_t(size_);

        oflag = O_CREAT | O_RDWR | O_EXCL;
        create_or_attach_shm_map(shm_fd, mode_, shm_name_.c_str(), oflag, 0666);


        switch (mode_) {
            case SharedMemoryMode::CREATE:
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
            case SharedMemoryMode::ATTACH: {
              // create_or_attach_shm_map already opened the object on the
              // EEXIST path; only open again if it did not.
              if (shm_fd == -1) {
                oflag = O_RDWR;
                shm_fd = shm_open(shm_name_.c_str(), oflag, 0666);
              }

              if (shm_fd == -1) {
                std::perror(
                    std::format("shm_open failed on attach for {}", shm_name_)
                        .c_str());
                return -1;
              }

              // Mapping more than the creator allocated faults on access,
              // so refuse instead of handing back a landmine.
              //
              // shm_open publishes the name before the creator can call
              // ftruncate, so a peer starting at the same moment can arrive
              // between the two and measure a segment that is still empty.
              // That window is microseconds wide; a genuinely wrong segment
              // stays wrong, so waiting briefly separates the two without
              // masking either.
              struct stat st;
              bool sized = false;
              for (int attempt = 0; attempt < kAttachAttempts; ++attempt) {
                if (fstat(shm_fd, &st) < 0) {
                  std::perror("fstat failed");
                  close(shm_fd);
                  return -1;
                }
                if (static_cast<std::size_t>(st.st_size) >= size_) {
                  sized = true;
                  break;
                }
                usleep(kAttachRetryUs);
              }

              if (!sized) {
                std::fprintf(stderr,
                             "shared memory %s is %lld bytes, need %zu; it was "
                             "created by a process built against a different "
                             "message layout\n",
                             shm_name_.c_str(),
                             static_cast<long long>(st.st_size), size_);
                close(shm_fd);
                return -1;
              }
              break;
            }
            default:
              return -1;
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

        ptr_ = shm_addr;
        return 0;
    }

    void reset() noexcept {
        if (ptr_ != nullptr) {
            if (munmap(ptr_, size_) == -1) {
                std::perror("munmap failed");
            }
            ptr_ = nullptr;
        }
    }
    std::string shm_name_{"error"};
    std::size_t size_{0};
    SharedMemoryMode mode_{SharedMemoryMode::CREATE};
    void* ptr_ = nullptr;
};
