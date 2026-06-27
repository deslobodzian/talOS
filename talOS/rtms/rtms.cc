#include "rtms.h"


RTMSQueue::RTMSQueue(std::string_view path, off_t message_size) :
    path_(path),
    message_size_(message_size),
    ptr_(
        path_.c_str(),
        static_cast<off_t>(sizeof(RTMSHeader)) + message_size) {
    header_ = static_cast<RTMSHeader*>(ptr_.ptr());
    header_->capacity = MAX_BUFFER_SIZE;
    header_->message_size = message_size_;
}
