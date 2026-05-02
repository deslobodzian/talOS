#include "ring_buffer.h"

#include <gtest/gtest.h>

TEST(Memory, RingBuffer) {
  RingBuffer<int> ring_buf(10);
  for (int i = 0; i < 10; i++) {
    std::cout << (ring_buf.get().has_value() ? ring_buf.get().value() : -1);
  }
}
