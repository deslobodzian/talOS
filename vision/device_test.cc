#include <gtest/gtest.h>
#include "vision/buffer.h"
#include "vision/cpu_device.h"

TEST(DeviceTest, Allocate) {
    CPUDevice dev{};
    Buffer<int> buffer(dev, 1);
    std::cout << buffer.raw().to_string();
}
