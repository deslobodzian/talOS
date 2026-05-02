#include <gtest/gtest.h>

#include "gmock/gmock.h"
#include "vision/buffer.h"
#include "vision/cpu_device.h"

TEST(DeviceTest, Allocate) {
  CPUDevice dev{};
  Buffer<int> buffer(dev, 1);
  std::cout << buffer.raw().to_string();
}

TEST(DeviceTest, DownloadSingle) {
  CPUDevice dev{};

  int test = 23;
  Buffer<int> buffer(dev, sizeof(test));
  buffer.upload(&test, sizeof(test));
  ASSERT_EQ(test, *buffer.data());
}

TEST(DeviceTest, DownloadArray) {
  CPUDevice dev{};

  std::vector<int> testArray = {1, 3, 10, 22, 0};

  Buffer<int> buffer(dev, testArray.size());
  buffer.upload(testArray.data(), testArray.size());

  std::vector<int> downloaded(testArray.size());
  buffer.download(downloaded.data(), buffer.count());

  EXPECT_THAT(downloaded, ::testing::ElementsAreArray(testArray));
}

TEST(DeviceTest, UploadArray) {
  CPUDevice dev{};

  std::vector<int> testArray = {1, 3, 10, 22, 0};
  Buffer<int> buffer(dev, testArray.size());
  buffer.upload(testArray.data(), testArray.size());

  EXPECT_THAT(std::vector<int>(buffer.data(), buffer.data() + buffer.count()),
              ::testing::ElementsAreArray(testArray));
}
