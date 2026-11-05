#include <gtest/gtest.h>

#include <string>

#include "shared_memory_ptr.h"

TEST(SharedMemory, SharedMemoryPtr) {
  // One path on both platforms. This was an #ifdef -- a flat name on Linux
  // and a slashed one elsewhere -- because glibc's shm_open rejects an
  // interior slash and macOS accepts one. ShmObjectName handles that at the
  // one boundary that opens the segment, so the difference no longer leaks
  // into every caller.
  auto path = "/rtms/test";
  shm_unlink(ShmObjectName(path).c_str());  // a crashed earlier run's
  {
    SharedMemoryPtr test{path, 100};
    EXPECT_NE(test.ptr(), nullptr);
    EXPECT_EQ(test.mode(), SharedMemoryMode::CREATE);
    // A second mapping of the same name attaches to it.
    SharedMemoryPtr again{path, 100};
    EXPECT_EQ(again.mode(), SharedMemoryMode::ATTACH);
  }
  // A mapping never unlinks its name: the segment outlives both mappings
  // until its owner (RTMS's lifetime rules) removes it.
  const int fd = shm_open(ShmObjectName(path).c_str(), O_RDWR, 0666);
  EXPECT_NE(fd, -1);
  if (fd != -1) close(fd);
  shm_unlink(ShmObjectName(path).c_str());
}

// Every talOS topic has two or three segments, so this mapping is on the path
// of every topic in the robot rather than being an edge case.
TEST(ShmObjectNameTest, ProducesAPortablePosixName) {
  for (const auto* topic : {"/hw/state", "/hw/state/driver_station",
                            "/drivetrain/target/teleop", "/talos/telemetry"}) {
    const std::string name = ShmObjectName(topic, "");
    ASSERT_FALSE(name.empty());
    EXPECT_EQ(name.front(), '/') << name;
    // POSIX allows exactly one slash, at the front.
    EXPECT_EQ(name.find('/', 1), std::string::npos)
        << topic << " maps to " << name << ", which glibc rejects";
    // Length is preserved, so RTMS's 30-character cap means the same thing
    // on both sides of the mapping.
    EXPECT_EQ(name.size(), std::string_view{topic}.size());
  }

  EXPECT_EQ(ShmObjectName("/hw/state/driver_station", ""),
            "/hw.state.driver_station");
  // A name that already conforms comes back unchanged, and the mapping is
  // idempotent -- RTMS applies it and then SharedMemoryPtr applies it again.
  EXPECT_EQ(ShmObjectName("/talos_registry.1", ""), "/talos_registry.1");
  EXPECT_EQ(ShmObjectName(ShmObjectName("/hw/state", ""), ""),
            ShmObjectName("/hw/state", ""));
}

// Two different topics must never land on one segment. That would join two
// unrelated streams while both ends reported success, which is a worse failure
// than either topic refusing to open.
TEST(ShmObjectNameTest, IsInjectiveOverTheTopicGrammar) {
  // The reason the separator is '.' and not '_': topic segments may contain
  // underscores (`driver_station`, `operator_interface`), so '_' would make
  // these two the same segment name. The grammar allows no '.' in a segment.
  EXPECT_NE(ShmObjectName("/hw/state/driver_station", ""),
            ShmObjectName("/hw/state_driver/station", ""));
  EXPECT_NE(ShmObjectName("/a/state/b", ""), ShmObjectName("/a/state_b", ""));
  EXPECT_NE(ShmObjectName("/operator_interface/state", ""),
            ShmObjectName("/operator/interface/state", ""));
}

TEST(ShmObjectNameTest, NamespacesAreStableIsolatedAndIdempotent) {
  const auto first = ShmObjectName("/hw/state", "run-a");
  const auto second = ShmObjectName("/hw/state", "run-b");
  EXPECT_NE(first, second);
  EXPECT_EQ(first, ShmObjectName("/hw/state", "run-a"));
  EXPECT_EQ(first, ShmObjectName("/hw.state", "run-a"));
  EXPECT_EQ(first, ShmObjectName(first, "run-a"));
  EXPECT_EQ(first, ShmObjectName(first, "run-b"));
  EXPECT_EQ(first, ShmObjectName(first, ""));
  EXPECT_EQ(first.size(), 18u);
  EXPECT_TRUE(first.starts_with("/~"));
  EXPECT_EQ(first.find_first_not_of("0123456789abcdef", 2), std::string::npos);
  EXPECT_NE(first, ShmObjectName("/hw/command", "run-a"));
  EXPECT_NE(ShmObjectName("/c", "ab"), ShmObjectName("/bc", "a"));
}
