#include "talOS/node_api/node_api.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

constexpr const char* kTopic = "/test/state/node_api";
constexpr uint32_t kBytes = 8;
constexpr uint32_t kAlign = 4;

TEST(NodeApi, AbiVersion) { EXPECT_EQ(talos_abi_version(), 3u); }

TEST(NodeApi, RtmsLayoutMatchesFrozenGeometry) {
  EXPECT_EQ(talos_rtms_layout(nullptr), 0u);
  TalosRtmsLayout layout{};
  EXPECT_EQ(talos_rtms_layout(&layout), sizeof(TalosRtmsLayout));
  // Frozen contract: sizeof(RTMSHeader)==640, writer@64, readers@128/64.
  EXPECT_EQ(layout.header_size, 640u);
  EXPECT_EQ(layout.off_writer_seq, 64u);
  EXPECT_EQ(layout.off_readers, 128u);
  EXPECT_EQ(layout.reader_stride, 64u);
}

TEST(NodeApi, MonotonicIncreases) {
  const int64_t a = talos_monotonic_ns();
  const int64_t b = talos_monotonic_ns();
  EXPECT_LE(a, b);
  EXPECT_GT(a, 0);
}

TEST(NodeApi, BadArgs) {
  EXPECT_EQ(talos_publish(nullptr, nullptr, 0), TALOS_ERR_ARG);
  EXPECT_EQ(talos_poll_next(nullptr, nullptr, 0, nullptr, nullptr),
            TALOS_ERR_ARG);
  EXPECT_TRUE(talos_topic_open_publisher(nullptr, kBytes, kAlign) == nullptr);
  EXPECT_TRUE(talos_topic_open_publisher("", kBytes, kAlign) == nullptr);
  EXPECT_TRUE(talos_topic_open_publisher(kTopic, 0, kAlign) == nullptr);
  talos_publisher_close(nullptr);
  talos_subscriber_close(nullptr);
}

TEST(NodeApi, PublishPollRoundTrip) {
  TalosPublisher* pub = talos_topic_open_publisher(kTopic, kBytes, kAlign);
  ASSERT_NE(pub, nullptr) << talos_last_error();
  TalosSubscriber* sub = talos_topic_open_subscriber(kTopic, kBytes, kAlign);
  ASSERT_NE(sub, nullptr) << talos_last_error();

  const uint64_t payload = 0x0102030405060708ULL;
  EXPECT_EQ(talos_publish(pub, &payload, kBytes), TALOS_OK);
  // Wrong size rejected.
  EXPECT_EQ(talos_publish(pub, &payload, kBytes - 1), TALOS_ERR_ARG);

  uint64_t got = 0;
  uint64_t seq = 0, dropped = 0;
  EXPECT_EQ(talos_poll_next(sub, &got, sizeof(got), &seq, &dropped), TALOS_OK);
  EXPECT_EQ(got, payload);
  EXPECT_EQ(talos_poll_next(sub, &got, sizeof(got), nullptr, nullptr),
            TALOS_EMPTY);
  // Small buffer rejected.
  EXPECT_EQ(talos_poll_next(sub, &got, 1, nullptr, nullptr), TALOS_ERR_ARG);

  talos_publisher_close(pub);
  talos_subscriber_close(sub);
}

TEST(NodeApi, DescribeEmitMatchesGroundTruth) {
  const TalosSource sources[] = {
      {TALOS_SOURCE_SENDER, "/test/state/x", 8, 0},
      {TALOS_SOURCE_WATCHER, "/test/state/y", 4, 1},
  };
  uint32_t need = 0;
  // Size probe first.
  EXPECT_EQ(talos_describe_emit("mynode", "//pkg:mynode", sources, 2, nullptr,
                               0, &need),
            TALOS_ERR_SMALL);
  ASSERT_GT(need, 0u);
  std::vector<char> buf(need + 1);
  uint32_t written = 0;
  EXPECT_EQ(talos_describe_emit("mynode", "//pkg:mynode", sources, 2, buf.data(),
                               static_cast<uint32_t>(buf.size()), &written),
            TALOS_OK);
  EXPECT_EQ(written, need);
  const std::string json(buf.data());
  EXPECT_NE(json.find("\"name\": \"mynode\""), std::string::npos);
  EXPECT_NE(json.find("\"describe_version\": 1"), std::string::npos);
  EXPECT_NE(json.find("/test/state/x"), std::string::npos);
  EXPECT_NE(json.find("\"external\": true"), std::string::npos);

  // Too-small buffer reports need.
  std::vector<char> tiny(4);
  uint32_t need2 = 0;
  EXPECT_EQ(talos_describe_emit("mynode", "//pkg:mynode", sources, 2, tiny.data(),
                               static_cast<uint32_t>(tiny.size()), &need2),
            TALOS_ERR_SMALL);
  EXPECT_EQ(need2, need);

  // Bad kind rejected; Python never formats the envelope itself.
  const TalosSource bad[] = {{99, "/test/state/z", 8, 0}};
  EXPECT_EQ(talos_describe_emit("mynode", "//pkg:mynode", bad, 1, buf.data(),
                               static_cast<uint32_t>(buf.size()), nullptr),
            TALOS_ERR_ARG);
}

TEST(NodeApi, RegisterPublishHeartbeatClose) {
  EXPECT_TRUE(talos_node_register(nullptr, "//pkg:mynode", 0, 0) == nullptr);
  EXPECT_TRUE(talos_node_register("", "//pkg:mynode", 0, 0) == nullptr);
  EXPECT_EQ(talos_node_publish(nullptr, nullptr, 0), TALOS_ERR_ARG);
  EXPECT_EQ(talos_node_heartbeat(nullptr, 0), TALOS_ERR_ARG);
  talos_node_close(nullptr);

  TalosNode* node =
      talos_node_register("node_api_test", "//talOS/node_api:node_api_test",
                          4242u, 1u);
  ASSERT_NE(node, nullptr) << talos_last_error();
  const TalosSource sources[] = {
      {TALOS_SOURCE_TIMER, "tick", 0, 0},
      {TALOS_SOURCE_SENDER, "/test/state/node_api_reg", 8, 0},
  };
  EXPECT_EQ(talos_node_publish(node, sources, 2), TALOS_OK);
  EXPECT_EQ(talos_node_publish(node, nullptr, 1), TALOS_ERR_ARG);
  EXPECT_EQ(talos_node_heartbeat(node, 1), TALOS_OK);
  EXPECT_EQ(talos_node_heartbeat(node, 2), TALOS_OK);
  talos_node_close(node);
}

}  // namespace
