load("//talOS/build:contract.bzl", "talos_messages")

# BridgeStatus (namespace bridge) and its topic: hardware_node's health,
# published every 20 ms on /hw/status. Kept in its own subpackage because
# talos_messages(topics = ...) emits one topics.h per package and
# //talOS/hardware already owns sim_messages.
talos_messages(
    name = "messages",
    srcs = ["bridge_status.fbs"],
    include_prefix = "bridge",
    topics = {"status": "/hw/status"},
    visibility = ["//visibility:public"],
)
