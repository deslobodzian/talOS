load("//talOS/build:contract.bzl", "talos_messages")

# TeleopStatus (namespace teleop) and its topic /teleop/status: operator
# arming state and blocking input, published every tick by //talOS/teleop:node.
talos_messages(
    name = "messages",
    srcs = ["status.fbs"],
    include_prefix = "teleop",
    topics = {"status": "/teleop/status"},
    visibility = ["//visibility:public"],
)
