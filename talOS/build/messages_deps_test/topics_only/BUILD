load("//talOS/build:contract.bzl", "talos_messages")

# A topics-only package: no schema of its own, its state type comes from
# `deps` (as a pose estimator publishes geometry.PoseEstimate). The targets
# still carry the topic names and make the dep's types reachable.
talos_messages(
    name = "messages",
    srcs = [],
    include_prefix = "deps_topics_only",
    topics = {
        "state": "/deps_topics_only/state",
    },
    deps = ["//talOS/build/messages_deps_test/geometry:messages"],
)

