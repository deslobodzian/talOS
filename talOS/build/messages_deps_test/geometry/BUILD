load("//talOS/build:contract.bzl", "talos_messages")

# Base messages: a shared struct with no topics. Consumer schemas include
# these through `deps` as include "deps_geometry/geometry.fbs".
talos_messages(
    name = "messages",
    srcs = ["geometry.fbs"],
    include_prefix = "deps_geometry",
)
