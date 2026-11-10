load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "shared_memory",
    hdrs = ["shared_memory_ptr.h"],
    visibility = ["//visibility:public"],
    deps = [":residency"],
)

# Prefaulting and locking memory a realtime loop touches; SharedMemoryPtr
# registers every mapping here, and talOS/process asks for them resident.
cc_library(
    name = "residency",
    srcs = ["residency.cc"],
    hdrs = ["residency.h"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "shared_memory_test",
    size = "small",
    srcs = ["shared_memory_test.cc"],
    deps = [
        ":residency",
        ":shared_memory",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
