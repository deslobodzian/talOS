load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_python//python:defs.bzl", "py_library", "py_test")

cc_library(
    name = "shared_memory",
    hdrs = ["shared_memory_ptr.h"],
    visibility = [
        "//studio:__subpackages__",
        "//talOS:__subpackages__",
        "//tools:__pkg__",
    ],
    deps = [":residency"],
)

# Prefaulting and locking memory a realtime loop touches; SharedMemoryPtr
# registers every mapping here, and talOS/process asks for them resident.
cc_library(
    name = "residency",
    srcs = ["residency.cc"],
    hdrs = ["residency.h"],
    visibility = ["//talOS:__subpackages__"],
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

# ShmObjectName in a namespace, for Python tools and tests that open or unlink
# a session's segments.
py_library(
    name = "shm_name",
    srcs = ["shm_name.py"],
    visibility = ["//visibility:public"],
)

py_test(
    name = "shm_name_test",
    size = "small",
    srcs = ["shm_name_test.py"],
    data = [":shm_name_probe"],
    deps = [
        ":shm_name",
        "@rules_python//python/runfiles",
    ],
)

cc_binary(
    name = "shm_name_probe",
    testonly = True,
    srcs = ["testdata/shm_name_probe.cc"],
    deps = [":shared_memory"],
)
