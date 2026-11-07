load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "metal-cpp-compute",
    hdrs = glob([
        "Foundation/**/*.hpp",
        "Metal/**/*.hpp",
    ]),
    includes = ["."],
    linkopts = [
        "-framework Foundation",
        "-framework Metal",
    ],
    target_compatible_with = ["@platforms//os:macos"],
    visibility = ["//visibility:public"],
)
