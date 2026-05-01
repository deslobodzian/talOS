load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "metal-cpp",
    hdrs = glob([
        "**/*.hpp"
    ]),
    includes = ["."],
    target_compatible_with = ["@platforms//os:osx"],
    linkopts = [
        "-framework Foundation",
        "-framework Metal",
        "-framework MetalFX",
        "-framework QuartzCore",
    ],
    visibility = ["//visibility:public"],
)
