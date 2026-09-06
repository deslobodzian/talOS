load("@flatbuffers//:build_defs.bzl", "flatbuffer_cc_library")

# The subsystem's own config file, so the robot manifest filegroup can ship it.
# Added by a later track alongside subsystem.toml; kept first so appends merge cleanly.
exports_files(["subsystem.toml"])

flatbuffer_cc_library(
    name = "intake_message",
    srcs = ["intake_message.fbs"],
    visibility = ["//visibility:public"],
)
