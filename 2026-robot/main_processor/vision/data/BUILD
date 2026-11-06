load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

genrule(
    name = "embed_field_layout",
    srcs = ["2026-rebuilt-welded.json"],
    outs = ["field_layout_2026.h"],
    cmd = """{ printf '#pragma once\n#include <string_view>\nnamespace vision { inline constexpr std::string_view kFieldLayoutJson = R\"talos_json('; cat $(location 2026-rebuilt-welded.json); printf ')talos_json\"; }\n'; } > $@""",
)

cc_library(
    name = "field_layout_2026",
    hdrs = [":field_layout_2026.h"],
    include_prefix = "vision/data",
    strip_include_prefix = ".",
)
