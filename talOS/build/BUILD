load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_test")

exports_files([
    "contract.bzl",
    "bundle_builder.py",
])

py_test(
    name = "bundle_builder_test",
    size = "small",
    srcs = ["bundle_builder_test.py"],
    data = ["bundle_builder.py"],
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)

# talos_messages' schema step: the field-name lint and the C++/Python
# descriptors, from each schema's .bfbs.
cc_binary(
    name = "descriptor_gen",
    srcs = ["descriptor_gen.cc"],
    visibility = ["//visibility:public"],
    deps = [
        "//talOS/schema:schema_hash",
        "//talOS/schema:unit_suffix",
        "@flatbuffers//:runtime_cc",
    ],
)

# Lint fixtures, compiled as talos_messages compiles schemas: a real copy of
# an include root, with declaration files recorded relative to it.
genrule(
    name = "lint_fixtures",
    srcs = [
        "testdata/lint_test/bad.fbs",
        "testdata/lint_test/good.fbs",
        "testdata/lint_test/included.fbs",
    ],
    outs = [
        "lint_fixtures/bad.bfbs",
        "lint_fixtures/good.bfbs",
        "lint_fixtures/included.bfbs",
    ],
    cmd = "tmp=$$(mktemp -d) && trap 'rm -rf $$tmp' EXIT && " +
          "mkdir $$tmp/lint_test && cp $(SRCS) $$tmp/lint_test && " +
          "$(execpath @flatbuffers//:flatc) --binary --schema " +
          "--bfbs-filenames $$tmp -I $$tmp -o $(RULEDIR)/lint_fixtures " +
          "$$tmp/lint_test/bad.fbs $$tmp/lint_test/good.fbs $$tmp/lint_test/included.fbs",
    tools = ["@flatbuffers//:flatc"],
    visibility = ["//visibility:private"],
)

py_test(
    name = "descriptor_gen_lint_test",
    size = "small",
    srcs = ["descriptor_gen_lint_test.py"],
    data = [
        "lint_fixtures/bad.bfbs",
        "lint_fixtures/good.bfbs",
        "lint_fixtures/included.bfbs",
        ":descriptor_gen",
    ],
    env = {
        "TALOS_DESCRIPTOR_GEN": "$(rlocationpath :descriptor_gen)",
        "TALOS_LINT_BAD": "$(rlocationpath lint_fixtures/bad.bfbs)",
        "TALOS_LINT_GOOD": "$(rlocationpath lint_fixtures/good.bfbs)",
        "TALOS_LINT_INCLUDED": "$(rlocationpath lint_fixtures/included.bfbs)",
    },
    legacy_create_init = False,
    deps = ["@rules_python//python/runfiles"],
)
