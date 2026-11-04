# WPILib C++ (wpimath + wpiutil) from WPILib's own Maven release artifacts:
# shared libraries per platform plus the headers zip (which bundles Eigen,
# gcem, Sleipnir and units/). Repositories are fetched lazily, so a macOS
# build downloads only the headers and osxuniversal zips.
#
# _VERSION must match GradleRIO in 2026-robot/controller_processor/rio/
# build.gradle and robotpy-native-wpimath in //talOS/python:requirements.txt
# (//2026-robot/controller_processor/rio:wpilib_lockstep_test): in one
# process dyld/ld.so share one libwpimath by
# install name, so a skew is an ABI hazard, not just a style issue.

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

_VERSION = "2026.2.1"
_MAVEN = "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first"

# sha256 from the `.sha256` files published next to each artifact.
_SHA256 = {
    "wpimath-headers": "25b5e461efb35ec097a6023bfe07fc9baa87bfbb2983a23880d2ea7e0ed75ac9",
    "wpimath-osxuniversal": "cf6bbe8a72810648b405b12e829b0a77221d18fdd15b180154aafcdd10e4082f",
    "wpimath-linuxx86-64": "57b17ca2472577df1d85d344504509832d7e156dc2d75849dfd452cb6c655e13",
    "wpimath-linuxarm64": "b0686a5c5630e420f2c664939c895dd602eb7b57e33f817ab0f2b0dfd8857c17",
    "wpiutil-headers": "edadccd3703e643ddbf06450cac82ce7ed9618527fb05c1bc972ddc4a89bff2b",
    "wpiutil-osxuniversal": "ac8992940ae5e1bc7ccd2816d27874cf6810f282ce5138de818e535a20373203",
    "wpiutil-linuxx86-64": "49e325a290306a6a53a047312c38ff336056daaa1300eeaab6d2678a922efd18",
    "wpiutil-linuxarm64": "6fd1e8199c6aa1065f2308ee8423bbc5d8b0deb2995f80a1e5e5e012781cd074",
}

# Maven classifier -> directory of the shared library inside the zip.
_PLATFORMS = {
    "osxuniversal": ("osx/universal/shared", "dylib"),
    "linuxx86-64": ("linux/x86-64/shared", "so"),
    "linuxarm64": ("linux/arm64/shared", "so"),
}

_HEADERS_BUILD = """\
load("@rules_cc//cc:defs.bzl", "cc_library")
cc_library(
    name = "headers",
    hdrs = glob(["**"], exclude = ["**/*.txt", "**/*.md", ".clang-format", "WORKSPACE", "BUILD.bazel", "MODULE.bazel"]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
"""

_LIB_BUILD = """\
load("@rules_cc//cc:defs.bzl", "cc_import")
cc_import(
    name = "lib",
    shared_library = "{dir}/lib{lib}.{ext}",
    visibility = ["//visibility:public"],
)
"""

def _artifact(lib, classifier):
    return "{m}/{lib}/{lib}-cpp/{v}/{lib}-cpp-{v}-{c}.zip".format(m = _MAVEN, lib = lib, v = _VERSION, c = classifier)

def _wpilib_impl(module_ctx):
    for lib in ("wpiutil", "wpimath"):
        http_archive(
            name = "wpilib_{}_headers".format(lib),
            url = _artifact(lib, "headers"),
            sha256 = _SHA256[lib + "-headers"],
            build_file_content = _HEADERS_BUILD,
        )
        for classifier, (dir, ext) in _PLATFORMS.items():
            http_archive(
                name = "wpilib_{}_{}".format(lib, classifier.replace("-", "_")),
                url = _artifact(lib, classifier),
                sha256 = _SHA256[lib + "-" + classifier],
                build_file_content = _LIB_BUILD.format(dir = dir, lib = lib, ext = ext),
            )
    return module_ctx.extension_metadata(reproducible = True)

wpilib = module_extension(implementation = _wpilib_impl)
