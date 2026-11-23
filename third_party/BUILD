# Marks third_party as a package so http_archive build_file labels like
# //third_party:eigen.BUILD.bazel resolve. Third-party shims only; no
# first-party code here.
package(default_visibility = ["//visibility:public"])

exports_files([
    "apriltag_pose_ambiguity.patch",
    "tomlplusplus_parser_ub.patch",
    "uws_security_and_ub.patch",
])

alias(
    name = "apriltag",
    actual = "@apriltag//:apriltag",
)

# talOS's patched toml++ (MODULE.bazel), for a repository that depends on
# talOS and reads TOML itself (`#include "toml.hpp"`): the repository's own
# name for it, @tomlplusplus, is visible only inside talOS.
alias(
    name = "tomlplusplus",
    actual = "@tomlplusplus//:toml++",
)
