"""Unit tests for tools/check_host_includes.py on fixture `.d` files.

TODO.md CI-6: a compile that read <toml++/toml.hpp> from /usr/include passed
on the workstation and failed in CI, where no such package is installed.
"""

import io
import os
import tempfile
import unittest
from contextlib import redirect_stdout

from tools import check_host_includes

GCC = "/usr/lib/gcc/x86_64-pc-linux-gnu/16"

# What GCC 16 on Arch and GCC 13 on Ubuntu write for a compile that reads
# only the toolchain's own headers and Bazel's.
TOOLCHAIN_ONLY = f"""\
bazel-out/k8-fastbuild/bin/talOS/telemetry/_objs/wire/wire.pic.o: \\
 talOS/telemetry/wire.cc /usr/include/stdc-predef.h talOS/telemetry/wire.h \\
 {GCC}/../../../../include/c++/16/cstddef \\
 {GCC}/../../../../include/c++/16/x86_64-pc-linux-gnu/bits/c++config.h \\
 /usr/include/features.h /usr/include/bits/wordsize.h /usr/include/sys/cdefs.h \\
 {GCC}/include/stddef.h /usr/include/linux/errno.h /usr/include/asm/errno.h \\
 /usr/include/asm-generic/errno-base.h /usr/include/netinet/in.h \\
 /usr/include/x86_64-linux-gnu/bits/types.h \\
 /usr/include/x86_64-linux-gnu/c++/13/bits/os_defines.h \\
 /usr/include/c++/13/vector \\
 external/tomlplusplus+/toml.hpp \\
 bazel-out/k8-fastbuild/bin/talOS/telemetry/telemetry_generated.h
"""

HOST_PACKAGES = """\
bazel-out/k8-fastbuild/bin/talOS/telemetry/_objs/dashboard_layout/dashboard_layout.pic.o: \\
 talOS/telemetry/dashboard_layout.cc /usr/include/stdc-predef.h \\
 /usr/include/toml++/toml.hpp /usr/include/toml++/impl/array.hpp
bazel-out/k8-fastbuild/bin/talOS/geometry/_objs/se2/se2.pic.o: \\
 talOS/geometry/se2.cc /usr/include/stdio.h /usr/include/eigen3/Eigen/Core \\
 /usr/local/include/foo.h
"""


def run(files, *extra):
    with tempfile.TemporaryDirectory() as tmp:
        for name, text in files.items():
            path = os.path.join(tmp, name)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
        out = io.StringIO()
        with redirect_stdout(out):
            status = check_host_includes.main([*extra, tmp])
    return status, out.getvalue()


class CheckHostIncludesTest(unittest.TestCase):
    def test_toolchain_libc_kernel_and_bazel_headers_pass(self):
        status, out = run({"a/wire.pic.d": TOOLCHAIN_ONLY})
        self.assertEqual(status, 0, out)

    # REQ-DEV-25
    def test_host_package_headers_fail_naming_source_and_header(self):
        status, out = run({"a/x.pic.d": TOOLCHAIN_ONLY, "b/y.d": HOST_PACKAGES})
        self.assertEqual(status, 1)
        for line in [
            "talOS/telemetry/dashboard_layout.cc: /usr/include/toml++/toml.hpp",
            "talOS/geometry/se2.cc: /usr/include/eigen3/Eigen/Core",
            "talOS/geometry/se2.cc: /usr/local/include/foo.h",
        ]:
            self.assertIn(line, out)
        self.assertNotIn("stdio.h", out)
        self.assertNotIn("wire.cc", out)

    def test_no_depfile_is_an_error_not_a_pass(self):
        # A wrong output path must not read as "no host headers".
        status, _ = run({"a/readme.txt": "x"})
        self.assertEqual(status, 2)


if __name__ == "__main__":
    unittest.main()
