"""Unit tests for tools/check_host_includes.py on fixture `.d` files.

REQ-DEV-25: a compile that read <toml++/toml.hpp> from /usr/include passed
on the workstation and failed in CI, where no such package is installed.
"""

import io
import os
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

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

    # REQ-DEV-25
    def test_double_leading_slash_host_header_is_caught(self):
        # POSIX normpath preserves leading `//`; it must still match HOST_PREFIXES.
        dep = (
            "bazel-out/k8-fastbuild/bin/a.o: \\\n"
            " talOS/telemetry/a.cc //usr/include/stdio.h \\\n"
            " //usr/include/toml++/toml.hpp //usr/local/include/foo.h\n"
        )
        status, out = run({"a/a.d": dep})
        self.assertEqual(status, 1, out)
        self.assertIn("talOS/telemetry/a.cc: /usr/include/toml++/toml.hpp", out)
        self.assertIn("talOS/telemetry/a.cc: /usr/local/include/foo.h", out)
        self.assertNotIn("stdio.h", out)

    # REQ-DEV-25
    def test_unreadable_depfile_fails_closed(self):
        # A dangling/unreadable .d file beside a valid .d file must fail closed
        # with exit 2 rather than being silently skipped.
        with tempfile.TemporaryDirectory() as tmp:
            good = os.path.join(tmp, "a/wire.pic.d")
            os.makedirs(os.path.dirname(good), exist_ok=True)
            with open(good, "w", encoding="utf-8") as f:
                f.write(TOOLCHAIN_ONLY)
            broken = os.path.join(tmp, "b/broken.d")
            os.makedirs(os.path.dirname(broken), exist_ok=True)
            os.symlink(os.path.join(tmp, "missing_target"), broken)
            out, err = io.StringIO(), io.StringIO()
            with redirect_stdout(out), redirect_stderr(err):
                status = check_host_includes.main([tmp])
        self.assertEqual(status, 2, out.getvalue() + err.getvalue())
        self.assertIn("broken.d", err.getvalue())

    # REQ-DEV-25
    def test_crlf_tab_and_malformed_depfiles_fail_closed(self):
        # CRLF line continuations and tab-after-colon rules must still catch
        # host headers rather than dropping the rule.
        crlf_tab_dep = (
            "bazel-out/a.o:\t\\\r\n"
            " talOS/a.cc /usr/include/toml++/toml.hpp\r\n"
        )
        status, out = run({"a/a.d": crlf_tab_dep})
        self.assertEqual(status, 1, out)
        self.assertIn("talOS/a.cc: /usr/include/toml++/toml.hpp", out)

        # A non-empty depfile with no parseable Make rule beside a valid .d
        # file must fail closed (exit 2) rather than being silently skipped.
        with redirect_stderr(io.StringIO()) as err:
            status, out = run({
                "a/wire.pic.d": TOOLCHAIN_ONLY,
                "b/corrupt.d": "not a valid make rule\n",
            })
        self.assertEqual(status, 2, out + err.getvalue())
        self.assertIn("corrupt.d", err.getvalue())


if __name__ == "__main__":
    unittest.main()
