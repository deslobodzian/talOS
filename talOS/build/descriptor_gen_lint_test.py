"""The talos_messages field-name lint (descriptor_gen lint) on fixtures.

bad.fbs mixes fields without a unit suffix with every exempt kind (bool,
enum, enum array, struct, struct array) and suffixed numbers; only the
unsuffixed numbers may be reported. good.fbs embeds included.fbs's
unsuffixed struct: a file answers for its own declarations only.
"""

import os
import re
import subprocess
import unittest

from python.runfiles import runfiles

_RUNFILES = runfiles.Create()


def _path(env):
    return _RUNFILES.Rlocation(os.environ[env])


def _lint(env, file):
    return subprocess.run(
        [_path("TALOS_DESCRIPTOR_GEN"), "lint", _path(env), file],
        capture_output=True, text=True, check=False)


def _reported(stderr):
    """{Struct.field} names the lint reported."""
    return set(re.findall(r"lint_test\.(\w+\.\w+)", stderr))


class FieldNameLintTest(unittest.TestCase):
    def test_unsuffixed_numbers_fail_and_exempt_kinds_pass(self):
        result = _lint("TALOS_LINT_BAD", "//lint_test/bad.fbs")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(_reported(result.stderr),
                         {"Bad.speed_bad", "Bad.count", "Bad.samples_bad"})

    def test_suffixed_struct_passes(self):
        result = _lint("TALOS_LINT_GOOD", "//lint_test/good.fbs")
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_included_struct_fails_in_its_own_file(self):
        result = _lint("TALOS_LINT_INCLUDED", "//lint_test/included.fbs")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(_reported(result.stderr), {"Included.raw"})


if __name__ == "__main__":
    unittest.main()
