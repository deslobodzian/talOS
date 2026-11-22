"""Unit tests for tools/check_runfiles_collisions.py."""

import io
import os
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

from tools import check_runfiles_collisions


def run_check(root):
    out = io.StringIO()
    err = io.StringIO()
    with redirect_stdout(out), redirect_stderr(err):
        status = check_runfiles_collisions.main(["check_runfiles_collisions.py", root])
    return status, out.getvalue(), err.getvalue()


class CheckRunfilesCollisionsTest(unittest.TestCase):
    def test_zero_manifests_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            # Even with unrelated files and a .runfiles directory, finding zero
            # *.runfiles_manifest files must fail closed (exit 2), not pass.
            os.makedirs(os.path.join(tmp, "pkg/foo.runfiles"))
            with open(os.path.join(tmp, "pkg/readme.txt"), "w", encoding="utf-8") as f:
                f.write("not a manifest\n")
            status, _, err = run_check(tmp)
        self.assertEqual(status, 2)
        self.assertIn("runfiles_manifest", err)

    def test_missing_root_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            missing = os.path.join(tmp, "does_not_exist")
            status, _, err = run_check(missing)
        self.assertEqual(status, 2)
        self.assertIn("not found", err)

    def test_clean_manifests_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest = os.path.join(tmp, "pkg/a_test.runfiles_manifest")
            os.makedirs(os.path.dirname(manifest), exist_ok=True)
            with open(manifest, "w", encoding="utf-8") as f:
                f.write("_main/pkg/a.py /src/pkg/a.py\n")
            status, out, err = run_check(tmp)
        self.assertEqual(status, 0, out + err)
        self.assertIn("checked 1 runfiles manifests", out)

    def test_empty_vs_real_collision_and_generated_init_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            m1 = os.path.join(tmp, "pkg/a_test.runfiles_manifest")
            m2 = os.path.join(tmp, "pkg/b_test.runfiles_manifest")
            os.makedirs(os.path.dirname(m1), exist_ok=True)
            with open(m1, "w", encoding="utf-8") as f:
                f.write("_main/pkg/mod.py \n")
                f.write("_main/pkg/__init__.py \n")
            with open(m2, "w", encoding="utf-8") as f:
                f.write("_main/pkg/mod.py /src/pkg/mod.py\n")
            status, out, _ = run_check(tmp)
        self.assertEqual(status, 1, out)
        self.assertIn("_main/pkg/mod.py: empty in 1 manifest(s)", out)
        self.assertIn("_main/pkg/__init__.py: generated __init__.py", out)


if __name__ == "__main__":
    unittest.main()
