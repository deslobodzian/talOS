"""Unit tests for tools/deploy.sh argument validation, prefix checks, and rollback.

REQ-OPS-50, REQ-OPS-54: deploy.sh rejects prefixes that would trigger systemd
specifier/variable expansion (% or $), rejects unexpected trailing arguments
in enable-boot/install/rollback, snapshots `previous` before rollback's
warm-up and graph check, and prints the target `--dry-run` rollback would
select.
"""

import os
import shlex
import subprocess
import tempfile
import unittest

from python.runfiles import runfiles


def put(path, text, mode=None):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    if mode is not None:
        os.chmod(path, mode)


def make_stub_bundle(bundle_dir, run_sh_body="exit 0\n"):
    put(
        os.path.join(bundle_dir, "run.sh"),
        "#!/usr/bin/env bash\n" + run_sh_body,
        0o755,
    )


class DeployShTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.deploy = runfiles.Create().Rlocation("_main/tools/deploy.sh")

    def setUp(self):
        tmp = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self.addCleanup(tmp.cleanup)
        self.tmp = os.path.realpath(tmp.name)

    def run_deploy(self, *args):
        return subprocess.run(
            ["bash", self.deploy, *args],
            capture_output=True,
            text=True,
            timeout=60,
        )

    # REQ-OPS-54
    def test_validate_prefix_rejects_systemd_specifier_and_variable_expansion(self):
        for bad_prefix in (
            "/opt/talos%n",
            "/opt/talos/foo%i",
            "/opt/talos$HOME",
            "/opt/talos/${USER}",
        ):
            with self.subTest(prefix=bad_prefix):
                proc = self.run_deploy(
                    "enable-boot", "--dry-run", "--prefix", bad_prefix
                )
                out = proc.stdout + proc.stderr
                self.assertNotEqual(proc.returncode, 0, out)
                self.assertIn("invalid --prefix", proc.stderr)
                self.assertNotIn("ExecStart=", proc.stdout)
                self.assertNotIn("Environment=", proc.stdout)

        # A clean absolute prefix still succeeds under --dry-run.
        ok = self.run_deploy(
            "enable-boot", "--dry-run", "--prefix", "/opt/talos-robot_1.0"
        )
        self.assertEqual(ok.returncode, 0, ok.stdout + ok.stderr)

    # REQ-OPS-50, REQ-OPS-54
    def test_rejects_unexpected_trailing_arguments(self):
        prefix = os.path.join(self.tmp, "prefix")
        bundle = os.path.join(self.tmp, "bundle")
        make_stub_bundle(bundle)

        # enable-boot: a typo like `--usre robot` must fail, never silently
        # default to User=talos.
        for extra in (("--usre", "robot"), ("stray",)):
            with self.subTest(cmd="enable-boot", extra=extra):
                proc = self.run_deploy(
                    "enable-boot", "--dry-run", "--prefix", prefix, *extra
                )
                out = proc.stdout + proc.stderr
                self.assertNotEqual(proc.returncode, 0, out)
                self.assertNotIn("User=talos", proc.stdout)

        # install: trailing arguments must fail before writing to prefix.
        for extra in (("--bogus",), ("extra",)):
            with self.subTest(cmd="install", extra=extra):
                proc = self.run_deploy(
                    "install", bundle, *extra, "--prefix", prefix, "--no-warm-up"
                )
                out = proc.stdout + proc.stderr
                self.assertNotEqual(proc.returncode, 0, out)
                self.assertFalse(os.path.exists(prefix), out)

        # rollback: trailing arguments must fail and leave selections intact.
        rb_prefix = os.path.join(self.tmp, "rb_args_prefix")
        old = os.path.join(rb_prefix, "bundles", "20260101_000000_1")
        new = os.path.join(rb_prefix, "bundles", "20260102_000000_2")
        make_stub_bundle(old)
        make_stub_bundle(new)
        os.symlink(new, os.path.join(rb_prefix, "current"))
        os.symlink(old, os.path.join(rb_prefix, "previous"))
        for extra in (("--bogus",), ("extra",)):
            with self.subTest(cmd="rollback", extra=extra):
                proc = self.run_deploy(
                    "rollback", *extra, "--prefix", rb_prefix, "--no-warm-up"
                )
                out = proc.stdout + proc.stderr
                self.assertNotEqual(proc.returncode, 0, out)
                self.assertEqual(os.readlink(os.path.join(rb_prefix, "current")), new)
                self.assertEqual(os.readlink(os.path.join(rb_prefix, "previous")), old)

    # REQ-OPS-50
    def test_rollback_snapshots_previous_and_dry_run_prints_selected_target(self):
        prefix = os.path.join(self.tmp, "rollback_prefix")
        old = os.path.join(prefix, "bundles", "20260101_000000_1")
        new = os.path.join(prefix, "bundles", "20260102_000000_2")
        hijack = os.path.join(prefix, "bundles", "20260103_000000_3")
        make_stub_bundle(new)
        make_stub_bundle(hijack)
        # During rollback's graph check, rewrite $PREFIX/previous to point to
        # `hijack`, simulating a concurrent mutation between check and swap.
        prev_link = os.path.join(prefix, "previous")
        make_stub_bundle(
            old,
            f"ln -sfn {shlex.quote(hijack)} {shlex.quote(prev_link)}\nexit 0\n",
        )
        cur_link = os.path.join(prefix, "current")
        os.symlink(new, cur_link)
        os.symlink(old, prev_link)

        # --dry-run must report `old` (the target rollback would select),
        # not the unchanged `new`, and leave both symlinks untouched.
        dry = self.run_deploy("rollback", "--dry-run", "--prefix", prefix)
        self.assertEqual(dry.returncode, 0, dry.stdout + dry.stderr)
        self.assertIn(f"current -> {old}", dry.stdout)
        self.assertNotIn(f"current -> {new}", dry.stdout)
        self.assertEqual(os.readlink(cur_link), new)
        self.assertEqual(os.readlink(prev_link), old)

        # Real rollback must select the snapshotted `old` bundle that passed
        # check_graph, never `hijack` written to `previous` mid-rollback.
        real = self.run_deploy("rollback", "--no-warm-up", "--prefix", prefix)
        self.assertEqual(real.returncode, 0, real.stdout + real.stderr)
        self.assertEqual(os.readlink(cur_link), old)
        self.assertEqual(os.readlink(prev_link), new)
        self.assertIn(f"current -> {old}", real.stdout)


if __name__ == "__main__":
    unittest.main()
