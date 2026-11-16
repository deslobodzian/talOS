"""Tests for tools/check_robot_repository.sh with a stub `bazel`.

REQ-DEV-30: a talOS change is checked against a robot repository that
consumes it. The check must refuse a path that is not a git repository and
must never write into the robot repository it reads (it builds a copy of its
committed tree). The Bazel path itself is proved by running the script
against a real robot repository by hand.
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


def git(cwd, *args):
    subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True)


def listing(root):
    out = []
    for d, dirs, files in os.walk(root):
        # `git status` itself refreshes the index; the work tree is what
        # the check must not touch.
        if d == root:
            dirs.remove(".git")
        for name in dirs + files:
            path = os.path.join(d, name)
            st = os.lstat(path)
            out.append((os.path.relpath(path, root), st.st_mode, st.st_mtime_ns))
    return sorted(out)


class CheckRobotRepositoryTest(unittest.TestCase):
    def setUp(self):
        r = runfiles.Create()
        self.script = r.Rlocation("_main/tools/check_robot_repository.sh")
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = os.path.realpath(tmp.name)
        self.tmpdir = os.path.join(self.tmp, "tmp")
        os.makedirs(self.tmpdir)
        self.log = os.path.join(self.tmp, "bazel_calls")
        bin_dir = os.path.join(self.tmp, "bin")
        # Records where it ran and its arguments, and writes into its
        # working directory as a real build would.
        put(os.path.join(bin_dir, "bazel"),
            "#!/bin/sh\n"
            f'echo "$PWD|$*" >> {shlex.quote(self.log)}\n'
            "touch bazel-out MODULE.bazel.lock\n"
            '[ "$1" = "${STUB_FAIL:-}" ] && { echo "//x:y_test FAILED in 1s"; exit 3; }\n'
            "exit 0\n",
            0o755)
        self.env = dict(os.environ,
                        PATH=bin_dir + os.pathsep + os.environ["PATH"],
                        TMPDIR=self.tmpdir,
                        GIT_CONFIG_GLOBAL="/dev/null",
                        GIT_CONFIG_NOSYSTEM="1")

        self.robot = os.path.join(self.tmp, "robot")
        put(os.path.join(self.robot, "MODULE.bazel"), 'module(name = "robot")\n')
        put(os.path.join(self.robot, "BUILD"), "")
        git(self.robot, "init", "-q")
        git(self.robot, "-c", "user.name=t", "-c", "user.email=t@t",
            "add", ".")
        git(self.robot, "-c", "user.name=t", "-c", "user.email=t@t",
            "commit", "-q", "-m", "init")
        # An uncommitted change, which the check reports and leaves alone.
        put(os.path.join(self.robot, "BUILD"), "# edited\n")

    def run_script(self, *args, **env):
        return subprocess.run(["bash", self.script, *args],
                              env=dict(self.env, **env), capture_output=True,
                              text=True, check=False)

    def calls(self):
        if not os.path.exists(self.log):
            return []
        with open(self.log, encoding="utf-8") as f:
            return f.read().splitlines()

    def test_refuses_a_path_that_is_not_a_git_repository(self):
        plain = os.path.join(self.tmp, "plain")
        os.makedirs(plain)
        result = self.run_script(plain)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("not a git repository", result.stderr)
        self.assertEqual(self.calls(), [])

    def test_builds_a_copy_and_never_writes_the_source(self):
        def status():
            return subprocess.run(
                ["git", "--no-optional-locks", "status", "--porcelain", "--untracked-files=all"],
                cwd=self.robot, capture_output=True, text=True,
                check=True).stdout

        before = (status(), listing(self.robot))
        for fail in ("", "test"):
            result = self.run_script(self.robot, "--jobs=1", STUB_FAIL=fail)
            self.assertEqual(result.returncode, 1 if fail else 0,
                             result.stdout + result.stderr)
            self.assertEqual((status(), listing(self.robot)), before)
            self.assertIn("uncommitted changes", result.stdout)
        calls = self.calls()
        self.assertTrue(calls)
        for call in calls:
            cwd, args = call.split("|", 1)
            self.assertNotEqual(os.path.realpath(cwd), self.robot)
            if args.startswith(("build", "test")):
                self.assertIn("--override_module=talos=", args)
                self.assertIn("--jobs=1", args)
        # The failing run says what failed and keeps its copy.
        self.assertIn("FAILED", result.stdout.splitlines()[-1])
        self.assertIn("//x:y_test FAILED", result.stdout)


if __name__ == "__main__":
    unittest.main()
