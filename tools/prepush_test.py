"""Tests for tools/prepush.sh with a stub `bazel`, so nothing builds.

TODO.md CI-7: CI tests with every tracked file read-only, and a test that
rewrote a copy of a source passed on the workstation and failed in CI. The
pre-push command must test a clean worktree of the commit under CI's
read-only step, leave the checkout as it was, and fail naming the step.

prepush.sh runs against a throwaway repository built here, with TMPDIR in
the test's own directory, so the worktree it makes is the test's to inspect.
"""

import os
import shlex
import subprocess
import tempfile
import unittest

from python.runfiles import runfiles

DEPFILE = "x.o: x.cc /usr/include/stdio.h\n"


def put(path, text, mode=None):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    if mode is not None:
        os.chmod(path, mode)


class PrepushTest(unittest.TestCase):
    def setUp(self):
        r = runfiles.Create()
        self.script = r.Rlocation("_main/tools/prepush.sh")
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = os.path.realpath(tmp.name)
        self.repo = os.path.join(self.tmp, "repo")
        self.tmpdir = os.path.join(self.tmp, "tmp")
        os.makedirs(self.tmpdir)
        self.log = os.path.join(self.tmp, "bazel_calls")
        self.out_path = os.path.join(self.tmp, "bazel-out")
        # What the stub's build writes under the output path.
        self.built = os.path.join(self.tmp, "built")
        put(os.path.join(self.built, "x.d"), DEPFILE)

        # Each call records its working directory, its arguments, and
        # whether the tracked file is writable there; a build writes the
        # `.d` files under self.built to the output path; STUB_FAIL fails
        # that subcommand.
        bin_dir = os.path.join(self.tmp, "bin")
        put(os.path.join(bin_dir, "bazel"),
            "#!/bin/sh\n"
            'if [ -w src/a.txt ]; then mode=writable; else mode=read-only; fi\n'
            f'echo "$PWD|$mode|$*" >> {shlex.quote(self.log)}\n'
            f'[ "$1" = info ] && echo {shlex.quote(self.out_path)}\n'
            f'[ "$1" = build ] && mkdir -p {shlex.quote(self.out_path)} &&'
            f' cp -R {shlex.quote(self.built)}/. {shlex.quote(self.out_path)}/\n'
            '[ "$1" = "${STUB_FAIL:-}" ] && { echo "stub: $1 failed"; exit 1; }\n'
            "exit 0\n",
            0o755)

        self.env = {k: v for k, v in os.environ.items()
                    if not k.startswith("GIT_")}
        self.env.update(
            PATH=bin_dir + os.pathsep + os.environ.get("PATH", ""),
            TMPDIR=self.tmpdir, HOME=self.tmp, GIT_CONFIG_GLOBAL=os.devnull,
            GIT_CONFIG_NOSYSTEM="1", GIT_AUTHOR_NAME="t",
            GIT_AUTHOR_EMAIL="t@example.com", GIT_COMMITTER_NAME="t",
            GIT_COMMITTER_EMAIL="t@example.com")
        put(os.path.join(self.repo, "src", "a.txt"), "a\n")
        put(os.path.join(self.repo, "MODULE.bazel.lock"), "{}\n")
        for args in (["init", "-q"], ["add", "."], ["commit", "-qm", "one"]):
            self.git(*args)
        self.addCleanup(self.git, "worktree", "prune")

    def git(self, *args):
        return subprocess.run(["git", *args], cwd=self.repo, env=self.env,
                              capture_output=True, text=True,
                              check=True).stdout

    def prepush(self, **env):
        proc = subprocess.run(
            ["bash", self.script, "--skip-gradle", "--skip-studio"],
            cwd=self.repo, env=dict(self.env, **env), capture_output=True,
            text=True, timeout=60)
        calls = []
        if os.path.exists(self.log):
            with open(self.log, encoding="utf-8") as f:
                calls = [line.rstrip("\n").split("|", 2) for line in f]
        return proc.returncode, proc.stdout + proc.stderr, calls

    def assert_cleaned_up(self):
        self.assertEqual(os.listdir(self.tmpdir),
                         [n for n in os.listdir(self.tmpdir)
                          if n.startswith("talos-prepush-logs.")])
        self.assertEqual(
            [line for line in self.git("worktree", "list", "--porcelain")
             .splitlines() if line.startswith("worktree ")],
            [f"worktree {self.repo}"])
        self.assertTrue(os.access(os.path.join(self.repo, "src", "a.txt"),
                                  os.W_OK))

    # REQ-DEV-25
    def test_builds_and_tests_a_read_only_worktree_then_removes_it(self):
        status, out, calls = self.prepush()
        self.assertEqual(status, 0, out)
        steps = {args.split()[0]: (cwd, mode, args) for cwd, mode, args in calls}
        for step in ("build", "test"):
            self.assertIn(step, steps, out)
            cwd, mode, args = steps[step]
            self.assertNotEqual(cwd, self.repo)
            self.assertTrue(cwd.startswith(self.tmpdir + os.sep), cwd)
            self.assertEqual(mode, "read-only", f"{step} ran on writable sources")
            self.assertIn("//...", args.split())
        self.assertIn("--nocache_test_results", steps["test"][2].split())
        # Without these a compile that hit the disk cache leaves no `.d`
        # file on disk, and host-includes cannot see what it read.
        self.assertIn("--experimental_inmemory_dotd_files=false",
                      steps["build"][2].split())
        self.assertIn(r"--remote_download_regex=.*\.d$",
                      steps["build"][2].split())
        self.assertIn("prepush: host-includes ok", out)
        self.assertIn("prepush: test ok", out)
        self.assert_cleaned_up()

    # REQ-DEV-25
    def test_a_failing_step_fails_the_run_naming_it(self):
        put(os.path.join(self.built, "y.d"),
            "y.o: y.cc /usr/include/toml++/toml.hpp\n")
        status, out, _ = self.prepush(STUB_FAIL="test")
        self.assertNotEqual(status, 0, out)
        self.assertIn("prepush: host-includes FAILED", out)
        self.assertIn("y.cc: /usr/include/toml++/toml.hpp", out)
        self.assertIn("prepush: test FAILED", out)
        self.assertIn("prepush: build ok", out)
        self.assert_cleaned_up()

    # REQ-DEV-25
    def test_host_includes_reads_only_this_builds_depfiles(self):
        # A `.d` file an earlier run left in the reused output base, for a
        # target this commit no longer has.
        put(os.path.join(self.out_path, "gone.d"),
            "gone.o: gone.cc /usr/include/toml++/toml.hpp\n")
        status, out, _ = self.prepush()
        self.assertEqual(status, 0, out)
        self.assertIn("prepush: host-includes ok", out)


if __name__ == "__main__":
    unittest.main()
