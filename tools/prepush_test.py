"""Tests for tools/prepush.sh with a stub `bazel`, so nothing builds.

REQ-DEV-25: CI tests with every tracked file read-only, and a test that
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
        # The network-namespace probe: STUB_NETNS=no refuses it.
        put(os.path.join(bin_dir, "unshare"),
            '#!/bin/sh\n[ "${STUB_NETNS:-yes}" = yes ]\n', 0o755)
        # pnpm: records each run; STUB_PNPM_INSTALL=fail fails an install.
        self.pnpm_log = os.path.join(self.tmp, "pnpm_calls")
        put(os.path.join(bin_dir, "pnpm"),
            f'#!/bin/sh\necho "$*" >> {shlex.quote(self.pnpm_log)}\n'
            '[ "$1" = install ] && [ "${STUB_PNPM_INSTALL:-}" = fail ] && exit 1\n'
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

    def prepush(self, *args, cwd=None, **env):
        proc = subprocess.run(
            ["bash", self.script, *(args or ("--skip-gradle", "--skip-studio"))],
            cwd=cwd or self.repo, env=dict(self.env, **env),
            capture_output=True, text=True, timeout=60)
        calls = []
        if os.path.exists(self.log):
            with open(self.log, encoding="utf-8") as f:
                calls = [line.rstrip("\n").split("|", 2) for line in f]
        return proc.returncode, proc.stdout + proc.stderr, calls

    def commit(self, path, message):
        """Commits a new file at `path` (relative to the repository)."""
        put(os.path.join(self.repo, path), message + "\n")
        self.git("add", path)
        self.git("commit", "-qm", message)
        return self.git("rev-parse", "HEAD").strip()

    def add_gradlew(self):
        """Commits a stub gradlew that records each run in self.gradle_log."""
        self.gradle_log = os.path.join(self.tmp, "gradle_calls")
        path = "2026-robot/controller_processor/rio/gradlew"
        put(os.path.join(self.repo, path),
            f'#!/bin/sh\necho "$*" >> {shlex.quote(self.gradle_log)}\n',
            0o755)
        self.git("add", path)
        self.git("commit", "-qm", "gradlew")

    def gradle_ran(self):
        ran = os.path.exists(self.gradle_log)
        if ran:
            os.remove(self.gradle_log)
        return ran

    def worktree_paths(self):
        return [line.split(" ", 1)[1]
                for line in self.git("worktree", "list", "--porcelain")
                .splitlines() if line.startswith("worktree ")]

    def assert_cleaned_up(self):
        self.assertEqual(os.listdir(self.tmpdir),
                         [n for n in os.listdir(self.tmpdir)
                          if n.startswith("talos-prepush-logs.")])
        self.assertEqual(self.worktree_paths(), [self.repo])
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


    # REQ-DEV-25
    def test_tests_require_network_namespaces_where_the_host_allows(self):
        # CI's Linux job allows them and sets TALOS_REQUIRE_NETNS=1, so
        # udp_address_change_test fails rather than skips there.
        require = "--test_env=TALOS_REQUIRE_NETNS=1"
        for netns, required in (("yes", True), ("no", False)):
            if os.path.exists(self.log):
                os.remove(self.log)
            status, out, calls = self.prepush(STUB_NETNS=netns)
            self.assertEqual(status, 0, out)
            test = [args for _, _, args in calls if args.startswith("test")]
            self.assertEqual(len(test), 1, out)
            self.assertEqual(require in test[0].split(), required, out)

    # REQ-DEV-25
    def test_studio_does_not_use_modules_installed_for_another_lockfile(self):
        put(os.path.join(self.repo, "studio", "node_modules", "m"), "\n")
        self.git("branch", "pushed")
        self.git("branch", "--set-upstream-to=pushed")
        self.commit("studio/pnpm-lock.yaml", "lockfile")
        status, out, _ = self.prepush("--skip-gradle",
                                      STUB_PNPM_INSTALL="fail")
        self.assertEqual(status, 1, out)
        self.assertIn("prepush: studio SKIPPED", out)
        with open(self.pnpm_log, encoding="utf-8") as f:
            self.assertEqual([line.split()[0] for line in f], ["install"])

    # REQ-DEV-25
    def test_gradle_runs_for_any_source_the_roborio_program_compiles(self):
        # build.gradle compiles talOS/driver_station/driver_station.cc and
        # takes headers from all of talOS/.
        self.add_gradlew()
        self.git("branch", "pushed")
        self.git("branch", "--set-upstream-to=pushed")
        self.commit("talOS/driver_station/x.h", "driver station")
        status, out, _ = self.prepush("--skip-studio")
        self.assertEqual(status, 0, out)
        self.assertTrue(self.gradle_ran(), out)

        self.git("branch", "-f", "pushed", "HEAD")
        self.commit("docs/x.md", "docs")
        status, out, _ = self.prepush("--skip-studio")
        self.assertEqual(status, 0, out)
        self.assertFalse(self.gradle_ran(), out)

    # REQ-DEV-25
    def test_optional_steps_follow_the_checked_refs_own_upstream(self):
        # The current branch's upstream already holds the change; the
        # branch checked, whose upstream does not, still runs gradle.
        self.add_gradlew()
        self.git("branch", "feature_pushed")
        change = self.commit("talOS/hardware/x.h", "hardware")
        self.git("branch", "feature", change)
        self.git("branch", "--set-upstream-to=feature_pushed", "feature")
        self.git("branch", "current_pushed", change)
        self.git("branch", "--set-upstream-to=current_pushed")
        status, out, _ = self.prepush("--ref", "feature", "--skip-studio")
        self.assertEqual(status, 0, out)
        self.assertTrue(self.gradle_ran(), out)

    # REQ-DEV-25
    def test_a_relative_tmpdir_from_a_subdirectory(self):
        # TMPDIR names self.tmpdir relative to the subdirectory run from,
        # not to the repository's top level.
        cwd = os.path.join(self.repo, "src")
        status, out, calls = self.prepush(
            cwd=cwd, TMPDIR=os.path.relpath(self.tmpdir, cwd))
        self.assertEqual(status, 0, out)
        build_cwd = [c for c, _, args in calls if args.startswith("build")]
        self.assertEqual(len(build_cwd), 1, out)
        self.assertTrue(build_cwd[0].startswith(self.tmpdir + os.sep),
                        build_cwd[0])
        self.assert_cleaned_up()

    # REQ-DEV-25
    def test_refuses_to_run_from_inside_its_own_worktree(self):
        wt = os.path.join(self.tmpdir, f"talos-prepush-{os.getuid()}")
        self.git("worktree", "add", "-q", "--detach", wt)
        status, out, calls = self.prepush(cwd=wt)
        self.assertEqual(status, 2, out)
        self.assertEqual(calls, [], out)
        self.assertTrue(os.path.isfile(os.path.join(wt, "src", "a.txt")), out)
        self.assertFalse(os.path.exists(wt + ".lock"), out)

    # REQ-DEV-25
    def test_a_lock_without_a_pid_is_held(self):
        # Another run between its mkdir and its pid write.
        lock = os.path.join(self.tmpdir, f"talos-prepush-{os.getuid()}.lock")
        os.makedirs(lock)
        status, out, calls = self.prepush()
        self.assertEqual(status, 2, out)
        self.assertEqual(calls, [], out)
        self.assertTrue(os.path.isdir(lock), out)

    # REQ-DEV-25
    def test_leaves_other_worktrees_registered(self):
        # Another worktree whose directory is missing for now (an unmounted
        # disk): `git worktree prune` would drop its registration.
        other = os.path.join(self.tmp, "other")
        self.git("worktree", "add", "-q", "--detach", other)
        os.rename(other, other + ".away")
        status, out, _ = self.prepush()
        self.assertEqual(status, 0, out)
        self.assertIn(other, self.worktree_paths())
        os.rename(other + ".away", other)


if __name__ == "__main__":
    unittest.main()
