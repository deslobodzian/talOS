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
                    if not k.startswith("GIT_")
                    and k != "TALOS_ROBOT_REPOSITORY"}
        self.env.update(
            PATH=bin_dir + os.pathsep + os.environ.get("PATH", ""),
            TMPDIR=self.tmpdir, HOME=self.tmp, GIT_CONFIG_GLOBAL=os.devnull,
            GIT_CONFIG_NOSYSTEM="1", GIT_AUTHOR_NAME="t",
            GIT_AUTHOR_EMAIL="t@example.com", GIT_COMMITTER_NAME="t",
            GIT_COMMITTER_EMAIL="t@example.com")
        put(os.path.join(self.repo, "src", "a.txt"), "a\n")
        put(os.path.join(self.repo, "MODULE.bazel.lock"), "{}\n")
        # The commit's own tools prepush runs; a commit without one fails
        # the run. The RoboRIO-in-the-loop stub records each run in
        # self.full_log (full_calls()).
        self.full_log = os.path.join(self.tmp, "full_calls")
        put(os.path.join(self.repo, "tools", "source_lints.sh"),
            "#!/bin/sh\nexit 0\n", 0o755)
        # format.sh and tidy.sh record their working directory, whether the
        # tracked file is writable there, and their arguments in
        # self.clang_log; STUB_FORMAT and STUB_TIDY are their exit codes,
        # and tidy's 3 prints tidy.sh's own reason it cannot run.
        self.clang_log = os.path.join(self.tmp, "clang_calls")
        for tool, var in (("format", "STUB_FORMAT"), ("tidy", "STUB_TIDY")):
            put(os.path.join(self.repo, "tools", f"{tool}.sh"),
                "#!/bin/sh\n"
                'if [ -w src/a.txt ]; then mode=writable; else mode=read-only; fi\n'
                f'echo "$PWD|$mode|{tool} $*" >> {shlex.quote(self.clang_log)}\n'
                f'status="${{{var}:-0}}"\n'
                '[ "$status" = 3 ] && {\n'
                '  echo "tidy: cannot run here: clang 18 cannot parse this'
                ' host\'s C++ standard library:" >&2\n'
                '  echo "  bits/atomic_wait.h:1:1: error: x" >&2; }\n'
                f'[ "$status" = 0 ] || echo "stub: {tool} exited $status"\n'
                'exit "$status"\n', 0o755)
        put(os.path.join(self.repo, "tools", "test_rio_in_the_loop.py"),
            "import os, sys\n"
            f"with open({self.full_log!r}, 'a', encoding='utf-8') as f:\n"
            "    f.write('|'.join([os.getcwd(), 'rio', *sys.argv[1:]]) + '\\n')\n")
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
        path = "controller_processor/rio/gradlew"
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

    def add_sanitizer_lane(self):
        """Commits a stub sanitizer lane script that records each run in
        self.full_log; STUB_LANE_FAIL fails that lane."""
        put(os.path.join(self.repo, "tools", "sanitizer_lane.sh"),
            f'#!/bin/sh\necho "$PWD|lane|$*" >> {shlex.quote(self.full_log)}\n'
            '[ "$1" = "${STUB_LANE_FAIL:-}" ] && { echo "stub: $1 failed"; exit 1; }\n'
            "exit 0\n", 0o755)
        self.git("add", "tools")
        self.git("commit", "-qm", "sanitizer lane")

    def full_calls(self):
        """The runs the rio and lane stubs recorded since the last call."""
        if not os.path.exists(self.full_log):
            return []
        with open(self.full_log, encoding="utf-8") as f:
            calls = [line.rstrip("\n").split("|") for line in f]
        os.remove(self.full_log)
        return calls

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

    # REQ-DEV-32: a contributor without the robot repository is unaffected.
    def test_robot_repository_is_skipped_without_its_variable(self):
        code, out, _ = self.prepush()
        self.assertEqual(code, 0, out)
        self.assertIn(
            "prepush: robot-repository skipped (TALOS_ROBOT_REPOSITORY unset)",
            out)

    # REQ-DEV-32: set but empty, as `TALOS_ROBOT_REPOSITORY="$MISSPELT"`
    # passes it, the run fails the step rather than skipping it or checking
    # the directory prepush was started in.
    def test_robot_repository_set_empty_fails_the_step(self):
        code, out, _ = self.prepush(TALOS_ROBOT_REPOSITORY="")
        self.assertEqual(code, 1, out)
        self.assertIn("prepush: robot-repository FAILED (TALOS_ROBOT_REPOSITORY"
                      " names no robot repository: '' is not a directory)",
                      out)
        self.assertNotIn("prepush: robot-repository skipped", out)
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
    def test_a_failing_source_lint_fails_the_run(self):
        # CI's lint job runs tools/source_lints.sh; a pushed commit whose
        # lint failed there passed a prepush that never ran it.
        path = "tools/source_lints.sh"
        put(os.path.join(self.repo, path),
            '#!/bin/sh\n[ -w src/a.txt ] && echo "lint: writable"\n'
            'echo "lint: node-matrix FAILED"; exit 1\n', 0o755)
        self.git("add", path)
        self.git("commit", "-qm", "lints")
        status, out, _ = self.prepush()
        self.assertEqual(status, 1, out)
        self.assertIn("prepush: source-lints FAILED", out)
        self.assertIn("lint: node-matrix FAILED", out)
        self.assertNotIn("lint: writable", out)
        self.assertIn("prepush: build ok", out)
        self.assert_cleaned_up()

    def clang_calls(self):
        """The format.sh and tidy.sh runs recorded since the last call."""
        if not os.path.exists(self.clang_log):
            return []
        with open(self.clang_log, encoding="utf-8") as f:
            calls = [line.rstrip("\n").split("|", 2) for line in f]
        os.remove(self.clang_log)
        return calls

    # REQ-DEV-25: CI's clang-format job failed on a pushed commit that
    # prepush had passed, since prepush never ran clang-format.
    def test_format_checks_the_branch_against_main_and_fails_the_run(self):
        remote = os.path.join(self.tmp, "remote.git")
        self.git("init", "-q", "--bare", remote)
        self.git("remote", "add", "origin", remote)
        self.git("push", "-q", "origin", "HEAD:refs/heads/main")
        self.git("fetch", "-q", "origin")
        main = self.git("rev-parse", "origin/main").strip()
        self.commit("talOS/x.cc", "source")
        status, out, _ = self.prepush()
        self.assertEqual(status, 0, out)
        self.assertIn("prepush: format ok", out)
        ran = {args.split()[0]: (cwd, mode, args.split()[1:])
               for cwd, mode, args in self.clang_calls()}
        for tool in ("format", "tidy"):
            cwd, mode, args = ran[tool]
            self.assertTrue(cwd.startswith(self.tmpdir + os.sep), cwd)
            self.assertEqual(mode, "read-only", out)
        self.assertEqual(ran["format"][2], ["--check", "--base", main], out)
        self.assertEqual(ran["tidy"][2], ["--base", main], out)

        # Unformatted files (1), and clang-format-18 missing or not
        # installable (format.sh's 2), both fail the run.
        for code in ("1", "2"):
            status, out, _ = self.prepush(STUB_FORMAT=code)
            self.assertEqual(status, 1, out)
            self.assertIn("prepush: format FAILED", out)
            self.assertIn(f"stub: format exited {code}", out)
            self.assertIn("prepush: test ok", out)
        self.assert_cleaned_up()

    # REQ-DEV-25: tidy.sh cannot run where clang 18 cannot parse the host's
    # standard library (exit 3); prepush says so instead of nothing, and
    # findings fail the run.
    def test_tidy_findings_fail_and_a_host_it_cannot_run_on_is_named(self):
        status, out, _ = self.prepush(STUB_TIDY="3")
        self.assertEqual(status, 0, out)
        self.assertIn("prepush: tidy NOT RUN (clang 18 cannot parse this "
                      "host's C++ standard library)", out)

        status, out, _ = self.prepush(STUB_TIDY="1")
        self.assertEqual(status, 1, out)
        self.assertIn("prepush: tidy FAILED", out)
        self.assertIn("stub: tidy exited 1", out)

        # Tidy reads the build's generated headers.
        self.clang_calls()
        status, out, _ = self.prepush(STUB_FAIL="build")
        self.assertEqual(status, 1, out)
        self.assertIn("prepush: tidy SKIPPED (the build failed)", out)
        self.assertEqual([a.split()[0] for _, _, a in self.clang_calls()],
                         ["format"], out)
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

    # REQ-DEV-25: on a branch without an upstream, studio reuses the checkout's
    # node_modules when its recorded .pnpm/lock.yaml matches the committed
    # pnpm-lock.yaml, and refuses stale node_modules whose recorded lockfile
    # differs.
    def test_studio_reuses_modules_only_when_recorded_lockfile_matches(self):
        self.commit("studio/pnpm-lock.yaml", "lockfile-v1")
        put(os.path.join(self.repo, "studio", "node_modules", ".pnpm",
                         "lock.yaml"), "lockfile-v1\n")
        status, out, _ = self.prepush("--skip-gradle",
                                      STUB_PNPM_INSTALL="fail")
        self.assertEqual(status, 0, out)
        self.assertIn("prepush: studio-test ok", out)
        self.assertIn("prepush: studio-build ok", out)
        with open(self.pnpm_log, encoding="utf-8") as f:
            self.assertEqual([line.split()[0] for line in f],
                             ["test", "build"])

        # Stale node_modules recorded an older lockfile: must not symlink.
        os.remove(self.pnpm_log)
        put(os.path.join(self.repo, "studio", "node_modules", ".pnpm",
                         "lock.yaml"), "lockfile-old\n")
        status, out, _ = self.prepush("--skip-gradle",
                                      STUB_PNPM_INSTALL="fail")
        self.assertEqual(status, 1, out)
        self.assertIn("prepush: studio SKIPPED", out)
        with open(self.pnpm_log, encoding="utf-8") as f:
            self.assertEqual([line.split()[0] for line in f], ["install"])

    # REQ-DEV-25: an unpushed branch with no upstream falls back to the merge
    # base with origin/main for optional steps, and still runs everything when
    # origin/main is absent.
    def test_optional_steps_fall_back_to_origin_main_without_upstream(self):
        self.add_gradlew()
        self.commit("studio/pnpm-lock.yaml", "lockfile-v1")
        put(os.path.join(self.repo, "studio", "node_modules", ".pnpm",
                         "lock.yaml"), "lockfile-v1\n")

        # Neither upstream nor origin/main: every optional step runs.
        self.commit("docs/before_remote.md", "docs before remote")
        status, out, _ = self.prepush("--ref", "HEAD",
                                      STUB_PNPM_INSTALL="fail")
        self.assertEqual(status, 0, out)
        self.assertTrue(self.gradle_ran(), out)
        self.assertEqual([c[1] for c in self.full_calls()], ["rio"], out)
        with open(self.pnpm_log, encoding="utf-8") as f:
            self.assertEqual([line.split()[0] for line in f],
                             ["test", "build"])
        os.remove(self.pnpm_log)

        # With origin/main present and no upstream, a doc-only change skips
        # gradle, rio-in-the-loop and studio.
        remote = os.path.join(self.tmp, "remote.git")
        self.git("init", "-q", "--bare", remote)
        self.git("remote", "add", "origin", remote)
        self.git("push", "-q", "origin", "HEAD:refs/heads/main")
        self.git("fetch", "-q", "origin")
        self.commit("docs/only_docs.md", "docs only")
        status, out, _ = self.prepush("--ref", "HEAD")
        self.assertEqual(status, 0, out)
        self.assertFalse(self.gradle_ran(), out)
        self.assertEqual(self.full_calls(), [], out)
        self.assertFalse(os.path.exists(self.pnpm_log), out)
        self.assertIn("gradle not needed (nothing under controller_processor/ "
                      "talOS/ since origin/main)", out)
        self.assertIn("studio not needed (nothing under studio/ since "
                      "origin/main)", out)

        # Touching talOS/ runs gradle and rio-in-the-loop, still skips studio.
        self.commit("talOS/x.h", "talos header")
        status, out, _ = self.prepush("--ref", "HEAD")
        self.assertEqual(status, 0, out)
        self.assertTrue(self.gradle_ran(), out)
        self.assertEqual([c[1] for c in self.full_calls()], ["rio"], out)
        self.assertFalse(os.path.exists(self.pnpm_log), out)

        # Touching studio/ runs studio too.
        self.commit("studio/x.ts", "studio source")
        status, out, _ = self.prepush("--ref", "HEAD",
                                      STUB_PNPM_INSTALL="fail")
        self.assertEqual(status, 0, out)
        self.assertTrue(self.gradle_ran(), out)
        self.assertEqual([c[1] for c in self.full_calls()], ["rio"], out)
        with open(self.pnpm_log, encoding="utf-8") as f:
            self.assertEqual([line.split()[0] for line in f],
                             ["test", "build"])

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
    def test_rio_in_the_loop_runs_when_the_commits_touch_what_it_runs(self):
        # A RoboRIO-in-the-loop scenario broken by an unpushed change was
        # seen only in CI's job, after the push.
        self.add_gradlew()
        self.git("branch", "pushed")
        self.git("branch", "--set-upstream-to=pushed")
        self.commit("talOS/bridge/x.h", "bridge")
        status, out, _ = self.prepush("--skip-studio")
        self.assertEqual(status, 0, out)
        rio = [c for c in self.full_calls() if c[1] == "rio"]
        self.assertEqual(len(rio), 1, out)
        cwd, _, *args = rio[0]
        self.assertTrue(cwd.startswith(self.tmpdir + os.sep), cwd)
        self.assertEqual(args[0], "--wpilib", out)
        self.assertEqual(args[1], "--output-dir", out)
        self.assertTrue(args[2].startswith(self.tmpdir + os.sep), args)
        self.assertIn("prepush: rio-in-the-loop ok", out)

        self.git("branch", "-f", "pushed", "HEAD")
        self.commit("docs/x.md", "docs")
        status, out, _ = self.prepush("--skip-studio")
        self.assertEqual(status, 0, out)
        self.assertEqual(self.full_calls(), [], out)

        # Anything the scenario's binaries are built from, not only the
        # packages on its command path.
        self.git("branch", "-f", "pushed", "HEAD")
        self.commit("talOS/events/x.h", "events")
        status, out, _ = self.prepush("--skip-studio")
        self.assertEqual(status, 0, out)
        self.assertEqual([c[1] for c in self.full_calls()], ["rio"], out)

    # REQ-DEV-25
    def test_full_runs_rio_in_the_loop_the_sanitizer_lanes_and_a_soak(self):
        self.add_sanitizer_lane()
        self.git("branch", "pushed")
        self.git("branch", "--set-upstream-to=pushed")
        self.commit("docs/x.md", "docs")
        status, out, calls = self.prepush("--full", "--skip-studio")
        self.assertEqual(status, 0, out)
        ran = self.full_calls()
        self.assertEqual([c[1] for c in ran], ["rio", "lane", "lane"], out)
        self.assertEqual([c[2].split()[0] for c in ran[1:]],
                         ["tsan", "asan"], out)
        self.assertIn("--nocache_test_results", ran[1][2].split())
        soak = [args.split() for _, _, args in calls
                if "//bench/main_processor/simulation:soak_test" in args]
        self.assertEqual(len(soak), 1, out)
        self.assertIn("--test_env=TALOS_SOAK_SECONDS=42", soak[0])
        self.assertIn("--nocache_test_results", soak[0])

    # REQ-DEV-25
    def test_full_fails_naming_a_failed_lane_and_skips_lanes_after_a_failed_build(self):
        self.add_sanitizer_lane()
        status, out, _ = self.prepush("--full", "--skip-studio",
                                      STUB_LANE_FAIL="tsan")
        self.assertEqual(status, 1, out)
        self.assertIn("prepush: tsan FAILED", out)
        self.assertIn("stub: tsan failed", out)
        self.assertIn("prepush: asan ok", out)
        self.assert_cleaned_up()

        # After a failed build every later Bazel step is skipped, failing
        # the run, rather than spending a quarter hour on a failed run.
        self.full_calls()
        os.remove(self.log)
        status, out, calls = self.prepush("--full", "--skip-studio",
                                          STUB_FAIL="build")
        self.assertEqual(status, 1, out)
        for step in ("rio-in-the-loop", "tsan", "asan", "soak"):
            self.assertIn(f"prepush: {step} SKIPPED (the build failed)", out)
        self.assertEqual(self.full_calls(), [], out)
        self.assertFalse([a for _, _, a in calls if "soak_test" in a], out)

    # REQ-DEV-25
    def test_more_than_twenty_commits_since_upstream_or_on_no_remote_need_full(self):
        # 110 commits passed the default steps and were red in CI on a
        # sanitizer lane and the RoboRIO-in-the-loop job.
        self.add_sanitizer_lane()
        # What is already on a remote, for the count without an upstream.
        remote = os.path.join(self.tmp, "remote.git")
        self.git("init", "-q", "--bare", remote)
        self.git("remote", "add", "origin", remote)
        self.git("push", "-q", "origin", "HEAD:refs/heads/main")
        self.git("branch", "pushed")
        self.git("branch", "--set-upstream-to=pushed")
        for i in range(20):
            self.commit(f"docs/{i}.md", f"docs {i}")
        status, out, _ = self.prepush()
        self.assertEqual(status, 0, out)
        self.assertNotIn("prepush: full", out)

        self.commit("docs/20.md", "docs 20")
        status, out, _ = self.prepush()
        self.assertEqual(status, 1, out)
        self.assertIn("prepush: full SKIPPED (21 commits since pushed", out)
        self.assertIn("prepush: test ok", out)

        status, out, _ = self.prepush("--full", "--skip-studio")
        self.assertEqual(status, 0, out)

        # A new branch: the commits on no remote-tracking branch.
        self.git("branch", "--unset-upstream")
        status, out, _ = self.prepush()
        self.assertEqual(status, 1, out)
        self.assertIn("prepush: full SKIPPED (21 commits on no "
                      "remote-tracking branch", out)
        self.assertIn("set an upstream or pass --full", out)

    # REQ-DEV-25
    def test_a_missing_tool_fails_the_run_unless_the_commit_predates_it(self):
        # A commit that deletes a tool prepush runs must not pass with the
        # step unrun. The fixture has no tools/sanitizer_lane.sh.
        self.add_gradlew()
        old = self.git("rev-parse", "HEAD").strip()
        status, out, _ = self.prepush("--full", "--skip-studio")
        self.assertEqual(status, 1, out)
        for lane in ("tsan", "asan"):
            self.assertIn(f"prepush: {lane} SKIPPED (this commit has no "
                          "tools/sanitizer_lane.sh", out)

        # An older commit than the tool, checked by --ref: nothing to run.
        self.add_sanitizer_lane()
        status, out, _ = self.prepush("--ref", old, "--full", "--skip-studio")
        self.assertEqual(status, 0, out)
        self.assertIn("prepush: tsan skipped", out)

        # Another branch deleting a tool its upstream has fails the run.
        self.git("branch", "has_lints")
        self.git("checkout", "-q", "-b", "deletes_lints")
        self.git("rm", "-q", "tools/source_lints.sh")
        self.git("commit", "-qm", "delete lints")
        self.git("branch", "--set-upstream-to=has_lints")
        self.git("checkout", "-q", "-")
        status, out, _ = self.prepush("--ref", "deletes_lints")
        self.assertEqual(status, 1, out)
        self.assertIn("prepush: source-lints SKIPPED (this commit has no "
                      "tools/source_lints.sh", out)

    # REQ-DEV-25
    def test_full_is_refused_with_skip_gradle(self):
        # --full promises rio-in-the-loop, which runs through Gradle.
        status, out, calls = self.prepush("--full", "--skip-gradle")
        self.assertEqual(status, 2, out)
        self.assertIn("drop --skip-gradle", out)
        self.assertEqual(calls, [], out)

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

    # REQ-DEV-25: two runs that both read a dead holder's pid must not both
    # take the lock; the retake is one at a time.
    def test_a_dead_holders_lock_is_retaken_by_one_run_only(self):
        lock = os.path.join(self.tmpdir, f"talos-prepush-{os.getuid()}.lock")
        dead = subprocess.Popen(["true"])
        dead.wait()
        put(os.path.join(lock, "pid"), f"{dead.pid}\n")
        # Another run in the middle of retaking it.
        os.makedirs(lock + ".retake")
        status, out, calls = self.prepush()
        self.assertEqual(status, 2, out)
        self.assertIn("retaking", out)
        self.assertEqual(calls, [], out)
        with open(os.path.join(lock, "pid"), encoding="utf-8") as f:
            self.assertEqual(f.read(), f"{dead.pid}\n")

        # Alone, a run retakes it and leaves neither directory behind.
        os.rmdir(lock + ".retake")
        status, out, _ = self.prepush()
        self.assertEqual(status, 0, out)
        self.assert_cleaned_up()

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
