"""tools/deploy.sh's warm-up runs each member of an installed bundle once,
in turn, before install's graph check and before anything is selected.

It is checked on bundles of stubs that record each run. On macOS each newly
written executable's first exec costs about 100 ms, and seconds under
parallel load (TODO.md MAC-2), and every installed stub is a new file. So a
stub runs under the default 60 s bound, except in the one case that waits
out a short bound: there every member has already run once from its
installed path, so only the stub that never exits can reach it."""

import os
import re
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import unittest

from python.runfiles import runfiles

from bench.main_processor.deploy_test_support import deploy_script


# Each stub records, under its installed path, "start" with its pid, what
# $STUB_PREFIX/current selected then ("-" for nothing) and its argv before
# acting, and "end" after, so the log orders every warm-up run.
STUB = """#!/usr/bin/env bash
me="$(cd "$(dirname "$0")" && pwd -P)/$(basename "$0")"
current="$(readlink "$STUB_PREFIX/current" || echo -)"
printf 'start %s %s %s %s\\n' "$me" "$$" "$current" "$*" >> "$STUB_LOG"
case "$(basename "$0")" in
  fails) printf 'end %s\\n' "$me" >> "$STUB_LOG"; exit 3 ;;
  dies) kill -KILL $$ ;;
  hangs)
    # STUB_PREWARM: run once to pay a first exec, then exit. STUB_FIFO: say
    # it is running (after spawning a child process so process-group kill is
    # exercised).
    [ -z "${STUB_PREWARM:-}" ] || exit 0
    bash -c '
      printf "child %s %s\n" "$1" "$$" >> "$STUB_LOG"
      [ -z "${STUB_FIFO:-}" ] || echo up > "$STUB_FIFO"
      exec tail -f /dev/null
    ' _ "$me" &
    wait "$!" ;;
  run.sh)
    # The graph check's run: STUB_CHECK_FIFO says it is running and never
    # exits; STUB_CHECK_EXIT is its exit status (the launcher refusing).
    if [ -n "${STUB_CHECK_FIFO:-}" ]; then
      echo up > "$STUB_CHECK_FIFO"
      exec tail -f /dev/null
    fi
    printf 'end %s\\n' "$me" >> "$STUB_LOG"
    exit "${STUB_CHECK_EXIT:-0}" ;;
esac
printf 'end %s\\n' "$me" >> "$STUB_LOG"
"""

def stub_bundle(root, nodes):
    """A bundle laid out as talos_bundle writes one, every executable a
    stub: `nodes` (`fails` exits 3, `dies` is killed by SIGKILL, `hangs`
    never exits, any other exits 0), a Studio bridge, the launcher and run.sh."""
    for sub in ("bin", "config", "studio"):
        os.makedirs(os.path.join(root, sub))
    with open(os.path.join(root, "config", "robot.toml"), "w",
              encoding="utf-8") as f:
        f.write("[robot]\n")
    for rel in [f"bin/{n}" for n in nodes] + [
            "studio/studio_bridge", "launcher", "run.sh"]:
        with open(os.path.join(root, rel), "w", encoding="utf-8") as f:
            f.write(STUB)
        os.chmod(os.path.join(root, rel), 0o755)
    lines = ['bundle_version = 1', 'config_path = "config/robot.toml"',
             'config_sha256 = "0"', '', '[studio]',
             'bridge = "studio/studio_bridge"', 'bridge_sha256 = "0"',
             'targets = ["//studio/bridge:studio_bridge"]',
             'webroot = "studio/webroot"']
    for n in nodes:
        lines += ['', '[[nodes]]', f'name = "{n}"',
                  f'targets = ["//x:{n}"]', f'binary = "bin/{n}"',
                  'binary_sha256 = "0"', 'language = "cpp"']
    with open(os.path.join(root, "bundle_manifest.toml"), "w",
              encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


NATIVE_MARK = "bin/alpha.runfiles/site/mark.cpython-311-darwin.so"
NATIVE_DYLIB = "bin/alpha.runfiles/site/mark_two.dylib"


def native_bundle(src):
    """A stub bundle with three native libraries in sorted name order: one that
    fails to load (`broken.so`, sorted first), the marking library as a `.so`
    (`NATIVE_MARK`), and a second marking library copy as a `.dylib`
    (`NATIVE_DYLIB`). Returns `(NATIVE_MARK, NATIVE_DYLIB)`."""
    stub_bundle(src, ("alpha",))
    site = os.path.dirname(os.path.join(src, NATIVE_MARK))
    os.makedirs(site)
    mark_src = runfiles.Create().Rlocation(
        "_main/bench/main_processor/bundle_test_native_mark.so")
    shutil.copy(mark_src, os.path.join(src, NATIVE_MARK))
    shutil.copy(mark_src, os.path.join(src, NATIVE_DYLIB))
    with open(os.path.join(site, "broken.so"), "wb") as f:
        f.write(b"not a Mach-O file")
    return NATIVE_MARK, NATIVE_DYLIB


class WarmUpTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self.addCleanup(tmp.cleanup)
        self.tmp = tmp.name
        self.prefix = os.path.join(self.tmp, "prefix")
        self.log = os.path.join(self.tmp, "stub.log")
        # The default bound: no fresh stub is killed for its first exec.
        self.env = dict(os.environ, STUB_LOG=self.log, STUB_PREFIX=self.prefix)
        self.env.pop("TALOS_DEPLOY_WARM_UP_TIMEOUT_S", None)

    def deploy(self, *args):
        proc = subprocess.run(
            ["bash", deploy_script(), *args, "--prefix", self.prefix],
            capture_output=True, text=True, env=self.env, timeout=60)
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        return proc

    def install(self, name, nodes, *args):
        src = os.path.join(self.tmp, name)
        stub_bundle(src, nodes)
        return self.deploy("install", src, *args)

    def records(self, check=False):
        """The warm-up's records, or with `check` install's graph check's
        (the installed run.sh, which the warm-up never runs)."""
        if not os.path.exists(self.log):
            return []
        with open(self.log, encoding="utf-8") as f:
            return [r for r in (line.split(" ", 4)
                                for line in f.read().splitlines())
                    if (os.path.basename(r[1]) == "run.sh") == check]

    def assert_checked_after_warm_up(self, bundle):
        """The installed run.sh ran once, with --describe-only, after every
        warm-up run and before `current` selected anything."""
        log = self.records(check=True)
        self.assertEqual([r[0] for r in log], ["start", "end"], log)
        self.assertEqual(log[0][1], os.path.join(bundle, "run.sh"), log)
        self.assertEqual(log[0][3], "-", log)
        self.assertTrue(log[0][4].startswith("--describe-only --output-dir "),
                        log)
        with open(self.log, encoding="utf-8") as f:
            last = f.read().splitlines()[-1]
        self.assertEqual(last, f"end {os.path.join(bundle, 'run.sh')}")

    def assert_dead(self, pid, msg):
        """Asserts `pid` is gone (or a SIGKILLed zombie awaiting init reap)."""
        def _cleanup_kill(p=pid):
            try:
                os.kill(p, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.addCleanup(_cleanup_kill)
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return
        try:
            with open(f"/proc/{pid}/stat", encoding="utf-8") as f:
                state = f.read().rsplit(")", 1)[1].split()[0]
            if state == "Z":
                return
        except OSError:
            return
        self.fail(f"{msg} (pid {pid} still alive)")

    def assert_warmed_in_turn(self, bundle, nodes, selected, proc):
        """Every member of the installed `bundle` ran exactly once, one at a
        time, while `current` still selected `selected`, and nothing else
        ran; a member that failed or hung is named in a warning."""
        records = self.records()
        out = proc.stdout + proc.stderr
        describe = "--describe --config config/robot.toml"
        # The installed files (the macOS charge is per file); run.sh is the
        # graph check's, which records() leaves out.
        started = sorted((r[1], r[4] if len(r) > 4 else "")
                         for r in records if r[0] == "start")
        want = sorted(
            [(os.path.join(bundle, "bin", n), describe) for n in nodes]
            + [(os.path.join(bundle, "studio", "studio_bridge"), describe),
               (os.path.join(bundle, "launcher"), "--help")])
        self.assertEqual(started, want, out)
        hung_pids = []
        child_pids = [int(r[2]) for r in records if r[0] == "child"]
        for i, r in enumerate(records):
            if r[0] != "start":
                continue
            self.assertEqual(r[3], selected, records)
            if os.path.basename(r[1]) == "hangs":
                # Killed before the next starts, its pid gone by the return.
                hung_pids.append(int(r[2]))
                self.assertNotEqual(records[i + 1:i + 2], [["end", r[1]]])
                continue
            self.assertEqual(records[i + 1][:2], ["end", r[1]], records)
        self.assertEqual(len(hung_pids), "hangs" in nodes, records)
        self.assertEqual(len(child_pids), "hangs" in nodes, records)
        for pid in hung_pids:
            self.assert_dead(pid, "hung member alive")
        for pid in child_pids:
            self.assertNotIn(pid, hung_pids, records)
            self.assert_dead(pid, "hung member's child alive")
        for name in ("fails", "hangs"):
            self.assertEqual(name in proc.stderr, name in nodes, out)

    # REQ-OPS-47
    def test_install_runs_each_member_once_in_turn_before_selecting(self):
        nodes = ("alpha", "fails", "omega")
        proc = self.install("bundle", nodes)
        bundle = os.path.realpath(os.path.join(self.prefix, "current"))
        self.assert_warmed_in_turn(bundle, nodes, "-", proc)
        self.assert_checked_after_warm_up(bundle)

    # REQ-OPS-47
    def test_no_warm_up_runs_nothing(self):
        self.install("bundle", ("alpha",), "--no-warm-up")
        self.assertEqual(self.records(), [])
        # The graph check is no part of the warm-up.
        self.assert_checked_after_warm_up(
            os.path.realpath(os.path.join(self.prefix, "current")))

    # REQ-OPS-47
    def test_rollback_no_warm_up_runs_nothing(self):
        self.install("old", ("alpha",), "--no-warm-up")
        old = os.readlink(os.path.join(self.prefix, "current"))
        self.install("new", ("alpha",), "--no-warm-up")
        os.remove(self.log)
        self.deploy("rollback", "--no-warm-up")
        self.assertEqual(os.readlink(os.path.join(self.prefix, "current")),
                         old)
        self.assertEqual(self.records(), [])
        self.assertEqual([r[0] for r in self.records(check=True)],
                         ["start", "end"])

    # REQ-OPS-47
    def test_rollback_warms_the_previous_bundle_before_selecting_it(self):
        # `previous` as a deploy.sh that predates the warm-up leaves it.
        nodes = ("alpha", "fails")
        self.install("old", nodes, "--no-warm-up")
        old = os.readlink(os.path.join(self.prefix, "current"))
        self.install("new", nodes, "--no-warm-up")
        new = os.readlink(os.path.join(self.prefix, "current"))
        proc = self.deploy("rollback")
        self.assertEqual(os.readlink(os.path.join(self.prefix, "current")),
                         old)
        self.assert_warmed_in_turn(os.path.realpath(old), nodes, new, proc)

    # REQ-OPS-47: the warm-up's last line is what was warmed. A member that
    # died or failed is counted as not warmed and named with how it ended,
    # not only in a warning above.
    def test_summary_counts_only_members_that_finished(self):
        proc = self.install("bundle", ("alpha", "dies", "fails"))
        summary = [line for line in proc.stdout.splitlines()
                   if line.startswith("deploy: warmed ")]
        self.assertEqual(len(summary), 1, proc.stdout + proc.stderr)
        counted, _, missed = summary[0].partition("; not warmed: ")
        # Of studio_bridge, alpha, dies, fails and the launcher.
        self.assertIn(" 3 of 5 members ", counted, summary)
        self.assertEqual(re.findall(r"(\w+) \(", missed), ["dies", "fails"],
                         summary)

    # REQ-OPS-47
    @unittest.skipUnless(sys.platform == "darwin",
                         "only macOS charges a native library's first load")
    def test_install_loads_each_native_library_once_before_selecting(self):
        src = os.path.join(self.tmp, "bundle")
        so_rel, dylib_rel = native_bundle(src)
        proc = self.deploy("install", src)
        bundle = os.path.realpath(os.path.join(self.prefix, "current"))
        # Both the installed .so and .dylib copies (the charge is per file),
        # loaded once each, while nothing was selected yet.
        marks = [(os.path.realpath(r[1]), r[2])
                 for r in self.records() if r[0] == "native"]
        self.assertEqual(marks, [(os.path.join(bundle, so_rel), "-"),
                                 (os.path.join(bundle, dylib_rel), "-")],
                         proc.stdout + proc.stderr)
        # Counted as loaded, not by what a library prints when loaded
        # (each marking one prints two lines).
        self.assertIn(" and 3 native libraries in ", proc.stdout)

    # REQ-OPS-47: a native-library pass that dies counts only the libraries
    # it loaded and is named as not warmed.
    @unittest.skipUnless(sys.platform == "darwin",
                         "only macOS charges a native library's first load")
    def test_summary_counts_only_native_libraries_loaded(self):
        src = os.path.join(self.tmp, "bundle")
        native_bundle(src)
        # The first marking library's load kills its process before the .dylib.
        self.env["STUB_NATIVE_KILL"] = "1"
        proc = self.deploy("install", src)
        summary = [line for line in proc.stdout.splitlines()
                   if line.startswith("deploy: warmed ")]
        self.assertEqual(len(summary), 1, proc.stdout + proc.stderr)
        counted, _, missed = summary[0].partition("; not warmed: ")
        # broken.so's load returned (failing); the marking .so never did.
        self.assertIn(" 3 members and 1 of 3 native libraries ", counted,
                      summary)
        self.assertTrue(missed.startswith("native libraries ("), summary)

    # REQ-OPS-47
    def test_a_hung_member_is_killed_at_its_bound(self):
        # Every member of `previous` has run once from its installed path,
        # so within the short bound only `hangs` is slow.
        nodes = ("alpha", "hangs", "omega")
        self.install("old", nodes, "--no-warm-up")
        old = os.path.realpath(os.path.join(self.prefix, "current"))
        prewarm = dict(self.env, STUB_PREWARM="1",
                       STUB_LOG=os.path.join(self.tmp, "prewarm.log"))
        for rel in [f"bin/{n}" for n in nodes] + [
                "studio/studio_bridge", "launcher"]:
            subprocess.run([os.path.join(old, rel)], env=prewarm, check=True,
                           timeout=60)
        self.install("new", nodes, "--no-warm-up")
        new = os.readlink(os.path.join(self.prefix, "current"))
        self.env["TALOS_DEPLOY_WARM_UP_TIMEOUT_S"] = "3"
        proc = self.deploy("rollback")
        self.assert_warmed_in_turn(old, nodes, new, proc)

    def stop_install(self, nodes, fifo_var, number, *args):
        """Starts an install of a stub bundle of `nodes`, sends `number` to
        it once the stub that `fifo_var` names says it is running, and
        asserts that deploy.sh exits 128 + `number`, selecting nothing and
        leaving no copy under bundles/ for prune to count."""
        src = os.path.join(self.tmp, "bundle")
        stub_bundle(src, nodes)
        fifo = os.path.join(self.tmp, "running")
        os.mkfifo(fifo)
        # Read and write: the stub's open never blocks, and no EOF wakes us.
        reader = os.open(fifo, os.O_RDWR | os.O_NONBLOCK)
        self.addCleanup(os.close, reader)
        deploy = subprocess.Popen(
            ["bash", deploy_script(), "install", src, "--prefix", self.prefix,
             *args],
            env=dict(self.env, **{fifo_var: fifo}), stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True)
        self.addCleanup(deploy.kill)
        ready, _, _ = select.select([reader], [], [], 60)
        self.assertTrue(ready, "the stub never started")
        deploy.send_signal(number)
        out, err = deploy.communicate(timeout=60)
        self.assertEqual(os.listdir(os.path.join(self.prefix, "bundles")), [],
                         out + err)
        self.assertFalse(os.path.lexists(os.path.join(self.prefix, "current")),
                         out + err)
        self.assertEqual(deploy.returncode, 128 + number, out + err)

    # REQ-OPS-47, REQ-OPS-48: a stop during the warm-up kills the running
    # member and its process group, selects nothing and removes the copy.
    def test_a_stopped_warm_up_kills_its_member_and_removes_the_copy(self):
        self.stop_install(("alpha", "hangs", "omega"), "STUB_FIFO",
                          signal.SIGTERM)
        records = self.records()
        started = [os.path.basename(r[1]) for r in records if r[0] == "start"]
        self.assertEqual(started, ["studio_bridge", "alpha", "hangs"], records)
        hangs_start = [int(r[2]) for r in records
                       if r[0] == "start" and os.path.basename(r[1]) == "hangs"]
        hangs_child = [int(r[2]) for r in records
                       if r[0] == "child" and os.path.basename(r[1]) == "hangs"]
        self.assertEqual(len(hangs_start), 1, records)
        self.assertEqual(len(hangs_child), 1, records)
        self.assertNotEqual(hangs_start[0], hangs_child[0], records)
        self.assert_dead(hangs_start[0], "hung member alive")
        self.assert_dead(hangs_child[0], "hung member's child alive")

    # REQ-OPS-50, REQ-OPS-48: a stop during install's graph check (SIGINT,
    # which bash leaves ignored in a background job) stops the launcher,
    # selects nothing and removes the copy.
    def test_a_stopped_graph_check_stops_it_and_removes_the_copy(self):
        self.stop_install(("alpha",), "STUB_CHECK_FIFO", signal.SIGINT,
                          "--no-warm-up")
        log = self.records(check=True)
        self.assertEqual([r[0] for r in log], ["start"], log)
        with self.assertRaises(ProcessLookupError, msg="graph check alive"):
            os.kill(int(log[0][2]), 0)

    # REQ-OPS-50: rollback selects `previous` only after its own launcher
    # accepts it, as install does; a refused rollback leaves `current` and
    # `previous` as they were.
    def test_rollback_refuses_a_bundle_the_launcher_refuses(self):
        self.install("old", ("alpha",), "--no-warm-up")
        old = os.readlink(os.path.join(self.prefix, "current"))
        self.install("new", ("alpha",), "--no-warm-up")
        new = os.readlink(os.path.join(self.prefix, "current"))
        proc = subprocess.run(
            ["bash", deploy_script(), "rollback", "--prefix", self.prefix,
             "--no-warm-up"],
            capture_output=True, text=True, timeout=60,
            env=dict(self.env, STUB_CHECK_EXIT="4"))
        out = proc.stdout + proc.stderr
        self.assertNotEqual(proc.returncode, 0, out)
        self.assertIn("failed (exit 4)", out)
        self.assertEqual(os.readlink(os.path.join(self.prefix, "current")),
                         new, out)
        self.assertEqual(os.readlink(os.path.join(self.prefix, "previous")),
                         old, out)
        # The check ran on `previous`, from its installed path.
        log = self.records(check=True)
        self.assertEqual(log[-2][:2],
                         ["start", os.path.join(os.path.realpath(old),
                                                "run.sh")], log)


if __name__ == "__main__":
    unittest.main()
