"""Tests for tools/format.sh against a throwaway git repository.

REQ-DEV-36, REQ-DEV-25: the local pre-push checks find what CI finds. CI formats every
C/C++ file the pushed commits add, so a new file not yet `git add`-ed must
not pass the local `format.sh --check` and then fail CI's.

The test proves format.sh's file selection, not clang-format's behaviour,
so the formatter is a stub placed in format.sh's private cache
(`$XDG_CACHE_HOME/talos/clang-18`): `bazel test //...` runs on hosts with no
clang-format 18, and installing the wheels needs the network.
"""

import os
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest

from python.runfiles import runfiles

BAD = "int f();  // MARKER_BAD\n"
GOOD = "int g() { return 2; }\n"


def put(path, text):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


# Stands in for clang-format. tools/clang18.sh's clang18_setup checks only
# that `--version` prints "version 18."; format.sh's check loop runs
# `clang-format-18 --dry-run --Werror "$f"` once per file, discards the
# output and reads the exit status (nonzero: needs formatting). The stub
# records each file it is given and fails a file containing the marker.
STUB = """#!/bin/sh
if [ "$1" = --version ]; then echo "clang-format version 18.1.8"; exit 0; fi
for f; do :; done
echo "$f" >> "$STUB_LOG"
if grep -q MARKER_BAD "$f"; then exit 1; fi
exit 0
"""


class FormatTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = os.path.realpath(tmp.name)
        cache = os.path.join(self.tmp, "cache", "talos", "clang-18")
        os.makedirs(os.path.join(cache, "venv", "bin"))
        for tool, text in (("clang-format", STUB),
                           ("clang-tidy", "#!/bin/sh\nexit 0\n")):
            path = os.path.join(cache, "venv", "bin", tool)
            put(path, text)
            os.chmod(path, 0o755)
        self.stub_log = os.path.join(self.tmp, "formatted")
        put(os.path.join(cache, "version"), "18.1.8\n")

        self.repo = os.path.join(self.tmp, "repo")
        os.makedirs(os.path.join(self.repo, "tools"))
        for f in ("format.sh", "clang18.sh"):
            shutil.copy(runfiles.Create().Rlocation(f"_main/tools/{f}"),
                        os.path.join(self.repo, "tools", f))
        self.env = {k: v for k, v in os.environ.items()
                    if not k.startswith("GIT_")}
        self.env.update(
            HOME=self.tmp, XDG_CACHE_HOME=os.path.join(self.tmp, "cache"),
            GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM="1",
            GIT_AUTHOR_NAME="t", GIT_AUTHOR_EMAIL="t@example.com",
            GIT_COMMITTER_NAME="t", GIT_COMMITTER_EMAIL="t@example.com",
            STUB_LOG=self.stub_log)

    def git(self, *args):
        return subprocess.run(["git", *args], cwd=self.repo, env=self.env,
                              check=True, capture_output=True,
                              text=True).stdout.strip()

    def test_check_names_changed_and_untracked_sources_only(self):
        self.git("init", "-q", "-b", "main")
        put(os.path.join(self.repo, ".clang-format"), "BasedOnStyle: Google\n")
        put(os.path.join(self.repo, ".gitignore"), "ignored/\n")
        put(os.path.join(self.repo, "tracked.cc"), GOOD)
        self.git("add", "-A")
        self.git("commit", "-q", "-m", "base")
        base = self.git("rev-parse", "HEAD")

        put(os.path.join(self.repo, "tracked.cc"), BAD)          # (a)
        put(os.path.join(self.repo, "new", "bad.cc"), BAD)       # (b)
        put(os.path.join(self.repo, "new", "bad.h"), BAD)        # (b)
        put(os.path.join(self.repo, "new", "good.cc"), GOOD)     # (c)
        put(os.path.join(self.repo, "ignored", "bad.cc"), BAD)   # (d)
        put(os.path.join(self.repo, "new", "notes.txt"), BAD)    # (e)

        r = subprocess.run(["bash", "tools/format.sh", "--check", "--base",
                            base], cwd=self.repo, env=self.env,
                           capture_output=True, text=True, check=False)
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 1, out)
        listed = sorted(line.strip() for line in r.stdout.splitlines()
                        if line.startswith("  "))
        self.assertEqual(listed, ["new/bad.cc", "new/bad.h", "tracked.cc"],
                         out)
        with open(self.stub_log, encoding="utf-8") as f:
            given = sorted(f.read().split())
        self.assertEqual(
            given, ["new/bad.cc", "new/bad.h", "new/good.cc", "tracked.cc"])

    # REQ-OPS-33: failing git diff / git ls-files must fail format.sh (not
    # silently report "no C/C++ files to check") and clean up its temp file.
    def test_fails_when_git_listing_fails_and_cleans_up_temp_file(self):
        self.git("init", "-q", "-b", "main")
        put(os.path.join(self.repo, "tracked.cc"), GOOD)
        self.git("add", "-A")
        self.git("commit", "-q", "-m", "base")
        base = self.git("rev-parse", "HEAD")

        real_git = shutil.which("git")
        self.assertIsNotNone(real_git)
        bin_dir = os.path.join(self.tmp, "bin")
        git_stub = os.path.join(bin_dir, "git")
        put(git_stub,
            "#!/bin/sh\n"
            'if [ "${1:-}" = "${STUB_FAIL_GIT:-}" ]; then\n'
            '  echo "fatal: simulated git $1 failure" >&2\n'
            '  exit 128\n'
            'fi\n'
            f'exec "{real_git}" "$@"\n')
        os.chmod(git_stub, 0o755)

        fmt_tmp = os.path.join(self.tmp, "fmt_tmp")
        os.makedirs(fmt_tmp)
        env = dict(self.env,
                   PATH=bin_dir + os.pathsep + self.env.get("PATH", ""),
                   TMPDIR=fmt_tmp)

        for args, fail_subcmd in (
                (["--check", "--all"], "ls-files"),
                (["--check", "--base", base], "diff"),
                (["--check", "--base", base], "ls-files")):
            with self.subTest(args=args, fail_subcmd=fail_subcmd):
                r = subprocess.run(
                    ["bash", "tools/format.sh", *args],
                    cwd=self.repo,
                    env=dict(env, STUB_FAIL_GIT=fail_subcmd),
                    capture_output=True, text=True, check=False)
                self.assertNotEqual(r.returncode, 0, r.stdout + r.stderr)
                self.assertIn(f"simulated git {fail_subcmd} failure", r.stderr)
                self.assertEqual(os.listdir(fmt_tmp), [])

    # REQ-DEV-36, REQ-DEV-25: invoking format.sh by relative path from a
    # subdirectory (even with CDPATH set) still resolves sibling clang18.sh.
    def test_relative_invocation_from_subdirectory(self):
        self.git("init", "-q", "-b", "main")
        put(os.path.join(self.repo, "sub", "ok.cc"), GOOD)
        self.git("add", "-A")
        self.git("commit", "-q", "-m", "base")
        base = self.git("rev-parse", "HEAD")

        put(os.path.join(self.repo, "sub", "ok.cc"), GOOD + "\n")
        env = dict(self.env, CDPATH=self.tmp)
        r = subprocess.run(
            ["bash", "../tools/format.sh", "--check", "--base", base],
            cwd=os.path.join(self.repo, "sub"),
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("1 file(s) clean", r.stdout)

    # REQ-DEV-36, REQ-DEV-25: concurrent format.sh runs on a cold or
    # version-bumped cache serialize install via lock + staging atomic swap and
    # rewrite staged console-script shebangs to the final cache path.
    def test_concurrent_cold_and_version_bump_setup_is_serialized(self):
        self.git("init", "-q", "-b", "main")
        put(os.path.join(self.repo, "tracked.cc"), GOOD)
        self.git("add", "-A")
        self.git("commit", "-q", "-m", "base")
        base = self.git("rev-parse", "HEAD")

        cache_root = os.path.join(self.tmp, "cold_cache")
        uv_log = os.path.join(self.tmp, "uv_installs")
        bin_dir = os.path.join(self.tmp, "uv_bin")
        uv_stub = os.path.join(bin_dir, "uv")
        put(
            uv_stub,
            "#!/bin/sh\n"
            'if [ "$1" = venv ]; then\n'
            '  venv_dir="$3"\n'
            '  mkdir -p "$venv_dir/bin"\n'
            f'  ln -sf {shlex.quote(sys.executable)} "$venv_dir/bin/python"\n'
            '  exit 0\n'
            'fi\n'
            'if [ "$1" = pip ] && [ "$2" = install ]; then\n'
            '  py_path="$5"\n'
            '  bin_path="$(dirname "$py_path")"\n'
            '  echo "install" >> "$UV_LOG"\n'
            '  sleep 0.2\n'
            '  cat > "$bin_path/clang-format" <<EOF\n'
            '#!/bin/sh\n'
            '# VENV_PY=$py_path\n'
            'if [ "\\$1" = --version ]; then echo "clang-format version 18.1.8"; exit 0; fi\n'
            'exit 0\n'
            'EOF\n'
            '  printf "#!/bin/sh\\n# VENV_PY=%s\\nexit 0\\n" "$py_path" > "$bin_path/clang-tidy"\n'
            '  chmod 755 "$bin_path/clang-format" "$bin_path/clang-tidy"\n'
            '  exit 0\n'
            'fi\n'
            'exit 1\n',
        )
        os.chmod(uv_stub, 0o755)

        env = dict(
            self.env,
            XDG_CACHE_HOME=cache_root,
            PATH=bin_dir + os.pathsep + self.env.get("PATH", ""),
            UV_LOG=uv_log,
        )
        home = os.path.join(cache_root, "talos", "clang-18")

        for label in ("cold", "version_bump"):
            with self.subTest(phase=label):
                if label == "version_bump":
                    put(os.path.join(home, "version"), "18.0.0\n")
                    os.remove(uv_log)
                procs = [
                    subprocess.Popen(
                        ["bash", "tools/format.sh", "--check", "--base", base],
                        cwd=self.repo,
                        env=env,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True,
                    )
                    for _ in range(2)
                ]
                results = [p.communicate(timeout=30) for p in procs]
                for p, (out, err) in zip(procs, results):
                    self.assertEqual(p.returncode, 0, out + err)
                with open(uv_log, encoding="utf-8") as f:
                    installs = f.read().splitlines()
                self.assertEqual(len(installs), 1, installs)
                with open(os.path.join(home, "version"), encoding="utf-8") as f:
                    self.assertEqual(f.read().strip(), "18.1.8")
                with open(os.path.join(home, "venv", "bin", "clang-format"),
                          encoding="utf-8") as f:
                    cf_text = f.read()
                self.assertIn(os.path.join(home, "venv", "bin", "python"),
                              cf_text)
                self.assertNotIn(".staging.", cf_text)


if __name__ == "__main__":
    unittest.main()
