"""Tests for tools/format.sh against a throwaway git repository.

REQ-DEV-25: the local pre-push checks find what CI finds. CI formats every
C/C++ file the pushed commits add, so a new file not yet `git add`-ed must
not pass the local `format.sh --check` and then fail CI's.

The test proves format.sh's file selection, not clang-format's behaviour,
so the formatter is a stub placed in format.sh's private cache
(`$XDG_CACHE_HOME/talos/clang-18`): `bazel test //...` runs on hosts with no
clang-format 18, and installing the wheels needs the network.
"""

import os
import shutil
import subprocess
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


if __name__ == "__main__":
    unittest.main()
