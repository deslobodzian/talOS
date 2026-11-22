"""Tests for tools/tidy.sh Linux GCC 14 libstdc++ fallback against a throwaway git repo.

REQ-DEV-25: local pre-push checks run clang-tidy-18 on Linux x86_64 hosts whose
system libstdc++ (such as Arch's GCC 16) clang 18 cannot parse, by using a
cached pinned Debian GCC 14 sysroot with --gcc-install-dir, while never
entering the fallback when the native standard library parses (Ubuntu 24.04 CI)
and preserving exit 3 (`tidy NOT RUN`) on offline, checksum-mismatch, or
unsupported-architecture hosts.
"""

import json
import os
import re
import shutil
import subprocess
import tempfile
import unittest

from python.runfiles import runfiles


def put(path, text, mode=None):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    if mode is not None:
        os.chmod(path, mode)


FORMAT_STUB = """#!/bin/sh
if [ "$1" = --version ]; then echo "clang-format version 18.1.8"; exit 0; fi
exit 0
"""

# Stub clang-tidy that records invocations and optionally simulates a host
# libstdc++ error unless --gcc-install-dir=<dir> (or --extra-arg=--gcc-install-dir=<dir>)
# is supplied.
TIDY_STUB = """#!/bin/sh
if [ "$1" = --version ]; then echo "LLVM version 18.1.8"; exit 0; fi
printf '%s\\n' "$*" >> "$TIDY_LOG"
has_gcc_dir=0
isysroot=""
prev=""
for arg in "$@"; do
  if [ "$prev" = "-isysroot" ]; then isysroot="$arg"; fi
  if [ "$prev" = "--extra-arg=-isysroot" ]; then isysroot="${arg#--extra-arg=}"; fi
  case "$arg" in
    --gcc-install-dir=*|--extra-arg=--gcc-install-dir=*) has_gcc_dir=1 ;;
  esac
  prev="$arg"
done
if [ "${STUB_PROBE_SELF_ERROR:-0}" = "1" ]; then
  for arg in "$@"; do
    case "$arg" in
      */probe.cc)
        echo "$arg:1:1: error: simulated probe error" >&2
        exit 1
        ;;
    esac
  done
fi
if [ -n "${STUB_DARWIN_SDK_OK:-}" ]; then
  if [ "$isysroot" != "$STUB_DARWIN_SDK_OK" ]; then
    echo "${isysroot:-/Applications/Xcode 16.app/SDKs/MacOSX16.0.sdk}/usr/include/c++/v1/atomic:10:5: error: use of undeclared identifier '__builtin_clzg'" >&2
    exit 1
  fi
fi
if [ "${STUB_NATIVE_FAIL:-0}" = "1" ] && [ "$has_gcc_dir" = "0" ]; then
  echo "/usr/include/c++/16/bits/atomic_wait.h:55:24: error: use of undeclared identifier '__builtin_popcountg'" >&2
  exit 1
fi
if [ "${STUB_FALLBACK_FAIL:-0}" = "1" ] && [ "$has_gcc_dir" = "1" ]; then
  echo "/fallback/include/c++/14/atomic:1:1: error: simulated fallback parse failure" >&2
  exit 1
fi
for arg in "$@"; do
  case "$arg" in
    *.cc)
      if [ -f "$arg" ] && grep -q "MARKER_BAD" "$arg"; then
        echo "$arg:1:1: warning: simulated finding [readability-braces-around-statements]"
        exit 1
      fi
      ;;
  esac
done
exit 0
"""

UNAME_STUB = """#!/bin/sh
case "$1" in
  -s) echo "${STUB_OS:-Linux}" ;;
  -m) echo "${STUB_ARCH:-x86_64}" ;;
  *) /usr/bin/uname "$@" ;;
esac
"""

CURL_STUB = """#!/bin/sh
printf '%s\\n' "$*" >> "$CURL_LOG"
if [ "${STUB_CURL_MODE:-offline}" = "offline" ]; then
  echo "curl: (6) Could not resolve host: snapshot.debian.org" >&2
  exit 6
fi
out=""
while [ $# -gt 0 ]; do
  if [ "$1" = "-o" ]; then out="$2"; shift 2; continue; fi
  shift
done
printf 'corrupt-deb-bytes\\n' > "$out"
exit 0
"""


class TidyFallbackTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = os.path.realpath(tmp.name)

        self.cache_dir = os.path.join(self.tmp, "cache")
        clang_cache = os.path.join(self.cache_dir, "talos", "clang-18")
        os.makedirs(os.path.join(clang_cache, "venv", "bin"))
        put(os.path.join(clang_cache, "venv", "bin", "clang-format"),
            FORMAT_STUB, 0o755)
        put(os.path.join(clang_cache, "venv", "bin", "clang-tidy"),
            TIDY_STUB, 0o755)
        put(os.path.join(clang_cache, "version"), "18.1.8\n")

        self.bin_dir = os.path.join(self.tmp, "bin")
        put(os.path.join(self.bin_dir, "uname"), UNAME_STUB, 0o755)
        put(os.path.join(self.bin_dir, "curl"), CURL_STUB, 0o755)

        self.tidy_log = os.path.join(self.tmp, "tidy.log")
        self.curl_log = os.path.join(self.tmp, "curl.log")

        self.repo = os.path.join(self.tmp, "repo")
        os.makedirs(os.path.join(self.repo, "tools"))
        rf = runfiles.Create()
        for f in ("tidy.sh", "clang18.sh", "run_clang_tidy.py"):
            shutil.copy(rf.Rlocation(f"_main/tools/{f}"),
                        os.path.join(self.repo, "tools", f))

        # Read the pinned version and SHA-256 fingerprint from tidy.sh so the
        # test stays locked to the script's constants.
        with open(os.path.join(self.repo, "tools", "tidy.sh"),
                  encoding="utf-8") as f:
            script = f.read()
        self.deb_version = re.search(r'^GCC14_DEB_VERSION="([^"]+)"',
                                     script, re.M).group(1)
        libstdcxx_sha = re.search(r'^GCC14_LIBSTDCXX_SHA256="([^"]+)"',
                                  script, re.M).group(1)
        libgcc_sha = re.search(r'^GCC14_LIBGCC_SHA256="([^"]+)"',
                               script, re.M).group(1)
        self.fingerprint = f"{self.deb_version}:{libstdcxx_sha}:{libgcc_sha}"
        self.gcc14_root = os.path.join(
            self.cache_dir, "talos", f"gcc-14-sysroot-{self.deb_version}")
        self.gcc_install_dir = os.path.join(
            self.gcc14_root, "usr", "lib", "gcc", "x86_64-linux-gnu", "14")

        self.env = {k: v for k, v in os.environ.items()
                    if not k.startswith("GIT_")}
        self.env.update(
            HOME=self.tmp,
            XDG_CACHE_HOME=self.cache_dir,
            PATH=f"{self.bin_dir}:{self.env.get('PATH', '')}",
            GIT_CONFIG_GLOBAL=os.devnull,
            GIT_CONFIG_NOSYSTEM="1",
            GIT_AUTHOR_NAME="t",
            GIT_AUTHOR_EMAIL="t@example.com",
            GIT_COMMITTER_NAME="t",
            GIT_COMMITTER_EMAIL="t@example.com",
            TIDY_LOG=self.tidy_log,
            CURL_LOG=self.curl_log,
        )

        self.git("init", "-q", "-b", "main")
        put(os.path.join(self.repo, ".bazelignore"), "")
        put(os.path.join(self.repo, "foo.cc"), "int f() { return 1; }\n")
        put(os.path.join(self.repo, "compile_commands.json"), json.dumps([{
            "directory": self.repo,
            "file": "foo.cc",
            "command": "g++ -std=c++23 -c foo.cc",
        }]))
        self.git("add", "-A")
        self.git("commit", "-q", "-m", "base")
        self.base = self.git("rev-parse", "HEAD")

        put(os.path.join(self.repo, "foo.cc"), "int f() { return 2; }\n")
        self.git("add", "foo.cc")
        self.git("commit", "-q", "-m", "change")

    def git(self, *args):
        return subprocess.run(
            ["git", *args], cwd=self.repo, env=self.env,
            check=True, capture_output=True, text=True).stdout.strip()

    def run_tidy(self, **extra_env):
        env = dict(self.env)
        env.update(extra_env)
        return subprocess.run(
            ["bash", "tools/tidy.sh", "--base", self.base],
            cwd=self.repo, env=env, capture_output=True, text=True,
            check=False)

    def populate_valid_cache(self):
        os.makedirs(self.gcc_install_dir, exist_ok=True)
        put(os.path.join(self.gcc14_root, ".ready"), self.fingerprint + "\n")

    # REQ-DEV-25: Ubuntu 24.04 CI parses natively and never enters fallback.
    def test_native_probe_ok_skips_fallback(self):
        r = self.run_tidy(STUB_NATIVE_FAIL="0")
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 0, out)
        self.assertFalse(os.path.exists(self.curl_log), out)
        self.assertFalse(os.path.exists(self.gcc14_root), out)
        with open(self.tidy_log, encoding="utf-8") as f:
            calls = f.read()
        self.assertNotIn("--gcc-install-dir", calls)

    # REQ-DEV-25: offline Linux x86_64 host without cache exits 3 cleanly.
    def test_offline_fallback_exits_3_with_reason_and_no_partial_cache(self):
        r = self.run_tidy(STUB_NATIVE_FAIL="1", STUB_CURL_MODE="offline")
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 3, out)
        self.assertIn("tidy: cannot run here:", out)
        self.assertIn("failed to download", out)
        self.assertFalse(os.path.exists(self.gcc14_root), out)
        talos_cache = os.path.join(self.cache_dir, "talos")
        staging = [p for p in os.listdir(talos_cache) if "staging" in p]
        self.assertEqual(staging, [], staging)

    # REQ-DEV-25: SHA-256 mismatch aborts before extraction and exits 3.
    def test_checksum_mismatch_exits_3_and_leaves_no_cache(self):
        r = self.run_tidy(STUB_NATIVE_FAIL="1", STUB_CURL_MODE="corrupt")
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 3, out)
        self.assertIn("tidy: cannot run here:", out)
        self.assertIn("SHA-256 mismatch", out)
        self.assertFalse(os.path.exists(self.gcc14_root), out)

    # REQ-DEV-25: valid cached sysroot is reused offline, wraps clang-tidy-18,
    # passes clean files and catches real findings (exit 1).
    def test_cached_sysroot_wraps_clang_tidy_and_reports_findings(self):
        self.populate_valid_cache()
        r = self.run_tidy(STUB_NATIVE_FAIL="1", STUB_CURL_MODE="offline")
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 0, out)
        self.assertFalse(os.path.exists(self.curl_log), out)
        with open(self.tidy_log, encoding="utf-8") as f:
            calls = f.read()
        self.assertIn(f"--extra-arg=--gcc-install-dir={self.gcc_install_dir}",
                      calls)

        # Commit a violation and confirm tidy.sh catches it (exit 1).
        put(os.path.join(self.repo, "foo.cc"),
            "int f() { return 3; } // MARKER_BAD\n")
        self.git("add", "foo.cc")
        self.git("commit", "-q", "-m", "bad")
        r = self.run_tidy(STUB_NATIVE_FAIL="1", STUB_CURL_MODE="offline")
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 1, out)
        self.assertIn("simulated finding", out)

    # REQ-DEV-25: incomplete cache without .ready marker is never trusted.
    def test_incomplete_cache_without_marker_is_rejected(self):
        os.makedirs(self.gcc_install_dir, exist_ok=True)
        r = self.run_tidy(STUB_NATIVE_FAIL="1", STUB_CURL_MODE="offline")
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 3, out)
        self.assertTrue(os.path.exists(self.curl_log), out)

    # REQ-DEV-25: non-x86_64 Linux skips amd64 deb download and exits 3.
    def test_non_x86_64_linux_exits_3_without_fetching(self):
        r = self.run_tidy(STUB_NATIVE_FAIL="1", STUB_ARCH="aarch64")
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 3, out)
        self.assertIn("tidy: cannot run here:", out)
        self.assertFalse(os.path.exists(self.curl_log), out)

    # REQ-OPS-33: macOS SDK paths with spaces are not word-split in probe/loop/
    # diagnostics, and $tmp/probe.cc: is matched as a literal prefix.
    def test_darwin_spaced_sdk_paths_and_literal_probe_prefix(self):
        dev = os.path.join(self.tmp, "Xcode 16.app", "Contents", "Developer")
        sdks_dir = os.path.join(
            dev, "Platforms", "MacOSX.platform", "Developer", "SDKs")
        sdk_16 = os.path.join(sdks_dir, "MacOSX16.0.sdk")
        sdk_15 = os.path.join(sdks_dir, "MacOSX15.2.sdk")
        os.makedirs(sdk_16)
        os.makedirs(sdk_15)
        put(os.path.join(self.bin_dir, "xcrun"),
            f'#!/bin/sh\nprintf "%s\\n" "{sdk_16}"\n', 0o755)
        put(os.path.join(self.bin_dir, "xcode-select"),
            f'#!/bin/sh\nprintf "%s\\n" "{dev}"\n', 0o755)

        special_tmp = os.path.join(self.tmp, "t.m+p dir")
        os.makedirs(special_tmp)

        # Fallback picks the spaced MacOSX15.2.sdk cleanly.
        r = self.run_tidy(
            STUB_OS="Darwin",
            STUB_DARWIN_SDK_OK=sdk_15,
            TMPDIR=special_tmp,
        )
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 0, out)
        self.assertIn(f"using {sdk_15}", out)
        with open(self.tidy_log, encoding="utf-8") as f:
            calls = f.read()
        self.assertIn(f"--extra-arg=-isysroot --extra-arg={sdk_15}", calls)

        # When every spaced SDK fails, each tried path stays intact on its line.
        r = self.run_tidy(
            STUB_OS="Darwin",
            STUB_DARWIN_SDK_OK="/nonexistent/sdk",
            TMPDIR=special_tmp,
        )
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 3, out)
        lines = r.stderr.splitlines()
        self.assertIn(f"  {sdk_16}", lines, out)
        self.assertIn(f"  {sdk_15}", lines, out)

        # A non-"file not found" error in $tmp/probe.cc with regex metachars in
        # TMPDIR still triggers hard probe failure (exit 2), not exit 3.
        r = self.run_tidy(
            STUB_OS="Linux",
            STUB_PROBE_SELF_ERROR="1",
            TMPDIR=special_tmp,
        )
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 2, out)
        self.assertIn("probe failed for another reason", out)


if __name__ == "__main__":
    unittest.main()
