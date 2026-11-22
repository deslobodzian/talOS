"""Tests for tools/setup.sh syntax and --check component detection.

REQ-DEV-12: tools/setup.sh installs a working dev environment on Linux, macOS
and Windows inside WSL2.

Runs tools/setup.sh from a scratch repository copy against a controlled PATH
containing only minimal POSIX utilities plus per-tool stubs, so every passing,
missing, wrong-version and --no-{studio,rio,lint} path is exercised without
installing anything or depending on host-installed dev tools.
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

from python.runfiles import runfiles

IS_MACOS = sys.platform == "darwin"

# Basic POSIX utilities setup.sh invokes while running --check.
POSIX_TOOLS = (
    "basename",
    "bash",
    "cat",
    "dirname",
    "grep",
    "readlink",
    "sed",
    "tr",
    "uname",
) + (("sw_vers",) if IS_MACOS else ())


def put(path, text, mode=None):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    if mode is not None:
        os.chmod(path, mode)


class SetupShTest(unittest.TestCase):
    def setUp(self):
        r = runfiles.Create()
        self.setup_src = r.Rlocation("_main/tools/setup.sh")
        self.clang18_src = r.Rlocation("_main/tools/clang18.sh")

        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = os.path.realpath(tmp.name)

        self.repo = os.path.join(self.tmp, "repo")
        os.makedirs(os.path.join(self.repo, "tools"))
        shutil.copy(self.setup_src, os.path.join(self.repo, "tools", "setup.sh"))
        shutil.copy(self.clang18_src,
                    os.path.join(self.repo, "tools", "clang18.sh"))
        put(os.path.join(self.repo, ".bazelversion"), "8.6.0\n")

        self.home = os.path.join(self.tmp, "home")
        self.cache = os.path.join(self.tmp, "cache")
        self.gradle = os.path.join(self.tmp, "gradle")
        self.bin_dir = os.path.join(self.tmp, "bin")
        self.sys_bin = os.path.join(self.tmp, "sys_bin")
        for d in (self.home, self.cache, self.gradle, self.bin_dir,
                  self.sys_bin):
            os.makedirs(d, exist_ok=True)

        for tool in POSIX_TOOLS:
            real = shutil.which(tool)
            self.assertIsNotNone(real, f"required POSIX tool missing: {tool}")
            os.symlink(real, os.path.join(self.sys_bin, tool))

        self.fake_jdk = os.path.join(self.tmp, "jdk17")
        self.clt_dir = os.path.join(self.tmp, "clt")
        os.makedirs(self.clt_dir, exist_ok=True)
        self._install_passing_environment()

    def _stub(self, name, body):
        path = os.path.join(self.bin_dir, name)
        put(path, "#!/bin/sh\n" + body, 0o755)
        return path

    def _remove_stub(self, name):
        path = os.path.join(self.bin_dir, name)
        if os.path.exists(path):
            os.remove(path)

    def _install_passing_environment(self):
        if IS_MACOS:
            self._stub("xcode-select", f'echo "{self.clt_dir}"\n')
            self._stub(
                "brew",
                'if [ "${1:-}" = "--version" ]; then echo "Homebrew 4.4.0"; '
                'exit 0; fi\n'
                'if [ "${1:-}" = "--prefix" ]; then echo "/nonexistent"; '
                'exit 0; fi\n'
                'exit 0\n',
            )
        else:
            self._stub("git", 'echo "git version 2.45.0"\n')
            self._stub("curl", "exit 0\n")
            self._stub("unzip", "exit 0\n")
            self._stub("zip", "exit 0\n")
            self._stub("g++", 'echo "14.2.0"\n')

        self._stub(
            "python3",
            'case "$*" in\n'
            '  *version_info\\[:3\\]*) echo "3.11.9"; exit 0 ;;\n'
            '  *sys.exit*) exit 0 ;;\n'
            'esac\n'
            'exit 0\n',
        )
        # is_bazelisk matches BAZELISK_SKIP_WRAPPER inside the executable.
        self._stub("bazel", "# BAZELISK_SKIP_WRAPPER\nexit 0\n")
        self._stub("node", 'echo "v22.14.0"\n')
        self._stub("pnpm", 'echo "9.15.9"\n')
        self._stub("buildifier", 'echo "buildifier version: 8.2.1"\n')

        put(
            os.path.join(self.fake_jdk, "bin", "java"),
            '#!/bin/sh\necho \'openjdk version "17.0.12"\' >&2\n',
            0o755,
        )
        put(os.path.join(self.fake_jdk, "bin", "javac"), "#!/bin/sh\nexit 0\n",
            0o755)

        self.rio_gxx = os.path.join(
            self.gradle, "toolchains", "frc", "2025", "roborio", "bin",
            "arm-frc2025-linux-gnueabi-g++",
        )
        put(self.rio_gxx, "#!/bin/sh\nexit 0\n", 0o755)

        self.clang_home = os.path.join(self.cache, "talos", "clang-18")
        for tool in ("clang-format", "clang-tidy"):
            put(
                os.path.join(self.clang_home, "venv", "bin", tool),
                "#!/bin/sh\nexit 0\n",
                0o755,
            )
        put(os.path.join(self.clang_home, "version"), "18.1.8\n")

    def _run_setup(self, *args, java_home=None):
        env = {
            "HOME": self.home,
            "XDG_CACHE_HOME": self.cache,
            "GRADLE_USER_HOME": self.gradle,
            "PATH": self.bin_dir + os.pathsep + self.sys_bin,
        }
        if java_home is not None:
            env["JAVA_HOME"] = java_home
        elif self.fake_jdk is not None:
            env["JAVA_HOME"] = self.fake_jdk
        return subprocess.run(
            ["bash", os.path.join(self.repo, "tools", "setup.sh"), *args],
            cwd=self.repo,
            env=env,
            capture_output=True,
            text=True,
            check=False,
            timeout=30,
        )

    # REQ-DEV-12
    def test_bash_syntax_is_valid(self):
        r = subprocess.run(
            ["bash", "-n", os.path.join(self.repo, "tools", "setup.sh")],
            capture_output=True,
            text=True,
            check=False,
            timeout=15,
        )
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)

    # REQ-DEV-12
    def test_check_passes_when_all_components_match(self):
        r = self._run_setup("--check")
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 0, out)
        self.assertIn("check only", out)
        for name in ("python3", "bazel (bazelisk)", "node 22", "pnpm 9.15.9",
                     "jdk 17", "roborio toolchain", "clang-format/tidy 18",
                     "buildifier"):
            self.assertRegex(out, rf"(?m)^{re.escape(name)}\s+ok\b", out)

    # REQ-DEV-12
    def test_check_reports_missing_tools_and_exits_nonzero(self):
        self._remove_stub("bazel")
        self._remove_stub("pnpm")
        self._remove_stub("buildifier")
        os.remove(self.rio_gxx)
        os.remove(os.path.join(self.clang_home, "version"))
        if IS_MACOS:
            self._stub("xcode-select", "exit 1\n")
        else:
            self._remove_stub("zip")
            self._remove_stub("g++")

        r = self._run_setup("--check")
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 1, out)
        self.assertRegex(out, r"(?m)^bazel \(bazelisk\)\s+missing\b", out)
        self.assertRegex(out, r"(?m)^pnpm 9\.15\.9\s+missing\b", out)
        self.assertRegex(out, r"(?m)^roborio toolchain\s+missing\b", out)
        self.assertRegex(out, r"(?m)^clang-format/tidy 18\s+missing\b", out)
        self.assertRegex(out, r"(?m)^buildifier\s+missing\b", out)
        if IS_MACOS:
            self.assertRegex(out, r"(?m)^xcode clt\s+missing\b", out)
        else:
            self.assertRegex(out, r"(?m)^git/curl/unzip/zip\s+missing\b", out)
            self.assertRegex(out, r"(?m)^c\+\+ compiler\s+missing\b", out)

    # REQ-DEV-12
    def test_check_reports_wrong_version_tools_and_exits_nonzero(self):
        self._stub(
            "python3",
            'case "$*" in\n'
            '  *version_info\\[:3\\]*) echo "3.7.9"; exit 0 ;;\n'
            '  *sys.exit*) exit 1 ;;\n'
            'esac\n'
            'exit 0\n',
        )
        # Plain bazel without BAZELISK_SKIP_WRAPPER marker.
        self._stub("bazel", 'echo "bazel 8.6.0"\n')
        self._stub("node", 'echo "v20.11.0"\n')
        self._stub("pnpm", 'echo "8.15.0"\n')
        self._stub("buildifier", 'echo "buildifier version: 7.0.0"\n')
        if not IS_MACOS:
            self._stub("g++", 'echo "12.3.0"\n')

        r = self._run_setup("--check")
        out = r.stdout + r.stderr
        self.assertEqual(r.returncode, 1, out)
        self.assertRegex(out, r"(?m)^python3\s+wrong-version\b", out)
        self.assertRegex(out, r"(?m)^bazel \(bazelisk\)\s+wrong-version\b", out)
        self.assertRegex(out, r"(?m)^node 22\s+wrong-version\b", out)
        self.assertRegex(out, r"(?m)^pnpm 9\.15\.9\s+wrong-version\b", out)
        if IS_MACOS:
            self.assertRegex(out, r"(?m)^buildifier\s+warn\b", out)
        else:
            self.assertRegex(out, r"(?m)^buildifier\s+wrong-version\b", out)
            self.assertRegex(out, r"(?m)^c\+\+ compiler\s+wrong-version\b", out)

    # REQ-DEV-12
    def test_no_studio_no_rio_no_lint_skip_optional_groups(self):
        self._remove_stub("node")
        self._remove_stub("pnpm")
        self._remove_stub("buildifier")
        os.remove(self.rio_gxx)
        os.remove(os.path.join(self.clang_home, "version"))

        # Without the skip flags, --check fails.
        failed = self._run_setup("--check", java_home="")
        self.assertEqual(failed.returncode, 1, failed.stdout + failed.stderr)

        # With --no-studio, --no-rio, --no-lint, all three groups are skipped
        # and --check exits 0.
        skipped = self._run_setup(
            "--check", "--no-studio", "--no-rio", "--no-lint", java_home=""
        )
        out = skipped.stdout + skipped.stderr
        self.assertEqual(skipped.returncode, 0, out)
        self.assertRegex(out, r"(?m)^studio node/pnpm\s+skipped\s+--no-studio\b",
                         out)
        self.assertRegex(out, r"(?m)^rio deps\s+skipped\s+--no-rio\b", out)
        self.assertRegex(out, r"(?m)^lint tools\s+skipped\s+--no-lint\b", out)

        # Each individual skip flag skips only its own group.
        only_studio = self._run_setup("--check", "--no-studio")
        self.assertEqual(only_studio.returncode, 1,
                         only_studio.stdout + only_studio.stderr)
        self.assertRegex(only_studio.stdout,
                         r"(?m)^studio node/pnpm\s+skipped\s+--no-studio\b")
        self.assertRegex(only_studio.stdout,
                         r"(?m)^roborio toolchain\s+missing\b")
        self.assertRegex(only_studio.stdout, r"(?m)^buildifier\s+missing\b")

    # REQ-DEV-12
    def test_unknown_argument_is_rejected(self):
        r = self._run_setup("--check", "--no-such-flag")
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn("unknown argument: --no-such-flag", r.stderr)


if __name__ == "__main__":
    unittest.main()
