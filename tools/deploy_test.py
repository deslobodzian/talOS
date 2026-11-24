"""Unit tests for tools/deploy.sh argument validation, prefix checks, and rollback.

REQ-OPS-50, REQ-OPS-54: deploy.sh rejects prefixes that would trigger systemd
specifier/variable expansion (% or $), rejects unexpected trailing arguments
in enable-boot/install/rollback, snapshots `previous` before rollback's
warm-up and graph check, and prints the target `--dry-run` rollback would
select.
"""

import os
import shlex
import shutil
import stat
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

    def run_deploy(self, *args, env=None):
        return subprocess.run(
            ["bash", self.deploy, *args],
            env=env,
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
            "/tmp/a;b",
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

    def _make_enable_boot_env(self, os_name="Linux", launchctl_body=None):
        bin_dir = os.path.join(self.tmp, f"stub_bin_{os_name}")
        fake_root = os.path.join(self.tmp, f"fake_root_{os_name}")
        sudo_log = os.path.join(self.tmp, f"sudo_{os_name}.log")
        os.makedirs(bin_dir, exist_ok=True)
        os.makedirs(fake_root, exist_ok=True)

        real_install = shutil.which("install")
        real_tee = shutil.which("tee")
        real_chmod = shutil.which("chmod")
        real_touch = shutil.which("touch")
        real_python3 = shutil.which("python3")
        real_uname = shutil.which("uname")

        put(
            os.path.join(bin_dir, "uname"),
            f'#!/bin/sh\nif [ "$1" = -s ]; then echo {shlex.quote(os_name)}; exit 0; fi\n'
            f'exec {shlex.quote(real_uname)} "$@"\n',
            0o755,
        )
        put(
            os.path.join(bin_dir, "id"),
            '#!/bin/sh\nif [ "$1" = -gn ]; then echo staff; exit 0; fi\n'
            'echo 1000\n',
            0o755,
        )
        put(os.path.join(bin_dir, "systemctl"), "#!/bin/sh\nexit 0\n", 0o755)
        put(
            os.path.join(bin_dir, "fdesetup"),
            '#!/bin/sh\necho "FileVault is Off."\n',
            0o755,
        )
        put(
            os.path.join(bin_dir, "pmset"),
            '#!/bin/sh\n'
            'if [ "$1" = -g ]; then printf " autorestart 1\\n sleep 0\\n"; fi\n'
            'exit 0\n',
            0o755,
        )
        put(os.path.join(bin_dir, "defaults"), "#!/bin/sh\necho 0\n", 0o755)
        if launchctl_body is not None:
            put(os.path.join(bin_dir, "launchctl"), launchctl_body, 0o755)

        # Stub sudo: logs every call, redirects /etc/... and /Library/... paths
        # under fake_root, drops -u/-o/-g ownership flags (unprivileged test),
        # and runs real tee/chmod/touch/install/python3/test/launchctl.
        put(
            os.path.join(bin_dir, "sudo"),
            f"""#!/usr/bin/env bash
printf '%s\\n' "$*" >> {shlex.quote(sudo_log)}
if [ "${{1:-}}" = "-u" ]; then
  shift 2
fi
cmd="$1"
shift
map_path() {{
  case "$1" in
    /etc/*|/Library/*) printf '%s%s' {shlex.quote(fake_root)} "$1" ;;
    *) printf '%s' "$1" ;;
  esac
}}
case "$cmd" in
  chown) exit 0 ;;
  tee)
    dest="$(map_path "$1")"
    mkdir -p "$(dirname "$dest")"
    exec {shlex.quote(real_tee)} "$dest"
    ;;
  chmod)
    mode="$1"; shift
    dest="$(map_path "$1")"
    exec {shlex.quote(real_chmod)} "$mode" "$dest"
    ;;
  touch)
    dest="$(map_path "$1")"
    mkdir -p "$(dirname "$dest")"
    exec {shlex.quote(real_touch)} "$dest"
    ;;
  install)
    args=()
    while [ $# -gt 0 ]; do
      case "$1" in
        -o|-g) shift 2 ;;
        *) args+=("$(map_path "$1")"); shift ;;
      esac
    done
    exec {shlex.quote(real_install)} "${{args[@]}}"
    ;;
  *)
    exec "$cmd" "$@"
    ;;
esac
""",
            0o755,
        )
        env = dict(
            os.environ,
            PATH=bin_dir + os.pathsep + os.environ.get("PATH", ""),
            TALOS_DEPLOY_SERVICE_PATH=os.path.dirname(real_python3)
            + os.pathsep
            + os.environ.get("PATH", ""),
        )
        return env, fake_root, sudo_log

    # REQ-OPS-50, REQ-OPS-54: enable-boot checks os.R_OK | os.X_OK on launcher,
    # bin/* (except *.repo_mapping), and studio/studio_bridge when present.
    def test_enable_boot_checks_executable_bits_on_bundle_binaries(self):
        prefix = os.path.join(self.tmp, "exec_prefix")
        bundle = os.path.join(prefix, "bundles", "b1")
        make_stub_bundle(bundle)
        launcher = os.path.join(bundle, "launcher")
        node_bin = os.path.join(bundle, "bin", "my_node")
        repo_map = os.path.join(bundle, "bin", "my_node.repo_mapping")
        bridge = os.path.join(bundle, "studio", "studio_bridge")
        put(launcher, "#!/bin/sh\nexit 0\n", 0o755)
        put(node_bin, "#!/bin/sh\nexit 0\n", 0o755)
        put(repo_map, "mapping\n", 0o644)
        put(bridge, "#!/bin/sh\nexit 0\n", 0o755)
        os.symlink(bundle, os.path.join(prefix, "current"))

        env, _, _ = self._make_enable_boot_env("Linux")

        # Each executable missing +x must fail enable-boot.
        for label, target in (
            ("launcher", launcher),
            ("bin/my_node", node_bin),
            ("studio/studio_bridge", bridge),
        ):
            with self.subTest(non_exec=label):
                os.chmod(target, 0o644)
                proc = self.run_deploy(
                    "enable-boot", "--prefix", prefix, "--user", "robot",
                    env=env,
                )
                os.chmod(target, 0o755)
                out = proc.stdout + proc.stderr
                self.assertNotEqual(proc.returncode, 0, out)
                self.assertIn("cannot read/execute", proc.stderr)
                self.assertIn(label, proc.stderr)

        # With all executables 0755 (and .repo_mapping 0644), enable-boot passes.
        ok = self.run_deploy(
            "enable-boot", "--prefix", prefix, "--user", "robot", env=env
        )
        self.assertEqual(ok.returncode, 0, ok.stdout + ok.stderr)

    # REQ-OPS-54: Linux enable-boot sets root:root and 0644 on talos.service
    # regardless of caller umask (077 or 000).
    def test_linux_enable_boot_sets_unit_mode_0644_under_umasks(self):
        prefix = os.path.join(self.tmp, "umask_prefix")
        bundle = os.path.join(prefix, "bundles", "b1")
        make_stub_bundle(bundle)
        os.symlink(bundle, os.path.join(prefix, "current"))

        for mask in ("077", "000"):
            with self.subTest(umask=mask):
                env, fake_root, sudo_log = self._make_enable_boot_env(
                    f"Linux_{mask}"
                )
                # Ensure uname returns Linux (not Linux_077).
                real_uname = shutil.which("uname")
                put(
                    os.path.join(
                        self.tmp, f"stub_bin_Linux_{mask}", "uname"
                    ),
                    '#!/bin/sh\nif [ "$1" = -s ]; then echo Linux; exit 0; fi\n'
                    f'exec {shlex.quote(real_uname)} "$@"\n',
                    0o755,
                )
                proc = subprocess.run(
                    [
                        "bash",
                        "-c",
                        'umask "$1" && shift && exec bash "$@"',
                        "_",
                        mask,
                        self.deploy,
                        "enable-boot",
                        "--prefix",
                        prefix,
                        "--user",
                        "robot",
                    ],
                    env=env,
                    capture_output=True,
                    text=True,
                    timeout=60,
                )
                self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
                unit_path = os.path.join(
                    fake_root, "etc", "systemd", "system", "talos.service"
                )
                self.assertTrue(os.path.isfile(unit_path))
                self.assertEqual(
                    stat.S_IMODE(os.stat(unit_path).st_mode), 0o644
                )
                with open(sudo_log, encoding="utf-8") as f:
                    log = f.read()
                self.assertIn(
                    "chown root:root /etc/systemd/system/talos.service", log
                )

    # REQ-OPS-50: macOS enable-boot polls launchctl print after bootout until
    # the LaunchDaemon finishes unloading before calling bootstrap.
    def test_macos_enable_boot_waits_for_bootout_before_bootstrap(self):
        prefix = os.path.join(self.tmp, "darwin_prefix")
        bundle = os.path.join(prefix, "bundles", "b1")
        make_stub_bundle(bundle)
        os.symlink(bundle, os.path.join(prefix, "current"))

        launchctl_log = os.path.join(self.tmp, "launchctl.log")
        state_file = os.path.join(self.tmp, "launchctl_print_count")
        launchctl_stub = f"""#!/bin/sh
echo "$1" >> {shlex.quote(launchctl_log)}
case "$1" in
  bootout)
    echo 0 > {shlex.quote(state_file)}
    exit 0
    ;;
  print)
    count="$(cat {shlex.quote(state_file)} 2>/dev/null || echo 99)"
    echo $((count + 1)) > {shlex.quote(state_file)}
    if [ "$count" -lt 2 ]; then
      exit 0
    fi
    exit 113
    ;;
  bootstrap)
    exit 0
    ;;
esac
exit 1
"""
        env, _, _ = self._make_enable_boot_env(
            "Darwin", launchctl_body=launchctl_stub
        )
        proc = self.run_deploy(
            "enable-boot", "--prefix", prefix, "--user", "robot", env=env
        )
        self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
        with open(launchctl_log, encoding="utf-8") as f:
            calls = f.read().splitlines()
        self.assertEqual(
            calls, ["bootout", "print", "print", "print", "bootstrap"]
        )


if __name__ == "__main__":
    unittest.main()
