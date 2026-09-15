"""Bundle deployment checks: install, logs, rollback, offline (items 56, 58)
and the compatibility record (item 57).

Builds a minimal fake bundle in an isolated temp dir (manifest + config +
one binary), drives compat_record.py as a library and deploy.sh as a real
subprocess, and asserts each deployment step. Network use stays at zero
except one loopback probe on an ephemeral port (item 54: no fixed ports).
"""

import json
import os
import socket
import stat
import subprocess
import tempfile
import threading
import unittest
from pathlib import Path

from compat_record import build_record, check_bundle, recompute_config_sha256
from python.runfiles import runfiles

DEPLOY_SH = runfiles.Create().Rlocation(os.environ["DEPLOY_SH"])


def write_exe(path):
    path.write_text("#!/bin/sh\nexit 0\n")
    path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def make_bundle(root):
    root = Path(root)
    (root / "config").mkdir(parents=True)
    (root / "bin").mkdir(parents=True)
    (root / "config" / "robot.toml").write_text(
        '[robot]\nname = "deploy_fixture"\ncommissioned = true\n')
    (root / "config" / "drive.toml").write_text(
        '[subsystem]\nname = "drive"\nnode = "//example/drive:node"\n')
    write_exe(root / "bin" / "drive")
    manifest = {
        "bundle_version": 1,
        "config_path": "config/robot.toml",
        "config_sha256": recompute_config_sha256(str(root / "config")),
        "nodes": [
            {"name": "drive", "target": "//example/drive:node",
             "binary_path": "bin/drive", "language": "cpp",
             "contract": "@talos_x//:contract"},
            {"name": "arbiter", "target": "//example/arbiter:node",
             "binary_path": "bin/drive", "language": "cpp", "contract": ""},
        ],
    }
    (root / "bundle_manifest.json").write_text(json.dumps(manifest, indent=2))
    return root


def run_deploy(*args, prefix):
    return subprocess.run(
        ["bash", DEPLOY_SH, *args, "--prefix", str(prefix)],
        capture_output=True, text=True, timeout=120)


class DeployTest(unittest.TestCase):
    def test_check_passes_with_schema_less_warning(self):
        with tempfile.TemporaryDirectory(prefix="talos_bundle_ok_") as tmp:
            bundle = make_bundle(Path(tmp) / "bundle")
            ok, problems = check_bundle(str(bundle))
            self.assertTrue(ok, problems)
            # The contract-less arbiter warns but does not fail the check.
            self.assertTrue(any("arbiter" in p for p in problems), problems)

    def test_tampered_config_fails_check(self):
        with tempfile.TemporaryDirectory(prefix="talos_bundle_tamper_") as tmp:
            bundle = make_bundle(Path(tmp) / "bundle")
            with open(bundle / "config" / "drive.toml", "a") as f:
                f.write("# pit hand edit\n")
            ok, problems = check_bundle(str(bundle))
            self.assertFalse(ok)
            self.assertTrue(any("sha256" in p for p in problems), problems)

    def test_missing_binary_fails_check(self):
        with tempfile.TemporaryDirectory(prefix="talos_bundle_nobin_") as tmp:
            bundle = make_bundle(Path(tmp) / "bundle")
            (bundle / "bin" / "drive").unlink()
            ok, problems = check_bundle(str(bundle))
            self.assertFalse(ok)
            self.assertTrue(any("missing binary" in p for p in problems),
                            problems)

    def test_record_carries_revisions_contracts_and_vendor(self):
        with tempfile.TemporaryDirectory(prefix="talos_bundle_rec_") as tmp:
            bundle = make_bundle(Path(tmp) / "bundle")
            vendor = {"devices": {"TalonFX": {"firmware": "25.3.2.1"}},
                      "wpilib": "2026.2.1"}
            record = build_record(str(bundle), vendor, workspace="abc123")
            self.assertTrue(record["config_match"])
            self.assertEqual(record["workspace_revision"], "abc123")
            self.assertEqual(record["nodes"][0]["contract"],
                             "@talos_x//:contract")
            self.assertTrue(record["vendor_expectations_recorded"])
            bare = build_record(str(bundle))
            self.assertFalse(bare["vendor_expectations_recorded"])

    def test_install_logs_and_rollback_roundtrip(self):
        with tempfile.TemporaryDirectory(prefix="talos_deploy_") as tmp:
            tmp = Path(tmp)
            old = make_bundle(tmp / "old")
            new = make_bundle(tmp / "new")
            r = run_deploy("install", str(old), prefix=tmp / "opt")
            self.assertEqual(r.returncode, 0, r.stderr)
            first = os.path.realpath(tmp / "opt" / "current")
            self.assertIn("bundles", first)
            r = run_deploy("install", str(new), prefix=tmp / "opt")
            self.assertEqual(r.returncode, 0, r.stderr)
            second = os.path.realpath(tmp / "opt" / "current")
            self.assertNotEqual(first, second)
            # After rollback, current points at the first bundle again and
            # previous at the second: whole-bundle rollback, never mixed.
            r = run_deploy("rollback", prefix=tmp / "opt")
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertEqual(os.path.realpath(tmp / "opt" / "current"), first)
            self.assertEqual(os.path.realpath(tmp / "opt" / "previous"), second)
            r = run_deploy("logs", prefix=tmp / "opt")
            self.assertEqual(r.returncode, 0, r.stderr)

    def test_verify_offline_and_dry_run_boot(self):
        with tempfile.TemporaryDirectory(prefix="talos_offline_") as tmp:
            bundle = make_bundle(Path(tmp) / "bundle")
            # Fake bundle entry point: honors the launcher flags
            # verify-offline passes a relocated bundle and writes a
            # conforming session manifest, the way the real launcher does.
            (bundle / "run.sh").write_text(
                '#!/bin/sh\n'
                'out=""\n'
                'while [ $# -gt 0 ]; do\n'
                '  case "$1" in\n'
                '    --output-dir) out="$2"; shift 2;;\n'
                '    *) shift;;\n'
                '  esac\n'
                'done\n'
                '[ -n "$out" ] || exit 2\n'
                'mkdir -p "$out"\n'
                'printf \'{"nodes": [{"name": "drive", "pid": %d,'
                ' "exit_code": 0}]}\' "$$" > "$out/manifest.json"\n'
                'exit 0\n')
            (bundle / "run.sh").chmod(
                (bundle / "run.sh").stat().st_mode | stat.S_IXUSR)
            r = subprocess.run(["bash", DEPLOY_SH, "verify-offline",
                                str(bundle)], capture_output=True, text=True,
                               timeout=120)
            self.assertEqual(r.returncode, 0, r.stderr)
            r = subprocess.run(["bash", DEPLOY_SH, "enable-boot",
                                "--dry-run", "--prefix", str(tmp)],
                               capture_output=True, text=True, timeout=60)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn("ExecStart", r.stdout)

    def test_check_network_loopback_ephemeral_port(self):
        # A UDP double answering like the controller gateway: any datagram
        # gets a valid protocol frame back (magic/version/header per
        # talOS/protocol/frame.h), preceded by one junk datagram the probe
        # must skip. Ephemeral ports on both ends (item 54: no fixed ports).
        import struct
        reply = struct.pack("<QHHHHQQII", 0x4F4952534F4C4154, 1, 40,
                            12, 0, 7, 0, 0, 0)
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", 0))
        port = server.getsockname()[1]
        server.settimeout(10)
        answered = []

        def serve():
            data, addr = server.recvfrom(2048)
            if data:
                server.sendto(b"junk", addr)
                server.sendto(reply, addr)
                answered.append(True)

        t = threading.Thread(target=serve, daemon=True)
        t.start()
        try:
            r = subprocess.run(
                ["bash", DEPLOY_SH, "check-network",
                 "--controller", "127.0.0.1", "--port", str(port),
                 "--local-port", "0"],
                capture_output=True, text=True, timeout=60)
            self.assertEqual(r.returncode, 0, r.stderr)
            t.join(timeout=10)
            self.assertTrue(answered)
            self.assertIn("frame type=12", r.stdout)
        finally:
            server.close()
        # A closed port fails clearly and fast (refused, not a hang past 3s).
        r = subprocess.run(
            ["bash", DEPLOY_SH, "check-network",
             "--controller", "127.0.0.1", "--port", "1",
             "--local-port", "0"],
            capture_output=True, text=True, timeout=60)
        self.assertNotEqual(r.returncode, 0)


if __name__ == "__main__":
    unittest.main()
