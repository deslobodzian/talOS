"""Unit tests for tools/run_clang_tidy.py header includer selection."""

import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest import mock

from tools import run_clang_tidy


class SelectIncludersTest(unittest.TestCase):
    def test_production_header_prefers_production_includers(self):
        includers = {
            "talOS/bridge/foo.h": {
                "talOS/bridge/foo_test.cc",
                "talOS/bridge/foo.cc",
                "talOS/launcher/launcher.cc",
            },
        }
        tus = {
            "talOS/bridge/foo_test.cc",
            "talOS/bridge/foo.cc",
            "talOS/launcher/launcher.cc",
        }
        found, fallback = run_clang_tidy.select_includers(
            "talOS/bridge/foo.h", includers, tus
        )
        self.assertEqual(
            found, ["talOS/bridge/foo.cc", "talOS/launcher/launcher.cc"]
        )
        self.assertFalse(fallback)

    def test_production_header_with_only_test_includers_falls_back(self):
        includers = {
            "talOS/bridge/in_process_gateway.h": {
                "talOS/bridge/node_test_support.h",
                "talOS/bridge/in_process_gateway_test.cc",
            },
            "talOS/bridge/node_test_support.h": {
                "talOS/bridge/node_test.cc",
                "talOS/other/integration_test.cc",
            },
        }
        tus = {
            "talOS/bridge/in_process_gateway_test.cc",
            "talOS/bridge/node_test.cc",
            "talOS/other/integration_test.cc",
        }
        found, fallback = run_clang_tidy.select_includers(
            "talOS/bridge/in_process_gateway.h", includers, tus
        )
        self.assertEqual(
            found,
            [
                "talOS/bridge/in_process_gateway_test.cc",
                "talOS/bridge/node_test.cc",
                "talOS/other/integration_test.cc",
            ],
        )
        self.assertTrue(fallback)

    def test_production_header_with_transitive_production_includer_skips_test_fallback(self):
        includers = {
            "talOS/bridge/detail.h": {
                "talOS/bridge/detail_test.cc",
                "talOS/bridge/api.h",
            },
            "talOS/bridge/api.h": {
                "talOS/bridge/node.cc",
            },
        }
        tus = {
            "talOS/bridge/detail_test.cc",
            "talOS/bridge/node.cc",
        }
        found, fallback = run_clang_tidy.select_includers(
            "talOS/bridge/detail.h", includers, tus
        )
        self.assertEqual(found, ["talOS/bridge/node.cc"])
        self.assertFalse(fallback)

    def test_test_header_selects_test_includers_without_fallback_flag(self):
        includers = {
            "talOS/bridge/testing.h": {
                "talOS/bridge/node_test.cc",
            },
        }
        tus = {"talOS/bridge/node_test.cc"}
        found, fallback = run_clang_tidy.select_includers(
            "talOS/bridge/testing.h", includers, tus
        )
        self.assertEqual(found, ["talOS/bridge/node_test.cc"])
        self.assertFalse(fallback)

    def test_unincluded_header_returns_empty(self):
        found, fallback = run_clang_tidy.select_includers(
            "talOS/bridge/orphan.h", {}, {"talOS/bridge/node.cc"}
        )
        self.assertEqual(found, [])
        self.assertFalse(fallback)

    def test_fallback_respects_tus_per_header_limit(self):
        tests = {f"talOS/bridge/t{i}_test.cc" for i in range(6)}
        includers = {"talOS/bridge/in_process_gateway.h": tests}
        found, fallback = run_clang_tidy.select_includers(
            "talOS/bridge/in_process_gateway.h", includers, tests, limit=4
        )
        self.assertEqual(len(found), 4)
        self.assertTrue(fallback)


class MainSymlinkRootTest(unittest.TestCase):
    def test_symlinked_cwd_matches_realpath_compile_commands(self):
        # Simulates macOS `/tmp` -> `/private/tmp`: `os.getcwd()` returns a
        # symlink path while `compile_commands.json` entries resolve through
        # `os.path.realpath(...)`.
        with tempfile.TemporaryDirectory() as tmp:
            real_root = os.path.join(tmp, "private_repo")
            link_root = os.path.join(tmp, "link_repo")
            src_rel = "talOS/bridge/foo.cc"
            os.makedirs(os.path.join(real_root, "talOS/bridge"), exist_ok=True)
            with open(os.path.join(real_root, ".bazelignore"), "w", encoding="utf-8") as f:
                f.write("")
            with open(os.path.join(real_root, src_rel), "w", encoding="utf-8") as f:
                f.write("int foo() { return 0; }\n")
            db = [{"directory": os.path.realpath(real_root), "file": src_rel}]
            with open(os.path.join(real_root, "compile_commands.json"), "w", encoding="utf-8") as f:
                json.dump(db, f)
            os.symlink(real_root, link_root)

            saved_cwd = os.getcwd()
            os.chdir(real_root)
            seen_cmds = []

            def fake_run(cmd, **kwargs):
                seen_cmds.append(cmd)
                return subprocess.CompletedProcess(cmd, 0, stdout="", stderr="")

            out = io.StringIO()
            try:
                with (
                    mock.patch.object(sys, "argv", ["run_clang_tidy.py", "HEAD~1"]),
                    mock.patch.object(os, "getcwd", return_value=link_root),
                    mock.patch.object(run_clang_tidy, "git", return_value=[src_rel]),
                    mock.patch.object(subprocess, "run", side_effect=fake_run),
                    redirect_stdout(out),
                ):
                    rc = run_clang_tidy.main()
            finally:
                os.chdir(saved_cwd)

        self.assertEqual(rc, 0, out.getvalue())
        self.assertEqual(len(seen_cmds), 1, out.getvalue())
        self.assertEqual(seen_cmds[0][-1], src_rel)


if __name__ == "__main__":
    unittest.main()
