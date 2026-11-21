"""Unit tests for studio/bridge/webroot_stamp.py.

REQ-DEV-30: Studio's webroot content stamp is deterministic over the Studio web
inputs and telemetry schema, ignores checked-in generated files, supports
`--list` for repository-rule watching, and fails closed on missing inputs or
malformed stamp files.
"""

import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from studio.bridge import webroot_stamp


class WebrootStampTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(tempfile.mkdtemp(dir=os.environ.get("TEST_TMPDIR")))
        for rel in webroot_stamp.FIXED_INPUTS:
            self.put(rel, f"content of {rel}\n")
        self.put("studio/src/main.tsx", "export const v = 1;\n")
        self.put("studio/src/drive/main.ts", "export const d = 1;\n")
        self.put("studio/src/generated/talos/telemetry.ts", "// generated\n")

    def put(self, rel, text):
        full = self.root / rel
        full.parent.mkdir(parents=True, exist_ok=True)
        full.write_text(text, encoding="utf-8")

    # REQ-DEV-30
    def test_deterministic_and_excludes_generated_dir(self):
        first = webroot_stamp.compute_stamp(self.root)
        second = webroot_stamp.compute_stamp(self.root)
        self.assertEqual(first, second)
        self.assertIsNotNone(webroot_stamp.STAMP_RE.match(first), first)

        # Editing checked-in studio/src/generated/** does not change the stamp
        # because talOS/telemetry/telemetry.fbs is hashed directly.
        self.put("studio/src/generated/talos/telemetry.ts", "// stale copy\n")
        self.assertEqual(webroot_stamp.compute_stamp(self.root), first)

    # REQ-DEV-30
    def test_changes_when_any_input_changes_or_is_added(self):
        base = webroot_stamp.compute_stamp(self.root)
        for rel in (
            "studio/agent/rpc.ts",
            "studio/drive.html",
            "studio/pnpm-lock.yaml",
            "talOS/telemetry/telemetry.fbs",
            "studio/src/drive/main.ts",
        ):
            with self.subTest(changed=rel):
                original = (self.root / rel).read_text(encoding="utf-8")
                self.put(rel, original + "edited\n")
                self.assertNotEqual(webroot_stamp.compute_stamp(self.root), base)
                self.put(rel, original)
                self.assertEqual(webroot_stamp.compute_stamp(self.root), base)

        self.put("studio/src/panels/new_panel.tsx", "export {};\n")
        self.assertNotEqual(webroot_stamp.compute_stamp(self.root), base)

    # REQ-DEV-30
    def test_list_returns_sorted_inputs_without_generated(self):
        out = io.StringIO()
        with mock.patch("sys.stdout", out):
            rc = webroot_stamp.main(["--root", str(self.root), "--list"])
        self.assertEqual(rc, 0)
        lines = out.getvalue().splitlines()
        self.assertEqual(lines, sorted(lines))
        for rel in webroot_stamp.FIXED_INPUTS:
            self.assertIn(rel, lines)
        self.assertIn("studio/agent/rpc.ts", lines)
        self.assertIn("studio/src/main.tsx", lines)
        self.assertIn("studio/src/drive/main.ts", lines)
        self.assertFalse(
            any(p.startswith("studio/src/generated/") for p in lines), lines)

    # REQ-DEV-30
    def test_parse_stamp_accepts_hash_with_optional_git_rev_and_rejects_bad(self):
        digest = webroot_stamp.compute_stamp(self.root)
        self.assertEqual(
            webroot_stamp.parse_stamp(webroot_stamp.format_stamp(digest)),
            digest)
        self.assertEqual(
            webroot_stamp.parse_stamp(
                webroot_stamp.format_stamp(digest, "0123456789abcdef")),
            digest)
        for bad in ("", "   \n", digest[:-1], digest.upper(), "z" * 64):
            with self.subTest(bad=bad):
                self.assertIsNone(webroot_stamp.parse_stamp(bad))

    # REQ-DEV-30
    def test_missing_required_input_fails_closed(self):
        (self.root / "talOS/telemetry/telemetry.fbs").unlink()
        err = io.StringIO()
        with mock.patch("sys.stderr", err):
            rc = webroot_stamp.main(["--root", str(self.root), "--hash-only"])
        self.assertEqual(rc, 1)
        self.assertIn("telemetry.fbs", err.getvalue())


if __name__ == "__main__":
    unittest.main()
