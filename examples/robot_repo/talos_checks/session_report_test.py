"""session_report runs from a repository that depends on talOS.

REQ-DEV-30 (a robot repository uses talOS only through @talos// labels),
REQ-OPS-16 (session_report is the session acceptance tool).
"""

import json
import os
import subprocess
import tempfile
import unittest

from python.runfiles import runfiles


class SessionReportTest(unittest.TestCase):
    def test_reports_a_recorded_session(self):
        files = runfiles.Create()
        fixture = files.Rlocation(os.environ["SESSION_REPORT_FIXTURE"])
        report = files.Rlocation(os.environ["SESSION_REPORT"])
        self.assertTrue(fixture and os.path.isfile(fixture), fixture)
        self.assertTrue(report and os.path.isfile(report), report)
        with tempfile.TemporaryDirectory(
                dir=os.environ.get("TEST_TMPDIR")) as session:
            subprocess.run([fixture, session], check=True, timeout=60)
            result = subprocess.run([report, "--json", session],
                                    capture_output=True, text=True,
                                    timeout=60)
        self.assertEqual(result.returncode, 0, result.stderr)
        nodes = [node["name"] for node in json.loads(result.stdout)["nodes"]]
        self.assertIn("fixture", nodes)


if __name__ == "__main__":
    unittest.main()
