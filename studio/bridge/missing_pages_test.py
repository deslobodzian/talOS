"""A bridge whose webroot holds no pages still runs, and its pages say how to build them."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parent))
from integration_test import bound_port

# Bounds only a hang: the bridge answers and stops in milliseconds.
HANG_S = 20


class MissingPages(unittest.TestCase):
    # REQ-DEV-30 REQ-OPS-34
    def test_a_webroot_never_built_is_answered_with_the_build_command(self):
        """A robot repository's talOS submodule that never built Studio's pages
        hands the bridge a webroot that does not exist. The bridge used to
        throw at startup (`fs::canonical`) and exit 1, so the launcher
        retried it for a minute and recorded the session member failed, and
        nothing named the command that builds the pages. It now runs, `/` and
        `/drive` answer 503 naming `//studio:web_dist` and, for a robot whose
        installed bundle lost them, `tools/deploy.sh install`; it stops with 0.
        """
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        with tempfile.TemporaryDirectory() as parent:
            webroot = Path(parent, "dist")  # never created
            # No publisher: the bridge serves HTTP and waits for the segment.
            bridge = subprocess.Popen(
                [str(workspace / "studio/bridge/studio_bridge"),
                 f"/test/telemetry/p{os.getpid()}", "0", str(webroot)],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                try:
                    port = bound_port(bridge)
                except AssertionError as error:
                    bridge.wait(timeout=HANG_S)
                    self.fail(f"{error}; exit {bridge.returncode}, stderr "
                              f"{bridge.stderr.read()!r}")
                for path in ("/", "/drive"):
                    with self.subTest(path=path):
                        with self.assertRaises(urllib.error.HTTPError) as answer:
                            urllib.request.urlopen(f"http://127.0.0.1:{port}{path}",
                                                   timeout=HANG_S)
                        self.assertEqual(answer.exception.code, 503)
                        body = answer.exception.read().decode()
                        # Both remedies: a checkout builds the pages, a
                        # robot installs its bundle again.
                        self.assertIn("//studio:web_dist", body)
                        self.assertIn("tools/deploy.sh install", body)
                # Any other file is simply absent.
                with self.assertRaises(urllib.error.HTTPError) as other:
                    urllib.request.urlopen(f"http://127.0.0.1:{port}/favicon.svg",
                                           timeout=HANG_S)
                self.assertEqual(other.exception.code, 404)
                bridge.send_signal(signal.SIGINT)
                bridge.wait(timeout=HANG_S)
                self.assertEqual(bridge.returncode, 0)
                # Its log names each missing page, as the answers do.
                log = bridge.stderr.read()
                for page in ("index.html", "drive.html"):
                    with self.subTest(log=page):
                        self.assertIn(page, log)
            finally:
                if bridge.poll() is None:
                    bridge.kill()
                bridge.communicate()


if __name__ == "__main__":
    unittest.main()
