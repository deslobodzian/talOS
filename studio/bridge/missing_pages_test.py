"""A bridge whose webroot holds no pages still runs, and its pages say how to build them;
a page that links out of the webroot is named so and never read."""
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
from bridge_client import bound_port

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
                        # REQ-DEV-30: no directory this bridge cannot know
                        # exists. `cd talos` was right only for a robot
                        # repository holding talOS as a submodule there.
                        self.assertNotIn("cd talos", body)
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

    # REQ-DEV-30 REQ-OPS-34
    def test_a_page_linking_out_of_the_webroot_is_named_and_never_read(self):
        """The bridge serves only files inside its webroot's real tree, so a
        request cannot read a file a link in the webroot points out to. A
        sandboxed Bazel test's runfiles stage the pages exactly so (each file
        a link to the execroot), and the bridge used to answer such built
        pages "Studio's pages are not built", which sent the reader to build
        what was built. It now says the page links out, and still reads
        nothing outside the webroot."""
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        with tempfile.TemporaryDirectory() as parent:
            webroot = Path(parent, "webroot")
            webroot.mkdir()
            Path(webroot, "index.html").write_text("<html>inside</html>")
            outside = Path(parent, "outside")
            outside.mkdir()
            Path(outside, "drive.html").write_text("<html>OUTSIDE-SECRET</html>")
            Path(outside, "secret.txt").write_text("OUTSIDE-SECRET")
            Path(webroot, "drive.html").symlink_to(Path(outside, "drive.html"))
            Path(webroot, "leak.txt").symlink_to(Path(outside, "secret.txt"))
            bridge = subprocess.Popen(
                [str(workspace / "studio/bridge/studio_bridge"),
                 f"/test/telemetry/p{os.getpid()}", "0", str(webroot)],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                port = bound_port(bridge)
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/",
                                            timeout=HANG_S) as page:
                    self.assertEqual(page.read().decode(), "<html>inside</html>")
                with self.assertRaises(urllib.error.HTTPError) as drive:
                    urllib.request.urlopen(f"http://127.0.0.1:{port}/drive",
                                           timeout=HANG_S)
                self.assertEqual(drive.exception.code, 503)
                body = drive.exception.read().decode()
                self.assertNotIn("OUTSIDE-SECRET", body)
                self.assertNotIn("not built", body)
                self.assertIn(f"links to {Path(outside, 'drive.html').resolve()}", body)
                with self.assertRaises(urllib.error.HTTPError) as leak:
                    urllib.request.urlopen(f"http://127.0.0.1:{port}/leak.txt",
                                           timeout=HANG_S)
                self.assertEqual(leak.exception.code, 404)
                self.assertNotIn("OUTSIDE-SECRET", leak.exception.read().decode())
                bridge.send_signal(signal.SIGINT)
                bridge.wait(timeout=HANG_S)
                self.assertEqual(bridge.returncode, 0)
                # The startup log says the same, not "not built".
                log = bridge.stderr.read()
                self.assertIn("links to", log)
                self.assertNotIn("not built", log)
            finally:
                if bridge.poll() is None:
                    bridge.kill()
                bridge.communicate()


if __name__ == "__main__":
    unittest.main()
