"""Studio's bridge runs as a member of this repository's simulated session.

Runs `//:sim --start-studio-bridge` on free loopback ports, waits for the
launcher's line that the bridge is up, fetches `/drive`, stops the launch
with SIGINT as Ctrl-C does, and reads the session's manifest.
"""

import glob
import json
import os
import re
import signal
import socket
import subprocess
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

from python.runfiles import runfiles

# Bounds only a hang, inside Bazel's 60 s timeout for a small test: the
# launcher declares the bridge up 1 s after it spawned it, a few seconds
# into a session on an idle host.
HANG_S = 40
STOP_WAIT_S = 10


def free_ports():
    """Free loopback ports for the gateway, the hardware bridge (UDP) and
    Studio (TCP), bound at 0 and released before the launch, since the
    launcher refuses port 0 (talOS docs/TESTING.md §6.4)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as gateway, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bridge, \
            socket.socket(socket.AF_INET, socket.SOCK_STREAM) as studio:
        gateway.bind(("127.0.0.1", 0))
        bridge.bind(("127.0.0.1", 0))
        studio.bind(("", 0))
        return (gateway.getsockname()[1], bridge.getsockname()[1],
                studio.getsockname()[1])


class StudioBridgeInSession(unittest.TestCase):
    # REQ-DEV-30
    def test_the_bridge_serves_drive_and_exits_zero(self):
        """Guards this repository's Studio wiring: talOS's bridge builds
        here through @talos, robot.toml's [studio] webroot and //:sim's
        data give it a folder that exists, and the launcher runs it as a
        member. A bridge the launcher drops leaves no manifest row, so the
        row is required here rather than only its exit."""
        out = tempfile.mkdtemp(dir=os.environ.get("TEST_TMPDIR"))
        try:
            self.check_session(out)
        except AssertionError as error:
            # The bridge's own output, which the launcher does not echo: a
            # port another process took first reads there, not in the
            # launch's lines (talOS docs/TESTING.md §6.4).
            logs = glob.glob(os.path.join(out, "*", "studio_bridge.log"))
            text = "".join(open(log, encoding="utf-8").read() for log in logs)
            raise AssertionError(f"{error}\n--- studio_bridge.log ---\n"
                                 f"{text or '(none written)'}") from None

    def check_session(self, out):
        sim = runfiles.Create().Rlocation("_main/sim")
        gateway, bridge, studio = free_ports()
        launch = subprocess.Popen(
            [sim, "--start-studio-bridge", "--studio-http-port", str(studio),
             "--output-dir", out, "--remote", "127.0.0.1",
             "--remote-port", str(gateway), "--local-port", str(bridge),
             "--duration-s", str(HANG_S)],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, start_new_session=True)
        output = []
        up = threading.Event()

        def read():
            for line in launch.stdout:
                output.append(line)
                if ("Started studio_bridge" in line
                        or "studio_bridge will not be restarted again" in line
                        or "studio_bridge not executable" in line):
                    up.set()
            up.set()  # the launch ended: nothing more will come

        reader = threading.Thread(target=read, daemon=True)
        reader.start()
        try:
            self.assertTrue(up.wait(HANG_S), f"no studio_bridge line in "
                            f"{HANG_S} s; the launch printed:\n"
                            + "".join(output[-60:]))
            log = "".join(output)
            self.assertIn("Started studio_bridge", log, log)
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{studio}/drive",
                                            timeout=HANG_S) as answer:
                    status, body = answer.status, answer.read().decode()
            except urllib.error.HTTPError as error:
                status, body = error.code, error.read().decode()
            # Which state this run is in is known before asking: studio_web
            # leaves NOT_BUILT.txt in //:studio_webroot exactly while the
            # talOS checkout has not built the pages. Built: /drive is the
            # page. Not built: 503 with the command that builds them, naming
            # the folder the bridge was given (//:studio_webroot, an existing
            # directory, so a webroot the config or //:sim's data miswires
            # fails here too) and the talOS checkout to build in.
            webroot = runfiles.Create().Rlocation("_main/studio_webroot")
            built = not os.path.exists(os.path.join(webroot, "NOT_BUILT.txt"))
            if built:
                self.assertEqual(status, 200, body)
                self.assertIn("<html", body.lower())
            else:
                self.assertEqual(status, 503, body)
                self.assertIn("//studio:web_dist", body)
                folder = re.search(r"not built: (.*) has no ", body)
                self.assertIsNotNone(folder, body)
                self.assertEqual(os.path.basename(folder[1]), "studio_webroot",
                                 body)
                self.assertTrue(os.path.isdir(folder[1]),
                                f"{folder[1]} is not a directory: {body}")
                # The checkout to build in is the one this repository's
                # studio_web reads (here MODULE.bazel's ../..), named as a
                # directory that exists and is talOS's, not a fixed `talos`
                # (a submodule this repository does not have).
                checkout = re.search(
                    r"\(cd (.+?) && bazel build //studio:web_dist\)", body)
                self.assertIsNotNone(checkout, body)
                self.assertTrue(os.path.isfile(os.path.join(
                    checkout[1], "studio", "bridge", "webroot.bzl")), body)
            launch.send_signal(signal.SIGINT)
            self.assertEqual(launch.wait(timeout=STOP_WAIT_S), 0,
                             "".join(output))
        finally:
            if launch.poll() is None:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
            reader.join(timeout=STOP_WAIT_S)
            launch.stdout.close()
        manifests = glob.glob(os.path.join(out, "*", "manifest.json"))
        self.assertEqual(len(manifests), 1, manifests)
        with open(manifests[0], encoding="utf-8") as f:
            nodes = {n["name"]: n for n in json.load(f)["nodes"]}
        self.assertIn("studio_bridge", nodes, sorted(nodes))
        runs = [(r["exit_code"], r["exit_signal"])
                for r in nodes["studio_bridge"].get("runs", [])]
        self.assertEqual(runs, [(0, 0)], nodes["studio_bridge"])


if __name__ == "__main__":
    unittest.main()
