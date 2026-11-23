"""Studio's bridge runs as a member of this repository's simulated session.

Runs `//:sim --start-studio-bridge` on free loopback ports, waits for the
launcher's line that the bridge is up, fetches `/drive`, stops the launch
with SIGINT as Ctrl-C does, and reads the session's manifest.
"""

import glob
import json
import os
import random
import re
import shutil
import signal
import socket
import subprocess
import sys
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
    """Two distinct free UDP ports on 0.0.0.0 below the OS ephemeral range
    (20000..31999) for the gateway and the hardware bridge, probed and released
    before the launch, since the launcher still refuses port 0 for the gateway
    link (talOS docs/TESTING.md §6.4). Studio uses `--studio-http-port 0` and
    records its bound port in `manifest.json`."""
    port_base, port_span = 20000, 12000
    start = random.randrange(port_span)
    held = []
    ports = []
    try:
        for offset in range(port_span):
            port = port_base + (start + offset) % port_span
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.bind(("0.0.0.0", port))
            except OSError:
                sock.close()
                continue
            held.append(sock)
            ports.append(port)
            if len(ports) == 2:
                return ports[0], ports[1]
    finally:
        for sock in held:
            sock.close()
    raise RuntimeError(
        f"fewer than 2 free UDP ports in "
        f"{port_base}-{port_base + port_span - 1}")


def _copy_tree_best_effort(src, dst):
    if not os.path.isdir(src):
        return
    try:
        os.makedirs(dst, exist_ok=True)
    except OSError:
        return
    for root, dirs, files in os.walk(src):
        rel = os.path.relpath(root, src)
        target_root = dst if rel == "." else os.path.join(dst, rel)
        for name in dirs:
            try:
                os.makedirs(os.path.join(target_root, name), exist_ok=True)
            except OSError:
                pass
        for name in files:
            src_path = os.path.join(root, name)
            dst_path = os.path.join(target_root, name)
            if os.path.islink(src_path) and not os.path.exists(src_path):
                continue
            try:
                os.makedirs(os.path.dirname(dst_path), exist_ok=True)
                shutil.copyfile(src_path, dst_path)
            except OSError:
                pass


def preserve_failure_artifacts(launch_output, sessions_dir,
                               subdir="studio_test_failure"):
    """Best-effort copy of `launch.log` and `sessions_dir` into
    `TEST_UNDECLARED_OUTPUTS_DIR` on failure (talOS docs/TESTING.md §8.2).
    Never raises so copy errors cannot mask the test failure."""
    outputs_dir = os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR")
    if not outputs_dir:
        return None
    has_sessions = False
    if sessions_dir and os.path.isdir(sessions_dir):
        try:
            has_sessions = bool(os.listdir(sessions_dir))
        except OSError:
            has_sessions = False
    if not launch_output and not has_sessions:
        return None
    try:
        os.makedirs(outputs_dir, exist_ok=True)
        dest = None
        for index in range(1, 1000):
            name = subdir if index == 1 else f"{subdir}_{index}"
            candidate = os.path.join(outputs_dir, name)
            try:
                os.makedirs(candidate, exist_ok=False)
                dest = candidate
                break
            except FileExistsError:
                continue
        if dest is None:
            dest = tempfile.mkdtemp(prefix=f"{subdir}_", dir=outputs_dir)
    except OSError:
        return None
    if launch_output:
        try:
            with open(os.path.join(dest, "launch.log"), "w",
                      encoding="utf-8", errors="replace") as f:
                f.write(launch_output)
        except OSError:
            pass
    if has_sessions:
        _copy_tree_best_effort(sessions_dir, os.path.join(dest, "sessions"))
    return dest


class StudioBridgeInSession(unittest.TestCase):
    # REQ-DEV-30
    def test_the_bridge_serves_drive_and_exits_zero(self):
        """Guards this repository's Studio wiring: talOS's bridge builds
        here through @talos, robot.toml's [studio] webroot and //:sim's
        data give it a folder that exists, and the launcher runs it as a
        member. A bridge the launcher drops leaves no manifest row, so the
        row is required here rather than only its exit."""
        with tempfile.TemporaryDirectory(
                dir=os.environ.get("TEST_TMPDIR")) as tmp:
            out = os.path.join(tmp, "sessions")
            os.makedirs(out)
            launch_output = []
            try:
                self.check_session(out, launch_output)
            except BaseException as error:
                kept = preserve_failure_artifacts(
                    "".join(launch_output), out)
                if isinstance(error, AssertionError):
                    # The bridge's own output, which the launcher does not
                    # echo: a port another process took first reads there, not
                    # in the launch's lines (talOS docs/TESTING.md §6.4).
                    logs = glob.glob(
                        os.path.join(out, "*", "studio_bridge.log"))
                    text = "".join(
                        open(log, encoding="utf-8", errors="replace").read()
                        for log in logs)
                    note = f"\nartifacts preserved: {kept}" if kept else ""
                    raise AssertionError(
                        f"{error}\n--- studio_bridge.log ---\n"
                        f"{text or '(none written)'}{note}") from None
                if kept:
                    print(f"studio_test: failure artifacts preserved: {kept}",
                          file=sys.stderr)
                raise

    def check_session(self, out, output):
        sim = runfiles.Create().Rlocation("_main/sim")
        gateway, bridge = free_ports()
        launch = subprocess.Popen(
            [sim, "--start-studio-bridge", "--studio-http-port", "0",
             "--output-dir", out, "--remote", "127.0.0.1",
             "--remote-port", str(gateway), "--local-port", str(bridge),
             "--duration-s", str(HANG_S)],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, start_new_session=True)
        up = threading.Event()

        def read():
            for line in launch.stdout:
                output.append(line)
                if ("Started studio_bridge" in line
                        or "studio_bridge will not be restarted again" in line
                        or "studio_bridge not executable" in line):
                    up.set()
            up.set()  # the launch ended: nothing more will come

        def read_http_port():
            for path in glob.glob(os.path.join(out, "*", "manifest.json")):
                try:
                    with open(path, encoding="utf-8") as f:
                        nodes = {n["name"]: n for n in json.load(f)["nodes"]}
                    port = nodes.get("studio_bridge", {}).get("http_port")
                    if isinstance(port, int) and port > 0:
                        return port
                except (OSError, json.JSONDecodeError, KeyError):
                    pass
            return None

        reader = threading.Thread(target=read, daemon=True)
        reader.start()
        try:
            self.assertTrue(up.wait(HANG_S), f"no studio_bridge line in "
                            f"{HANG_S} s; the launch printed:\n"
                            + "".join(output[-60:]))
            log = "".join(output)
            self.assertIn("Started studio_bridge", log, log)
            studio = read_http_port()
            if studio is None:
                wait = threading.Event()
                for _ in range(HANG_S * 20):
                    studio = read_http_port()
                    if studio is not None or launch.poll() is not None:
                        break
                    wait.wait(0.05)
            self.assertIsNotNone(
                studio,
                f"manifest.json did not record studio_bridge http_port:\n{log}")
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
            # fails here too) and the talOS checkout to build in; when
            # TALOS_REQUIRE_STUDIO_PAGES=1 is set (as in CI), unbuilt pages
            # fail the test instead.
            webroot = runfiles.Create().Rlocation("_main/studio_webroot")
            built = not os.path.exists(os.path.join(webroot, "NOT_BUILT.txt"))
            require_pages = os.environ.get(
                "TALOS_REQUIRE_STUDIO_PAGES", "").strip() not in ("", "0")
            if require_pages and not built:
                self.fail(
                    f"TALOS_REQUIRE_STUDIO_PAGES is set, but Studio's pages "
                    f"are not built ({webroot}/NOT_BUILT.txt exists; HTTP "
                    f"{status}): {body}")
            if built:
                self.assertEqual(status, 200, body)
                self.assertIn("<html", body.lower())
                self.assertTrue(
                    os.path.isfile(os.path.join(webroot, "STUDIO_STAMP.txt")),
                    f"built webroot {webroot} is missing STUDIO_STAMP.txt")
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
        self.assertEqual(nodes["studio_bridge"].get("http_port"), studio,
                         nodes["studio_bridge"])
        runs = [(r["exit_code"], r["exit_signal"])
                for r in nodes["studio_bridge"].get("runs", [])]
        self.assertEqual(runs, [(0, 0)], nodes["studio_bridge"])


if __name__ == "__main__":
    unittest.main()
