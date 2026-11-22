"""The driver dashboard as a browser lays it out, against a live simulated session.

The unit tests (studio/tests/drive.test.ts) render the page's markup against
a fake surface; layout, clipping, scrolling and the page's own timers exist
only in a browser. This launches the bench robot's physics simulation with
the Studio bridge and a dashboard of its own (studio/tests/browser/robot.toml
and dashboard.toml), opens /drive in the host's headless
Chromium or Chrome over the DevTools protocol, and judges the live DOM
(studio/tests/browser/probe.js): at 1366x768 and 1280x720 nothing is hidden,
cut, smaller than 12 px or without a value, lists show whole rows and count
what is out of view; the alerts and subsystems lists update their out-of-view
counts on scroll and subsystems keeps its scroll position while a restarted
node redraws it; at a narrow viewport the subsystems session label yields
(ellipsizes) while the heading and out-of-view count stay whole; once the
telemetry node stops, every telemetry widget transitions to stale past
STALE_MS (300 ms) and within REQ-OPS-36's 500 ms bound with STALE styling
and its last value kept; a wall-clock jump past STALE_MS triggers sleep
detection until a new tick arrives; returning from a hidden tab repaints
synchronously inside `visibilitychange`; and live HTTP endpoints (`/drive`,
`/`, hashed assets, `/drive/layout.json`, `/drive/summary.json`) serve the
expected status, bodies, and `Cache-Control` headers.

A reserved port another process takes first fails the startup at once,
naming the member and the port, not after STARTUP_S
(HeldHttpPortIsNamedAtOnce holds the bridge's port to prove it), and a graph
the launcher refuses fails it at once naming the refusal
(RefusedGraphIsNamedAtOnce); neither needs a browser. With no browser on the host the browser checks skip, naming why; with
TALOS_REQUIRE_BROWSER=1 that is a failure. A failed check leaves a
screenshot and the failing elements (outerHTML and geometry) in the test's
undeclared outputs and prints their paths.
"""

import atexit
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import tomllib
import unittest
import urllib.request
import uuid

sys.path.insert(0, str(Path(__file__).parent))
import cdp  # noqa: E402
import children  # noqa: E402
import watch  # noqa: E402

CONFIG = "studio/tests/browser/robot.toml"
# The telemetry node's subsystem file, for its period.
TELEMETRY_CONFIG = "bench/main_processor/configuration/subsystems/telemetry.toml"
# Bounds only a hang (docs/TESTING.md §6.3): the whole graph builds its
# nodes, the Python nodes load, and the bridge comes up.
STARTUP_S = 60
WAIT_MS = 30000
# REQ-OPS-36: every value stale within 500 ms of its last update; the page's
# own freshness deadline is STALE_MS (300 ms) + STALE_DRAW_LAG_MS (1 ms).
STALE_MS = 300
STALE_DRAW_LAG_MS = 1
STALE_BOUND_MS = 500
# The viewports REQ-OPS-49 names, one short enough that the subsystems list
# scrolls, one short enough that the alerts list scrolls, and one narrow
# enough that the subsystems header's session label must yield to the heading
# and out-of-view count.
VIEWPORTS = ((1366, 768), (1280, 720))
SHORT_VIEWPORT = (1366, 500)
ALERTS_SCROLL_VIEWPORT = (1366, 300)
NARROW_VIEWPORT = (1280, 720)
# A node the launcher restarts, whose row changes when it does.
RESTARTED = "swerve_module"
PRODUCER = "telemetry"


def workspace():
    return Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")


# The processes the run started (the launcher, the browser): stopped on any
# exit of this process, a SIGTERM (Bazel's timeout) included. Each runs in a
# session of its own, so a signal to this process's group misses them; under
# Bazel on Linux the test runner reaps them anyway, run directly or elsewhere
# nothing else does. Each stop runs once: `_stop` takes it off the list
# first, and each is idempotent besides.
_STOPS = []


def _stop(stop):
    if stop in _STOPS:
        _STOPS.remove(stop)
    stop()


def _stop_all():
    while _STOPS:
        try:
            _stop(_STOPS[-1])
        except Exception as error:  # noqa: BLE001: stop the rest regardless
            print(f"cleanup: {error!r}", flush=True)


def _on_sigterm(signum, _frame):
    print("SIGTERM: stopping the session and the browser", flush=True)
    _stop_all()
    signal.signal(signum, signal.SIG_DFL)
    os.kill(os.getpid(), signum)


atexit.register(_stop_all)
signal.signal(signal.SIGTERM, _on_sigterm)


def free_ports(count):
    """`count` ports free for UDP on every address now, below the host's
    ephemeral range.

    The launcher still refuses port 0 for the gateway link (`--remote-port`,
    `--local-port`; `sim_gateway --port` too), so these are reserved and
    released, and its members bind them a second or more later
    (docs/TESTING.md §6.4). Studio uses `--studio-http-port 0` and reports
    its OS-assigned port in `manifest.json`. Below the ephemeral range no
    connect() or DNS lookup on the host draws one meanwhile; a port taken
    anyway fails the wait for the bridge, naming the port and the member's
    own log line (Session.bind_failure)."""
    ports, port = [], 20000 + os.getpid() % 10000
    for _ in range(12000):
        port = 20000 + (port - 20000 + 1) % 12000  # stays below 32768
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tcp, \
                    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                tcp.bind(("0.0.0.0", port))
                udp.bind(("0.0.0.0", port))
        except OSError:
            continue
        ports.append(port)
        if len(ports) == count:
            return ports
    raise RuntimeError(f"fewer than {count} free ports in 20000-31999")


class Session:
    """The launched bench simulation with the Studio bridge on an OS-assigned
    ephemeral port (`--studio-http-port 0`), or on `http_port` when given."""

    def __init__(self, root, tmp, http_port=None, config=None):
        self.output = tmp / "sessions"
        self.namespace = f"drive-browser-{uuid.uuid4().hex}"
        gw_port, bridge_port = free_ports(2)
        self.http_port = http_port
        http_port_arg = "0" if http_port is None else str(http_port)
        # Each member that binds an explicit port, the port, and what its log
        # says when the bind fails (studio/bridge/main.cc, which
        # studio/bridge/listen_test.py holds to naming the port;
        # sim_gateway.cc; talOS/bridge/main.cc). HeldHttpPortIsNamedAtOnce
        # below proves the bridge's still matches.
        binds = []
        if http_port is not None:
            binds.append(("studio_bridge", http_port,
                          f"cannot listen on port {http_port}"))
        binds.extend([("sim_gateway", gw_port, "cannot open simulation socket"),
                      ("hardware_node", bridge_port,
                       "failed to open hardware node network/ipc resources")])
        self.binds = tuple(binds)
        # The launcher resolves node labels and [studio] webroot under
        # bazel-bin/ of its working directory: the runfiles tree.
        (tmp / "bazel-bin").symlink_to(root, target_is_directory=True)
        self.log_path = tmp / "launcher.txt"
        self.log = self.log_path.open("w")
        args = [str(root / "talOS/launcher/launcher"), "--config", str(config or root / CONFIG),
                "--sim", "--start-sim-gateway", "--start-studio-bridge",
                "--studio-http-port", http_port_arg,
                "--remote", "127.0.0.1", "--remote-port", str(gw_port),
                "--local-port", str(bridge_port), "--output-dir", str(self.output),
                # The telemetry node is held stopped for about 400 ms below,
                # inside its 10 ms period's 500 ms floor: a longer stall
                # timeout keeps the launcher from killing it meanwhile.
                "--heartbeat-periods", "300",
                "--sim-gateway-arg", "--physics"]
        env = dict(os.environ, TALOS_SHM_NAMESPACE=self.namespace)
        if sys.platform == "darwin":
            env["TALOS_REALTIME"] = "1"
        self.launcher = children.Child(args, cwd=tmp, env=env, stdout=self.log,
                                       stderr=subprocess.STDOUT)
        _STOPS.append(self.stop)
        self.stopped = False
        self.tmp = tmp
        self.dir = None
        # The launcher's exit writes nothing wait_listening watches (its log
        # is this process's open file), so a refused graph would wait out
        # STARTUP_S: a helper waits for the exit and creates a file there.
        exited = tmp / "launcher.exited"
        threading.Thread(target=lambda: self.launcher.wait_exited(STARTUP_S)
                         and exited.touch(), daemon=True).start()

    def wait_listening(self):
        """The session directory, once manifest.json records studio_bridge's http_port."""

        def listening():
            # A failed bind first: a critical member's can end the session,
            # and the launcher's exit would then hide which port was lost.
            failure = self.bind_failure()
            if failure:
                raise RuntimeError(failure)
            if self.launcher.exited():
                lines = [line.strip() for line in self.log_path.read_text(
                    errors="replace").splitlines() if line.strip()]
                # Its error lines, or else its last words (a refusal that
                # says "cannot ..." without the word).
                said = [line for line in lines if "error" in line] or lines[-3:]
                raise RuntimeError("launcher exited during startup: " +
                                   (" | ".join(said) or "it printed nothing"))
            for manifest in self.output.glob("*/manifest.json"):
                try:
                    nodes = json.loads(manifest.read_text(errors="replace"))["nodes"]
                except (OSError, json.JSONDecodeError, KeyError):
                    continue
                bridge = next((n for n in nodes if n.get("name") == "studio_bridge"), None)
                if bridge is not None:
                    port = bridge.get("http_port")
                    if isinstance(port, int) and port > 0 and (
                            self.http_port is None or port == self.http_port):
                        self.http_port = port
                        return manifest.parent
            return None

        def paths():
            return [self.tmp, self.output, *self.output.glob("*/"),
                    *self.output.glob("*/manifest.json"),
                    *self.output.glob("*/*.log")]

        self.dir = watch.wait_for(listening, paths, STARTUP_S,
                                  "the Studio bridge's http_port in manifest.json")

    def bind_failure(self):
        """A member's failed bind of its reserved port, named, or None."""
        for member, port, marker in self.binds:
            for log in self.output.glob(f"*/{member}.log"):
                line = next((line for line in log.read_text(errors="replace").splitlines()
                             if marker in line), None)
                if line:
                    return (f"{member} could not bind its reserved port {port} (taken after "
                            f"free_ports released it?): {log.name} says {line!r} "
                            "(the session's logs are kept with the outputs)")
        return None

    def pid(self, name):
        nodes = json.loads((self.dir / "manifest.json").read_text())["nodes"]
        return next(n["pid"] for n in nodes if n["name"] == name)

    def stop(self, grace_s=10):
        """Stops the session as an operator does (SIGINT: the launcher stops
        its members, each in a process group of its own); SIGKILLs the
        launcher after grace_s, whose members then die with it (Linux:
        PR_SET_PDEATHSIG). Once: a second call does nothing."""
        if self.stopped:
            return
        self.stopped = True
        if not self.launcher.exited():
            self.launcher.signal_group(signal.SIGINT)
        try:
            self.launcher.reap(timeout=grace_s)
        except subprocess.TimeoutExpired:
            self.launcher.signal_group(signal.SIGKILL)
            self.launcher.reap()
        self.log.close()


class DriveDashboardInBrowser(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        binary = cdp.find_browser()
        if binary is None:
            reason = ("no Chromium or Chrome on this host (TALOS_BROWSER, "
                      f"{', '.join(cdp.BROWSER_NAMES)} on PATH {os.environ.get('PATH', '')!r}, "
                      f"or {cdp.MACOS_CHROME})")
            if os.environ.get("TALOS_REQUIRE_BROWSER") == "1":
                raise AssertionError(f"TALOS_REQUIRE_BROWSER=1, but {reason}")
            # Bazel reports a skipped py_test as passed: the log says why.
            print(f"SKIPPED: {reason}", flush=True)
            raise unittest.SkipTest(reason)
        root = workspace()
        cls.tmp = Path(tempfile.mkdtemp(prefix="drive-browser-", dir=os.environ.get("TEST_TMPDIR")))
        cls.outputs = Path(os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR", cls.tmp))
        cls.session = None
        cls.browser = None
        try:
            cls.session = Session(root, cls.tmp)
            cls.session.wait_listening()
            cls.browser = cdp.Browser(binary, cls.tmp, register=_STOPS.append)
            cls.page = cls.browser.page
            cls.page.viewport(*VIEWPORTS[0])
            probe = (Path(__file__).parent / "probe.js").read_text()
            cls.page.call("Page.addScriptToEvaluateOnNewDocument", source=probe)
            cls.page.call("Page.navigate", url=f"http://127.0.0.1:{cls.session.http_port}/drive")
            cls.page.evaluate(f"__probe.waitFor(() => document.querySelector('#drive .page'), "
                              f"{WAIT_MS}, 'the dashboard drawn')")
            # The session is up once every node's row reads UP (the Python
            # nodes are the slowest to register).
            cls.page.evaluate(f"__probe.waitFor(__probe.allUp, {STARTUP_S * 1000}, "
                              "'every subsystems row UP')")
        except BaseException:
            cls.kept = True
            cls.tearDownClass()
            raise

    @classmethod
    def tearDownClass(cls):
        failed = False
        if cls.browser is not None:
            _stop(cls.browser.close)
        if cls.session is not None:
            _stop(cls.session.stop)
            failed = cls.session.launcher.returncode not in (0, -signal.SIGINT)
        if cls.session is not None and (failed or getattr(cls, "kept", False)):
            kept = cls.outputs / "session"
            if cls.session.output.exists():
                shutil.copytree(cls.session.output, kept, dirs_exist_ok=True,
                                ignore=shutil.ignore_patterns("*.tlog"))
            kept.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(cls.session.log_path, kept / "launcher.txt")
            if (cls.tmp / "browser.log").exists():
                shutil.copyfile(cls.tmp / "browser.log", kept / "browser.log")
            print(f"session logs kept: {kept}", flush=True)
            for log in sorted(kept.glob("**/studio_bridge.log")) + [kept / "launcher.txt"]:
                print(f"--- {log} (tail)\n" + log.read_text(errors="replace")[-4000:], flush=True)
        (cls.tmp / "bazel-bin").unlink(missing_ok=True)
        shutil.rmtree(cls.tmp, ignore_errors=True)

    # --- helpers -----------------------------------------------------------

    def call_probe(self, expression):
        return self.page.evaluate(f"(async () => {{ {expression} }})()")

    def evidence(self, label, failures):
        """Saves a screenshot and the failures; returns their paths."""
        type(self).kept = True
        shot = self.outputs / f"{label}.png"
        dump = self.outputs / f"{label}.json"
        self.page.screenshot(shot)
        dump.write_text(json.dumps(failures, indent=2))
        print(f"{label}: screenshot {shot}\n{label}: failing elements {dump}", flush=True)
        return shot, dump

    def check(self, label, failures):
        if not failures:
            return
        shot, dump = self.evidence(label, failures)
        lines = [f"{f['check']}: {f['element']}: {f['detail']}" for f in failures[:20]]
        more = f"\n... and {len(failures) - 20} more" if len(failures) > 20 else ""
        self.fail(f"{label}: {len(failures)} failures\n" + "\n".join(lines) + more +
                  f"\nscreenshot {shot}\nelements {dump}")

    def wait(self, label, predicate, what, bound_ms=WAIT_MS):
        """Awaits a page condition (a JS expression), saving evidence on a timeout."""
        try:
            return self.call_probe(
                f"return __probe.waitFor(() => {predicate}, {bound_ms}, {json.dumps(what)})")
        except cdp.EvaluateError as error:
            failures = self.page.evaluate("__probe.audit()")
            shot, dump = self.evidence(label, failures)
            self.fail(f"{label}: {error}\nscreenshot {shot}\nelements {dump}")

    def settle(self, label, size):
        self.page.viewport(*size)
        self.call_probe("await __probe.frames(); return true")
        self.wait(label, "__probe.current()", "status OK and no widget stale")

    # --- the checks ----------------------------------------------------------

    # REQ-OPS-49: in the viewports it names nothing is hidden, cut or
    # smaller than 12 px; every list row shows whole; every widget a value.
    def test_layout_shows_everything_whole(self):
        for width, height in VIEWPORTS:
            label = f"layout-{width}x{height}"
            with self.subTest(viewport=f"{width}x{height}"):
                self.settle(label, (width, height))
                # The checks and summary fill in as the graph starts.
                self.wait(label, "__probe.valued()", "every widget with a value")
                self.call_probe("await __probe.frames(); return true")
                # Kept whatever the result, to see what was judged.
                self.page.screenshot(self.outputs / f"{label}-seen.png")
                self.check(label, self.page.evaluate("__probe.audit()"))
                # dashboard.toml's lists fit whole at these sizes (REQ-OPS-49).
                for kind in ("subsystems", "checks"):
                    state = self.page.evaluate(f"__probe.listState('{kind}')")
                    self.assertLessEqual(state["scrollHeight"], state["clientHeight"],
                                         f"{label}: the {kind} list scrolls: {state}")

    # REQ-OPS-37, REQ-OPS-49: the alerts list updates its out-of-view badge on
    # scroll in a browser, counting alerts above/below as the body scrolls.
    def test_alerts_out_of_view_count_updates_on_scroll(self):
        label = f"alerts-scroll-{ALERTS_SCROLL_VIEWPORT[0]}x{ALERTS_SCROLL_VIEWPORT[1]}"
        self.settle(label, ALERTS_SCROLL_VIEWPORT)
        # Pause sim_gateway briefly so hardware_node raises alert_error_link_lost
        # (creating both an Active row and a Raised/latched row in .w-alerts).
        gw_pid = self.session.pid("sim_gateway")
        os.kill(gw_pid, signal.SIGSTOP)
        try:
            self.wait(label, "__probe.listState('alerts').rows.length >= 2",
                      "active and latched link-lost alert rows")
            self.call_probe("""
                document.querySelector('.w-alerts .body').scrollTop = 0;
                await __probe.frames();
                return true;""")
            top_state = self.page.evaluate("__probe.listState('alerts')")
            self.assertGreater(top_state["scrollHeight"], top_state["clientHeight"],
                               f"{label}: the alerts list does not overflow: {top_state}")
            self.assertRegex(top_state["badge"], r"^\+\d+ below$", f"{label}: {top_state}")
            self.assertEqual(top_state["badge"], top_state["measured"], f"{label}: {top_state}")

            bottom_state = self.call_probe(f"""
                const b = document.querySelector('.w-alerts .body');
                b.scrollTop = b.scrollHeight - b.clientHeight;
                await __probe.waitFor(() => {{
                  const s = __probe.listState('alerts');
                  return s.badge === s.measured && s.badge.includes('above');
                }}, {WAIT_MS}, 'alerts heading counts rows above on scroll');
                return __probe.listState('alerts');""")
            self.assertGreater(bottom_state["scrollTop"], 0, label)
            self.assertRegex(bottom_state["badge"], r"^\d+ above", f"{label}: {bottom_state}")

            back_state = self.call_probe(f"""
                const b = document.querySelector('.w-alerts .body');
                b.scrollTop = 0;
                await __probe.waitFor(() => {{
                  const s = __probe.listState('alerts');
                  return s.badge === s.measured && /^\\+\\d+ below$/.test(s.badge);
                }}, {WAIT_MS}, 'alerts heading counts rows below after scrolling back to top');
                return __probe.listState('alerts');""")
            self.assertEqual(back_state["scrollTop"], 0, label)
            print(f"{label}: alerts badge {top_state['badge']!r} -> {bottom_state['badge']!r} "
                  f"-> {back_state['badge']!r}", flush=True)
        finally:
            os.kill(gw_pid, signal.SIGCONT)
        self.wait(label, "__probe.current()", "current again after sim_gateway resumed")
        # Reload so the latched link-lost alert from this test does not leak
        # into subsequent tests sharing the browser page.
        self.page.reload()
        self.page.evaluate(f"__probe.waitFor(() => document.querySelector('#drive .page'), "
                           f"{WAIT_MS}, 'the dashboard redrawn after reload')")
        self.page.evaluate(f"__probe.waitFor(__probe.allUp, {WAIT_MS}, "
                           "'every subsystems row UP after reload')")
        self.wait(label, "__probe.current()", "current again after reload")

    # REQ-OPS-49: a list that overflows says how many rows are out of view,
    # keeps its scroll position while a row's markup changes (a restarted
    # node), and keeps showing whole rows.
    def test_list_keeps_its_scroll_across_redraws(self):
        label = f"scroll-{SHORT_VIEWPORT[0]}x{SHORT_VIEWPORT[1]}"
        self.settle(label, SHORT_VIEWPORT)
        state = self.page.evaluate("__probe.listState('subsystems')")
        self.assertGreater(state["scrollHeight"], state["clientHeight"],
                           f"{label}: the subsystems list does not overflow: {state}")
        self.assertRegex(state["badge"], r"^\+\d+ below$", f"{label}: {state}")
        self.check(label, self.page.evaluate("__probe.audit('rows')"))
        for where in ("bottom", "middle"):
            step = f"{label}-{where}"
            # Scroll by script, on a row boundary; the heading follows.
            top = self.call_probe(f"""
                const b = document.querySelector('.w-subsystems .body');
                const max = b.scrollHeight - b.clientHeight;
                b.scrollTop = {'max' if where == 'bottom' else 'Math.floor(max / 2 / 22) * 22'};
                const top = b.scrollTop;
                await __probe.waitFor(() => {{
                  const s = __probe.listState('subsystems');
                  return s.badge === s.measured && s.badge.includes('above');
                }}, {WAIT_MS}, 'the heading counts rows above');
                return top;""")
            self.assertGreater(top, 0, step)
            scrolled = self.page.evaluate("__probe.listState('subsystems')")
            if where == "middle":
                self.assertRegex(scrolled["badge"], r"^\d+ above · \+\d+ below$",
                                 f"{step}: middle scroll should show both above and below: {scrolled}")
            else:
                self.assertRegex(scrolled["badge"], r"^\d+ above$",
                                 f"{step}: bottom scroll should show only above: {scrolled}")
            self.check(f"{step}-scrolled", self.page.evaluate("__probe.audit('rows')"))
            before = self.call_probe(f"""
                const row = [...document.querySelectorAll('.w-subsystems li')]
                    .find(li => li.querySelector('.name')?.textContent === '{RESTARTED}');
                return row.textContent;""")
            os.kill(self.session.pid(RESTARTED), signal.SIGKILL)
            after = self.wait(step, f"""(() => {{
                const row = [...document.querySelectorAll('.w-subsystems li')]
                    .find(li => li.querySelector('.name')?.textContent === '{RESTARTED}');
                return row && row.textContent !== {json.dumps(before)} && row.textContent;
              }})()""", f"the {RESTARTED} row to change")
            state = self.page.evaluate("__probe.listState('subsystems')")
            if state["scrollTop"] != top:
                failures = self.page.evaluate("__probe.audit('rows')") + [{
                    "check": "scroll position lost", "element": ".w-subsystems > div.body",
                    "detail": f"scrollTop {top} before the redraw, {state['scrollTop']} after",
                    "state": state}]
                self.check(step, failures)
            self.check(f"{step}-redrawn", self.page.evaluate("__probe.audit('rows')"))
            print(f"{step}: {RESTARTED} row {before!r} -> {after!r}, scrollTop {top} kept",
                  flush=True)
            # Back up before the next kill, so it is a restart, not a stall.
            self.wait(step, f"""(() => {{
                const row = [...document.querySelectorAll('.w-subsystems li')]
                    .find(li => li.querySelector('.name')?.textContent === '{RESTARTED}');
                return row && row.querySelector('.tag-good') !== null;
              }})()""", f"{RESTARTED} up again")
        self.call_probe("document.querySelector('.w-subsystems .body').scrollTop = 0; "
                        "await __probe.frames(); return true")

    # REQ-OPS-49: where a list overflows, its heading still shows whole:
    # the heading, the out-of-view count and every word in the list (the
    # session label gives way first, README.md "Driver dashboard").
    def test_overflowing_list_heading_is_whole(self):
        label = f"heading-{SHORT_VIEWPORT[0]}x{SHORT_VIEWPORT[1]}"
        self.settle(label, SHORT_VIEWPORT)
        self.call_probe("document.querySelector('.w-subsystems .body').scrollTop = 0; "
                        "await __probe.frames(); return true")
        state = self.page.evaluate("__probe.listState('subsystems')")
        self.assertGreater(state["scrollHeight"], state["clientHeight"],
                           f"{label}: the subsystems list does not overflow: {state}")
        self.check(label, self.page.evaluate("__probe.audit('lists')"))

    # REQ-OPS-49: where the subsystems header cannot fit its title, session
    # label and (when overflowing) out-of-view count together, the session
    # label ellipsizes first while the heading and out-of-view count stay whole
    # inside the header.
    def test_session_label_yields_at_narrow_viewport(self):
        for viewport, expect_badge in ((NARROW_VIEWPORT, False), (SHORT_VIEWPORT, True)):
            label = f"session-yield-{viewport[0]}x{viewport[1]}"
            with self.subTest(viewport=f"{viewport[0]}x{viewport[1]}"):
                self.settle(label, viewport)
                self.call_probe("document.querySelector('.w-subsystems .body').scrollTop = 0; "
                                "await __probe.frames(); return true")
                header = self.page.evaluate("__probe.headerState('subsystems')")
                meta = header["meta"]
                self.assertIsNotNone(meta, f"{label}: no session label in subsystems header: {header}")
                self.assertTrue(meta["text"].startswith("session "), f"{label}: {meta}")
                self.assertEqual(meta["textOverflow"], "ellipsis", f"{label}: {meta}")
                self.assertTrue(meta["insideHeader"], f"{label}: session label outside header: {header}")
                self.assertTrue(
                    meta["scrollWidth"] > meta["clientWidth"] or meta["textWidth"] > meta["contentWidth"],
                    f"{label}: expected session label to yield/ellipsize: {header}",
                )
                self.assertTrue(header["h2"]["insideHeader"], f"{label}: {header}")
                if expect_badge:
                    self.assertTrue(header["badge"]["insideHeader"], f"{label}: {header}")
                    self.assertRegex(header["badge"]["text"], r"^\+\d+ below$", f"{label}: {header}")
                self.check(label, self.page.evaluate("__probe.audit('lists')"))
                print(f"{label}: session label {meta['text']!r} yielded "
                      f"({meta['textWidth']}px in {meta['contentWidth']}px), "
                      f"heading {header['h2']['text']!r} and badge {header['badge']['text']!r} whole",
                      flush=True)

    # REQ-OPS-36: once the producer stops, every telemetry widget reads
    # stale past STALE_MS (~301 ms after its last tick) and within 500 ms of
    # the producer's last tick, keeping its last value dimmed and struck
    # through, and the status line names telemetry; when it resumes the page
    # is current again.
    def test_stopped_telemetry_reads_stale_in_time(self):
        label = "freshness"
        self.settle(label, VIEWPORTS[0])
        sub = tomllib.loads((workspace() / TELEMETRY_CONFIG).read_text())["subsystem"]
        period_ms = sub["period_us"] / 1000
        pid = self.session.pid(PRODUCER)
        armed_at = self.page.evaluate(f"__probe.armStale({WAIT_MS})")
        os.kill(pid, signal.SIGSTOP)
        # Not part of the bound: how much of the measurement is the harness
        # (the page's reply reaching this process, and this process's turn
        # to stop the producer), printed so a failure shows it.
        stopped_by = self.page.evaluate("performance.now()")
        try:
            try:
                result = self.page.evaluate("window.__stale")
            except cdp.EvaluateError as error:
                failures = self.page.evaluate("__probe.audit()")
                shot, dump = self.evidence(label, failures)
                self.fail(f"{label}: {error}\nscreenshot {shot}\nelements {dump}")
        finally:
            os.kill(pid, signal.SIGCONT)
        # The producer's last tick was no earlier than one period before the
        # stop, which followed armed_at: this bounds stale-after-last-tick.
        latency_ms = result["staleAt"] - armed_at + period_ms
        deadline_timer = result["deadlineTimer"]
        self.assertIsNotNone(deadline_timer, f"{label}: DrawSchedule did not arm a stale timer: {result}")
        last_tick_at = deadline_timer["targetAt"] - (STALE_MS + STALE_DRAW_LAG_MS)
        since_last_tick_ms = result["staleAt"] - last_tick_at
        print(f"{label}: {result['widgets']} widgets stale {since_last_tick_ms:.1f} ms after last tick "
              f"(target {STALE_MS + STALE_DRAW_LAG_MS} ms, deadline_timer={result['firedByDeadlineTimer']}), "
              f"{result['staleAt'] - armed_at:.1f} ms after clock read before SIGSTOP (+{period_ms:.0f} ms "
              f"= {latency_ms:.1f} ms, bound {STALE_BOUND_MS} ms); status {result['status']['text']!r} "
              f"at {result['namedAt'] - armed_at:.1f} ms; stop within {stopped_by - armed_at:.1f} ms",
              flush=True)
        # Must not go stale before STALE_MS (300 ms) from its last new tick.
        self.assertGreaterEqual(since_last_tick_ms, STALE_MS - 1.0,
                                f"{label}: widgets went stale early ({since_last_tick_ms:.1f} ms < {STALE_MS} ms): {result}")
        if latency_ms > STALE_BOUND_MS:
            self.check(label, [{
                "check": "stale too late", "element": ".w", "detail":
                f"{latency_ms:.1f} ms from the producer's last tick at the latest, "
                f"bound {STALE_BOUND_MS} ms (REQ-OPS-36); the stop came within "
                f"{stopped_by - armed_at:.1f} ms of the page's clock read", "result": result}])
        # Every telemetry widget must render stale visually (STALE tag, dimmed
        # body, struck-through value) while preserving its last value.
        for style in result["styles"]:
            self.assertTrue(style["staleClass"], f"{label}: {style}")
            self.assertEqual(style["tag"], "STALE", f"{label}: {style}")
            self.assertLess(style["opacity"], 0.6, f"{label}: body not dimmed: {style}")
            if style["lineThrough"] is not None:
                self.assertTrue(style["lineThrough"], f"{label}: value not struck through: {style}")
            self.assertEqual(style["absentCount"], 0, f"{label}: stale widget lost its value: {style}")
        self.assertIn("TELEMETRY", result["status"]["text"], result)
        self.wait(label, "__probe.current()", "current again after the producer resumed")

    # REQ-OPS-36: when the wall clock jumps more than STALE_MS ahead of
    # performance.now() (laptop sleep), DrawSchedule suspends the model so
    # every widget (including subsystems) reads STALE and the status line
    # names the sleep until a new telemetry tick arrives.
    def test_sleep_detection_marks_all_widgets_stale_until_new_tick(self):
        label = "sleep-detection"
        self.settle(label, VIEWPORTS[0])
        pid = self.session.pid(PRODUCER)
        os.kill(pid, signal.SIGSTOP)
        try:
            self.page.evaluate(f"__probe.shiftWallClock({STALE_MS + 700})")
            suspended = self.wait(label, """(() => {
                const s = __probe.status();
                const widgets = [...document.querySelectorAll('.w')];
                if (!s || s.link !== 'link-telemetry' || !s.text.includes('laptop asleep'))
                  return false;
                if (widgets.length === 0 || !widgets.every(w => w.classList.contains('stale')))
                  return false;
                return {status: s, widgets: widgets.length,
                        subsystemsStale: document.querySelector('.w-subsystems')?.classList.contains('stale')};
              })()""", "sleep detection marks every widget stale and names laptop asleep")
            self.assertTrue(suspended["subsystemsStale"], f"{label}: {suspended}")
            self.assertIn("TELEMETRY", suspended["status"]["text"], suspended)
            print(f"{label}: {suspended['widgets']} widgets (subsystems included) stale; "
                  f"status {suspended['status']['text']!r}", flush=True)
        finally:
            os.kill(pid, signal.SIGCONT)
        self.wait(label, "__probe.current()", "current again after new tick clears sleep suspend")

    # REQ-OPS-36: returning to a hidden tab triggers an immediate synchronous
    # DrawSchedule.frame() redraw inside Chromium's visibilitychange event,
    # rather than waiting for the periodic render timer.
    def test_visibilitychange_redraws_immediately_on_return(self):
        label = "visibilitychange"
        self.settle(label, VIEWPORTS[0])
        self.page.evaluate(f"__probe.armVisibilityReturn({STALE_MS + 700}, {WAIT_MS})")
        other = self.browser.browser.call("Target.createTarget", url="about:blank")["targetId"]
        try:
            self.browser.browser.call("Target.activateTarget", targetId=other)
            self.page.evaluate("window.__visHidden")
            self.browser.browser.call("Target.activateTarget", targetId=self.browser.target_id)
            result = self.page.evaluate("window.__visReturn")
        finally:
            self.browser.browser.call("Target.closeTarget", targetId=other)
            self.browser.browser.call("Target.activateTarget", targetId=self.browser.target_id)
        self.assertTrue(result["sawHidden"], f"{label}: {result}")
        self.assertEqual(result["before"]["status"]["link"], "link-ok", f"{label}: {result}")
        self.assertEqual(result["before"]["staleCount"], 0, f"{label}: {result}")
        self.assertEqual(result["after"]["status"]["link"], "link-telemetry", f"{label}: {result}")
        self.assertIn("laptop asleep", result["after"]["status"]["text"], f"{label}: {result}")
        self.assertEqual(result["after"]["staleCount"], result["after"]["totalWidgets"],
                         f"{label}: visibilitychange did not repaint all widgets synchronously: {result}")
        print(f"{label}: hidden -> visible repainted synchronously "
              f"({result['before']['status']['link']} -> {result['after']['status']['link']}, "
              f"{result['after']['staleCount']}/{result['after']['totalWidgets']} stale)",
              flush=True)
        self.wait(label, "__probe.current()", "current again on next live tick after tab return")

    # REQ-OPS-34, REQ-OPS-39: live bridge serves /drive, /, its Vite
    # content-hashed assets, /drive/layout.json, and /drive/summary.json with
    # the required Cache-Control headers and layout contents.
    def test_live_http_endpoints_and_cache_control(self):
        base = f"http://127.0.0.1:{self.session.http_port}"

        def fetch(path):
            with urllib.request.urlopen(f"{base}{path}", timeout=10) as resp:
                return resp.status, dict(resp.headers.items()), resp.read().decode("utf-8")

        status, headers, html = fetch("/drive")
        self.assertEqual(status, 200)
        self.assertTrue(headers.get("Content-Type", "").startswith("text/html"), headers)
        self.assertEqual(headers.get("Cache-Control"), "no-cache", headers)
        self.assertEqual(headers.get("X-Content-Type-Options"), "nosniff", headers)
        self.assertIn('id="drive"', html)

        assets = re.findall(r'(?:src|href)="(/assets/[^"]+)"', html)
        self.assertGreaterEqual(len(assets), 2, f"/drive HTML missing hashed assets: {html[:500]}")
        for asset in assets:
            a_status, a_headers, a_body = fetch(asset)
            self.assertEqual(a_status, 200, asset)
            self.assertEqual(a_headers.get("Cache-Control"),
                             "public, max-age=31536000, immutable",
                             f"{asset}: {a_headers}")
            self.assertEqual(a_headers.get("X-Content-Type-Options"), "nosniff", f"{asset}: {a_headers}")
            self.assertGreater(len(a_body), 0, asset)

        idx_status, idx_headers, _ = fetch("/")
        self.assertEqual(idx_status, 200)
        self.assertEqual(idx_headers.get("Cache-Control"), "no-cache", idx_headers)

        l_status, l_headers, l_body = fetch("/drive/layout.json")
        self.assertEqual(l_status, 200)
        self.assertEqual(l_headers.get("Content-Type"), "application/json", l_headers)
        self.assertEqual(l_headers.get("Cache-Control"), "no-store", l_headers)
        self.assertEqual(l_headers.get("X-Content-Type-Options"), "nosniff", l_headers)
        layout = json.loads(l_body)
        self.assertEqual(layout["title"], "Drive")
        self.assertEqual((layout["columns"], layout["rows"]), (12, 8))
        self.assertEqual(layout["update_hz"], 20)
        self.assertTrue(layout["alerts"])
        self.assertEqual(len(layout["widgets"]), 14)
        self.assertIn("hw/status/alert_error_link_lost", layout["channels"])
        self.assertIn("hw/status/alert_error_protocol_mismatch", layout["channels"])

        s_status, s_headers, s_body = fetch("/drive/summary.json")
        self.assertEqual(s_status, 200)
        self.assertEqual(s_headers.get("Content-Type"), "application/json", s_headers)
        self.assertEqual(s_headers.get("Cache-Control"), "no-store", s_headers)
        summary = json.loads(s_body)
        self.assertEqual(summary["kind"], "talos.drive.summary")
        self.assertTrue(summary["session"])
        self.assertGreaterEqual(len(summary["nodes"]), 8)
        print(f"live HTTP: /drive ({len(assets)} immutable assets), /, "
              f"/drive/layout.json ({len(layout['widgets'])} widgets), "
              f"/drive/summary.json ({len(summary['nodes'])} nodes) verified", flush=True)


def held_port():
    """A listening socket on the wildcard address (dual-stack where the host
    has IPv6), on a port the OS picked: another process's listener, as the
    kernel sees it. Returns (socket, port)."""
    try:
        holder = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        holder.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        address = ("::", 0)
    except OSError:
        holder = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        address = ("0.0.0.0", 0)
    holder.bind(address)
    holder.listen()
    return holder, holder.getsockname()[1]


# REQ-OPS-36: keeps this target from failing for a reason that is not the
# dashboard's without saying so (helpers_test.py does the same for its
# helpers). Needs no browser, so it runs on every host this target does.
class HeldHttpPortIsNamedAtOnce(unittest.TestCase):
    """The HTTP port free_ports reserved is taken by another process before
    the bridge binds it (TODO.md TEST-24's suspected mechanism; ten agents
    build on one host). The bridge exits naming the port (REQ-OPS-34,
    studio/bridge/listen_test.py) and the launcher restarts it, so a harness
    that does not recognise that line waits out STARTUP_S for a listening
    line that never comes and reports only the timeout. The harness holds
    the port itself here, before the launch: the wait must end on the
    bridge's own line, naming the member and the port, not on its bound."""

    def test_a_held_port_fails_the_wait_naming_it(self):
        holder, port = held_port()
        tmp = Path(tempfile.mkdtemp(prefix="held-port-", dir=os.environ.get("TEST_TMPDIR")))
        session = None
        try:
            with holder:
                session = Session(workspace(), tmp, http_port=port)
                with self.assertRaises(RuntimeError) as raised:
                    session.wait_listening()
            print(f"held port {port}: {raised.exception}", flush=True)
            self.assertIn("studio_bridge", str(raised.exception))
            self.assertIn(str(port), str(raised.exception))
        except BaseException:
            if session is not None:
                print("--- launcher.txt (tail)\n" + session.log_path.read_text(
                    errors="replace")[-4000:], flush=True)
            raise
        finally:
            if session is not None:
                _stop(session.stop)
            (tmp / "bazel-bin").unlink(missing_ok=True)
            shutil.rmtree(tmp, ignore_errors=True)


def refused_config(root, tmp):
    """This target's graph with a key no node reads in the motor's file, which
    the launcher refuses as a graph error (talOS/launcher/README.md,
    "Parameter keys"). Returns the config's path."""
    browser = root / CONFIG
    here = browser.parent
    bench = "../../../bench/main_processor/configuration/"
    motor = (here / bench / "subsystems/motor_py.toml").resolve()
    text = browser.read_text()
    for relative, absolute in ((bench + "robot.toml", here / bench / "robot.toml"),
                               ("dashboard.toml", here / "dashboard.toml"),
                               (bench + "subsystems/motor_py.toml", tmp / "motor.toml")):
        text = text.replace(f'"{relative}"', f'"{Path(absolute).resolve()}"')
    (tmp / "motor.toml").write_text(
        motor.read_text().replace('defaults = "motor.toml"',
                                  f'defaults = "{motor.parent / "motor.toml"}"')
        + "\n[unread]\nvalue_m = 1.0\n")
    (tmp / "robot.toml").write_text(text)
    return tmp / "robot.toml"


# REQ-OPS-36, as HeldHttpPortIsNamedAtOnce. Needs no browser.
class RefusedGraphIsNamedAtOnce(unittest.TestCase):
    """The launcher refuses the graph and exits before any member starts, so
    no member log ever appears and its exit writes nothing the wait watches:
    the wait must still end on that exit, naming the launcher's refusal, not
    wait out STARTUP_S for a listening line."""

    def test_a_refused_graph_fails_the_wait_naming_it(self):
        tmp = Path(tempfile.mkdtemp(prefix="refused-", dir=os.environ.get("TEST_TMPDIR")))
        session = None
        try:
            session = Session(workspace(), tmp, config=refused_config(workspace(), tmp))
            with self.assertRaises(RuntimeError) as raised:
                session.wait_listening()
            print(f"refused graph: {raised.exception}", flush=True)
            self.assertIn("unread.value_m", str(raised.exception))
        except BaseException:
            if session is not None:
                print("--- launcher.txt (tail)\n" + session.log_path.read_text(
                    errors="replace")[-4000:], flush=True)
            raise
        finally:
            if session is not None:
                _stop(session.stop)
            (tmp / "bazel-bin").unlink(missing_ok=True)
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
