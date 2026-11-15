"""The driver dashboard as a browser lays it out, against a live simulated session.

The unit tests (studio/tests/drive.test.ts) render the page's markup against
a fake surface; layout, clipping, scrolling and the page's own timers exist
only in a browser. This launches the 2026 physics simulation with the Studio
bridge (as `./sim.sh --sim-gateway` does), opens /drive in the host's headless
Chromium or Chrome over the DevTools protocol, and judges the live DOM
(studio/tests/browser/probe.js): at 1366x768 and 1280x720 nothing is hidden,
cut, smaller than 12 px or without a value, lists show whole rows and count
what is out of view; at 1366x500 the subsystems list keeps its scroll
position while a restarted node redraws it; and the page shows telemetry
stale within REQ-OPS-36's bound once the telemetry node stops.

With no browser on the host the test skips, naming why; with
TALOS_REQUIRE_BROWSER=1 that is a failure. A failed check leaves a
screenshot and the failing elements (outerHTML and geometry) in the test's
undeclared outputs and prints their paths.
"""

import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import tomllib
import unittest
import uuid

sys.path.insert(0, str(Path(__file__).parent))
import cdp  # noqa: E402
import watch  # noqa: E402

import autonomous_progress  # noqa: E402

CONFIG = "2026-robot/main_processor/configuration/robot_physics_sim.toml"
# Bounds only a hang (docs/TESTING.md §6.3): the whole graph builds its
# nodes, the cameras load their models, and the bridge comes up.
STARTUP_S = 60
WAIT_MS = 30000
# REQ-OPS-36: every value stale within 500 ms of its last update.
STALE_BOUND_MS = 500
# The viewports REQ-OPS-49 names, and one short enough that the 2026
# subsystems list scrolls.
VIEWPORTS = ((1366, 768), (1280, 720))
SHORT_VIEWPORT = (1366, 500)
# A node the launcher restarts, whose row changes when it does.
RESTARTED = "shooter"
PRODUCER = "telemetry"


def workspace():
    return Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")


def free_ports(count):
    """`count` ports free for both TCP and UDP now, below the host's
    ephemeral range: the launcher spawns the bridge and the gateway a second
    or more after this returns, and a port the OS picked for bind(0) is in
    the range every connect() and DNS lookup on the host draws from, so it
    could be taken by then (the launcher refuses port 0)."""
    ports, port = [], 20000 + os.getpid() % 10000
    for _ in range(12000):
        port =20000 + (port - 20000 + 1) % 12000  # stays below 32768
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tcp, \
                    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                tcp.bind(("127.0.0.1", port))
                udp.bind(("127.0.0.1", port))
        except OSError:
            continue
        ports.append(port)
        if len(ports) == count:
            return ports
    raise RuntimeError(f"fewer than {count} free ports in 20000-31999")


class Session:
    """The launched 2026 simulation with the Studio bridge on a free port."""

    def __init__(self, root, tmp):
        self.output = tmp / "sessions"
        self.namespace = f"drive-browser-{uuid.uuid4().hex}"
        self.http_port, gw_port, bridge_port = free_ports(3)
        # The launcher resolves node labels and [studio] webroot under
        # bazel-bin/ of its working directory: the runfiles tree.
        (tmp / "bazel-bin").symlink_to(root, target_is_directory=True)
        self.log_path = tmp / "launcher.txt"
        self.log = self.log_path.open("w")
        args = [str(root / "talOS/launcher/launcher"), "--config", str(root / CONFIG),
                "--sim", "--start-sim-gateway", "--start-studio-bridge",
                "--studio-http-port", str(self.http_port),
                "--remote", "127.0.0.1", "--remote-port", str(gw_port),
                "--local-port", str(bridge_port), "--output-dir", str(self.output),
                # The telemetry node is held stopped for about 400 ms below,
                # inside its 10 ms period's 500 ms floor: a longer stall
                # timeout keeps the launcher from killing it meanwhile.
                "--heartbeat-periods", "300",
                "--sim-gateway-arg", "--physics",
                "--sim-gateway-arg", "--port", "--sim-gateway-arg", str(gw_port),
                "--sim-gateway-arg", "--remote-port", "--sim-gateway-arg", str(bridge_port)]
        env = dict(os.environ, TALOS_SHM_NAMESPACE=self.namespace)
        if sys.platform == "darwin":
            env["TALOS_REALTIME"] = "1"
        self.proc = subprocess.Popen(args, cwd=tmp, env=env, stdout=self.log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        self.tmp = tmp
        self.dir = None

    def wait_listening(self):
        """The session directory, once the bridge's log says it listens."""

        def listening():
            if self.proc.poll() is not None:
                raise RuntimeError(f"launcher exited {self.proc.returncode} during startup")
            for log in self.output.glob("*/studio_bridge.log"):
                if f"listening {self.http_port}\n" in log.read_text(errors="replace"):
                    return log.parent
            return None

        def paths():
            return [self.tmp, self.output, *self.output.glob("*/"),
                    *self.output.glob("*/studio_bridge.log")]

        self.dir = watch.wait_for(listening, paths, STARTUP_S, "the Studio bridge's listening line")

    def pid(self, name):
        nodes = json.loads((self.dir / "manifest.json").read_text())["nodes"]
        return next(n["pid"] for n in nodes if n["name"] == name)

    def stop(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self.log.close()
        autonomous_progress.remove(self.namespace)


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
            cls.browser = cdp.Browser(binary, cls.tmp)
            cls.page = cls.browser.page
            cls.page.viewport(*VIEWPORTS[0])
            probe = (Path(__file__).parent / "probe.js").read_text()
            cls.page.call("Page.addScriptToEvaluateOnNewDocument", source=probe)
            cls.page.call("Page.navigate", url=f"http://127.0.0.1:{cls.session.http_port}/drive")
            cls.page.evaluate(f"__probe.waitFor(() => document.querySelector('#drive .page'), "
                              f"{WAIT_MS}, 'the dashboard drawn')")
            # The session is up once every node's row reads UP (the Python
            # pose estimator is the slowest to register).
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
            cls.browser.close()
        if cls.session is not None:
            cls.session.stop()
            failed = cls.session.proc.returncode not in (0, -signal.SIGINT)
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
                # The pose, checks and summary fill in as the graph starts.
                self.wait(label, "__probe.valued()", "every widget with a value")
                self.call_probe("await __probe.frames(); return true")
                # Kept whatever the result, to see what was judged.
                self.page.screenshot(self.outputs / f"{label}-seen.png")
                self.check(label, self.page.evaluate("__probe.audit()"))
                # The 2026 lists fit whole at these sizes (REQ-OPS-49).
                for kind in ("subsystems", "checks"):
                    state = self.page.evaluate(f"__probe.listState('{kind}')")
                    self.assertLessEqual(state["scrollHeight"], state["clientHeight"],
                                         f"{label}: the {kind} list scrolls: {state}")

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

    # REQ-OPS-49: where a list overflows, its heading still shows whole:
    # the heading, the out-of-view count and every word in the list (the
    # session label gives way first, README.md "Driver dashboard").
    def test_overflowing_list_heading_is_whole(self):
        label = f"heading-{SHORT_VIEWPORT[0]}x{SHORT_VIEWPORT[1]}"
        self.settle(label, SHORT_VIEWPORT)
        self.call_probe("await __probe.frames(); return true")
        state = self.page.evaluate("__probe.listState('subsystems')")
        self.assertGreater(state["scrollHeight"], state["clientHeight"],
                           f"{label}: the subsystems list does not overflow: {state}")
        self.check(label, self.page.evaluate("__probe.audit('lists')"))

    # REQ-OPS-36: once the producer stops, every telemetry widget reads
    # stale within 500 ms of its last update, and the status line names
    # telemetry; when it resumes the page is current again.
    def test_stopped_telemetry_reads_stale_in_time(self):
        label = "freshness"
        self.settle(label, VIEWPORTS[0])
        robot_dir = workspace() / CONFIG.rsplit("/", 1)[0]
        sub = tomllib.loads((robot_dir / "subsystems/telemetry.toml").read_text())["subsystem"]
        period_ms = sub["period_us"] / 1000
        pid = self.session.pid(PRODUCER)
        armed_at = self.page.evaluate(f"__probe.armStale({WAIT_MS})")
        os.kill(pid, signal.SIGSTOP)
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
        print(f"{label}: {result['widgets']} widgets stale {result['staleAt'] - armed_at:.1f} ms "
              f"after the page clock read before SIGSTOP (+{period_ms:.0f} ms producer period "
              f"= {latency_ms:.1f} ms, bound {STALE_BOUND_MS} ms); status "
              f"{result['status']['text']!r} at {result['namedAt'] - armed_at:.1f} ms", flush=True)
        if latency_ms > STALE_BOUND_MS:
            self.check(label, [{
                "check": "stale too late", "element": ".w", "detail":
                f"{latency_ms:.1f} ms from the producer's last tick at the latest, "
                f"bound {STALE_BOUND_MS} ms (REQ-OPS-36)", "result": result}])
        self.assertIn("TELEMETRY", result["status"]["text"], result)
        self.wait(label, "__probe.current()", "current again after the producer resumed")


if __name__ == "__main__":
    unittest.main()
