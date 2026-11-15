"""A headless Chromium driven over the DevTools protocol, with no dependency.

`find_browser()` names the host's Chromium or Chrome (TALOS_BROWSER first);
`Browser` starts it headless with its own profile in a temporary directory,
reads the port it chose from the profile's DevToolsActivePort file and opens
one page over a minimal RFC 6455 client (text frames, client masking,
fragmentation, ping); `Page.evaluate` runs an expression in the page and
returns its JSON value, awaiting a promise.
"""

import base64
import hashlib
import itertools
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import struct
import subprocess
import sys
from urllib.parse import urlsplit

import children
import watch

BROWSER_NAMES = ("chromium", "chromium-browser", "google-chrome", "google-chrome-stable")
MACOS_CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def find_browser():
    """The browser binary: $TALOS_BROWSER, then the PATH names, then macOS's
    Chrome; None when there is none."""
    override = os.environ.get("TALOS_BROWSER")
    if override:
        return override
    for name in BROWSER_NAMES:
        found = shutil.which(name)
        if found:
            return found
    if sys.platform == "darwin" and Path(MACOS_CHROME).exists():
        return MACOS_CHROME
    return None


class WebSocket:
    """The client side of RFC 6455 for one text-message conversation.

    Every read is bounded by the socket timeout, which only bounds a hang.
    """

    def __init__(self, url, timeout_s):
        parts = urlsplit(url)
        self.sock = socket.create_connection((parts.hostname, parts.port), timeout=timeout_s)
        key = base64.b64encode(os.urandom(16))
        self.sock.sendall(
            f"GET {parts.path} HTTP/1.1\r\nHost: {parts.hostname}:{parts.port}\r\n"
            "Upgrade: websocket\r\nConnection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key.decode()}\r\nSec-WebSocket-Version: 13\r\n\r\n".encode())
        response = b""
        while b"\r\n\r\n" not in response:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("DevTools closed the WebSocket handshake")
            response += chunk
        head, self.buffer = response.split(b"\r\n\r\n", 1)
        lines = head.decode("latin-1").split("\r\n")
        if " 101 " not in lines[0] + " ":
            raise ConnectionError(f"DevTools refused the WebSocket: {lines[0]}")
        accept = base64.b64encode(hashlib.sha1(key + _GUID).digest()).decode()
        if not any(line.lower() == f"sec-websocket-accept: {accept}".lower() for line in lines):
            raise ConnectionError("DevTools answered with a wrong Sec-WebSocket-Accept")

    def _read(self, n):
        while len(self.buffer) < n:
            chunk = self.sock.recv(max(65536, n - len(self.buffer)))
            if not chunk:
                raise ConnectionError("DevTools closed the WebSocket")
            self.buffer += chunk
        data, self.buffer = self.buffer[:n], self.buffer[n:]
        return data

    def _frame(self, opcode, payload):
        n = len(payload)
        if n < 126:
            header = struct.pack("!BB", 0x80 | opcode, 0x80 | n)
        elif n < 1 << 16:
            header = struct.pack("!BBH", 0x80 | opcode, 0x80 | 126, n)
        else:
            header = struct.pack("!BBQ", 0x80 | opcode, 0x80 | 127, n)
        mask = os.urandom(4)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        self.sock.sendall(header + mask + masked)

    def send(self, text):
        self._frame(0x1, text.encode())

    def receive(self):
        """The next whole text message."""
        message = b""
        while True:
            first, second = self._read(2)
            opcode, n = first & 0x0F, second & 0x7F
            if n == 126:
                n = struct.unpack("!H", self._read(2))[0]
            elif n == 127:
                n = struct.unpack("!Q", self._read(8))[0]
            mask = self._read(4) if second & 0x80 else None
            payload = self._read(n)
            if mask:
                payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
            if opcode == 0x8:
                raise ConnectionError("DevTools closed the WebSocket")
            if opcode == 0x9:
                self._frame(0xA, payload)
                continue
            if opcode == 0xA:
                continue
            message += payload
            if first & 0x80:
                return message.decode()

    def close(self):
        self.sock.close()


class EvaluateError(Exception):
    """An expression threw, or its promise rejected, in the page."""


class Page:
    """One page target, through a flattened session on the browser socket."""

    def __init__(self, ws, session):
        self.ws, self.session = ws, session
        self.ids = itertools.count(1)

    def call(self, method, **params):
        message_id = next(self.ids)
        message = {"id": message_id, "method": method, "params": params}
        if self.session is not None:
            message["sessionId"] = self.session
        self.ws.send(json.dumps(message))
        while True:
            reply = json.loads(self.ws.receive())
            if reply.get("id") != message_id or reply.get("sessionId") != self.session:
                continue  # an event, or the reply to another session's call
            if "error" in reply:
                raise RuntimeError(f"{method}: {reply['error']}")
            return reply["result"]

    def evaluate(self, expression):
        """The expression's JSON value, awaited if it is a promise."""
        result = self.call("Runtime.evaluate", expression=expression, returnByValue=True,
                           awaitPromise=True)
        if "exceptionDetails" in result:
            details = result["exceptionDetails"]
            text = details.get("exception", {}).get("description") or details.get("text")
            raise EvaluateError(text)
        return result["result"].get("value")

    def viewport(self, width, height):
        self.call("Emulation.setDeviceMetricsOverride", width=width, height=height,
                  deviceScaleFactor=1, mobile=False)

    def screenshot(self, path):
        data = self.call("Page.captureScreenshot", format="png")["data"]
        Path(path).write_bytes(base64.b64decode(data))


class Browser:
    """A headless browser with a fresh profile under `tmp`, and its one page."""

    def __init__(self, binary, tmp, timeout_s=60, register=None):
        """`register(close)` is called once the browser runs, so a caller
        can stop it from a signal handler while this still waits on it."""
        self.profile = Path(tmp) / "browser-profile"
        self.profile.mkdir(parents=True)
        self.log_path = Path(tmp) / "browser.log"
        self.log = self.log_path.open("w")
        self.closed = False
        self.child = children.Child(
            [binary, "--headless=new", "--remote-debugging-port=0",
             f"--user-data-dir={self.profile}", "--no-first-run",
             "--no-default-browser-check", "--disable-extensions",
             "--disable-background-networking", "--disable-component-update",
             "--disable-sync", "--force-device-scale-factor=1",
             # A page out of view must still draw at its own pace.
             "--disable-background-timer-throttling",
             "--disable-renderer-backgrounding",
             "--disable-backgrounding-occluded-windows", "about:blank"],
            stdout=self.log, stderr=subprocess.STDOUT)
        self.ws = None
        if register is not None:
            register(self.close)
        try:
            active = self.profile / "DevToolsActivePort"

            def port_file():
                if self.child.exited():
                    raise RuntimeError(f"{binary} exited; see {self.log_path}")
                text = active.read_text() if active.exists() else ""
                return text if text.count("\n") >= 1 and text.split("\n")[1] else None

            port, path = watch.wait_for(port_file, lambda: [self.profile, active], timeout_s,
                                        f"{binary} DevTools port").split("\n")[:2]
            self.ws = WebSocket(f"ws://127.0.0.1:{port.strip()}{path.strip()}", timeout_s)
            browser = Page(self.ws, None)
            target = browser.call("Target.createTarget", url="about:blank")["targetId"]
            session = browser.call("Target.attachToTarget", targetId=target,
                                   flatten=True)["sessionId"]
            self.page = Page(self.ws, session)
            self.page.call("Page.enable")
        except BaseException:
            self.close()
            raise

    def close(self):
        """Kills the browser's whole group (renderers and helpers may outlive
        the main process) while its leader is unreaped, then reaps it. Once:
        a second call does nothing."""
        if self.closed:
            return
        self.closed = True
        if self.ws is not None:
            self.ws.close()
        self.child.signal_group(signal.SIGKILL)
        self.child.reap()
        self.log.close()
