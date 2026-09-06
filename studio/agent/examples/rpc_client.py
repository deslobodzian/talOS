"""Shared stdlib-only JSON-RPC helper for the agent examples."""
import json
import urllib.request


class RpcError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(f"RPC error {code}: {message}")
        self.code = code


def rpc(url, method, params=None, timeout=5):
    payload = json.dumps({
        "jsonrpc": "2.0", "id": 1, "method": method, "params": params or {},
    }).encode()
    req = urllib.request.Request(
        url, data=payload, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as response:
        body = json.load(response)
    if "error" in body:
        raise RpcError(body["error"]["code"], body["error"]["message"])
    return body["result"]
