"""Shared Python runner for talOS loop nodes.

Mirrors the shape, flags, and semantics of talOS/process/node_main.h:
--describe (import-light manifest output via C ABI), --sim, --session-id,
--log, --config, --replay, --duration-s.
"""

import argparse
import importlib
import json
import os
import signal
import sys
import time
from typing import Any, Callable, Optional, Sequence

from talos_runtime import node_api


class NodeContext:
    """Hardware and runtime context passed to a Python node factory/class."""

    def __init__(
        self,
        node_name: str,
        config: Optional[node_api.Config],
        simulation: bool,
        session_id: int,
        loop: Optional[node_api.Loop] = None,
    ):
        self.node_name = node_name
        self.config = config
        self.simulation = simulation
        self.session_id = session_id
        self.loop = loop

    @property
    def config_id(self) -> int:
        return self.config.config_id if self.config else 0

    @property
    def period_us(self) -> int:
        return self.config.period_us if self.config else 5000

    @property
    def command_timeout_us(self) -> int:
        return self.config.command_timeout_us if self.config else 100000

    def get_device_id(self, device_name: str, subsystem: Optional[str] = None) -> int:
        if not self.config:
            raise RuntimeError("No configuration loaded")
        sub = subsystem or self.node_name
        dev_id = self.config.get_device_id(sub, device_name)
        if dev_id == 0:
            raise RuntimeError(f"device '{device_name}' not found for subsystem '{sub}'")
        return dev_id


def resolve_config_path(config_spec: str) -> str:
    """Resolve a config path or package resource specifier ('pkg:resource')."""
    if not config_spec:
        return ""
    if os.path.exists(config_spec):
        return os.path.abspath(config_spec)
    ws = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
    if ws:
        p = os.path.join(ws, config_spec)
        if os.path.exists(p):
            return os.path.abspath(p)
    rf = os.environ.get("RUNFILES_DIR") or os.environ.get("PYTHON_RUNFILES")
    if rf:
        for candidate in [os.path.join(rf, config_spec), os.path.join(rf, "_main", config_spec)]:
            if os.path.exists(candidate):
                return os.path.abspath(candidate)
    if ":" in config_spec:
        pkg, resource = config_spec.split(":", 1)
        try:
            from importlib.resources import files
            p = str(files(pkg).joinpath(resource))
            if os.path.exists(p):
                return os.path.abspath(p)
        except Exception:
            pass
    return config_spec


def parse_args(argv: Optional[Sequence[str]] = None, default_config: str = "") -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="node")
    parser.add_argument("--describe", action="store_true")
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--session-id", default="0")
    parser.add_argument("--log", default="")
    parser.add_argument("--config", default=default_config)
    parser.add_argument("--replay", default="")
    parser.add_argument("--duration-s", type=int, default=0)
    args = parser.parse_args(argv)
    if args.duration_s < 0:
        parser.error("duration must be nonnegative")
    return args


def run_node_main(
    node_name: str,
    node_target: str,
    entry_point: str,
    describe_sources_fn: Optional[Callable[[], list]] = None,
    default_config: str = "",
    argv: Optional[Sequence[str]] = None,
) -> int:
    args = parse_args(argv, default_config=default_config)

    # Replay branch
    if args.replay:
        config_path = resolve_config_path(args.config)
        config = None
        if config_path and os.path.exists(config_path):
            config = node_api.load_config(config_path, simulation=False)

        mod_name, attr_name = entry_point.rsplit(":", 1) if ":" in entry_point else entry_point.rsplit(".", 1)
        mod = importlib.import_module(mod_name)
        factory = getattr(mod, attr_name)

        if not os.path.exists(args.replay):
            if not (hasattr(factory, "register_sources") or hasattr(factory, "register_loop")):
                sys.stderr.write("node: replay unsupported for Python nodes\n")
                return 2
            sys.stderr.write(f"node: replay log not found: {args.replay}\n")
            return 1

        loop = None
        node = None
        try:
            loop = node_api.Loop.create_replay(args.replay)
            ctx = NodeContext(
                node_name=node_name,
                config=config,
                simulation=False,
                session_id=int(args.session_id),
                loop=loop,
            )

            if hasattr(factory, "from_context"):
                node = factory.from_context(ctx)
            else:
                try:
                    node = factory(ctx)
                except TypeError:
                    node = factory()

            if hasattr(node, "register_sources"):
                if not getattr(node, "_sources_registered", False):
                    node.register_sources(loop)
                    try:
                        node._sources_registered = True
                    except Exception:
                        pass
            elif hasattr(node, "register_loop"):
                if not getattr(node, "_sources_registered", False):
                    node.register_loop(loop)
                    try:
                        node._sources_registered = True
                    except Exception:
                        pass

            if not (hasattr(node, "register_sources") or hasattr(node, "register_loop") or len(getattr(loop, "_callbacks", {})) > 0):
                sys.stderr.write("node: replay unsupported for Python nodes\n")
                return 2

            loop.run()
            print(f"replay: diverged={int(loop.diverged)} clean_exit={int(loop.reached_exit)}")
            return 0 if (not loop.diverged and loop.reached_exit) else 1
        finally:
            if loop:
                try:
                    loop.close()
                except Exception:
                    pass
            if hasattr(node, "close"):
                try:
                    node.close()
                except Exception:
                    pass
            if config:
                try:
                    config.close()
                except Exception:
                    pass

    # Import-light describe branch: must not touch shm, flatbuffers, or heavy modules.
    if args.describe:
        if describe_sources_fn is not None:
            sources = describe_sources_fn()
        else:
            mod_name, attr_name = entry_point.rsplit(":", 1) if ":" in entry_point else entry_point.rsplit(".", 1)
            mod = importlib.import_module(mod_name)
            target = getattr(mod, attr_name)
            if hasattr(target, "describe_sources"):
                sources = target.describe_sources()
            elif hasattr(mod, "describe_sources"):
                sources = mod.describe_sources()
            else:
                raise RuntimeError(f"No describe_sources found for {entry_point}")
        print(node_api.describe_json(node_name, node_target, sources))
        return 0

    # Live / Simulation branch:
    # 1. Load config through native C ABI (talOS/node_api)
    config_path = resolve_config_path(args.config)
    config = None
    if config_path and os.path.exists(config_path):
        config = node_api.load_config(config_path, simulation=args.sim)

    loop = None
    registry = None
    node = None
    log_handle = None
    try:
        loop = node_api.Loop.create_realtime(
            log_path=args.log,
            stream_name=node_name,
            session_id=int(args.session_id),
        )

        ctx = NodeContext(
            node_name=node_name,
            config=config,
            simulation=args.sim,
            session_id=int(args.session_id),
            loop=loop,
        )

        # 2. Deferred import of the entry point (so describe stays import-light)
        mod_name, attr_name = entry_point.rsplit(":", 1) if ":" in entry_point else entry_point.rsplit(".", 1)
        mod = importlib.import_module(mod_name)
        factory = getattr(mod, attr_name)

        # Instantiate node
        if hasattr(factory, "from_context"):
            node = factory.from_context(ctx)
        else:
            try:
                node = factory(ctx)
            except TypeError:
                # Fallback for old style constructors if applicable
                node = factory()

        # Check if node supports native loop
        uses_native_loop = False
        if hasattr(node, "register_sources"):
            if not getattr(node, "_sources_registered", False):
                node.register_sources(loop)
                try:
                    node._sources_registered = True
                except Exception:
                    pass
            uses_native_loop = True
        elif hasattr(node, "register_loop"):
            if not getattr(node, "_sources_registered", False):
                node.register_loop(loop)
                try:
                    node._sources_registered = True
                except Exception:
                    pass
            uses_native_loop = True
        elif getattr(node, "uses_native_loop", False):
            uses_native_loop = True
        elif len(getattr(loop, "_callbacks", {})) > 0:
            uses_native_loop = True

        if uses_native_loop:
            flags = node_api.FLAG_SIMULATION if args.sim else 0
            loop.attach_reporter(node_name, node_target, int(args.session_id), flags)

            def _on_signal(signum, _frame):
                del signum, _frame
                loop.exit()

            signal.signal(signal.SIGINT, _on_signal)
            try:
                signal.signal(signal.SIGTERM, _on_signal)
            except (OSError, ValueError):
                pass

            if args.duration_s > 0:
                loop.run_for(args.duration_s * 1_000_000_000)
            else:
                loop.run()

            summary = {
                "dispatches": loop.dispatch_count,
                "sim": bool(args.sim),
                "session_id": args.session_id,
            }
            print(json.dumps(summary))
            return 0
        else:
            if loop:
                loop.close()
                loop = None

            # Determine describe sources for live registration
            if describe_sources_fn is not None:
                sources = describe_sources_fn()
            elif hasattr(node, "describe_sources"):
                sources = node.describe_sources()
            elif hasattr(factory, "describe_sources"):
                sources = factory.describe_sources()
            elif hasattr(mod, "describe_sources"):
                sources = mod.describe_sources()
            else:
                sources = []

            # 3. Register in Studio live graph
            flags = node_api.FLAG_SIMULATION if args.sim else 0
            try:
                registry = node_api.Node(node_name, node_target, int(args.session_id), flags, sources)
            except Exception:
                # A missing registry costs the heartbeat, never the node
                pass

            # Optional start event logging
            if args.log:
                try:
                    log_handle = open(args.log, "a", encoding="utf-8")
                    log_handle.write(json.dumps({"event": "start",
                                                 "session_id": args.session_id,
                                                 "node": node_name,
                                                 "period_us": ctx.period_us}) + "\n")
                    log_handle.flush()
                except Exception:
                    pass

            # 4. Signal handling
            stop = {"hit": False}
            def _on_signal_legacy(signum, _frame):
                del signum
                stop["hit"] = True

            signal.signal(signal.SIGINT, _on_signal_legacy)
            try:
                signal.signal(signal.SIGTERM, _on_signal_legacy)
            except (OSError, ValueError):
                pass

            period_s = (ctx.period_us if ctx.period_us else 5000) / 1e6
            deadline_ns = 0
            if args.duration_s > 0:
                deadline_ns = time.monotonic_ns() + args.duration_s * 1_000_000_000

            dispatches = 0
            while not stop["hit"]:
                now_ns = time.monotonic_ns()
                if deadline_ns and now_ns >= deadline_ns:
                    break

                dispatches += 1
                if registry:
                    registry.heartbeat(dispatches)

                if hasattr(node, "poll_once"):
                    node.poll_once(now_ns)
                elif hasattr(node, "tick"):
                    node.tick(now_ns)

                elapsed_s = (time.monotonic_ns() - now_ns) / 1e9
                sleep_s = period_s - elapsed_s
                if sleep_s > 0:
                    time.sleep(sleep_s)

            if log_handle:
                try:
                    log_handle.write(json.dumps({"event": "stop",
                                                 "session_id": args.session_id,
                                                 "dispatches": dispatches}) + "\n")
                    log_handle.flush()
                except Exception:
                    pass

            summary = {"dispatches": dispatches, "sim": bool(args.sim),
                       "session_id": args.session_id}
            print(json.dumps(summary))
            return 0

    finally:
        if loop:
            try:
                loop.close()
            except Exception:
                pass
        if log_handle:
            try:
                log_handle.close()
            except Exception:
                pass
        if registry:
            try:
                registry.close()
            except Exception:
                pass
        if hasattr(node, "close"):
            try:
                node.close()
            except Exception:
                pass
        if config:
            try:
                config.close()
            except Exception:
                pass
