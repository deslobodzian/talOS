"""Runs the launcher on a config from a Bazel runfiles tree.

The runner behind talos_graph_test and talos_sim (contract.bzl). The
launcher resolves node labels as a workspace's `bazel-bin` paths
(`//p:t` as bazel-bin/p/t, `@repo//p:t` via `_repo_mapping` as
bazel-bin/external/<canonical>/p/t), so the runner builds that layout over the
runfiles tree: a work directory mirroring the main repository's runfiles, with
a bazel-bin holding the same entries and `external` naming the runfiles root,
where every other repository's runfiles live. It works the same whether talOS
is the main repository or a dependency.

    launcher_runner.py --launcher RLOCATION --config RLOCATION
        (--graph-nodes N | --sim) [-- LAUNCHER_ARGS...]

--graph-nodes N runs `--describe-only` and fails unless the graph lists N
nodes, each described. --sim runs the config under the simulator
(`--sim --start-sim-gateway --sim-gateway-arg --physics`, then
LAUNCHER_ARGS) and exits with the launcher's status.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile

from python.runfiles import runfiles


def parse_repo_mapping(path):
    """Reads Bazel's 3-column CSV `_repo_mapping` into
    [(source_canonical, target_apparent, target_canonical)]."""
    entries = []
    if not path or not os.path.isfile(path):
        return entries
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.rstrip("\r\n").split(",")
            if len(parts) == 3:
                entries.append(tuple(parts))
    return entries


def split_config(config, root=None):
    """Splits a config's `$(rlocationpath ...)` into `(main_repo, rel_path)`.

    Derives the main repository's runfiles directory from the rlocationpath's
    leading segment rather than hardcoding `_main`. When `root/_repo_mapping`
    exists, rejects a config belonging to an external repository.
    """
    if "/" not in config:
        sys.exit(f"launcher_runner: {config} has no repository prefix; "
                 "pass $(rlocationpath <config>)")
    repo, rel = config.split("/", 1)
    if not repo or not rel:
        sys.exit(f"launcher_runner: invalid config rlocationpath: {config}")
    if "+" in repo or "~" in repo:
        sys.exit(f"launcher_runner: {config} is not in the main "
                 "repository; the launcher reads a config by its "
                 "workspace path")
    if root and repo != "_main":
        mapping = parse_repo_mapping(os.path.join(root, "_repo_mapping"))
        if mapping:
            sources = {source for source, _, _ in mapping if source}
            external_repos = set(sources) | {
                canonical for source, _, canonical in mapping
                if canonical and canonical != "_main" and (
                    source != "" or "+" in canonical or "~" in canonical
                    or canonical in sources)
            }
            if repo in external_repos:
                sys.exit(f"launcher_runner: {config} is not in the main "
                         "repository; the launcher reads a config by its "
                         "workspace path")
    return repo, rel


def find_runfiles_root(launcher, main_repo, r=None):
    """Locates the Bazel runfiles root containing `main_repo`."""
    if r is None:
        r = runfiles.Create()
    candidates = [os.environ.get("RUNFILES_DIR"),
                  r.EnvVars().get("RUNFILES_DIR") if r else None,
                  # <root>/<repo>/talOS/launcher/launcher
                  os.path.dirname(os.path.dirname(os.path.dirname(
                      os.path.dirname(launcher))))]
    root = next((c for c in candidates
                 if c and os.path.isdir(os.path.join(c, main_repo))), None)
    if root is None:
        sys.exit(f"launcher_runner: no runfiles tree for '{main_repo}' "
                 f"among {candidates}")
    return root


def workspace(tmp, launcher, config_rlocation):
    """Builds the work directory in `tmp`; returns (work_dir, config, env)."""
    r = runfiles.Create()
    main_repo, _ = split_config(config_rlocation)
    root = find_runfiles_root(launcher, main_repo, r)
    main_repo, config = split_config(config_rlocation, root)
    main_dir = os.path.join(root, main_repo)
    work_dir = os.path.join(tmp, "ws")
    bazel_bin = os.path.join(work_dir, "bazel-bin")
    os.makedirs(bazel_bin)
    for entry in os.listdir(main_dir):
        if entry == "bazel-bin":
            continue
        src = os.path.join(main_dir, entry)
        os.symlink(src, os.path.join(work_dir, entry))
        if entry != "external":
            os.symlink(src, os.path.join(bazel_bin, entry))
    os.symlink(root, os.path.join(bazel_bin, "external"))
    env = dict(os.environ, **(r.EnvVars() if r else {}))
    env["RUNFILES_DIR"] = root
    env.pop("PYTHONPATH", None)
    env.pop("BUILD_WORKSPACE_DIRECTORY", None)
    return work_dir, config, env


def rlocation(path, what):
    """`path` is a canonical runfiles path ($(rlocationpath)'s), so it is
    looked up without a caller's repository mapping."""
    found = runfiles.Create().Rlocation(path, source_repo="")
    if not found or not os.path.exists(found):
        sys.exit(f"launcher_runner: missing {what}: {path}")
    return found


def graph_test(launcher, config_rlocation, expected_nodes):
    with tempfile.TemporaryDirectory() as tmp:
        work_dir, config, env = workspace(tmp, launcher, config_rlocation)
        out_dir = os.path.join(tmp, "out")
        os.makedirs(out_dir)
        proc = subprocess.run(
            [launcher, "--config", config, "--describe-only", "--output-dir",
             out_dir],
            cwd=work_dir, env=env, capture_output=True, text=True,
            timeout=120, check=False)
        if proc.returncode != 0:
            sys.exit(f"launcher --describe-only exited {proc.returncode}:\n"
                     f"{proc.stdout}\n{proc.stderr}")
        # The launcher writes into <output-dir>/<session_id>/.
        sessions = [os.path.join(out_dir, d) for d in os.listdir(out_dir)
                    if os.path.isdir(os.path.join(out_dir, d))]
        if len(sessions) != 1:
            sys.exit(f"expected one session directory, found {sessions}")
        with open(os.path.join(sessions[0], "graph.json"), "r",
                  encoding="utf-8") as f:
            nodes = json.load(f)["nodes"]
        names = [n.get("name") for n in nodes]
        if len(nodes) != expected_nodes:
            sys.exit(f"expected {expected_nodes} nodes, the graph has "
                     f"{len(nodes)}: {names}")
        failed = [f"node {n.get('name')} ({n.get('target')}) failed "
                  f"--describe: {n.get('describe_error')}"
                  for n in nodes if not n.get("described")]
        if failed:
            sys.exit("\n".join(failed))


def from_invocation(args):
    """`args` with a relative --output-dir taken from where `bazel run` was
    invoked rather than from the runner's work directory."""
    cwd = os.environ.get("BUILD_WORKING_DIRECTORY")
    out = list(args)
    for i, arg in enumerate(out[:-1]):
        if arg == "--output-dir" and cwd and not os.path.isabs(out[i + 1]):
            out[i + 1] = os.path.join(cwd, out[i + 1])
    return out


def sim(launcher, config_rlocation, launcher_args):
    with tempfile.TemporaryDirectory() as tmp:
        work_dir, config, env = workspace(tmp, launcher, config_rlocation)
        command = [launcher, "--config", config, "--sim",
                   "--start-sim-gateway", "--sim-gateway-arg", "--physics"]
        child = None
        pending = []

        # The launcher stops its session in order on SIGINT, SIGTERM, or SIGHUP
        # (REQ-OPS-33); install handlers before Popen so an early signal never
        # tears down the work directory while the launcher runs, and forward
        # any signal that arrives during Popen.
        def forward(signum, _):
            if child is not None:
                try:
                    child.send_signal(signum)
                except ProcessLookupError:
                    pass
            else:
                pending.append(signum)

        forwarded = [signal.SIGINT, signal.SIGTERM]
        if (hasattr(signal, "SIGHUP")
                and signal.getsignal(signal.SIGHUP) != signal.SIG_IGN):
            forwarded.append(signal.SIGHUP)
        previous = {sig: signal.signal(sig, forward) for sig in forwarded}
        try:
            if pending:
                return 128 + pending[0]
            child = subprocess.Popen(command + from_invocation(launcher_args),
                                     cwd=work_dir, env=env)
            for sig in pending:
                try:
                    child.send_signal(sig)
                except ProcessLookupError:
                    pass
            pending.clear()
            rc = child.wait()
            return 128 + (-rc) if rc < 0 else rc
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)


def main(argv):
    if "--" in argv:
        split = argv.index("--")
        argv, launcher_args = argv[:split], argv[split + 1:]
    else:
        launcher_args = []
    parser = argparse.ArgumentParser(prog="launcher_runner",
                                     allow_abbrev=False)
    parser.add_argument("--launcher", required=True)
    parser.add_argument("--config", required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--graph-nodes", type=int)
    mode.add_argument("--sim", action="store_true")
    args = parser.parse_args(argv)
    launcher = rlocation(args.launcher, "launcher")
    if args.sim:
        # talos_sim's own arguments end in `--`, so `bazel run //:sim --
        # ARGS` hands every one of ARGS to the launcher.
        return sim(launcher, args.config, launcher_args)
    if launcher_args:
        parser.error(f"unexpected launcher arguments: {launcher_args}")
    graph_test(launcher, args.config, args.graph_nodes)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
