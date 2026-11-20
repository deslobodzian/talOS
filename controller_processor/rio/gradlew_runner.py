"""Runs controller_processor/rio's Gradle wrapper from a staged cache directory.

    bazel run @talos//controller_processor/rio:gradlew -- <gradle args>

Copies the Gradle project and the talOS sources/headers build.gradle reads into
a stable cache directory (never mutating the talOS checkout or Bazel execroot),
stamps -PtalosRevision and -PtalosDirty from the pinned talOS checkout when it
is a git work tree, forwards the calling workspace's
.wpilib/wpilib_preferences.json if present, and runs ./gradlew.
"""

import contextlib
import fcntl
import hashlib
import json
import os
import re
import shlex
import signal
import stat
import subprocess
import sys

from python.runfiles import runfiles

MANIFEST_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "rio_stage_manifest.json"
)

# The Gradle wrapper script shares its filename with the :gradlew Bazel rule,
# so it is staged directly from the talOS root beside MODULE.bazel.
GRADLEW_RELPATH = "controller_processor/rio/gradlew"

# Desktop-test headers included only by src/test/cpp (phoenixTest); copied from
# the talOS checkout when present beside the production sources.
OPTIONAL_TEST_HEADERS = (
    "talOS/hardware/gateway_test_support.h",
    "talOS/protocol/udp_test_helpers.h",
)

# Bazel runfiles variables that can confuse a child JVM or Gradle wrapper.
STRIP_ENV_VARS = (
    "JAVA_RUNFILES",
    "PYTHON_RUNFILES",
    "RUNFILES_DIR",
    "RUNFILES_MANIFEST_FILE",
    "RUNFILES_MANIFEST_ONLY",
)

STAGED_INDEX = ".talos-staged-files.json"
PREFS_RELPATH = "controller_processor/rio/.wpilib/wpilib_preferences.json"
STAMP_PROPERTIES = ("talosRevision", "talosDirty")
SYSTEM_PROP_PREFIX = "org.gradle.project."
ENV_PROP_PREFIX = "ORG_GRADLE_PROJECT_"


def _same_dir(a, b):
    return os.path.realpath(a) == os.path.realpath(b)


def _run_git(repo_dir, *args):
    try:
        proc = subprocess.run(
            ["git", "-C", repo_dir, *args],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        return None
    if proc.returncode != 0:
        return None
    return proc.stdout.strip()


def compute_talos_stamp(talos_root):
    """Returns ['-PtalosRevision=...', '-PtalosDirty=...'] if talos_root is
    its own git work tree, else []."""
    top = _run_git(talos_root, "rev-parse", "--show-toplevel")
    if not top or not _same_dir(top, talos_root):
        return []
    head = _run_git(talos_root, "rev-parse", "HEAD")
    if not head or not re.fullmatch(r"[0-9a-f]{16,}", head):
        return []
    status = _run_git(talos_root, "--no-optional-locks", "status", "--porcelain")
    dirty = "true" if (status is None or status != "") else "false"
    return [f"-PtalosRevision={head[:16]}", f"-PtalosDirty={dirty}"]


def _is_stamp_project_prop(spec):
    return spec.split("=", 1)[0] in STAMP_PROPERTIES


def _is_stamp_system_prop(spec):
    name = spec.split("=", 1)[0]
    return (
        name.startswith(SYSTEM_PROP_PREFIX)
        and name[len(SYSTEM_PROP_PREFIX) :] in STAMP_PROPERTIES
    )


def caller_stamp_overrides(args, env=None):
    """Returns caller arguments or environment settings that supply
    talosRevision or talosDirty in any Gradle property spelling."""
    found = []
    i = 0
    while i < len(args):
        arg = args[i]
        if arg == "-P" and i + 1 < len(args):
            if _is_stamp_project_prop(args[i + 1]):
                found.append(f"-P {args[i + 1]}")
            i += 2
            continue
        if arg.startswith("-P") and len(arg) > 2:
            if _is_stamp_project_prop(arg[2:]):
                found.append(arg)
        elif arg == "--project-prop" and i + 1 < len(args):
            if _is_stamp_project_prop(args[i + 1]):
                found.append(f"--project-prop {args[i + 1]}")
            i += 2
            continue
        elif arg.startswith("--project-prop="):
            if _is_stamp_project_prop(arg[len("--project-prop=") :]):
                found.append(arg)
        elif arg == "-D" and i + 1 < len(args):
            if _is_stamp_system_prop(args[i + 1]):
                found.append(f"-D {args[i + 1]}")
            i += 2
            continue
        elif arg.startswith("-D") and len(arg) > 2:
            if _is_stamp_system_prop(arg[2:]):
                found.append(arg)
        elif arg == "--system-prop" and i + 1 < len(args):
            if _is_stamp_system_prop(args[i + 1]):
                found.append(f"--system-prop {args[i + 1]}")
            i += 2
            continue
        elif arg.startswith("--system-prop="):
            if _is_stamp_system_prop(arg[len("--system-prop=") :]):
                found.append(arg)
        i += 1

    if env:
        for prop in STAMP_PROPERTIES:
            env_name = ENV_PROP_PREFIX + prop
            if env_name in env:
                found.append(env_name)
        for opts_var in ("GRADLE_OPTS", "JAVA_OPTS"):
            raw = env.get(opts_var)
            if not raw:
                continue
            try:
                tokens = shlex.split(raw)
            except ValueError:
                tokens = raw.split()
            for tok in tokens:
                if tok.startswith("-D") and _is_stamp_system_prop(tok[2:]):
                    found.append(f"{opts_var}:{tok}")
    return found


def _caller_prefs_path(workspace_dir):
    if not workspace_dir:
        return None
    candidate = os.path.join(
        workspace_dir, ".wpilib", "wpilib_preferences.json"
    )
    return candidate if os.path.isfile(candidate) else None


def cache_dir_for(talos_root, workspace_dir="", prefs_path=None):
    override = os.environ.get("TALOS_RIO_CACHE_DIR")
    if override:
        return os.path.abspath(override)
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        os.path.expanduser("~"), ".cache"
    )
    if prefs_path is None:
        prefs_path = _caller_prefs_path(workspace_dir)
    talos_id = os.path.realpath(talos_root)
    workspace_id = os.path.realpath(workspace_dir) if workspace_dir else ""
    prefs_id = (
        os.path.realpath(prefs_path)
        if prefs_path and os.path.isfile(prefs_path)
        else ""
    )
    payload = "\0".join((talos_id, workspace_id, prefs_id)).encode("utf-8")
    key = hashlib.sha256(payload).hexdigest()[:16]
    return os.path.join(base, "talos", "rio", key)


def _sync_file(src, dst):
    """Copies src to dst only when bytes or executable bit differ, preserving
    dst's mtime for Gradle's incremental build when unchanged. Always leaves
    dst user-writable so read-only source checkouts stage cleanly."""
    with open(src, "rb") as f:
        data = f.read()
    executable = bool(os.stat(src).st_mode & stat.S_IXUSR)
    mode = 0o755 if executable else 0o644
    os.makedirs(os.path.dirname(dst), mode=0o755, exist_ok=True)
    if os.path.lexists(dst):
        if os.path.islink(dst):
            os.unlink(dst)
        else:
            st = os.stat(dst)
            dst_exec = bool(st.st_mode & stat.S_IXUSR)
            if st.st_size == len(data) and dst_exec == executable:
                with open(dst, "rb") as f:
                    if f.read() == data:
                        if not (st.st_mode & stat.S_IWUSR):
                            os.chmod(dst, mode)
                        return
            if not (st.st_mode & stat.S_IWUSR):
                os.chmod(dst, mode)
    tmp = f"{dst}.tmp.{os.getpid()}"
    with open(tmp, "wb") as f:
        f.write(data)
    os.chmod(tmp, mode)
    os.replace(tmp, dst)


@contextlib.contextmanager
def cache_lock(cache_dir):
    """Holds an exclusive flock for cache_dir across staging and Gradle.

    The lock fd is non-inheritable (and Gradle runs with close_fds=True) so a
    background Gradle daemon cannot inherit and hold the lock after gradlew
    exits.
    """
    normalized = os.path.normpath(os.path.abspath(cache_dir))
    os.makedirs(normalized, mode=0o755, exist_ok=True)
    lock_path = normalized + ".lock"
    os.makedirs(os.path.dirname(lock_path) or ".", mode=0o755, exist_ok=True)
    with open(lock_path, "a+", encoding="utf-8") as lock_file:
        os.set_inheritable(lock_file.fileno(), False)
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        try:
            yield lock_path
        finally:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def _stage_project_locked(cache_dir, file_map):
    index_path = os.path.join(cache_dir, STAGED_INDEX)
    previous = []
    if os.path.isfile(index_path):
        try:
            with open(index_path, encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, list):
                previous = loaded
        except (OSError, ValueError):
            previous = []

    for relpath, src in sorted(file_map.items()):
        _sync_file(src, os.path.join(cache_dir, relpath))

    current = set(file_map)
    for stale in previous:
        if stale not in current:
            target = os.path.join(cache_dir, stale)
            if os.path.lexists(target):
                try:
                    os.chmod(target, 0o644)
                except OSError:
                    pass
                os.unlink(target)

    tmp_index = f"{index_path}.tmp.{os.getpid()}"
    with open(tmp_index, "w", encoding="utf-8") as f:
        json.dump(sorted(current), f, indent=2)
        f.write("\n")
    os.replace(tmp_index, index_path)


def stage_project(cache_dir, file_map):
    """Synchronizes file_map ({relpath: src_path}) into cache_dir, deleting
    files staged in an earlier run that are no longer in file_map while leaving
    Gradle's build/ and .gradle/ directories intact."""
    with cache_lock(cache_dir):
        _stage_project_locked(cache_dir, file_map)


def load_manifest():
    with open(MANIFEST_FILE, encoding="utf-8") as f:
        return json.load(f)


def collect_files(rf, manifest, talos_root, workspace_dir):
    file_map = {}
    for relpath, rlocation in manifest["manifest"]:
        resolved = rf.Rlocation(rlocation)
        if not resolved or not os.path.isfile(resolved):
            raise FileNotFoundError(
                f"runfile missing for {relpath} ({rlocation} -> {resolved})"
            )
        file_map[relpath] = resolved

    gradlew_src = os.path.join(talos_root, GRADLEW_RELPATH)
    if not os.path.isfile(gradlew_src):
        raise FileNotFoundError(
            f"Gradle wrapper missing at {gradlew_src}"
        )
    file_map[GRADLEW_RELPATH] = gradlew_src

    for relpath in OPTIONAL_TEST_HEADERS:
        candidate = os.path.join(talos_root, relpath)
        if os.path.isfile(candidate):
            file_map[relpath] = candidate

    caller_prefs = _caller_prefs_path(workspace_dir)
    if caller_prefs:
        file_map[PREFS_RELPATH] = caller_prefs

    return file_map


def _run_gradle(cmd, cwd, env):
    proc = None
    received_signal = None

    def _forward(signum, _frame):
        nonlocal received_signal
        if received_signal is None:
            received_signal = signum
        if proc is not None and proc.poll() is None:
            try:
                proc.send_signal(signum)
            except ProcessLookupError:
                pass

    prev_int = signal.signal(signal.SIGINT, _forward)
    prev_term = signal.signal(signal.SIGTERM, _forward)
    try:
        if received_signal is not None:
            return 128 + received_signal
        proc = subprocess.Popen(cmd, cwd=cwd, env=env, close_fds=True)
        if received_signal is not None and proc.poll() is None:
            try:
                proc.send_signal(received_signal)
            except ProcessLookupError:
                pass
        rc = proc.wait()
    finally:
        signal.signal(signal.SIGINT, prev_int)
        signal.signal(signal.SIGTERM, prev_term)

    if rc < 0:
        return 128 + (-rc)
    if received_signal is not None and rc == 0:
        return 128 + received_signal
    return rc


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    rf = runfiles.Create()
    manifest = load_manifest()
    module_rlocation = manifest["module_rlocation"]
    module_path = rf.Rlocation(module_rlocation)
    if not module_path or not os.path.exists(module_path):
        print(
            f"rio gradlew: cannot locate {module_rlocation}",
            file=sys.stderr,
        )
        return 2
    talos_root = os.path.dirname(os.path.realpath(module_path))
    workspace_dir = os.environ.get("BUILD_WORKSPACE_DIRECTORY") or ""

    stamp_args = compute_talos_stamp(talos_root)
    if stamp_args:
        overrides = caller_stamp_overrides(args, os.environ)
        if overrides:
            joined = ", ".join(overrides)
            print(
                f"rio gradlew: {joined} refused: {talos_root} is a talOS git "
                "work tree, which stamps its own -PtalosRevision and "
                "-PtalosDirty; drop those properties (they are only for a "
                "talOS copy with no git history)",
                file=sys.stderr,
            )
            return 2

    cache_dir = cache_dir_for(talos_root, workspace_dir)
    file_map = collect_files(rf, manifest, talos_root, workspace_dir)
    rio_dir = os.path.join(cache_dir, "controller_processor", "rio")
    gradlew = os.path.join(rio_dir, "gradlew")

    env = {k: v for k, v in os.environ.items() if k not in STRIP_ENV_VARS}
    if not env.get("JAVA_HOME"):
        wpilib_jdk = os.path.expanduser("~/wpilib/2026/jdk")
        if os.path.isdir(wpilib_jdk):
            env["JAVA_HOME"] = wpilib_jdk

    with cache_lock(cache_dir):
        _stage_project_locked(cache_dir, file_map)
        return _run_gradle([gradlew, *stamp_args, *args], rio_dir, env)


if __name__ == "__main__":
    sys.exit(main())
