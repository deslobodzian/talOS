"""Runs controller_processor/rio's Gradle wrapper from a staged cache directory.

    bazel run @talos//controller_processor/rio:gradlew -- <gradle args>

Copies the Gradle project and the talOS sources/headers build.gradle reads into
a stable cache directory (never mutating the talOS checkout or Bazel execroot),
stamps -PtalosRevision and -PtalosDirty from the pinned talOS checkout when it
is a git work tree, forwards the calling workspace's
.wpilib/wpilib_preferences.json if present, and runs ./gradlew.
"""

import fcntl
import hashlib
import json
import os
import re
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


def caller_sets_stamp(args):
    """True when the caller already passed -PtalosRevision or -PtalosDirty."""
    for arg in args:
        if arg.startswith("-PtalosRevision") or arg.startswith("-PtalosDirty"):
            return True
    return False


def cache_dir_for(talos_root):
    override = os.environ.get("TALOS_RIO_CACHE_DIR")
    if override:
        return os.path.abspath(override)
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        os.path.expanduser("~"), ".cache"
    )
    key = hashlib.sha256(os.path.realpath(talos_root).encode("utf-8")).hexdigest()[:12]
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


def stage_project(cache_dir, file_map):
    """Synchronizes file_map ({relpath: src_path}) into cache_dir, deleting
    files staged in an earlier run that are no longer in file_map while leaving
    Gradle's build/ and .gradle/ directories intact."""
    os.makedirs(cache_dir, mode=0o755, exist_ok=True)
    lock_path = cache_dir + ".lock"
    os.makedirs(os.path.dirname(lock_path) or ".", mode=0o755, exist_ok=True)
    with open(lock_path, "a+", encoding="utf-8") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
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

    if workspace_dir:
        caller_prefs = os.path.join(
            workspace_dir, ".wpilib", "wpilib_preferences.json"
        )
        if os.path.isfile(caller_prefs):
            file_map[PREFS_RELPATH] = caller_prefs

    return file_map


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

    cache_dir = cache_dir_for(talos_root)
    file_map = collect_files(rf, manifest, talos_root, workspace_dir)
    stage_project(cache_dir, file_map)

    stamp_args = [] if caller_sets_stamp(args) else compute_talos_stamp(talos_root)
    rio_dir = os.path.join(cache_dir, "controller_processor", "rio")
    gradlew = os.path.join(rio_dir, "gradlew")

    env = {k: v for k, v in os.environ.items() if k not in STRIP_ENV_VARS}
    if not env.get("JAVA_HOME"):
        wpilib_jdk = os.path.expanduser("~/wpilib/2026/jdk")
        if os.path.isdir(wpilib_jdk):
            env["JAVA_HOME"] = wpilib_jdk

    os.chdir(rio_dir)
    os.execvpe(gradlew, [gradlew, *stamp_args, *args], env)


if __name__ == "__main__":
    sys.exit(main())
