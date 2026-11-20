#!/usr/bin/env bash
#
# talOS coprocessor deployment: install a bundle, start it on boot, inspect
# session logs, roll back, prune old bundles.
#
# Layout under the prefix (default /opt/talos):
#   bundles/<stamp>/   one installed bundle each (immutable after install)
#   current -> bundles/<stamp>   what boot and `run` start
#   previous -> bundles/<stamp>  rollback target
#
#   ./tools/deploy.sh install <bundle> [--prefix DIR] [--no-warm-up]
#   ./tools/deploy.sh enable-boot [--prefix DIR] [--user NAME] [--dry-run]
#   ./tools/deploy.sh boot-check
#   ./tools/deploy.sh run [--prefix DIR] [--dry-run] [--] [run.sh arguments]
#   ./tools/deploy.sh logs [--prefix DIR] [LOG_ROOT]
#   ./tools/deploy.sh fetch [--prefix DIR] DEST
#   ./tools/deploy.sh rollback [--prefix DIR] [--no-warm-up]
#   ./tools/deploy.sh prune [--prefix DIR] [--keep N]
#
# install and rollback select a bundle only once its own launcher accepts it
# as the boot service will start it: run.sh --describe-only
# --describe-as-launch (a config with [robot] simulation_only is refused).
#
set -euo pipefail

PREFIX="/opt/talos"
DRY_RUN=0

say() { printf 'deploy: %s\n' "$*"; }
die() { printf 'deploy: error: %s\n' "$*" >&2; exit 1; }
run() { if [ "$DRY_RUN" = "1" ]; then printf '+ %s\n' "$*"; else "$@"; fi; }

# Rename the link itself even when the destination names a directory.
select_bundle() {
  run python3 -c '
import os, sys, uuid
target, link = sys.argv[1:]
temporary = link + ".tmp." + uuid.uuid4().hex
try:
    os.symlink(target, temporary)
    os.replace(temporary, link)
finally:
    if os.path.lexists(temporary):
        os.unlink(temporary)
' "$1" "$2"
}

# --- warm-up ----------------------------------------------------------------
# Runs every member of the installed bundle at $1 once, one at a time, before
# anything selects it: each node binary and the Studio bridge as the launcher's
# probe runs them (`--describe --config <config>`), and the launcher itself with
# --help, then on macOS every native library once (below). macOS charges the
# first exec and first load of every newly written executable file (about 100 ms
# each, serialized host-wide), per file and only on a real exec or load; a fresh
# bundle's first launch pays it inside the launcher's parallel 15 s describe
# deadline and, with other new executables on the host, misses it. Paid here, in
# turn and against the files at their final path, it costs seconds once. A
# --describe run opens no topic, socket or registry row, and builds no backend
# (talOS/node RunMain, talOS/bridge main.cc, studio/bridge main.cc), so this
# touches no hardware and no network, and may run beside a live robot; the
# native pass runs only the libraries' static initializers, in a process of its
# own. A member that fails or outlives its bound (TALOS_DEPLOY_WARM_UP_TIMEOUT_S
# seconds, default 60; tests shorten it) is a warning naming it and never fails
# the command: the graph check that follows judges members. The last line counts what
# finished (members that exited 0, native libraries whose load returned) and
# names each member that did not, with how it ended. A stop signal to deploy.sh kills the
# running member's group and selects nothing (stop traps, below). Members run
# as the invoking user
# (root under sudo). Linux pays no such charge but runs the same path, without
# the native pass.
WARM_UP=1
warm_up() {
  local bundle="$1"
  if [ "$WARM_UP" != "1" ]; then
    # The graph check that follows (install's and rollback's) runs every
    # member and the launcher (all at once, inside the probes' deadline) but
    # loads no native library.
    say "warm-up skipped (--no-warm-up): the graph check pays each member's first run, all at once; on macOS the first launch pays each native library's first load"
    return 0
  fi
  if [ "$DRY_RUN" = "1" ]; then
    printf '+ warm up %s (each member once, in turn)\n' "$bundle"
    return 0
  fi
  say "warming up $bundle: each member's first run, one at a time"
  # In the background so that a stop reaches the warm-up, which kills its
  # member before exiting: bash would otherwise die alone and leave the
  # member running.
  hold_stops "the warm-up"
  python3 - "$bundle" "${TALOS_DEPLOY_WARM_UP_TIMEOUT_S:-60}" <<'EOF' &
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

bundle, bound_text = sys.argv[1:]


def warn(message):
    print(f"deploy: warning: warm-up: {message}", file=sys.stderr, flush=True)


try:
    bound = float(bound_text)
except ValueError:
    bound = 0.0
if not bound > 0:
    warn(f"bound {bound_text!r} is not a positive number of seconds; using 60")
    bound = 60.0

# bundle_manifest.toml as talOS/build/bundle_builder.py writes it: one
# `key = <JSON literal>` per line. Read by line, since macOS's python3 (3.9)
# has no tomllib.
config = None
members = []
section = None
name = None
try:
    with open(os.path.join(bundle, "bundle_manifest.toml"),
              encoding="utf-8") as f:
        manifest = f.read().splitlines()
except OSError as e:
    warn(f"cannot read bundle_manifest.toml ({e}); nothing warmed")
    sys.exit(0)
for line in manifest:
    line = line.strip()
    if line.startswith("["):
        section, name = line, None
        continue
    key, sep, value = line.partition(" = ")
    if not sep:
        continue
    try:
        value = json.loads(value)
    except ValueError:
        continue
    if section is None and key == "config_path":
        config = value
    elif section == "[[nodes]]" and key == "name":
        name = value
    elif section == "[[nodes]]" and key == "binary":
        members.append((name or value, value, ["--describe"]))
    elif section == "[studio]" and key == "bridge":
        members.append(("studio_bridge", value, ["--describe"]))
if config is None:
    warn("bundle_manifest.toml names no config_path; nothing warmed")
    sys.exit(0)
members = [(label, binary, args + ["--config", config])
           for label, binary, args in members]
members.append(("launcher", "launcher", ["--help"]))

# macOS also charges the first load of each newly written native library (a
# Python extension module, a dylib a node loads; docs/DEPLOYMENT.md "Warm-up"
# has the numbers), and --describe imports no backend. The charge is paid by
# dyld's first mapping of the file, even when the load then fails on an
# unresolved symbol, and not by reading it or `codesign -v`; it belongs to the
# file, so the installed copy pays again. So on macOS one more member loads
# every .so and .dylib in the bundle once, in a process of its own, ignoring
# each failure: their static initializers run there, nothing calls into them,
# and it exits without running any destructor. Linux charges nothing, so it is
# skipped there.
# It writes a line to the descriptor it is given (argv[2]; not stdout,
# which a loaded library may print to) as each load returns, unbuffered,
# so a loader that dies part way says how many it loaded.
NATIVE_LOADER = r"""
import ctypes, os, sys
for root, _, files in os.walk(sys.argv[1]):
    for name in sorted(files):
        path = os.path.join(root, name)
        if name.endswith((".so", ".dylib")) and not os.path.islink(path):
            try:
                ctypes.CDLL(path, mode=os.RTLD_LAZY | os.RTLD_LOCAL)
            except OSError:
                pass
            os.write(int(sys.argv[2]), b"loaded\n")
os._exit(0)
"""
natives = 0
if sys.platform == "darwin":
    for root, _, files in os.walk(bundle):
        natives += sum(1 for name in files
                       if name.endswith((".so", ".dylib"))
                       and not os.path.islink(os.path.join(root, name)))
if natives:
    members.append(("native libraries", sys.executable,
                    ["-c", NATIVE_LOADER, bundle]))

# As run.sh starts the launcher: from the bundle, its bin/ first on PATH,
# nothing of a Bazel run or a Python environment inherited. No .pyc is
# written: the bundle stays as installed even when deploy.sh runs as root.
env = dict(os.environ)
for var in ("BUILD_WORKSPACE_DIRECTORY", "RUNFILES_DIR",
            "RUNFILES_MANIFEST_FILE", "JAVA_RUNFILES", "PYTHON_RUNFILES",
            "PYTHONPATH", "VIRTUAL_ENV", "PYTHONHOME"):
    env.pop(var, None)
env["PATH"] = os.path.join(bundle, "bin") + os.pathsep + env.get("PATH", "")
env["PYTHONDONTWRITEBYTECODE"] = "1"

class Interrupted(Exception):
    pass


def interrupted(signum, frame):
    raise Interrupted(signum)


# deploy.sh forwards SIGTERM on its own SIGTERM, SIGINT or SIGHUP; the member
# is in its own session, so no terminal or service stop reaches it.
for number in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
    signal.signal(number, interrupted)


def kill_group(proc):
    """Kills a member's whole process group while its leader is unreaped,
    so the group id cannot have been reused, and reaps it."""
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    proc.wait()


def ended(code):
    if code < 0:
        try:
            return f"killed by {signal.Signals(-code).name}"
        except ValueError:
            return f"killed by signal {-code}"
    return f"exited {code}"


def warm(label, binary, args, err, marks=None):
    """Runs one member; returns None when it exited 0, else how it ended.
    With `marks`, a file, the member gets its descriptor as its last
    argument."""
    proc = None
    passed = (marks.fileno(),) if marks else ()
    try:
        # Its own process group, so a hung member is killed whole.
        proc = subprocess.Popen(
            [os.path.join(bundle, binary)] + args
            + [str(fd) for fd in passed], cwd=bundle, env=env,
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=err,
            pass_fds=passed, start_new_session=True)
    except OSError as e:
        warn(f"{label}: cannot run {binary}: {e}")
        return "could not run"
    try:
        code = proc.wait(timeout=bound)
    except subprocess.TimeoutExpired:
        warn(f"{label}: still running after {bound:g} s; killed")
        return f"killed at the {bound:g} s bound"
    finally:
        # Whatever ends the wait (the bound, an interrupt), the member and
        # its group go with it.
        kill_group(proc)
    if code == 0:
        return None
    err.seek(0)
    lines = err.read()[-4096:].decode(errors="replace").splitlines()
    # Its last line naming an error, else its last line.
    said = [line.strip() for line in lines if line.strip()]
    cause = next((line for line in reversed(said)
                  if "error" in line.lower()), said[-1] if said else "")
    warn(f"{label}: {ended(code)}" + (f": {cause}" if cause else ""))
    return ended(code)


began = time.monotonic()
finished = 0
loaded = 0
missed = []
try:
    for index, (label, binary, args) in enumerate(members):
        native = natives and index == len(members) - 1
        with tempfile.TemporaryFile() as marks, \
                tempfile.TemporaryFile() as err:
            why = warm(label, binary, args, err, marks if native else None)
            if native:
                marks.seek(0)
                loaded = marks.read().count(b"\n")
        if why is not None:
            missed.append(f"{label} ({why})")
        elif not native:
            finished += 1
except Interrupted:
    print("deploy: warm-up interrupted; its member was killed",
          file=sys.stderr, flush=True)
    sys.exit(130)


def of(done, total):
    return f"{done}" if done == total else f"{done} of {total}"


summary = f"deploy: warmed {of(finished, len(members) - bool(natives))} members"
if natives:
    summary += f" and {of(loaded, natives)} native libraries"
summary += f" in {time.monotonic() - began:.1f} s"
if missed:
    summary += "; not warmed: " + ", ".join(missed)
print(summary, flush=True)
EOF
  STOP_CHILD=$!
  local status=0
  wait "$STOP_CHILD" || status=$?
  STOP_CHILD=""
  [ "$status" = "0" ] ||
    printf 'deploy: warning: warm-up did not finish; the launch pays the rest\n' >&2
}

# --- graph check ------------------------------------------------------------
# The launch's own first phase, run on the installed copy before anything
# selects it: the copy's run.sh with --describe-only --describe-as-launch, so
# the bundle's launcher probes every member with --describe on the config
# run.sh passes (Studio's bridge included when the bundle carries it), lints
# the graph and exits, and refuses too what the boot service's launch (run.sh
# bare, so without --sim) would refuse before its probes: a config that sets
# `[robot] simulation_only = true`. A graph it refuses (a naming or
# graph-lint error, a critical member that does not answer, a parameter key
# no node reads) would make the boot service restart a refusing launcher
# forever, so install refuses it. Its warnings (the graph's `  warning: `
# lines; a non-critical member that did not answer is one) do not refuse and
# are printed. A bundle's launcher built before --describe-as-launch refuses
# the flag while it parses its arguments, before it reads the config, with
# exactly one line, `launcher error: unknown argument: --describe-as-launch`;
# on that output alone the check runs again with --describe-only only, which
# judges everything but simulation_only, and warns that it did not check
# that (a refusal prints other lines, so it is never retried). Not covered:
# the refusals of a launch that need what the check must not take beside a
# running robot (the session lock, the live registry, the bridges' sockets),
# and the check runs as the installing person with their environment, not
# as the boot service's account (check_interpreters runs what members find
# through the service's PATH). The check needs no controller, shared memory,
# registry, session lock or realtime privilege: --describe-only takes none
# of them and a --describe run opens no device. Its session directory goes
# to a temporary root, removed afterwards. --no-warm-up does not skip it; on
# macOS a fresh bundle's probes then pay the first-run charge inside their
# deadline. There is no override: the launcher accepts --allow-graph-errors
# only with --sim, so a robot could never launch such a bundle. rollback
# runs it too, on `previous`. A stop stops the launcher and removes the
# temporary root (stop traps, below).
AS_LAUNCH=--describe-as-launch
check_graph() {
  local bundle="$1"
  if [ "$DRY_RUN" = "1" ]; then
    printf '+ %s/run.sh --describe-only --output-dir <temporary root> %s\n' \
      "$bundle" "$AS_LAUNCH"
    return 0
  fi
  say "checking the graph: $bundle/run.sh --describe-only $AS_LAUNCH"
  local scratch
  # macOS's TMPDIR ends in a slash; the launcher prints paths under this.
  scratch="${TMPDIR:-/tmp}"
  if ! scratch="$(mktemp -d "${scratch%/}/talos_install_check.XXXXXX")"; then
    # install's copy (STOP_DISCARD; rollback has none) goes as a refused
    # bundle's does.
    local discard="$STOP_DISCARD"
    release_stops
    [ -z "$discard" ] || discard_copy "$discard"
    die "cannot create a temporary directory for the graph check; nothing selected${discard:+, $discard removed}"
  fi
  local status=0
  # In the background, as the warm-up, so that a stop reaches the launcher.
  hold_stops "the graph check"
  STOP_SCRATCH="$scratch"
  PYTHONDONTWRITEBYTECODE=1 "$bundle/run.sh" --describe-only \
    --output-dir "$scratch" "$AS_LAUNCH" </dev/null >"$scratch/output" 2>&1 &
  STOP_CHILD=$!
  wait "$STOP_CHILD" || status=$?
  STOP_CHILD=""
  if [ "$status" = "1" ] && [ "$(cat "$scratch/output")" = \
      "launcher error: unknown argument: $AS_LAUNCH" ]; then
    printf "deploy: warning: this bundle's launcher predates %s: checked with --describe-only alone, so a config that sets [robot] simulation_only = true is not refused\n" "$AS_LAUNCH" >&2
    status=0
    PYTHONDONTWRITEBYTECODE=1 "$bundle/run.sh" --describe-only \
      --output-dir "$scratch" </dev/null >"$scratch/output" 2>&1 &
    STOP_CHILD=$!
    wait "$STOP_CHILD" || status=$?
    STOP_CHILD=""
  fi
  if [ "$status" = "0" ]; then
    # PrintDeclaredGraph writes each diagnostic as "  warning: <subject>: ...".
    local warnings
    warnings="$(grep '^  warning: ' "$scratch/output" || true)"
    if [ -n "$warnings" ]; then
      say "the launcher accepts the graph, with warnings:"
      printf '%s\n' "$warnings" | sed 's/^/  | /'
    else
      say "the launcher accepts the graph"
    fi
  else
    printf "deploy: the launcher's check of this bundle failed (exit %s); its output:\n" "$status" >&2
    sed 's/^/  | /' "$scratch/output" >&2
  fi
  rm -rf "$scratch"
  STOP_SCRATCH=""
  return "$status"
}

# --- interpreters -----------------------------------------------------------
# A member that starts `#!/usr/bin/env <program>` (run.sh: bash; a Python
# node's entry script under bin/: python3) finds <program> on the PATH it is
# started with, and the boot service starts run.sh with its service
# manager's default PATH, not the installer's: systemd's for a system unit,
# launchd's /usr/bin:/bin:/usr/sbin:/sbin for a LaunchDaemon, where neither
# Homebrew's nor a pyenv python3 is. run.sh puts the bundle's bin/ first. So
# install runs each such program once, `<program> --version` with only that
# PATH and no other environment, and refuses the bundle before writing
# anything when one does not run: absent from that PATH, or macOS's
# /usr/bin/python3 without the Command Line Tools. It runs as the installer;
# the service account reads the same PATH, and enable-boot checks the
# account's access to the bundle with that PATH's python3.
# TALOS_DEPLOY_SERVICE_PATH replaces the service manager's PATH (tests).
service_path() {
  if [ -n "${TALOS_DEPLOY_SERVICE_PATH:-}" ]; then
    printf '%s' "$TALOS_DEPLOY_SERVICE_PATH"
  elif [ "$(uname -s)" = "Darwin" ]; then
    printf '/usr/bin:/bin:/usr/sbin:/sbin'
  else
    printf '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
  fi
}

check_interpreters() {
  local bundle="$1" path file line program rel checked=" "
  path="$bundle/bin:$(service_path)"
  for file in "$bundle/run.sh" "$bundle/launcher" "$bundle"/bin/* \
      "$bundle/studio/studio_bridge"; do
    [ -f "$file" ] || continue
    # The first two bytes, so that no binary is read as a line.
    [ "$(head -c 2 "$file" 2>/dev/null)" = '#!' ] || continue
    line=""
    IFS= read -r line <"$file" || true
    case "$line" in '#!/usr/bin/env '*) ;; *) continue ;; esac
    program="${line#'#!/usr/bin/env '}"
    program="${program%% *}"
    case "$checked" in *" $program "*) continue ;; esac
    checked="$checked$program "
    env -i PATH="$path" "$program" --version </dev/null >/dev/null 2>&1 &&
      continue
    rel="${file#"$bundle/"}"
    die "$rel starts #!/usr/bin/env $program, which the boot service resolves on its PATH, $path, where $program does not run; nothing was written to $PREFIX. Install $program in a directory on that PATH (python3 on macOS: xcode-select --install, whose /usr/bin/python3 launchd finds; on Linux: the distribution's python3 package)"
  done
}

# --- copy -------------------------------------------------------------------
# Copies the bundle at $1 (a real path) to $2, which must not exist, so that
# the copy runs whatever becomes of $1. talos_bundle writes no symlinks
# (docs/DEPLOYMENT.md), but a bundle staged by Bazel's sandbox, or copied
# with its links, is a tree of links into a build tree, which an install
# must not keep: after a `bazel clean` it would dangle. So a link whose
# target leaves the bundle (an absolute target, a relative one climbing out
# of it, or one that resolves outside it) is copied as the file or
# directory it names, and one line counts them; a relative link that stays
# inside is copied as a link, as cp -RP does (GNU and BSD alike), since
# dereferencing it would duplicate its target. Before anything is written,
# every leaving link is checked as the copy will follow it, through each
# directory it names and every link there, and the install is refused,
# naming each, for a link that names nothing (dangling, or a loop of links);
# one naming neither a regular file nor a directory (a FIFO, socket or
# device, which the copy would block on or read forever); and one naming a
# directory that holds the link itself, the bundle or the install (a loop
# the copy would follow until ELOOP or a full disk). Python, as elsewhere
# here: BSD's readlink has no -f before macOS 12.3, and macOS's bash is 3.2.
# Run it only in the background (`copy_bundle ... &`): it execs Python, so
# that $! is the copy's own process, which a stop signals; a subshell in
# between would die of the signal alone and leave the copy running.
copy_bundle() {
  exec python3 - "$1" "$2" <<'EOF'
import os
import shutil
import signal
import stat
import subprocess
import sys

bundle, dest = sys.argv[1:]


class Stopped(Exception):
    pass


STOPS = (signal.SIGTERM, signal.SIGINT, signal.SIGHUP)


def stopped(signum, frame):
    # Once: Ctrl-C reaches this process and install's forwarded SIGTERM
    # follows, which must not interrupt stopping `cp`.
    for number in STOPS:
        signal.signal(number, signal.SIG_IGN)
    raise Stopped(signum)


# install forwards its stop as SIGTERM; the copy stops where it is and
# install removes what it wrote.
for number in STOPS:
    signal.signal(number, stopped)


def contains(outer, path):
    return os.path.commonpath([outer, path]) == outer


# The install's real path: the copy must never follow a link into it.
install = os.path.join(os.path.realpath(os.path.dirname(dest)),
                       os.path.basename(dest))
refusals = []


def check_link(rel, path, holders, checked):
    """Checks the link at `path` (`rel` in the bundle) as the copy follows
    it: what it names, and, for a directory, everything in it. `holders`
    are the real directories the copy is inside of when it reaches the
    link; `checked` the directories already walked."""
    target = os.readlink(path)
    try:
        mode = os.stat(path).st_mode
    except OSError as e:
        refusals.append(f"{rel} -> {target}: names nothing ({e.strerror})")
        return
    if stat.S_ISREG(mode):
        return
    if not stat.S_ISDIR(mode):
        refusals.append(f"{rel} -> {target}: names neither a regular file "
                        "nor a directory")
        return
    real = os.path.realpath(path)
    if any(contains(real, d)
           for d in [bundle, install, os.path.realpath(os.path.dirname(path))]
           + holders):
        refusals.append(f"{rel} -> {target}: names a directory holding the "
                        "link, the bundle or the install")
        return
    if real not in checked:
        checked.add(real)
        check_directory(rel, real, holders + [real], checked)


def check_directory(rel, real, holders, checked):
    try:
        entries = sorted(os.scandir(real), key=lambda e: e.name)
    except OSError as e:
        refusals.append(f"{rel}: cannot be read ({e.strerror})")
        return
    for entry in entries:
        sub = os.path.join(rel, entry.name)
        if entry.is_symlink():
            check_link(sub, entry.path, holders, checked)
        elif entry.is_dir(follow_symlinks=False):
            check_directory(sub, entry.path, holders + [entry.path], checked)
        elif not entry.is_file(follow_symlinks=False):
            refusals.append(f"{sub}: is neither a regular file nor a "
                            "directory")


leaving = []
checked = set()
# Does not descend into a linked directory: the link is judged whole.
for parent, dirs, files in os.walk(bundle):
    for name in dirs + files:
        path = os.path.join(parent, name)
        if not os.path.islink(path):
            continue
        rel = os.path.relpath(path, bundle)
        target = os.readlink(path)
        if not os.path.exists(path):
            refusals.append(f"{rel} -> {target}: names nothing")
            continue
        named = os.path.normpath(os.path.join(os.path.dirname(rel), target))
        if (os.path.isabs(target) or named == os.pardir
                or named.startswith(os.pardir + os.sep)
                or not contains(bundle, os.path.realpath(path))):
            before = len(refusals)
            check_link(rel, path, [], checked)
            if len(refusals) == before:
                leaving.append(rel)
for refusal in refusals:
    print(f"deploy: error: {refusal}", file=sys.stderr)
if refusals:
    sys.exit(1)
copier = None
try:
    # Held while `cp` starts, so a stop cannot fall between its start and
    # `copier` naming it; `cp` inherits the mask, and is stopped with
    # SIGKILL.
    signal.pthread_sigmask(signal.SIG_BLOCK, STOPS)
    try:
        copier = subprocess.Popen(["cp", "-RP", bundle, dest])
    finally:
        signal.pthread_sigmask(signal.SIG_UNBLOCK, STOPS)
    if copier.wait() != 0:
        sys.exit(1)
    for rel in leaving:
        path = os.path.join(dest, rel)
        # cp gives a copied directory its source's mode, read-only in a
        # Bazel tree; install makes the whole copy read-only next.
        parent = os.path.dirname(path)
        os.chmod(parent, os.stat(parent).st_mode | stat.S_IWUSR)
        os.unlink(path)
        source = os.path.join(bundle, rel)
        if os.path.isdir(source):
            shutil.copytree(source, path, symlinks=False)
        else:
            shutil.copy2(source, path)
except Stopped:
    if copier is not None and copier.poll() is None:
        copier.kill()
        copier.wait()
    sys.exit(1)
except (OSError, shutil.Error) as e:
    print(f"deploy: error: copying a link's target: {e}", file=sys.stderr)
    sys.exit(1)
if leaving:
    print(f"deploy: copied {len(leaving)} links that leave the bundle as "
          "the files they name", flush=True)
EOF
}

# Removes an install's copy, which install made read-only.
discard_copy() {
  chmod -R u+w "$1" 2>/dev/null || true
  rm -rf "$1"
}

# What stays selected when an install fails.
kept_selection() {
  if [ -L "$PREFIX/current" ]; then
    printf 'current stays %s' "$(readlink "$PREFIX/current")"
  else
    printf 'nothing is selected'
  fi
}

# --- stop traps -------------------------------------------------------------
# install holds them from its copy until it selects, rollback through its
# warm-up and graph check. A stop (SIGTERM, SIGINT or SIGHUP to deploy.sh)
# stops the step running in the background (STOP_CHILD: the copy's Python,
# which kills its `cp`; the warm-up, which kills its member; the graph
# check's launcher) and waits for it, removes the graph check's temporary
# root (STOP_SCRATCH) and install's copy (STOP_DISCARD), selects nothing and
# exits 128 + the signal. Bash runs a background job with SIGINT ignored, so
# every stop is forwarded as SIGTERM. A stop during a foreground command
# (install's chmod and sync) is handled once that command ends.
STOP_PHASE="" STOP_CHILD="" STOP_SCRATCH="" STOP_DISCARD=""
on_stop() {
  trap - TERM INT HUP
  if [ -n "$STOP_CHILD" ]; then
    kill -TERM "$STOP_CHILD" 2>/dev/null || true
    wait "$STOP_CHILD" 2>/dev/null || true
  fi
  [ -z "$STOP_SCRATCH" ] || rm -rf "$STOP_SCRATCH"
  if [ -n "$STOP_DISCARD" ]; then
    discard_copy "$STOP_DISCARD"
    printf 'deploy: error: stopped during %s; %s removed, %s\n' \
      "$STOP_PHASE" "$STOP_DISCARD" "$(kept_selection)" >&2
  else
    printf 'deploy: error: stopped during %s; nothing selected, %s\n' \
      "$STOP_PHASE" "$(kept_selection)" >&2
  fi
  exit "$1"
}

# Holds the stop traps for the step $1 names (the error line says it).
hold_stops() {
  STOP_PHASE="$1"
  trap 'on_stop 143' TERM
  trap 'on_stop 130' INT
  trap 'on_stop 129' HUP
}

release_stops() {
  trap - TERM INT HUP
  STOP_PHASE="" STOP_CHILD="" STOP_SCRATCH="" STOP_DISCARD=""
}

# --- test-only bundles -------------------------------------------------------
# A testonly talos_bundle (one that may carry testonly nodes, test fixtures)
# writes `testonly = true` in its bundle_manifest.toml's top table
# (talOS/build/bundle_builder.py). Install refuses such a bundle before
# writing anything, unless the prefix lies inside the directory TEST_TMPDIR
# names and TEST_TARGET is set, as Bazel sets both for a test: the tests that
# install their fixture bundles do so there, and a stale TEST_TMPDIR left
# exported in a robot's shell is not enough. There is no flag to override it.
refuse_testonly() {
  local bundle="$1" manifest="$1/bundle_manifest.toml"
  [ -f "$manifest" ] || return 0
  awk '/^\[/ { exit } $0 == "testonly = true" { found = 1 } END { exit !found }' \
    "$manifest" || return 0
  if [ -n "${TEST_TMPDIR:-}" ] && [ -n "${TEST_TARGET:-}" ] && python3 -c '
import os, sys
test, prefix = map(os.path.realpath, sys.argv[1:])
sys.exit(os.path.commonpath([test, prefix]) != test)
' "$TEST_TMPDIR" "$PREFIX"; then
    return 0
  fi
  die "$bundle is a test-only bundle (its bundle_manifest.toml says testonly = true: a testonly talos_bundle, which may carry test fixtures); install refuses it outside a Bazel test (TEST_TMPDIR and TEST_TARGET), so nothing was written to $PREFIX; install the robot's own bundle"
}

# --- install ---------------------------------------------------------------
# Copies the bundle (copy_bundle), warms it in place, checks its graph and
# flips `current` atomically, keeping `previous` for rollback. Selects only a
# complete copy, synced to disk first: a power cut right after the flip must
# not boot a half-written bundle. A copy that fails, that the launcher
# refuses, or whose later step fails before the selection, is removed, as is one stopped (SIGTERM, SIGINT or SIGHUP to
# deploy.sh) while it is copied, made read-only, synced, warmed or checked
# (stop traps, above): the step's processes are stopped first, and
# deploy.sh exits 128 + the signal. So neither a selection nor prune's
# newest --keep N ever counts it. Not covered: SIGKILL. A bundle whose
# `#!/usr/bin/env` interpreter the boot service cannot run is refused before
# anything is written (check_interpreters).
cmd_install() {
  local bundle="${1:?usage: deploy.sh install <bundle> [--prefix DIR]}"
  [ -x "$bundle/run.sh" ] || die "not a bundle (no executable run.sh): $bundle"
  refuse_testonly "$bundle"
  check_interpreters "$bundle"
  # PID-suffixed: two installs in the same second must not share a dir.
  local stamp; stamp="$(date +%Y%m%d_%H%M%S)_$$"
  local dest="$PREFIX/bundles/$stamp"
  say "installing to $dest"
  run mkdir -p "$PREFIX/bundles"
  # Resolve the operand first: the copy would otherwise be of a symlinked
  # bundle path (a runfiles entry, a link into bazel-out), not the bundle,
  # and copy_bundle judges each link against the bundle's real path.
  bundle="$(cd "$bundle" && pwd -P)" || die "cannot resolve $bundle"
  if [ "$DRY_RUN" = "1" ]; then
    printf '+ copy %s %s\n' "$bundle" "$dest"
  else
    # In the background, as the warm-up, so that a stop reaches the copy;
    # a stop during the foreground chmod or sync below runs once it ends.
    local status=0
    STOP_DISCARD="$dest"
    hold_stops "the copy"
    copy_bundle "$bundle" "$dest" &
    STOP_CHILD=$!
    wait "$STOP_CHILD" || status=$?
    STOP_CHILD=""
    if [ "$status" != "0" ]; then
      release_stops
      discard_copy "$dest"
      die "$bundle not installed; $(kept_selection)"
    fi
  fi
  run chmod -R a-w "$dest" 2>/dev/null || true
  if [ "$DRY_RUN" != "1" ] && [ ! -x "$dest/run.sh" ]; then
    release_stops
    discard_copy "$dest"
    die "copy has no executable run.sh, removed: $dest; $(kept_selection)"
  fi
  run sync
  warm_up "$dest"
  if ! check_graph "$dest"; then
    release_stops
    discard_copy "$dest"
    # The launcher's paths name the removed copy and check directory; the
    # bundle given holds the same files at the same relative paths.
    die "$bundle not installed (the paths above were its removed copy, $dest); $(kept_selection)"
  fi
  # Selected from here on: a stop must no longer remove the copy.
  release_stops
  if [ -L "$PREFIX/current" ]; then
    select_bundle "$(readlink "$PREFIX/current")" "$PREFIX/previous"
  fi
  select_bundle "$dest" "$PREFIX/current"
  say "current -> $dest (previous kept); restart the service to activate"
}

# --- boot-check -------------------------------------------------------------
# The macOS settings a robot that boots with nobody logged in needs (TODO.md
# DEPLOY-1). enable-boot sets the pmset ones; FileVault and automatic updates
# it leaves to a person at the machine.
PMSET_BOOT=(autorestart 1 sleep 0)
# The LaunchDaemon's stderr. Its directory is root's: newsyslog, as root,
# chowns and chmods rotated copies by path, so a directory the service
# account could write would let it plant a link there to a root file.
# TALOS_LAUNCHD_STDERR moves where `logs` looks (tests); enable-boot ignores it.
LAUNCHD_STDERR_DIR="/Library/Logs/talos"
LAUNCHD_STDERR="$LAUNCHD_STDERR_DIR/launchd.stderr.log"

# The first line `fdesetup status` prints; empty when it says nothing.
filevault_status() { { fdesetup status 2>&1 || true; } | sed -n 1p; }

# Read-only, no sudo. Fails unless FileVault is off (with it on, the
# LaunchDaemon cannot start until someone logs in and unlocks the disk) and
# each PMSET_BOOT setting holds (autorestart: power back on after a power
# loss; sleep 0: never sleep while idle). Reports automatic updates, which
# can download and install a macOS update, and restart, on their own.
cmd_boot_check() {
  [ $# -eq 0 ] || die "usage: deploy.sh boot-check"
  if [ "$(uname -s)" != "Darwin" ]; then
    say "boot-check covers macOS only; on Linux set the firmware to power on after AC loss"
    return 0
  fi
  local failed=0 status settings key want have
  status="$(filevault_status)"
  if [ "$status" = "FileVault is Off." ]; then
    say "ok    FileVault is off"
  else
    say "FAIL  ${status:-fdesetup status gave no answer}: the LaunchDaemon cannot start before a login; turn FileVault off"
    failed=1
  fi
  settings="$(pmset -g 2>&1 || true)"
  local i=0
  while [ "$i" -lt "${#PMSET_BOOT[@]}" ]; do
    key="${PMSET_BOOT[$i]}" want="${PMSET_BOOT[$((i + 1))]}"
    i=$((i + 2))
    have="$(printf '%s\n' "$settings" | awk -v k="$key" '$1 == k { print $2; exit }')"
    if [ "$have" = "$want" ]; then
      say "ok    pmset $key $have"
    else
      say "FAIL  pmset $key is ${have:-not reported}, want $want (deploy.sh enable-boot sets it with sudo pmset -a)"
      failed=1
    fi
  done
  local prefs=/Library/Preferences/com.apple.SoftwareUpdate
  for key in AutomaticCheckEnabled AutomaticDownload \
      AutomaticallyInstallMacOSUpdates CriticalUpdateInstall; do
    have="$(defaults read "$prefs" "$key" 2>/dev/null || echo "not set")"
    say "info  automatic updates: $key = $have"
    if [ "$key" = AutomaticallyInstallMacOSUpdates ] && [ "$have" = 1 ]; then
      say "WARN  macOS updates install (and restart the Mini) on their own; turn off in System Settings > General > Software Update"
    fi
  done
  return "$failed"
}

# --- enable-boot ------------------------------------------------------------
# Starts the installed bundle on boot as --user: a systemd unit on Linux, a
# launchd LaunchDaemon on macOS (a daemon, not a LaunchAgent, so it starts
# without anyone logging in). --dry-run prints it instead of installing.
cmd_enable_boot() {
  local user="talos"
  while [ $# -gt 0 ]; do
    case "$1" in --user) user="$2"; shift 2 ;; *) break ;; esac
  done
  local darwin=0
  [ "$(uname -s)" = "Darwin" ] && darwin=1
  if [ "$darwin" = "1" ]; then
    # Refused before anything is written: a daemon that cannot start until
    # someone logs in is no boot service. --dry-run still shows the rest.
    local filevault; filevault="$(filevault_status)"
    if [ "$filevault" != "FileVault is Off." ]; then
      local why="${filevault:-fdesetup status gave no answer}: the LaunchDaemon could not start before a login; turn FileVault off first"
      [ "$DRY_RUN" = "1" ] || die "$why"
      say "would refuse: $why"
    fi
  fi
  # Check with the actual service identity, including ancestor traversal and
  # symlink targets. Never make the immutable bundle writable to fix access.
  # The python3 the service's PATH resolves, as check_interpreters runs it,
  # not the one on sudo's secure_path.
  run id -u "$user"
  run sudo -u "$user" env -i PATH="$(service_path)" python3 -c '
import os, sys
root = sys.argv[1]
def require(path, mode):
    if not os.access(path, mode):
        sys.exit("deploy: error: service account cannot read/traverse " + path)
require(root, os.R_OK | os.X_OK)
require(os.path.join(root, "run.sh"), os.R_OK | os.X_OK)
for parent, dirs, files in os.walk(root, onerror=lambda e: sys.exit(str(e))):
    for name in dirs:
        require(os.path.join(parent, name), os.R_OK | os.X_OK)
    for name in files:
        require(os.path.join(parent, name), os.R_OK)
' "$PREFIX/current"
  run sudo install -d -m 0755 -o "$user" "$PREFIX/logs"
  run sudo -u "$user" test -w "$PREFIX/logs"
  # StartLimitIntervalSec=0: systemd's default start limit (5 starts in 10 s,
  # manual restarts included) would leave the unit failed and end
  # Restart=always. LimitRTPRIO/LimitMEMLOCK let the unprivileged service
  # take the realtime scheduling nodes request: SCHED_FIFO up to the bridge's
  # priority 49, and mlockall (docs/NODE_DEVELOPMENT_GUIDE.md).
  # KillMode=mixed: `systemctl stop` sends SIGTERM to the launcher alone,
  # which stops its members itself (SIGINT, then SIGKILL after 2 s); systemd
  # SIGKILLs whatever remains once the launcher exits or TimeoutStopSec
  # passes. The default, control-group, SIGTERMs every member too, and one
  # still starting dies of it, failing the session and the unit.
  local unit="[Unit]
Description=talOS robot bundle
After=network-online.target
Wants=network-online.target
StartLimitIntervalSec=0

[Service]
Type=notify
User=$user
Environment=\"TALOS_LOG_DIR=$PREFIX/logs\"
ExecStart=$PREFIX/current/run.sh --serve-refusal
TimeoutStartSec=1200
KillMode=mixed
TimeoutStopSec=90
WatchdogSec=3
Restart=always
RestartSec=2
LimitRTPRIO=49
LimitMEMLOCK=infinity

[Install]
WantedBy=multi-user.target
"
  if [ "$darwin" = "1" ]; then
    # ProcessType Interactive: launchd applies no resource limits, as for an
    # app. Left unset, launchd applies light limits that throttle the job's
    # CPU and I/O (launchd.plist(5)); Background throttles more and Adaptive
    # follows XPC activity the launcher has none of.
    # stderr (launcher errors: a bad config or refused graph, before any
    # session manifest exists) goes to a file newsyslog bounds below; stdout
    # (progress lines and the echo of node output, which is uncapped and
    # already in each <session>/<name>.log) to /dev/null.
    local stderr_log="$LAUNCHD_STDERR"
    local plist="<?xml version=\"1.0\" encoding=\"UTF-8\"?>
<!DOCTYPE plist PUBLIC \"-//Apple//DTD PLIST 1.0//EN\" \"http://www.apple.com/DTDs/PropertyList-1.0.dtd\">
<plist version=\"1.0\">
<dict>
  <key>Label</key>
  <string>org.talos.robot</string>
  <key>UserName</key>
  <string>$user</string>
  <key>ProgramArguments</key>
  <array>
    <string>$PREFIX/current/run.sh</string>
    <string>--serve-refusal</string>
  </array>
  <key>EnvironmentVariables</key>
  <dict>
    <key>TALOS_LOG_DIR</key>
    <string>$PREFIX/logs</string>
  </dict>
  <key>RunAtLoad</key>
  <true/>
  <key>KeepAlive</key>
  <true/>
  <key>ProcessType</key>
  <string>Interactive</string>
  <key>StandardOutPath</key>
  <string>/dev/null</string>
  <key>StandardErrorPath</key>
  <string>$stderr_log</string>
</dict>
</plist>
"
    # newsyslog (hourly, as root) rotates the file past 1024 KiB and keeps
    # four old copies, so a crash loop cannot fill the disk. A running
    # launcher keeps appending to the copy it opened until it restarts. N:
    # no process to signal; B: no "turned over" line in the new file, which
    # would read as the launcher's. The new file is the account's, as the
    # one created below, whichever way launchd opens it; safe by path only
    # because the account cannot write the directory.
    local group; group="$(id -gn "$user" 2>/dev/null || echo staff)"
    local rotation="# logfilename [owner:group] mode count size(KiB) when flags
$stderr_log $user:$group 644 4 1024 * NB"
    local plist_dest="/Library/LaunchDaemons/org.talos.robot.plist"
    local rotation_dest="/etc/newsyslog.d/org.talos.robot.conf"
    say "setting power with sudo: power on after a power loss, never sleep"
    run sudo pmset -a "${PMSET_BOOT[@]}"
    if [ "$DRY_RUN" = "1" ]; then
      say "would create $LAUNCHD_STDERR_DIR (root:wheel 0755) and $stderr_log ($user:$group 0644)"
      say "would write $plist_dest:"
      printf '%s\n' "$plist"
      say "would write $rotation_dest:"
      printf '%s\n' "$rotation"
      return 0
    fi
    id -u "$user" >/dev/null 2>&1 || die "no user '$user' (pass --user)"
    sudo install -d -m 0755 -o root -g wheel "$LAUNCHD_STDERR_DIR"
    sudo touch "$stderr_log"
    sudo chown "$user:$group" "$stderr_log"
    sudo chmod 644 "$stderr_log"
    printf '%s\n' "$rotation" | sudo tee "$rotation_dest" >/dev/null
    sudo chown root:wheel "$rotation_dest"
    sudo chmod 644 "$rotation_dest"
    printf '%s' "$plist" | sudo tee "$plist_dest" >/dev/null
    sudo chown root:wheel "$plist_dest"
    sudo chmod 644 "$plist_dest"
    sudo launchctl bootout system/org.talos.robot 2>/dev/null || true
    sudo launchctl bootstrap system "$plist_dest"
    say "boot start enabled ($plist_dest, as $user; launcher stderr in $stderr_log)"
    cmd_boot_check || die "boot settings above do not hold"
    return 0
  fi
  if [ "$DRY_RUN" = "1" ]; then
    printf '%s\n' "$unit"
    return 0
  fi
  echo "$unit" | sudo tee /etc/systemd/system/talos.service >/dev/null
  sudo systemctl daemon-reload
  sudo systemctl enable talos.service
  say "boot start enabled (talos.service)"
}

# --- run --------------------------------------------------------------------
# Starts the selected bundle by hand, in the foreground, as the boot service
# does: <prefix>/current/run.sh with TALOS_LOG_DIR=<prefix>/logs and every
# argument after `run` (all of them after `--`) passed on, so `--remote
# <address>` reaches the launcher. On Linux it first takes the boot unit's
# LimitRTPRIO=49 and LimitMEMLOCK=infinity with `ulimit` (soft and hard, as
# systemd sets them), which needs no privilege when the caller's hard limits
# already allow them (docs/INSTALL.md gives the one-time grant); otherwise
# it starts nothing. macOS's realtime policy needs no limit. It then execs
# run.sh, which execs the launcher, so the launcher runs in this process:
# the pid printed first is the launcher's, and Ctrl-C reaches it alone.
# Refused while the boot service is running the same run.sh: the launcher
# would refuse a second session in its namespace anyway, but this says why.

# The program the boot service is running, empty when it is not running or
# cannot be asked. Read-only, no sudo.
boot_service_program() {
  if [ "$1" = "1" ]; then
    command -v launchctl >/dev/null 2>&1 || return 0
    local status
    status="$(launchctl print system/org.talos.robot 2>/dev/null)" || return 0
    printf '%s\n' "$status" | grep -q '^[[:space:]]*state = running$' || return 0
    printf '%s\n' "$status" |
      awk '$1 == "program" && $2 == "=" { print $3; exit }'
  else
    command -v systemctl >/dev/null 2>&1 || return 0
    case "$(systemctl show -p ActiveState --value talos.service 2>/dev/null)" in
      active|activating|reloading|deactivating|refreshing) ;;
      *) return 0 ;;
    esac
    systemctl show -p ExecStart --value talos.service 2>/dev/null |
      sed -n 's/.*path=\([^ ;]*\) ;.*/\1/p' | sed -n 1p
  fi
}

cmd_run() {
  local run_sh="$PREFIX/current/run.sh"
  [ -x "$run_sh" ] ||
    die "no bundle selected: $run_sh is not an executable run.sh (deploy.sh install one first)"
  local darwin=0
  [ "$(uname -s)" = "Darwin" ] && darwin=1
  local running; running="$(boot_service_program "$darwin")"
  # -ef: the same file however a link spells it.
  if [ -n "$running" ] && [ "$running" -ef "$run_sh" ]; then
    if [ "$darwin" = "1" ]; then
      die "the boot service is running $run_sh; stop it first (sudo launchctl bootout system/org.talos.robot; deploy.sh enable-boot restores it)"
    fi
    die "the boot service is running $run_sh; stop it first (sudo systemctl stop talos)"
  fi
  # run.sh's own rule: an --output-dir argument replaces TALOS_LOG_DIR.
  local root="$PREFIX/logs" arg
  for arg in "$@"; do
    case "$arg" in --output-dir|--output-dir=*) root="" ;; esac
  done
  if [ -n "$root" ] && [ -d "$root" ] && [ ! -w "$root" ]; then
    die "cannot write the session root $root (enable-boot gives it to the service account); pass --output-dir DIR"
  fi
  if [ "$darwin" = "0" ]; then
    # Through `run`, a function, so the builtin sets this shell's limits
    # and --dry-run prints exactly these commands.
    if ! { run ulimit -r 49 && run ulimit -l unlimited; } 2>/dev/null; then
      die "the realtime limits the boot service has need a one-time grant: your hard rtprio is $(ulimit -H -r) (need 49) and hard memlock $(ulimit -H -l) KiB (need unlimited); add /etc/security/limits.d/talos.conf for $(id -un) and log in again (docs/INSTALL.md \"Realtime grant for deploy.sh run\"), or for this terminal only: sudo prlimit --pid $PPID --rtprio=49:49 --memlock=unlimited:unlimited"
    fi
  fi
  local where="$root"
  [ -n "$where" ] || where="the --output-dir given"
  local pid="launcher pid $$"
  [ "$DRY_RUN" = "0" ] || pid="would start the launcher"
  say "$pid; its session directory is a new <session id>/ under $where (the launcher prints it as Output Dir)"
  if [ "$DRY_RUN" = "1" ]; then
    printf '+ exec env TALOS_LOG_DIR=%s %s' "$PREFIX/logs" "$run_sh"
    [ $# -eq 0 ] || printf ' %s' "$@"
    printf '\n'
    return 0
  fi
  exec env TALOS_LOG_DIR="$PREFIX/logs" "$run_sh" "$@"
}

# --- logs -------------------------------------------------------------------
# List session directories under the output root, oldest first by the start
# time their manifest records (a session id is not a time), and summarize the
# newest manifest. Never search unrelated bundles or the machine-wide /tmp
# logs.
cmd_logs() {
  [ $# -le 1 ] || die "usage: deploy.sh logs [--prefix DIR] [LOG_ROOT]"
  local dir="${1:-$PREFIX/logs}"
  python3 - "$dir" "${TALOS_LAUNCHD_STDERR:-$LAUNCHD_STDERR}" <<'EOF'
import collections
import datetime
import json
import os
import sys

root, launchd_log = sys.argv[1:]
if not os.path.isdir(root):
    print(f"deploy: no session directories at {root}")
    sys.exit(0)


def manifest_of(path):
    try:
        with open(os.path.join(path, "manifest.json"), encoding="utf-8") as f:
            manifest = json.load(f)
    except (OSError, ValueError):
        return None
    return manifest if isinstance(manifest, dict) else None


def session_class(path):
    """The launcher's cached verdict: match, other or unknown."""
    try:
        with open(os.path.join(path, "session_class"), encoding="utf-8") as f:
            words = f.read().split()
    except OSError:
        words = []
    return words[0] if words else "unclassified"


def when(ns):
    return datetime.datetime.fromtimestamp(ns / 1e9).strftime(
        "%Y-%m-%d %H:%M:%S")


def revision(m):
    rev = str(m.get("git_revision") or "")[:12] or "unstamped"
    return rev + (" dirty" if m.get("git_dirty") else "")


def outcomes(m):
    reasons = collections.Counter()
    for n in m.get("nodes", []):
        for run in n.get("runs", []):
            reasons[str(run.get("reason"))] += 1
    dropped = sum(int(n.get("runs_dropped") or 0) for n in m.get("nodes", []))
    text = ", ".join(f"{count} {reason}"
                     for reason, count in sorted(reasons.items()))
    if dropped:
        text += f", {dropped} not kept"
    return text or "none recorded"


sessions = []
for entry in os.scandir(root):
    if not entry.is_dir(follow_symlinks=False):
        continue
    m = manifest_of(entry.path)
    start = m.get("start_wall_ns") if m else None
    from_manifest = isinstance(start, int) and start > 0
    if not from_manifest:
        start = entry.stat(follow_symlinks=False).st_mtime_ns
    sessions.append((start, entry.path, m, from_manifest))
sessions.sort(key=lambda s: (s[0], s[1]))

print(f"deploy: sessions under {root}, oldest first:")
for start, path, m, from_manifest in sessions:
    if m is None:
        print(f"  {path}  modified {when(start)}  no manifest")
        continue
    started = when(start) if from_manifest else f"{when(start)} (modified)"
    print(f"  {path}  started {started}  {revision(m)}  "
          f"{session_class(path)}  runs: {outcomes(m)}")

with_manifest = [s for s in sessions if s[2] is not None]
newest = with_manifest[-1] if with_manifest else None
if not sessions:
    print("deploy: no sessions yet")
elif sessions[-1][2] is None:
    print("deploy: the newest launch left no manifest (a bad config, a "
          "refused graph, or a launch still starting)")
if sessions and newest is None:
    print("deploy: no session manifest yet")
elif newest is not None:
    start, path, m, _ = newest
    session_dir = path
    print(f"deploy: newest session manifest: {path}/manifest.json")
    print(f"session {m.get('session_id')} started {when(start)} "
          f"config {m.get('config_path')} "
          f"sha {str(m.get('config_sha256') or '')[:16]} "
          f"revision {revision(m)} class {session_class(path)}")
    for n in m.get("nodes", []):
        signal = n.get("exit_signal") or 0  # Absent before the launcher wrote it.
        ended = f" (signal {signal})" if signal else ""
        runs = n.get("runs") or [{}]
        print(f"  {n.get('name')}: exit={n.get('exit_code')}{ended} "
              f"last={runs[-1].get('reason')} "
              f"restarts={n.get('restart_count')} "
              f"failed={n.get('recorder_failed')} "
              f"records={n.get('record_count')} "
              f"{n.get('log_path')} "
              f"output={os.path.join(session_dir, str(n.get('name')) + '.log')}")

# What a launch that died before its manifest said (macOS LaunchDaemon; on
# Linux, journalctl -u talos). Shown when written after the newest session
# started, as by a launch that failed before creating any directory.
if os.path.isfile(launchd_log):
    modified = os.stat(launchd_log).st_mtime_ns
    if newest is None or modified > newest[0]:
        after = ("," if newest is None else
                 " was written after the newest session started;")
        print(f"deploy: {launchd_log}{after} its last lines:")
        with open(launchd_log, encoding="utf-8", errors="replace") as f:
            for line in collections.deque(f, maxlen=20):
                print(f"  | {line.rstrip()}")
    else:
        print(f"deploy: launcher stderr under launchd: {launchd_log}")
elif not sessions or sessions[-1][2] is None:
    print("deploy: launcher errors are in the boot service's log "
          f"(macOS: {launchd_log}; Linux: journalctl -u talos)")
EOF
}

# --- fetch ------------------------------------------------------------------
# Copy whole sessions: manifests, every run's event log and each node's
# output log (<name>.log).
cmd_fetch() {
  [ $# -eq 1 ] || die "usage: deploy.sh fetch [--prefix DIR] DEST"
  python3 - "$PREFIX/logs" "$1" <<'EOF'
import os
import shutil
import sys

root, dest = map(os.path.realpath, sys.argv[1:])
if not os.path.isdir(root):
    sys.exit(f"deploy: error: no session directories at {root}")
if os.path.commonpath([root, dest]) == root:
    sys.exit("deploy: error: fetch destination must be outside the log root")
sessions = sorted(entry.name for entry in os.scandir(root)
                  if entry.is_dir(follow_symlinks=False))
if not sessions:
    sys.exit(f"deploy: error: no session directories at {root}")
# Refuse collisions before copying anything: fetching must not mix sessions.
for name in sessions:
    if os.path.lexists(os.path.join(dest, name)):
        sys.exit(f"deploy: error: destination session already exists: {name}")
os.makedirs(dest, exist_ok=True)
for name in sessions:
    shutil.copytree(os.path.join(root, name), os.path.join(dest, name),
                    symlinks=True)
    print(f"deploy: fetched {os.path.join(dest, name)}")
EOF
}

# --- rollback ---------------------------------------------------------------
# Whole-bundle rollback with its matching configuration and runtime: swaps
# `current` back to `previous`. Never mixes node versions across bundles, and
# never selects a `previous` that no longer holds a bundle. Warms `previous`
# first: one installed by a deploy.sh without the warm-up, and never
# launched, has never run (an already-run one costs about 10 ms a member).
# Then runs install's graph check on it and refuses, changing neither link,
# a bundle its launcher refuses, such as one installed by a deploy.sh
# without the check. A stop selects nothing.
cmd_rollback() {
  [ -L "$PREFIX/current" ] || die "no current bundle at $PREFIX/current"
  [ -L "$PREFIX/previous" ] && [ -x "$PREFIX/previous/run.sh" ] ||
    die "no previous bundle to roll back to ($PREFIX/previous missing or not a bundle)"
  local cur previous
  cur="$(readlink "$PREFIX/current")"
  previous="$(cd "$PREFIX/previous" && pwd -P)"
  warm_up "$previous"
  check_graph "$previous" ||
    die "not rolled back: the launcher refuses $previous; $(kept_selection)"
  release_stops
  select_bundle "$(readlink "$PREFIX/previous")" "$PREFIX/current"
  select_bundle "$cur" "$PREFIX/previous"
  say "rolled back: current -> $(readlink "$PREFIX/current"); restart the service to activate"
}

# --- prune ------------------------------------------------------------------
# Deletes installed bundles except the newest --keep N (default 3) and the
# ones `current` and `previous` select, however old. A running launcher
# restarts nodes from the bundle it started from, so prune only after the
# service has restarted into the current selection.
cmd_prune() {
  local synopsis="usage: deploy.sh prune [--prefix DIR] [--keep N]"
  local keep=3
  while [ $# -gt 0 ]; do
    case "$1" in
      --keep) [ $# -ge 2 ] || die "$synopsis"; keep="$2"; shift 2 ;;
      *) die "$synopsis" ;;
    esac
  done
  # At most nine digits: `[` cannot compare a larger integer, and a failed
  # keep test below would remove the bundle instead of keeping it.
  case "$keep" in ''|*[!0-9]*|??????????*) die "--keep takes a count, not '$keep'" ;; esac
  # Stamps start with a fixed-width install time, so name order is install
  # order. Real directories only: rm -rf must never follow a link out.
  local dir newest_first=()
  for dir in "$PREFIX"/bundles/*; do
    if [ -d "$dir" ] && [ ! -L "$dir" ]; then
      newest_first=("$dir" ${newest_first[@]+"${newest_first[@]}"})
    fi
  done
  local seen=0
  for dir in ${newest_first[@]+"${newest_first[@]}"}; do
    seen=$((seen + 1))
    # -ef compares the directories themselves, however a link spells them.
    if [ "$seen" -le "$keep" ] || [ "$dir" -ef "$PREFIX/current" ] ||
        [ "$dir" -ef "$PREFIX/previous" ]; then
      continue
    fi
    say "removing $dir"
    # Installed read-only; rm needs write access to each directory it empties
    # and reports what it still cannot remove.
    run chmod -R u+w "$dir" 2>/dev/null || true
    run rm -rf "$dir"
  done
}

usage() { sed -n '2,22p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

main() {
  local cmd="${1:-}"; shift || true
  # Global flags ride anywhere on the command line; subcommand flags
  # (--user, --keep) are consumed by each subcommand below.
  local rest=()
  # After `--` nothing is read here: `run` passes it all to run.sh.
  while [ $# -gt 0 ]; do
    case "$1" in
      --) shift; [ $# -eq 0 ] || rest+=("$@"); break ;;
      --prefix) PREFIX="$2"; shift 2 ;;
      --dry-run) DRY_RUN=1; shift ;;
      --no-warm-up) WARM_UP=0; shift ;;
      *) rest+=("$1"); shift ;;
    esac
  done
  # Absolute: install writes it into the `current`/`previous` link targets
  # and enable-boot into the unit; neither may depend on the caller's
  # working directory.
  case "$PREFIX" in /*) ;; *) PREFIX="$(pwd -P)/$PREFIX" ;; esac
  if [ "${#rest[@]}" -gt 0 ]; then
    set -- "${rest[@]}"
  else
    set --
  fi
  case "$cmd" in
    install) cmd_install "$@" ;;
    enable-boot) cmd_enable_boot "$@" ;;
    boot-check) cmd_boot_check "$@" ;;
    logs) cmd_logs "$@" ;;
    fetch) cmd_fetch "$@" ;;
    run) cmd_run "$@" ;;
    rollback) cmd_rollback "$@" ;;
    prune) cmd_prune "$@" ;;
    *) usage; exit 2 ;;
  esac
}

main "$@"
