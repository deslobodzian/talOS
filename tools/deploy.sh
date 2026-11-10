#!/usr/bin/env bash
#
# talOS coprocessor deployment: install a bundle, start it on boot, inspect
# session logs, roll back, prune old bundles.
#
# Layout under the prefix (default /opt/talos):
#   bundles/<stamp>/   one installed bundle each (immutable after install)
#   current -> bundles/<stamp>   what boot and `run` use
#   previous -> bundles/<stamp>  rollback target
#
#   ./tools/deploy.sh install <bundle> [--prefix DIR]
#   ./tools/deploy.sh enable-boot [--prefix DIR] [--user NAME] [--dry-run]
#   ./tools/deploy.sh boot-check
#   ./tools/deploy.sh logs [--prefix DIR] [LOG_ROOT]
#   ./tools/deploy.sh fetch [--prefix DIR] DEST
#   ./tools/deploy.sh rollback [--prefix DIR]
#   ./tools/deploy.sh prune [--prefix DIR] [--keep N]
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

# --- install ---------------------------------------------------------------
# Copies the bundle and flips `current` atomically, keeping `previous` for
# rollback. Selects only a complete copy, synced to disk first: a power cut
# right after the flip must not boot a half-written bundle.
cmd_install() {
  local bundle="${1:?usage: deploy.sh install <bundle> [--prefix DIR]}"
  [ -x "$bundle/run.sh" ] || die "not a bundle (no executable run.sh): $bundle"
  # PID-suffixed: two installs in the same second must not share a dir.
  local stamp; stamp="$(date +%Y%m%d_%H%M%S)_$$"
  local dest="$PREFIX/bundles/$stamp"
  say "installing to $dest"
  run mkdir -p "$PREFIX/bundles"
  # Resolve the operand first: -P would copy a symlinked bundle path (a
  # runfiles entry, a link into bazel-out) as a link, not the bundle.
  bundle="$(cd "$bundle" && pwd -P)" || die "cannot resolve $bundle"
  # -RP keeps the bundle's relative symlinks (GNU and BSD alike). macOS
  # `cp -r` dereferences them; a copied venv python3 then cannot find
  # @executable_path/../lib/libpython3.11.dylib and every Python node fails.
  run cp -RP "$bundle" "$dest"
  run chmod -R a-w "$dest" 2>/dev/null || true
  [ "$DRY_RUN" = "1" ] || [ -x "$dest/run.sh" ] ||
    die "copy has no executable run.sh; current unchanged: $dest"
  run sync
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
  run id -u "$user"
  run sudo -u "$user" python3 -c '
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
  local unit="[Unit]
Description=talOS robot bundle
After=network-online.target
Wants=network-online.target
StartLimitIntervalSec=0

[Service]
Type=notify
User=$user
Environment=\"TALOS_LOG_DIR=$PREFIX/logs\"
ExecStart=$PREFIX/current/run.sh
TimeoutStartSec=1200
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
# never selects a `previous` that no longer holds a bundle.
cmd_rollback() {
  [ -L "$PREFIX/current" ] || die "no current bundle at $PREFIX/current"
  [ -L "$PREFIX/previous" ] && [ -x "$PREFIX/previous/run.sh" ] ||
    die "no previous bundle to roll back to ($PREFIX/previous missing or not a bundle)"
  local cur; cur="$(readlink "$PREFIX/current")"
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

usage() { sed -n '2,17p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

main() {
  local cmd="${1:-}"; shift || true
  # Global flags ride anywhere on the command line; subcommand flags
  # (--user, --keep) are consumed by each subcommand below.
  local rest=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --prefix) PREFIX="$2"; shift 2 ;;
      --dry-run) DRY_RUN=1; shift ;;
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
    rollback) cmd_rollback "$@" ;;
    prune) cmd_prune "$@" ;;
    *) usage; exit 2 ;;
  esac
}

main "$@"
