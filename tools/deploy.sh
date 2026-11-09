#!/usr/bin/env bash
#
# talOS coprocessor deployment: install a bundle, start it on boot, inspect
# session logs, roll back.
#
# Layout under the prefix (default /opt/talos):
#   bundles/<stamp>/   one installed bundle each (immutable after install)
#   current -> bundles/<stamp>   what boot and `run` use
#   previous -> bundles/<stamp>  rollback target
#
#   ./tools/deploy.sh install <bundle> [--prefix DIR]
#   ./tools/deploy.sh enable-boot [--prefix DIR] [--user NAME] [--dry-run]
#   ./tools/deploy.sh logs [--prefix DIR]
#   ./tools/deploy.sh fetch [--prefix DIR] DEST
#   ./tools/deploy.sh rollback [--prefix DIR]
#
set -euo pipefail

PREFIX="/opt/talos"
DRY_RUN=0

say() { printf 'deploy: %s\n' "$*"; }
die() { printf 'deploy: error: %s\n' "$*" >&2; exit 1; }
run() { if [ "$DRY_RUN" = "1" ]; then printf '+ %s\n' "$*"; else "$@"; fi; }

# --- install ---------------------------------------------------------------
# Copies the bundle and flips `current` atomically, keeping `previous` for
# rollback.
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
  if [ -L "$PREFIX/current" ]; then
    run rm -f "$PREFIX/previous"
    run cp -P "$PREFIX/current" "$PREFIX/previous"
  fi
  run ln -sfn "$dest" "$PREFIX/current"
  say "current -> $dest (previous kept for rollback)"
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
  local unit="[Unit]
Description=talOS robot bundle
After=network-online.target
Wants=network-online.target

[Service]
Type=notify
User=$user
Environment=\"TALOS_LOG_DIR=$PREFIX/logs\"
ExecStart=$PREFIX/current/run.sh
WatchdogSec=3
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
"
  if [ "$(uname -s)" = "Darwin" ]; then
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
</dict>
</plist>
"
    local plist_dest="/Library/LaunchDaemons/org.talos.robot.plist"
    if [ "$DRY_RUN" = "1" ]; then
      say "would write $plist_dest:"
      printf '%s\n' "$plist"
      return 0
    fi
    id -u "$user" >/dev/null 2>&1 || die "no user '$user' (pass --user)"
    printf '%s' "$plist" | sudo tee "$plist_dest" >/dev/null
    sudo chown root:wheel "$plist_dest"
    sudo chmod 644 "$plist_dest"
    sudo launchctl bootout system/org.talos.robot 2>/dev/null || true
    sudo launchctl bootstrap system "$plist_dest"
    say "boot start enabled ($plist_dest, as $user)"
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
# List session directories under the output root and summarize the newest
# manifest. Never search unrelated bundles or the machine-wide /tmp logs.
cmd_logs() {
  [ $# -le 1 ] || die "usage: deploy.sh logs [--prefix DIR] [LOG_ROOT]"
  local dir="${1:-$PREFIX/logs}"
  python3 - "$dir" <<'EOF'
import json
import os
import sys

root = sys.argv[1]
if not os.path.isdir(root):
    print(f"deploy: no session directories at {root}")
    sys.exit(0)
sessions = sorted(entry.path for entry in os.scandir(root)
                  if entry.is_dir(follow_symlinks=False))
print(f"deploy: session directories under {root}:")
for path in sessions:
    print(f"  {path}")
manifests = [os.path.join(path, "manifest.json") for path in sessions
             if os.path.isfile(os.path.join(path, "manifest.json"))]
if not manifests:
    print("deploy: no session manifest yet")
    sys.exit(0)
manifest = max(manifests, key=lambda path: (os.stat(path).st_mtime_ns, path))
print(f"deploy: newest session manifest: {manifest}")
with open(manifest, encoding="utf-8") as source:
    m = json.load(source)
print(f"session {m.get('session_id')} config {m.get('config_path')} "
      f"sha {m.get('config_sha256', '')[:16]}")
for n in m.get("nodes", []):
    print(f"  {n.get('name')}: exit={n.get('exit_code')} "
          f"failed={n.get('recorder_failed')} records={n.get('record_count')} "
          f"{n.get('log_path')}")
EOF
}

# --- fetch ------------------------------------------------------------------
# Copy whole sessions, including manifests and every node's log attempts.
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
# `current` back to `previous`. Never mixes node versions across bundles.
cmd_rollback() {
  [ -L "$PREFIX/previous" ] || die "no previous bundle to roll back to"
  local cur; cur="$(readlink "$PREFIX/current")"
  run ln -sfn "$(readlink "$PREFIX/previous")" "$PREFIX/current"
  run ln -sfn "$cur" "$PREFIX/previous"
  say "rolled back: current -> $(readlink "$PREFIX/current")"
}

usage() { sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

main() {
  local cmd="${1:-}"; shift || true
  # Global flags ride anywhere on the command line; subcommand flags
  # (--user) are consumed by each subcommand below.
  local rest=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --prefix) PREFIX="$2"; shift 2 ;;
      --dry-run) DRY_RUN=1; shift ;;
      *) rest+=("$1"); shift ;;
    esac
  done
  if [ "${#rest[@]}" -gt 0 ]; then
    set -- "${rest[@]}"
  else
    set --
  fi
  case "$cmd" in
    install) cmd_install "$@" ;;
    enable-boot) cmd_enable_boot "$@" ;;
    logs) cmd_logs "$@" ;;
    fetch) cmd_fetch "$@" ;;
    rollback) cmd_rollback "$@" ;;
    *) usage; exit 2 ;;
  esac
}

main "$@"
