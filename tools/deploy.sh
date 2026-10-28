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
  run cp -r "$bundle" "$dest"
  run chmod -R a-w "$dest" 2>/dev/null || true
  if [ -L "$PREFIX/current" ]; then
    run rm -f "$PREFIX/previous"
    run cp -P "$PREFIX/current" "$PREFIX/previous"
  fi
  run ln -sfn "$dest" "$PREFIX/current"
  say "current -> $dest (previous kept for rollback)"
}

# --- enable-boot ------------------------------------------------------------
# Writes a systemd unit that starts the installed bundle on boot. Prints the
# unit (and the macOS note) under --dry-run instead of touching /etc.
cmd_enable_boot() {
  local user="talos"
  while [ $# -gt 0 ]; do
    case "$1" in --user) user="$2"; shift 2 ;; *) break ;; esac
  done
  local unit="[Unit]
Description=talOS robot bundle
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$user
ExecStart=$PREFIX/current/run.sh
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
"
  if [ "$DRY_RUN" = "1" ] || [ "$(uname -s)" = "Darwin" ]; then
    printf '%s\n' "$unit"
    [ "$(uname -s)" = "Darwin" ] && \
      say "macOS has no systemd: copy the ExecStart line into a launchd plist"
    [ "$DRY_RUN" = "1" ] && return 0
  fi
  echo "$unit" | sudo tee /etc/systemd/system/talos.service >/dev/null
  sudo systemctl daemon-reload
  sudo systemctl enable talos.service
  say "boot start enabled (talos.service)"
}

# --- logs -------------------------------------------------------------------
# Manifest first (which session, which config hash), then the per-node logs
# it names. Falls back to a directory listing when no manifest exists.
cmd_logs() {
  local dir="$PREFIX/current"
  [ $# -gt 0 ] && dir="$1"
  [ -e "$dir" ] || die "nothing installed at $dir"
  local manifest; manifest="$(find -L "$dir" -maxdepth 3 -name manifest.json 2>/dev/null | head -1)"
  if [ -n "$manifest" ]; then
    say "session manifest: $manifest"
    python3 - "$manifest" <<'EOF'
import json, sys
m = json.load(open(sys.argv[1]))
print(f"session {m.get('session_id')} config {m.get('config_path')} "
      f"sha {m.get('config_sha256', '')[:16]}")
for n in m.get("nodes", []):
    print(f"  {n.get('name')}: exit={n.get('exit_code')} "
          f"failed={n.get('recorder_failed')} records={n.get('record_count')} "
          f"{n.get('log_path')}")
EOF
  else
    say "no session manifest yet; bundle contents:"
    ls "$dir"
  fi
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
    rollback) cmd_rollback "$@" ;;
    *) usage; exit 2 ;;
  esac
}

main "$@"
