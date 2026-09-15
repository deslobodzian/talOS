#!/usr/bin/env bash
#
# talOS bundle deployment: install, networking, boot start, log inspection,
# rollback (item 56). Offline-first (item 58): install and run touch only
# files inside the bundle; the single network operation is the explicit
# `check-network` probe of the controller endpoint.
#
# Layout under the prefix (default /opt/talos):
#   bundles/<stamp>/   one installed bundle each (immutable after install)
#   current -> bundles/<stamp>   what boot and `run` use
#   previous -> bundles/<stamp>  rollback target
#
#   ./tools/delivery/deploy.sh install <bundle> [--prefix DIR]
#   ./tools/delivery/deploy.sh check-network [--controller ADDR] [--port N]
#   ./tools/delivery/deploy.sh enable-boot [--prefix DIR] [--dry-run]
#   ./tools/delivery/deploy.sh logs [--prefix DIR]
#   ./tools/delivery/deploy.sh rollback [--prefix DIR]
#   ./tools/delivery/deploy.sh verify-offline <bundle>
#
set -euo pipefail

PREFIX="/opt/talos"
DRY_RUN=0

say() { printf 'deploy: %s\n' "$*"; }
die() { printf 'deploy: error: %s\n' "$*" >&2; exit 1; }
run() { if [ "$DRY_RUN" = "1" ]; then printf '+ %s\n' "$*"; else "$@"; fi; }

repo_root() { cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd; }

# --- install ---------------------------------------------------------------
# Copies the bundle, verifies its manifest hash (rebuild, never hand-edit),
# and flips `current` atomically, keeping `previous` for rollback.
cmd_install() {
  local bundle="${1:?usage: deploy.sh install <bundle> [--prefix DIR]}"
  [ -f "$bundle/bundle_manifest.json" ] || die "not a bundle: $bundle"
  say "verifying bundle manifest"
  if [ "$DRY_RUN" = "1" ]; then
    echo "+ python3 tools/delivery/compat_record.py --bundle $bundle --check"
  else
    python3 "$(repo_root)/tools/delivery/compat_record.py" \
      --bundle "$bundle" --check || die "bundle verification failed"
  fi
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

# --- check-network ----------------------------------------------------------
# The only network touch in this script: a 3s TCP probe of the controller
# gateway port (default 5802, the runtime contract). Everything else here
# and in run.sh works with the cable unplugged.
cmd_check_network() {
  local addr="10.56.87.2" port="5802"
  while [ $# -gt 0 ]; do
    case "$1" in
      --controller) addr="$2"; shift 2 ;;
      --port) port="$2"; shift 2 ;;
      *) die "usage: deploy.sh check-network [--controller ADDR] [--port N]" ;;
    esac
  done
  say "probing controller $addr:$port"
  python3 - "$addr" "$port" <<'EOF'
import socket, sys
addr, port = sys.argv[1], int(sys.argv[2])
s = socket.create_connection((addr, port), timeout=3)
s.close()
print(f"deploy: controller reachable at {addr}:{port}")
EOF
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
# it names. Falls back to a directory scan when no manifest exists.
cmd_logs() {
  local dir="$PREFIX/current"
  while [ $# -gt 0 ]; do
    case "$1" in --prefix) dir="$2/current"; shift 2 ;; *) dir="$1"; shift ;; esac
  done
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

# --- verify-offline ----------------------------------------------------------
# Proves the installed bundle starts from its own files: refuses when any
# run.sh/launcher path references the network (curl/wget) or the build tree
# (bazel-bin), then does a manifest --check offline.
cmd_verify_offline() {
  local bundle="${1:?usage: deploy.sh verify-offline <bundle>}"
  [ -f "$bundle/bundle_manifest.json" ] || die "not a bundle: $bundle"
  if grep -rEn 'curl|wget|bazel-bin|bazel-out' "$bundle/run.sh" \
      "$bundle/launcher" 2>/dev/null | head -5; then
    die "bundle references the network or the build tree (see above)"
  fi
  say "no network/build-tree references in the launch path"
  python3 "$(repo_root)/tools/delivery/compat_record.py" \
    --bundle "$bundle" --check || die "bundle verification failed"
  say "offline startup verified: install, then $bundle/run.sh --sim"
}

usage() { sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

main() {
  local cmd="${1:-}"; shift || true
  # Global flags ride anywhere on the command line; subcommand flags
  # (--controller, --port, --user) are consumed by each subcommand below.
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
    check-network) cmd_check_network "$@" ;;
    enable-boot) cmd_enable_boot "$@" ;;
    logs) cmd_logs "$@" ;;
    rollback) cmd_rollback "$@" ;;
    verify-offline) cmd_verify_offline "$@" ;;
    *) usage; exit 2 ;;
  esac
}

main "$@"
