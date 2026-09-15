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
#   ./tools/delivery/deploy.sh check-network [--controller ADDR] [--port N] [--local-port N]
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
# The only network touch in this script: a UDP frame exchange with the
# controller gateway (default 5802, the runtime contract). Sends one
# Heartbeat frame and waits up to 3s for a valid frame back, validated
# against the wire contract in talOS/protocol (40-byte header, magic,
# version). Everything else here and in run.sh works with the cable unplugged.
# The gateway's socket is connected to the companion port (default 5803),
# so probing a real controller needs that local port free; pass
# --local-port 0 for an ephemeral port (test doubles answer from any port).
cmd_check_network() {
  local addr="10.56.87.2" port="5802" local_port="5803"
  while [ $# -gt 0 ]; do
    case "$1" in
      --controller) addr="$2"; shift 2 ;;
      --port) port="$2"; shift 2 ;;
      --local-port) local_port="$2"; shift 2 ;;
      *) die "usage: deploy.sh check-network [--controller ADDR] [--port N] [--local-port N]" ;;
    esac
  done
  say "probing controller $addr:$port over UDP (local port $local_port)"
  python3 - "$addr" "$port" "$local_port" <<'EOF'
import socket, struct, sys, time
addr, port, local_port = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])

# Wire contract from talOS/protocol (types.h, frame.h): little-endian
# 40-byte header; the probe is a Heartbeat (12) with an empty payload.
MAGIC = 0x4F4952534F4C4154
VERSION = 1
HEADER_SIZE = 40
TYPE_HEARTBEAT = 12
FLAG_ACK_REQUESTED = 1
MAX_PAYLOAD = 1400

frame = struct.pack('<QHHHHQQII', MAGIC, VERSION, HEADER_SIZE,
                    TYPE_HEARTBEAT, FLAG_ACK_REQUESTED, 1,
                    time.monotonic_ns() // 1000, 0, 0)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    s.bind(('0.0.0.0', local_port))
except OSError as e:
    sys.exit(f"deploy: error: cannot bind local UDP port {local_port}: {e} "
             f"(stop the process holding it, or pass --local-port 0)")
try:
    s.connect((addr, port))
except OSError as e:
    sys.exit(f"deploy: error: controller unreachable at {addr}:{port}: {e}")
s.send(frame)
deadline = time.monotonic() + 3
while True:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        sys.exit(f"deploy: error: no valid frame from {addr}:{port} within 3s")
    s.settimeout(remaining)
    try:
        data = s.recv(2048)
    except socket.timeout:
        sys.exit(f"deploy: error: no valid frame from {addr}:{port} within 3s")
    except ConnectionRefusedError:
        sys.exit(f"deploy: error: controller refused UDP at {addr}:{port}")
    if len(data) < HEADER_SIZE:
        continue
    magic, version, header_size, ftype, flags, seq, ts, payload_size, _ = \
        struct.unpack('<QHHHHQQII', data[:HEADER_SIZE])
    if magic != MAGIC or version != VERSION or header_size != HEADER_SIZE:
        continue
    if payload_size > MAX_PAYLOAD or len(data) < HEADER_SIZE + payload_size:
        continue
    print(f"deploy: controller reachable at {addr}:{port} "
          f"(frame type={ftype} seq={seq})")
    break
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
# Proves a relocated bundle starts from its own files: refuses when any
# run.sh/launcher path references the network (curl/wget) or the build tree
# (bazel-bin), checks the manifest, then copies the bundle to a temp dir and
# launches the copy briefly (a bounded sim session), failing unless every
# node process comes up and exits cleanly. The temp copy is removed afterwards.
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
  [ -x "$bundle/run.sh" ] || die "bundle has no executable run.sh"
  local tmp; tmp="$(mktemp -d)"
  trap "rm -rf \"$tmp\"" EXIT
  say "copying bundle to $tmp for a relocated launch"
  cp -r "$bundle" "$tmp/bundle"
  chmod -R u+w "$tmp/bundle"
  say "launching relocated bundle (bounded sim session)"
  python3 - "$tmp/bundle" "$tmp/session" <<'EOF'
import json, os, signal, subprocess, sys
bundle, session_dir = sys.argv[1], sys.argv[2]
proc = subprocess.Popen(
    ["./run.sh", "--sim", "--start-sim-gateway",
     "--duration-s", "10", "--output-dir", session_dir],
    cwd=bundle, start_new_session=True)
try:
    rc = proc.wait(timeout=120)
except subprocess.TimeoutExpired:
    os.killpg(proc.pid, signal.SIGTERM)
    try:
        rc = proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        rc = proc.wait()
    sys.exit("deploy: error: relocated bundle did not stop within 120s")
if rc != 0:
    sys.exit(f"deploy: error: relocated bundle exited with code {rc}")
manifest_path = os.path.join(session_dir, "manifest.json")
try:
    nodes = json.load(open(manifest_path))["nodes"]
except (OSError, ValueError, KeyError) as e:
    sys.exit(f"deploy: error: no session manifest at {manifest_path}: {e}")
if not nodes:
    sys.exit("deploy: error: session started no nodes")
bad = [n for n in nodes
       if n.get("pid", 0) <= 0 or n.get("exit_code", -1) != 0]
if bad:
    detail = ", ".join(f"{n.get('name')}(pid={n.get('pid')} "
                       f"exit={n.get('exit_code')})" for n in bad)
    sys.exit(f"deploy: error: nodes failed to come up cleanly: {detail}")
print(f"deploy: {len(nodes)} processes came up and exited cleanly")
EOF
  say "offline startup verified: relocated bundle launched, all nodes clean"
}

usage() { sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

main() {
  local cmd="${1:-}"; shift || true
  # Global flags ride anywhere on the command line; subcommand flags
  # (--controller, --port, --local-port, --user) are consumed by each
  # subcommand below.
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
