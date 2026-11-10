#!/usr/bin/env bash
#
# Deployment-host qualification in one run: the evidence TODO.md MAC-1 and INC-3
# ask for from the Mac mini. Also runs on Linux.
#
#   tools/mac_qualify.sh [--log FILE]
#
# In order: host information; on macOS the boot settings a robot that starts
# with nobody logged in needs (tools/deploy.sh boot-check: FileVault off,
# pmset autorestart 1 and sleep 0, automatic updates reported; read-only, no
# sudo); bazel build //...; bazel test --test_output=errors //...; launcher
# --describe-only on every robot config;
# restart latency on the physics simulation, idle and under disk load
# (//talOS/launcher:restart_latency); the pose_estimator update() benchmark
# for both backends. Every step runs even after a failure; a summary lists
# each step's result, and the exit status is 1 if any failed. All output is
# also written to the log (default /tmp/talos-qualify-<host>-<time>.log).
#
# The config's nodes come from bazel-bin, so the build step must pass for the
# later steps to mean anything. On macOS the restart-latency step asks for
# sudo once (`sudo -v`) so it can `purge` the page cache before Python node
# restarts; without it those spawn times are reported as warm.
#
# Portable to macOS's bash 3.2: no associative arrays, no mapfile.

set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1

LOG="/tmp/talos-qualify-$(hostname -s 2>/dev/null || echo host)-$(date +%Y%m%d-%H%M%S).log"
while [ $# -gt 0 ]; do
  case "$1" in
    --log)
      [ $# -ge 2 ] || { echo "mac_qualify.sh: --log needs a file" >&2; exit 2; }
      LOG="$2"
      shift
      ;;
    -h|--help)
      sed -n '3,/^$/p' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *) echo "mac_qualify.sh: unknown argument '$1'" >&2; exit 2 ;;
  esac
  shift
done

exec > >(tee -a "$LOG") 2>&1

CONFIGS="2026-robot/main_processor/configuration/robot.toml
2026-robot/main_processor/configuration/robot_field.toml
2026-robot/main_processor/configuration/robot_physics_sim.toml
bench/main_processor/configuration/robot.toml
bench/main_processor/configuration/robot_bench.toml
examples/superstructure/config/robot.toml
examples/superstructure/config/robot_py.toml"
PHYSICS_SIM=2026-robot/main_processor/configuration/robot_physics_sim.toml

SUMMARY=""
FAILED=0

section() {
  printf '\n==================== %s ====================\n' "$*"
}

# run_step NAME COMMAND...: runs the command, prints it and its duration, and
# records PASS or FAIL in the summary.
run_step() {
  local name="$1" start status result
  shift
  section "$name"
  printf '$ %s\n' "$*"
  start=$(date +%s)
  "$@"
  status=$?
  if [ "$status" -eq 0 ]; then
    result="PASS"
  else
    result="FAIL ($status)"
    FAILED=1
  fi
  SUMMARY="${SUMMARY}$(printf '%-10s %5ss  %s' "$result" "$(( $(date +%s) - start ))" "$name")
"
}

section "host"
echo "date: $(date)"
echo "revision: $(git rev-parse HEAD 2>/dev/null) ($(git status --porcelain 2>/dev/null | wc -l | tr -d ' ') changed files)"
uname -a
case "$(uname -s)" in
  Darwin)
    sw_vers
    echo "cpu: $(sysctl -n machdep.cpu.brand_string)"
    echo "cores: $(sysctl -n hw.physicalcpu) physical, $(sysctl -n hw.logicalcpu) logical"
    echo "memory: $(( $(sysctl -n hw.memsize) / 1073741824 )) GiB"
    pmset -g 2>/dev/null | grep -E '^ *(lowpowermode|powermode) '
    ;;
  *)
    sed -n 's/^model name[[:space:]]*: //p' /proc/cpuinfo | sed -n 1p
    echo "cores: $(getconf _NPROCESSORS_ONLN) logical"
    sed -n 's/^MemTotal: *//p' /proc/meminfo
    ;;
esac
df -h "${TMPDIR:-/tmp}" | sed -n '1,2p'
bazel --version

if [ "$(uname -s)" = "Darwin" ]; then
  run_step "boot settings" "$ROOT/tools/deploy.sh" boot-check
fi
run_step "bazel build //..." bazel build //...
run_step "bazel test //..." bazel test --test_output=errors //...

for config in $CONFIGS; do
  run_step "describe-only $config" \
    bazel run //talOS/launcher -- --config "$ROOT/$config" --describe-only
done

if [ "$(uname -s)" = "Darwin" ]; then
  section "sudo for purge"
  sudo -v || echo "no sudo credentials: Python node restarts will be reported warm"
fi
GATEWAY="--start-sim-gateway --sim-gateway-arg=--physics
  --sim-gateway-arg=--port --sim-gateway-arg={remote_port}
  --sim-gateway-arg=--remote-port --sim-gateway-arg={local_port}"
# shellcheck disable=SC2086
run_step "restart latency, idle" \
  bazel run //talOS/launcher:restart_latency -- --config "$PHYSICS_SIM" \
  --evict-cache $GATEWAY
# shellcheck disable=SC2086
run_step "restart latency, 8 disk writers" \
  bazel run //talOS/launcher:restart_latency -- --config "$PHYSICS_SIM" \
  --disk-load 8 --python-kills 0 $GATEWAY

run_step "pose_estimator update() benchmark" \
  bazel run //2026-robot/main_processor/pose_estimator:update_benchmark -- --backend both

section "summary"
printf '%s' "$SUMMARY"
echo "log: $LOG"
exit "$FAILED"
