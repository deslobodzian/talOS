#!/usr/bin/env bash
#
# One command to run this robot in simulation.
#
# Replaces the three-terminal sequence in 2026-robot/README.md: the WPILib
# simulation (which stands in for the RoboRIO and is the Driver Station), the
# talOS node graph, and Studio. Everything starts in dependency order and
# everything dies together on Ctrl-C.
#
#   ./sim.sh                 WPILib GUI simulation + node graph + Studio
#   ./sim.sh --headless      no sim GUI; teleop enabled via TALOS_SIM_DS
#   ./sim.sh --no-rio        use an existing controller (no WPILib, no gateway)
#   ./sim.sh --sim-gateway   lightweight simulator instead of WPILib
#   ./sim.sh --no-studio     robot only, no bridge and no browser
#   ./sim.sh --duration-s 30 stop the node graph after 30 seconds
#
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

HEADLESS=0
WANT_STUDIO=1
WANT_RIO=1
WANT_GATEWAY=0
WANT_OPEN=1
DURATION=""
# HTTP port only: the telemetry topic and UDP port live in the robot config
# ([subsystems.telemetry] topic, [studio] udp_port), read by the nodes that
# own each end, so this script never spells them twice.
HTTP_PORT=5800

usage() {
  sed -n '3,${/^[^#]/q;p;}' "$0" | sed 's/^# \{0,1\}//'
  cat <<'USAGE'

Options:
  --headless        run the WPILib sim without its GUI (sets TALOS_SIM_DS=1)
  --no-studio       skip the Studio bridge and the browser
  --no-rio          use an existing controller (skip WPILib sim and gateway)
  --sim-gateway     start the lightweight simulator instead of WPILib
                    (implies --no-rio; conflicts with WPILib sim)
  --no-open         start Studio but do not open a browser
  --duration-s N    pass --duration-s N to the launcher
  --http-port N     Studio HTTP port (default 5800)
  -h, --help        this message
USAGE
}

while [ $# -gt 0 ]; do
  case "$1" in
    --headless)    HEADLESS=1 ;;
    --no-studio)   WANT_STUDIO=0 ;;
    --no-rio)      WANT_RIO=0 ;;
    --sim-gateway|--with-sim-gateway|--lightweight-sim) WANT_GATEWAY=1; WANT_RIO=0 ;;
    --no-open)     WANT_OPEN=0 ;;
    --duration-s)  DURATION="$2"; shift ;;
    --http-port)   HTTP_PORT="$2"; shift ;;
    -h|--help)     usage; exit 0 ;;
    *) echo "sim.sh: unknown argument '$1'" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

# Three explicit controller modes (never combine WPILib + gateway: two
# publishers on /hw/state would conflict):
#   1. start WPILib  (default: WANT_RIO=1, WANT_GATEWAY=0)
#   2. use existing  (--no-rio: WANT_RIO=0, WANT_GATEWAY=0)
#   3. lightweight   (--sim-gateway: WANT_RIO=0, WANT_GATEWAY=1)
if [ "$WANT_RIO" = "1" ] && [ "$WANT_GATEWAY" = "1" ]; then
  die "--sim-gateway conflicts with the WPILib simulation; pass --no-rio to use the lightweight simulator instead"
fi

SESSION_ID="$(date +%s)"
OUT_DIR="/tmp/talos_logs/${SESSION_ID}"
LOG_DIR="${OUT_DIR}/sim"
mkdir -p "$LOG_DIR"

PIDS=""
NAMES=""

# Kill a process and everything it spawned, children first. Gradle and the
# launcher both fork, and killing only the parent orphans a simulation that
# then holds the UDP port against the next run.
kill_tree() {
  local pid="$1" child
  for child in $(pgrep -P "$pid" 2>/dev/null); do
    kill_tree "$child"
  done
  kill -TERM "$pid" 2>/dev/null
}

CLEANED=0
cleanup() {
  [ "$CLEANED" = "1" ] && return
  CLEANED=1
  trap - EXIT INT TERM
  echo ""
  echo "sim.sh: shutting down"
  # Reverse start order, so the bridge stops before the graph it reads.
  local ordered="" p
  for p in $PIDS; do ordered="$p $ordered"; done
  for p in $ordered; do kill_tree "$p"; done
  sleep 1
  for p in $ordered; do kill -KILL "$p" 2>/dev/null; done
  wait 2>/dev/null
  echo "sim.sh: logs in $LOG_DIR"
}
trap cleanup EXIT INT TERM

track() { PIDS="$PIDS $1"; NAMES="$NAMES $2"; }

say() { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
die() { printf '\033[1;31msim.sh:\033[0m %s\n' "$*" >&2; exit 1; }

# --- build ------------------------------------------------------------------
# One bazel invocation for everything, so a compile error stops the run before
# any process starts rather than halfway through bringing the system up.
say "building"
# The platform subsystem configs name standalone @talos_*//:node targets, so
# those externals must be built into bazel-bin/external/talos_*+/ for the
# launcher's dev-mode resolution to find them. Mechanisms build in-tree via
# //2026-robot/main_processor/... (hybrid policy: mechanisms live per year).
BUILD_TARGETS="//talOS/launcher:launcher //talOS/bridge:hardware_node //2026-robot/main_processor/..."
BUILD_TARGETS="$BUILD_TARGETS @talos_driver_station//:node @talos_operator_interface//:node"
BUILD_TARGETS="$BUILD_TARGETS @talos_odometry//:node @talos_telemetry//:node"
if [ "$WANT_STUDIO" = "1" ]; then
  BUILD_TARGETS="$BUILD_TARGETS //studio/bridge:studio_bridge //studio:web_dist"
fi
# shellcheck disable=SC2086
bazel build $BUILD_TARGETS 2>&1 | tail -5 || die "build failed"

# --- 1. the controller processor (WPILib sim) -------------------------------
if [ "$WANT_RIO" = "1" ]; then
  RIO_DIR="$ROOT/2026-robot/controller_processor/rio"
  [ -x "$RIO_DIR/gradlew" ] || die "no gradlew at $RIO_DIR"
  GRADLE_ARGS="simulateNative"
  [ "$HEADLESS" = "1" ] && GRADLE_ARGS="$GRADLE_ARGS -Pheadless"
  say "starting WPILib simulation (log: $LOG_DIR/rio.log)"
  if [ "$HEADLESS" = "1" ]; then
    say "  headless: enabling teleop through TALOS_SIM_DS=1"
    ( cd "$RIO_DIR" && TALOS_SIM_DS=1 ./gradlew $GRADLE_ARGS ) \
      > "$LOG_DIR/rio.log" 2>&1 &
  else
    ( cd "$RIO_DIR" && ./gradlew $GRADLE_ARGS ) > "$LOG_DIR/rio.log" 2>&1 &
  fi
  track $! "wpilib-sim"
  disown $! 2>/dev/null
  # Gradle has to resolve and compile before the simulation exists. Wait for
  # the task to actually be running rather than guessing with a fixed sleep.
  say "  waiting for the simulation to come up"
  for i in $(seq 1 240); do
    if grep -q "Robot program starting" "$LOG_DIR/rio.log" 2>/dev/null; then
      break
    fi
    if grep -qE "FAILURE:|BUILD FAILED" "$LOG_DIR/rio.log" 2>/dev/null; then
      tail -25 "$LOG_DIR/rio.log" >&2
      die "the WPILib simulation failed to start"
    fi
    sleep 0.5
  done
  if [ "$HEADLESS" != "1" ]; then
    say "  enable the robot and pick Teleoperated in the sim GUI"
  fi
fi

# --- 2. the node graph ------------------------------------------------------
say "starting node graph (log: $LOG_DIR/launcher.log)"
LAUNCH_ARGS="--config 2026-robot/main_processor/configuration/robot.toml --sim --session-id $SESSION_ID --output-dir $OUT_DIR"
[ -n "$DURATION" ] && LAUNCH_ARGS="$LAUNCH_ARGS --duration-s $DURATION"
# Studio's bridge is a declared node in the launcher's graph ([studio] in the
# robot config): the launcher starts and supervises it, so sim.sh only asks
# for it and forwards the HTTP port override.
if [ "$WANT_STUDIO" = "1" ]; then
  LAUNCH_ARGS="$LAUNCH_ARGS --start-studio-bridge"
  [ "$HTTP_PORT" != "5800" ] && LAUNCH_ARGS="$LAUNCH_ARGS --studio-http-port $HTTP_PORT"
fi
# The lightweight sim gateway stands in for the controller processor only
# in mode 3 (--sim-gateway). In mode 1 WPILib publishes /hw/state; in mode 2
# an existing controller does. Starting the gateway alongside either would
# create two publishers on /hw/state.
if [ "$WANT_GATEWAY" = "1" ]; then
  LAUNCH_ARGS="$LAUNCH_ARGS --start-sim-gateway"
fi
# shellcheck disable=SC2086
./bazel-bin/talOS/launcher/launcher $LAUNCH_ARGS > "$LOG_DIR/launcher.log" 2>&1 &
LAUNCHER_PID=$!
track "$LAUNCHER_PID" "launcher"

GRAPH="$OUT_DIR/graph.json"
for i in $(seq 1 60); do
  [ -f "$GRAPH" ] && break
  sleep 0.5
done
if [ ! -f "$GRAPH" ]; then
  tail -25 "$LOG_DIR/launcher.log" >&2
  die "the launcher never wrote $GRAPH; the declared graph probably has errors"
fi
say "  declared graph: $GRAPH"

# --- 3. Studio --------------------------------------------------------------
# Nothing to spawn: the launcher owns the bridge (see section 2). Wait for
# its HTTP endpoint, then offer the browser. A viewer that never comes up is
# a warning, not a fatal error: the launcher supervises the bridge and the
# robot session runs regardless (bridge log: $OUT_DIR/studio_bridge.log).
if [ "$WANT_STUDIO" = "1" ]; then
  URL="http://localhost:${HTTP_PORT}"
  STUDIO_UP=0
  for _ in $(seq 1 60); do
    if (echo > /dev/tcp/127.0.0.1/"$HTTP_PORT") 2>/dev/null; then
      STUDIO_UP=1
      break
    fi
    sleep 1
  done
  if [ "$STUDIO_UP" = "1" ]; then
    say "Studio on $URL"
  else
    say "Studio bridge not reachable on $URL yet; continuing without it"
  fi
  if [ "$WANT_OPEN" = "1" ] && [ "$STUDIO_UP" = "1" ]; then
    case "$(uname -s)" in
      Darwin) open "$URL" 2>/dev/null ;;
      Linux)  xdg-open "$URL" >/dev/null 2>&1 ;;
    esac
  fi
fi

say "running. Ctrl-C stops everything."
# Wait on the launcher: when the node graph exits (--duration-s, or a fault),
# the whole session is over and the trap tears down the rest.
wait "$LAUNCHER_PID" 2>/dev/null
