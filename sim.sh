#!/usr/bin/env bash
#
# One command to run a robot in simulation (2026-robot unless --config).
#
# Starts the WPILib simulation (which stands in for the RoboRIO and is the
# Driver Station), the talOS node graph, and Studio, in dependency order.
# Everything dies together on Ctrl-C.
#
#   ./sim.sh                 WPILib GUI simulation + node graph + Studio
#   ./sim.sh --headless      no sim GUI; teleop enabled via TALOS_SIM_DS
#   ./sim.sh --headless --mode-schedule teleop:60,auto:20,disabled:5
#                            no sim GUI; scripted Driver Station modes
#   ./sim.sh --headless --mode-schedule teleop:30,disabled:5 \
#       --joystick-schedule '0:axes=0,0,0;buttons=0;pov=-1|3:axes=0,-0.5,0;buttons=0;pov=-1'
#                            ... and a scripted joystick 0
#   ./sim.sh --no-rio        use an existing controller (no WPILib, no gateway)
#   ./sim.sh --sim-gateway   lightweight physics simulator instead of WPILib
#   ./sim.sh --no-studio     robot only, no bridge and no browser
#   ./sim.sh --no-open       Studio without opening a browser
#   ./sim.sh --sim-gateway --gateway-arg ARG
#                            forward ARG to the lightweight simulator
#   ./sim.sh --duration-s 30 stop the node graph after 30 seconds
#   ./sim.sh --config my_robot/main_processor/configuration/robot.toml \
#       --sim-gateway --no-studio
#                            another robot (bazel run //tools:new_robot)
#
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CALLER_DIR="$PWD"
cd "$ROOT" || exit 1

HEADLESS=0
WANT_STUDIO=1
WANT_RIO=1
WANT_GATEWAY=0
WANT_OPEN=1
DURATION=""
MODE_SCHEDULE=""
JOYSTICK_SCHEDULE=""
ALLIANCE=""
GATEWAY_ARGS=()
CONFIG=""
# HTTP port only: the telemetry topic lives in the robot config
# ([subsystems.telemetry] topic), read by the nodes that own each end, so this
# script never spells it twice.
HTTP_PORT=5800

say() { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
die() { printf '\033[1;31msim.sh:\033[0m %s\n' "$*" >&2; exit 1; }

usage() {
  sed -n '3,${/^[^#]/q;p;}' "$0" | sed 's/^# \{0,1\}//'
  cat <<'USAGE'

Options:
  --config FILE     the robot config to run, relative to the repository root
                    or the current directory (default: 2026-robot's robot.toml,
                    or robot_physics_sim.toml with --sim-gateway); its
                    generated manifest must be current
  --headless       run the WPILib sim without its GUI (sets TALOS_SIM_DS=auto)
  --mode-schedule S script the Driver Station as mode:seconds phases
                    (disabled, auto, teleop, test; the last is held). With
                    --headless it sets TALOS_SIM_DS_SCHEDULE, counted from the
                    gateway's configuration; with --sim-gateway it is that
                    simulator's --mode-schedule, counted from its start
  --joystick-schedule S
                    script joystick 0 as 'seconds:axes=A0,...;buttons=MASK;
                    pov=DEG|...' snapshots (syntax: the lightweight
                    simulator's README), on the same clock as
                    --mode-schedule; requires --headless or --sim-gateway
  --alliance A      blue or red (default blue): TALOS_SIM_DS_ALLIANCE with
                    --headless, the simulator's --alliance with --sim-gateway
  --no-studio       skip the Studio bridge and the browser
  --no-rio          use an existing controller (skip WPILib sim and gateway)
  --sim-gateway     start the lightweight physics simulator instead of WPILib
                    (implies --no-rio; conflicts with WPILib sim)
  --gateway-arg ARG forward one argument verbatim to sim_gateway (repeatable;
                    requires --sim-gateway, which always enables --physics)
  --no-open         start Studio but do not open a browser
  --duration-s N    pass --duration-s N to the launcher
  --http-port N     Studio HTTP port (default 5800)
  -h, --help        this message
USAGE
}

while [ $# -gt 0 ]; do
  case "$1" in
    --config)
      [ $# -ge 2 ] || { echo "sim.sh: --config needs an argument" >&2; exit 2; }
      CONFIG="$2"
      shift
      ;;
    --headless)    HEADLESS=1 ;;
    --no-studio)   WANT_STUDIO=0 ;;
    --no-rio)      WANT_RIO=0 ;;
    --sim-gateway|--with-sim-gateway|--lightweight-sim) WANT_GATEWAY=1; WANT_RIO=0 ;;
    --gateway-arg)
      [ $# -ge 2 ] || { echo "sim.sh: --gateway-arg needs an argument" >&2; exit 2; }
      GATEWAY_ARGS+=(--sim-gateway-arg "$2")
      shift
      ;;
    --no-open)     WANT_OPEN=0 ;;
    --duration-s)  DURATION="$2"; shift ;;
    --mode-schedule)
      [ $# -ge 2 ] || { echo "sim.sh: --mode-schedule needs an argument" >&2; exit 2; }
      MODE_SCHEDULE="$2"
      shift
      ;;
    --joystick-schedule)
      [ $# -ge 2 ] || { echo "sim.sh: --joystick-schedule needs an argument" >&2; exit 2; }
      JOYSTICK_SCHEDULE="$2"
      shift
      ;;
    --alliance)
      [ $# -ge 2 ] || { echo "sim.sh: --alliance needs an argument" >&2; exit 2; }
      ALLIANCE="$2"
      shift
      ;;
    --http-port)   HTTP_PORT="$2"; shift ;;
    -h|--help)     usage; exit 0 ;;
    *) echo "sim.sh: unknown argument '$1'" >&2; usage >&2; exit 2 ;;
  esac
  shift
done
if [ "${#GATEWAY_ARGS[@]}" -gt 0 ] && [ "$WANT_GATEWAY" != "1" ]; then
  echo "sim.sh: --gateway-arg requires --sim-gateway" >&2
  exit 2
fi
if [ -n "$MODE_SCHEDULE" ]; then
  if [ "$WANT_GATEWAY" = "1" ]; then
    GATEWAY_ARGS+=(--sim-gateway-arg --mode-schedule --sim-gateway-arg "$MODE_SCHEDULE")
  elif [ "$WANT_RIO" != "1" ] || [ "$HEADLESS" != "1" ]; then
    # With the GUI, its Robot State window is the Driver Station; with --no-rio
    # the existing controller's Driver Station is.
    echo "sim.sh: --mode-schedule requires --headless or --sim-gateway" >&2
    exit 2
  fi
fi
if [ -n "$JOYSTICK_SCHEDULE" ]; then
  if [ "$WANT_GATEWAY" = "1" ]; then
    GATEWAY_ARGS+=(--sim-gateway-arg --joystick-schedule --sim-gateway-arg "$JOYSTICK_SCHEDULE")
  elif [ "$WANT_RIO" != "1" ] || [ "$HEADLESS" != "1" ]; then
    echo "sim.sh: --joystick-schedule requires --headless or --sim-gateway" >&2
    exit 2
  fi
fi
if [ -n "$ALLIANCE" ]; then
  if [ "$WANT_GATEWAY" = "1" ]; then
    GATEWAY_ARGS+=(--sim-gateway-arg --alliance --sim-gateway-arg "$ALLIANCE")
  elif [ "$WANT_RIO" != "1" ] || [ "$HEADLESS" != "1" ]; then
    echo "sim.sh: --alliance requires --headless or --sim-gateway" >&2
    exit 2
  fi
fi

# Three explicit controller modes (never combine WPILib + gateway: two
# publishers on /hw/state would conflict):
#   1. start WPILib  (default: WANT_RIO=1, WANT_GATEWAY=0)
#   2. use existing  (--no-rio: WANT_RIO=0, WANT_GATEWAY=0)
#   3. lightweight   (--sim-gateway: WANT_RIO=0, WANT_GATEWAY=1)
if [ "$WANT_RIO" = "1" ] && [ "$WANT_GATEWAY" = "1" ]; then
  die "--sim-gateway conflicts with the WPILib simulation; pass --no-rio to use the lightweight simulator instead"
fi

# --- the robot config -------------------------------------------------------
# Every target follows the config: the nodes its manifest lists, its [robot]
# hardware_node and sim_gateway, and its [studio] bridge. Bazel and the
# launcher take it relative to the repository root.
if [ -z "$CONFIG" ]; then
  CONFIG="2026-robot/main_processor/configuration/robot.toml"
  if [ "$WANT_GATEWAY" = "1" ]; then
    CONFIG="2026-robot/main_processor/configuration/robot_physics_sim.toml"
  fi
else
  case "$CONFIG" in
    /*) ;;
    *) [ -f "$CONFIG" ] || CONFIG="$CALLER_DIR/$CONFIG" ;;
  esac
  [ -f "$CONFIG" ] || die "no robot config at $CONFIG"
  CONFIG="$(cd "$(dirname "$CONFIG")" && pwd)/$(basename "$CONFIG")"
  case "$CONFIG" in
    "$ROOT"/*) CONFIG="${CONFIG#"$ROOT"/}" ;;
    *) die "$CONFIG is outside the repository ($ROOT)" ;;
  esac
fi

# A stale manifest would build the wrong nodes; sync_manifest prints the
# command that regenerates it. Its run also builds the resolver used below.
say "checking $CONFIG"
CHECK_OUTPUT="$(bazel run --noshow_progress --ui_event_filters=-info \
  //tools:sync_manifest -- --check "$CONFIG" 2>&1)" || {
  printf '%s\n' "$CHECK_OUTPUT" >&2
  die "$CONFIG cannot run as it stands (above)"
}
# The effective config, through the launcher's own parser: the manifest's
# node labels (current, per the check) and the labels of [robot] and
# [studio], defaults where unset.
TARGETS="$(./bazel-bin/talOS/configuration/resolve "$CONFIG" | python3 -c '
import json, sys
config = json.load(sys.stdin)
robot = config["effective"].get("robot", {})
studio = config["effective"].get("studio", {})
print("nodes", *config["nodes"])
print("hardware_node", robot.get("hardware_node", "//talOS/bridge:hardware_node"))
print("sim_gateway", robot.get("sim_gateway", ""))
print("studio_bridge", studio.get("bridge", "//studio/bridge:studio_bridge"))
')" || die "cannot resolve $CONFIG"
target() { printf '%s\n' "$TARGETS" | sed -n "s|^$1 ||p"; }
if [ "$WANT_GATEWAY" = "1" ] && [ -z "$(target sim_gateway)" ]; then
  die "$CONFIG sets no [robot] sim_gateway for --sim-gateway to start"
fi

SESSION_ID="$(date +%s)"
OUT_DIR="/tmp/talos_logs"
SESSION_DIR="${OUT_DIR}/${SESSION_ID}"
LOG_DIR="${SESSION_DIR}/sim"
mkdir -p "$LOG_DIR"

PIDS=""
NAMES=""

# Print a process and everything it spawned, children first. Gradle and the
# launcher both fork, and killing only the parent orphans a simulation that
# then holds the UDP port against the next run.
descendants() {
  local child
  for child in $(pgrep -P "$1" 2>/dev/null); do
    descendants "$child"
  done
  echo "$1"
}

CLEANED=0
cleanup() {
  [ "$CLEANED" = "1" ] && return
  CLEANED=1
  trap - EXIT INT TERM
  echo ""
  echo "sim.sh: shutting down"
  # Reverse start order, so the bridge stops before the graph it reads.
  local ordered="" p tree=""
  for p in $PIDS; do ordered="$p $ordered"; done
  # Snapshot the trees before signalling: a parent that exits on TERM
  # reparents its children, and they could no longer be found to KILL.
  for p in $ordered; do tree="$tree $(descendants "$p")"; done
  for p in $tree; do kill -TERM "$p" 2>/dev/null; done
  sleep 1
  for p in $tree; do kill -KILL "$p" 2>/dev/null; done
  wait 2>/dev/null
  echo "sim.sh: session logs in $SESSION_DIR (launcher/RIO output in $LOG_DIR)"
}
trap cleanup EXIT INT TERM

track() { PIDS="$PIDS $1"; NAMES="$NAMES $2"; }

# --- build ------------------------------------------------------------------
# One bazel invocation for everything, so a compile error stops the run before
# any process starts rather than halfway through bringing the system up.
say "building"
BUILD_TARGETS="//talOS/launcher:launcher $(target hardware_node) $(target nodes)"
if [ "$WANT_GATEWAY" = "1" ]; then
  BUILD_TARGETS="$BUILD_TARGETS $(target sim_gateway)"
fi
if [ "$WANT_STUDIO" = "1" ]; then
  BUILD_TARGETS="$BUILD_TARGETS $(target studio_bridge) //studio:web_dist"
fi
# shellcheck disable=SC2086
bazel build $BUILD_TARGETS 2>&1 | tail -5 || die "build failed"

# --- 1. the controller processor (WPILib sim) -------------------------------
if [ "$WANT_RIO" = "1" ]; then
  # The robot's own RoboRIO program when it has one beside its
  # main_processor/, else 2026-robot's; the bridge sends either its hardware
  # config.
  RIO_DIR="$ROOT/2026-robot/controller_processor/rio"
  case "$CONFIG" in
    */main_processor/configuration/*)
      OWN_RIO="$ROOT/${CONFIG%%/main_processor/configuration/*}/controller_processor/rio"
      [ -x "$OWN_RIO/gradlew" ] && RIO_DIR="$OWN_RIO"
      ;;
  esac
  [ -x "$RIO_DIR/gradlew" ] || die "no gradlew at $RIO_DIR"
  # --no-daemon keeps the build, and the simulation it runs, inside this
  # script's process tree; a Gradle daemon is detached, so cleanup could
  # never reach a simulation it started.
  GRADLE_ARGS=(simulateNative --no-daemon)
  [ "$HEADLESS" = "1" ] && GRADLE_ARGS+=(-Pheadless)
  # Empty unless --joystick-schedule or --alliance; SimDriverStation reads
  # each only when set.
  RIO_ENV=()
  if [ "$HEADLESS" = "1" ] && [ -n "$ALLIANCE" ]; then
    RIO_ENV+=("TALOS_SIM_DS_ALLIANCE=$ALLIANCE")
  fi
  if [ "$HEADLESS" = "1" ] && [ -n "$JOYSTICK_SCHEDULE" ]; then
    say "  headless: joystick 0 schedule $JOYSTICK_SCHEDULE (from configuration)"
    RIO_ENV+=("TALOS_SIM_JOYSTICK_SCHEDULE=$JOYSTICK_SCHEDULE")
  fi
  say "starting WPILib simulation (log: $LOG_DIR/rio.log)"
  if [ "$HEADLESS" = "1" ] && [ -n "$MODE_SCHEDULE" ]; then
    say "  headless: Driver Station schedule $MODE_SCHEDULE (from configuration)"
    ( cd "$RIO_DIR" && env ${RIO_ENV[@]+"${RIO_ENV[@]}"} TALOS_SIM_DS_SCHEDULE="$MODE_SCHEDULE" ./gradlew "${GRADLE_ARGS[@]}" ) \
      > "$LOG_DIR/rio.log" 2>&1 &
  elif [ "$HEADLESS" = "1" ]; then
    say "  headless: enabling teleop through TALOS_SIM_DS=auto (once configured)"
    ( cd "$RIO_DIR" && env ${RIO_ENV[@]+"${RIO_ENV[@]}"} TALOS_SIM_DS=auto ./gradlew "${GRADLE_ARGS[@]}" ) \
      > "$LOG_DIR/rio.log" 2>&1 &
  else
    ( cd "$RIO_DIR" && ./gradlew "${GRADLE_ARGS[@]}" ) > "$LOG_DIR/rio.log" 2>&1 &
  fi
  track $! "wpilib-sim"
  disown $! 2>/dev/null
  # Gradle has to resolve and compile before the simulation exists. Wait for
  # the task to actually be running rather than guessing with a fixed sleep.
  say "  waiting for the simulation to come up"
  SIM_UP=0
  for _ in $(seq 1 240); do
    if grep -q "Robot program starting" "$LOG_DIR/rio.log" 2>/dev/null; then
      SIM_UP=1
      break
    fi
    if grep -qE "FAILURE:|BUILD FAILED" "$LOG_DIR/rio.log" 2>/dev/null; then
      tail -25 "$LOG_DIR/rio.log" >&2
      die "the WPILib simulation failed to start"
    fi
    sleep 0.5
  done
  if [ "$SIM_UP" != "1" ]; then
    tail -25 "$LOG_DIR/rio.log" >&2
    die "the WPILib simulation did not start within 120 s (log: $LOG_DIR/rio.log)"
  fi
  if [ "$HEADLESS" != "1" ]; then
    say "  enable the robot and pick Teleoperated in the sim GUI"
  fi
fi

# --- 2. the node graph ------------------------------------------------------
say "starting node graph (log: $LOG_DIR/launcher.log)"
LAUNCH_ARGS="--config $CONFIG --sim --session-id $SESSION_ID --output-dir $OUT_DIR"
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
  LAUNCH_ARGS="$LAUNCH_ARGS --start-sim-gateway --sim-gateway-arg --physics"
fi
# Bash 3.2 treats an empty array as unset under nounset; the + expansion
# preserves each forwarded argument without expanding an unset array.
# shellcheck disable=SC2086
./bazel-bin/talOS/launcher/launcher $LAUNCH_ARGS ${GATEWAY_ARGS[@]+"${GATEWAY_ARGS[@]}"} > "$LOG_DIR/launcher.log" 2>&1 &
LAUNCHER_PID=$!
track "$LAUNCHER_PID" "launcher"

GRAPH="$SESSION_DIR/graph.json"
for _ in $(seq 1 60); do
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
# robot session runs regardless (bridge log: $SESSION_DIR/studio_bridge.log).
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
