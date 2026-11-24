#!/usr/bin/env bash
#
# One command to run a robot in simulation: talOS's bench robot
# (bench/main_processor/configuration/) unless --config names another robot
# in this checkout. A robot in its own repository, such as the 2026 robot,
# runs there with `bazel run //:sim` (docs/FIRST_MECHANISM.md).
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
#                            another robot in this checkout
#                            (bazel run //tools:new_robot)
#
set -uo pipefail

# The configs run without --config, relative to the repository root. Studio
# needs no [studio] table: every key it reads has a default
# (talOS/launcher/README.md "Studio bridge").
#   the WPILib simulation, or --no-rio: the bench on the loopback controller
DEFAULT_CONFIG="bench/main_processor/configuration/robot.toml"
#   --sim-gateway: the bench against the physics simulator
DEFAULT_SIM_CONFIG="bench/main_processor/configuration/robot_physics_sim.toml"

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
# HTTP port only: defaults to the resolved config's [studio] http_port (else
# 5800) unless --http-port overrides it; the telemetry topic lives in the
# robot config ([subsystems.telemetry] topic), read by the nodes that own
# each end, so this script never spells it twice.
HTTP_PORT=""
HTTP_PORT_EXPLICIT=0

say() { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
die() { printf '\033[1;31msim.sh:\033[0m %s\n' "$*" >&2; exit 1; }

usage() {
  sed -n '3,${/^[^#]/q;p;}' "$0" | sed 's/^# \{0,1\}//'
  cat <<USAGE

Options:
  --config FILE     the robot config to run, inside the repository: absolute,
                    or relative to the repository root, else to the current
                    directory (refused when it names a different file under
                    each); its generated manifest must be current.
                    Default, the bench robot:
                      $DEFAULT_CONFIG
                    and with --sim-gateway:
                      $DEFAULT_SIM_CONFIG
                    A robot in its own repository runs there instead:
                    bazel run //:sim
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
  --no-rio          use an existing controller (skip WPILib sim and gateway);
                    without --sim-gateway it refuses --mode-schedule,
                    --joystick-schedule and --alliance (that controller's
                    Driver Station decides them)
  --sim-gateway     start the lightweight physics simulator instead of WPILib
                    (implies --no-rio; aliases --with-sim-gateway,
                    --lightweight-sim)
  --gateway-arg ARG forward one argument verbatim to sim_gateway (repeatable;
                    requires --sim-gateway, which always enables --physics)
  --no-open         start Studio but do not open a browser
  --duration-s N    pass --duration-s N to the launcher
  --http-port N     Studio HTTP port (default: config [studio] http_port, else 5800)
  -h, --help        this message

Each run's session logs go to /tmp/talos_logs/<session_id> (override the root
with TALOS_SIM_LOG_DIR; launcher and controller output go in its sim/
subdirectory).
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
    --duration-s)
      [ $# -ge 2 ] || { echo "sim.sh: --duration-s needs an argument" >&2; exit 2; }
      DURATION="$2"
      shift
      ;;
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
    --http-port)
      [ $# -ge 2 ] || { echo "sim.sh: --http-port needs an argument" >&2; exit 2; }
      HTTP_PORT="$2"
      HTTP_PORT_EXPLICIT=1
      shift
      ;;
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
  CONFIG="$DEFAULT_CONFIG"
  if [ "$WANT_GATEWAY" = "1" ]; then
    CONFIG="$DEFAULT_SIM_CONFIG"
  fi
else
  # A relative path names the file under the repository root, else the one
  # under the caller's directory; one that names a different file under each
  # is refused rather than silently taking the root's.
  case "$CONFIG" in
    /*) ;;
    *)
      if [ -f "$CONFIG" ] && [ -f "$CALLER_DIR/$CONFIG" ] &&
         ! [ "$CONFIG" -ef "$CALLER_DIR/$CONFIG" ]; then
        die "--config $CONFIG is ambiguous: it names both $ROOT/$CONFIG and $CALLER_DIR/$CONFIG; pass an absolute path"
      fi
      [ -f "$CONFIG" ] || CONFIG="$CALLER_DIR/$CONFIG"
      ;;
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
# [studio], defaults where unset. A config the parser refuses stops here with
# the parser's reason on one line; its empty output never reaches the reader.
RESOLVE_ERRORS="$(mktemp)"
RESOLVED="$(./bazel-bin/talOS/configuration/resolve "$CONFIG" 2>"$RESOLVE_ERRORS")"
RESOLVE_STATUS=$?
RESOLVE_REASON="$(tr '\n' ' ' < "$RESOLVE_ERRORS" | sed 's/ *$//')"
rm -f "$RESOLVE_ERRORS"
[ "$RESOLVE_STATUS" = 0 ] ||
  die "cannot resolve $CONFIG: ${RESOLVE_REASON:-the resolver exited $RESOLVE_STATUS}"
TARGETS="$(printf '%s\n' "$RESOLVED" | python3 -c '
import json, sys
config = json.load(sys.stdin)
robot = config["effective"].get("robot", {})
studio = config["effective"].get("studio", {})
print("nodes", *config["nodes"])
print("hardware_node", robot.get("hardware_node", "//talOS/bridge:hardware_node"))
print("sim_gateway", robot.get("sim_gateway", ""))
print("studio_bridge", studio.get("bridge", "//studio/bridge:studio_bridge"))
http_port = studio.get("http_port")
print("studio_http_port", 5800 if http_port is None else http_port)
')" || die "cannot resolve $CONFIG"
target() { printf '%s\n' "$TARGETS" | sed -n "s|^$1 ||p"; }
if [ "$HTTP_PORT_EXPLICIT" != "1" ]; then
  HTTP_PORT="$(target studio_http_port)"
  [ -n "$HTTP_PORT" ] || HTTP_PORT=5800
fi
if [ "$WANT_GATEWAY" = "1" ] && [ -z "$(target sim_gateway)" ]; then
  die "$CONFIG sets no [robot] sim_gateway for --sim-gateway to start"
fi

# In WPILib mode, fail fast if another session already holds this namespace's
# session lock so we never spend time building or booting Gradle/WPILib only
# to have the launcher refuse startup afterwards. Before the build, a stale
# pre-build binary may lack --check-session-lock or fail to load; only an
# active-holder message aborts early, while other failures fall through to
# `bazel build` and the strict post-build check below.
check_session_lock() {
  local strict="${1:-strict}"
  local err status
  err="$(./bazel-bin/talOS/launcher/launcher --check-session-lock 2>&1)"
  status=$?
  [ "$status" = 0 ] && return 0
  case "$err" in
    *"another launcher"*|*"session lock held"*)
      printf '%s\n' "$err" >&2
      die "another session holds the session lock; stop it before starting WPILib simulation"
      ;;
  esac
  if [ "$strict" = "strict" ]; then
    printf '%s\n' "$err" >&2
    die "session lock check failed (exit $status); refusing to start WPILib simulation"
  fi
  return 0
}
if [ "$WANT_RIO" = "1" ] && [ -x ./bazel-bin/talOS/launcher/launcher ]; then
  check_session_lock prebuild
fi

# The launcher takes a number (uint64). The second alone collides for two
# starts in one second; the shell's pid (below 2^22 on Linux, 99999 on macOS,
# so seven digits) makes the key unique among concurrent runs.
SESSION_ID="$(date +%s)$(printf '%07d' "$$")"
OUT_DIR="${TALOS_SIM_LOG_DIR:-/tmp/talos_logs}"
SESSION_DIR="${OUT_DIR}/${SESSION_ID}"
LOG_DIR="${SESSION_DIR}/sim"
mkdir -p "$LOG_DIR"

PIDS=""
NAMES=""
SLEEP_PID=""

# True while PID exists and is not an exited zombie waiting for wait().
is_alive() {
  kill -0 "$1" 2>/dev/null || return 1
  local state
  if state="$(ps -o stat= -p "$1" 2>/dev/null)"; then
    case "$state" in
      ""|Z*|*" "Z*) return 1 ;;
    esac
  else
    kill -0 "$1" 2>/dev/null || return 1
  fi
  return 0
}

# Print a live process and every live process it spawned, children first.
# Gradle and the launcher both fork, and killing only the parent after a
# timeout would orphan a simulation that holds the UDP port against the next
# run.
descendants() {
  local child
  for child in $(pgrep -P "$1" 2>/dev/null); do
    descendants "$child"
  done
  is_alive "$1" && echo "$1"
}

CLEANED=0
cleanup() {
  [ "$CLEANED" = "1" ] && return
  CLEANED=1
  trap - EXIT INT TERM
  [ -n "$SLEEP_PID" ] && kill "$SLEEP_PID" 2>/dev/null
  echo ""
  echo "sim.sh: shutting down"
  # Reverse start order, so the node graph stops before the controller.
  local ordered="" p live="" tree="" live_any tree_any orphans_signalled=0 i
  for p in $PIDS; do ordered="$p $ordered"; done
  # Snapshot live top-level PIDs and their descendant trees before signalling:
  # a parent that exits on TERM reparents any stray child, so it could no
  # longer be found if a fallback KILL is needed after the timeout.
  for p in $ordered; do
    if is_alive "$p"; then
      live="$live $p"
      tree="$tree $(descendants "$p")"
    fi
  done
  if [ -n "$live" ]; then
    # Signal only live tracked top-level PIDs so the launcher runs its own
    # orderly shutdown (stopping nodes, flushing logs, writing manifest.json,
    # unlinking shared memory) without its children or guardian killed early.
    for p in $live; do kill -TERM "$p" 2>/dev/null; done
    # Poll up to 5 s (50 x 0.1 s), returning immediately once everything exits.
    tree_any=0
    for (( i = 0; i < 50; i++ )); do
      live_any=0
      for p in $live; do
        if is_alive "$p"; then
          live_any=1
          break
        fi
      done
      tree_any=0
      for p in $tree; do
        if is_alive "$p"; then
          tree_any=1
          break
        fi
      done
      [ "$tree_any" = "0" ] && break
      # If every tracked parent has exited but a reparented descendant stayed
      # up, signal the remaining descendants once before the timeout expires.
      if [ "$live_any" = "0" ] && [ "$orphans_signalled" = "0" ]; then
        orphans_signalled=1
        for p in $tree; do
          is_alive "$p" && kill -TERM "$p" 2>/dev/null
        done
      fi
      sleep 0.1
    done
    if [ "$tree_any" = "1" ]; then
      for p in $tree; do
        is_alive "$p" && kill -KILL "$p" 2>/dev/null
      done
    fi
  fi
  wait 2>/dev/null
  echo "sim.sh: session logs in $SESSION_DIR (launcher/RIO output in $LOG_DIR)"
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

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
# The per-target "up-to-date" lines and progress are not printed. The full
# output goes to the session's build.log; a failure prints its last 60 lines
# (the errors and the compiler output before them) and the log's path.
BUILD_LOG="$LOG_DIR/build.log"
# shellcheck disable=SC2086
if ! bazel build --noshow_progress --ui_event_filters=-info --show_result=0 \
  $BUILD_TARGETS > "$BUILD_LOG" 2>&1; then
  tail -60 "$BUILD_LOG" >&2
  die "build failed (full output: $BUILD_LOG)"
fi
tail -5 "$BUILD_LOG"

# --- 1. the controller processor (WPILib sim) -------------------------------
if [ "$WANT_RIO" = "1" ]; then
  check_session_lock
  # The robot's own RoboRIO program when it has one beside its
  # main_processor/, else talOS's; the bridge sends either its hardware
  # config.
  RIO_DIR="$ROOT/controller_processor/rio"
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
    say "  headless: enabling teleop through TALOS_SIM_DS=auto (once a push is acknowledged)"
    ( cd "$RIO_DIR" && env ${RIO_ENV[@]+"${RIO_ENV[@]}"} TALOS_SIM_DS=auto ./gradlew "${GRADLE_ARGS[@]}" ) \
      > "$LOG_DIR/rio.log" 2>&1 &
  else
    ( cd "$RIO_DIR" && ./gradlew "${GRADLE_ARGS[@]}" ) > "$LOG_DIR/rio.log" 2>&1 &
  fi
  RIO_PID=$!
  track "$RIO_PID" "wpilib-sim"
  disown "$RIO_PID" 2>/dev/null
  # Gradle has to resolve and compile before the simulation exists. Wait for
  # the task to actually be running rather than guessing with a fixed sleep.
  say "  waiting for the simulation to come up"
  SIM_UP=0
  RIO_EXITED=0
  for (( _i = 0; _i < 240; _i++ )); do
    if grep -q "Robot program starting" "$LOG_DIR/rio.log" 2>/dev/null; then
      SIM_UP=1
      break
    fi
    if grep -qE "FAILURE:|BUILD FAILED" "$LOG_DIR/rio.log" 2>/dev/null; then
      tail -25 "$LOG_DIR/rio.log" >&2
      die "the WPILib simulation failed to start"
    fi
    if ! is_alive "$RIO_PID"; then
      if grep -q "Robot program starting" "$LOG_DIR/rio.log" 2>/dev/null; then
        SIM_UP=1
      else
        RIO_EXITED=1
      fi
      break
    fi
    sleep 0.5 &
    SLEEP_PID=$!
    wait "$SLEEP_PID" 2>/dev/null
    SLEEP_PID=""
  done
  if [ "$SIM_UP" != "1" ]; then
    tail -25 "$LOG_DIR/rio.log" >&2
    if [ "$RIO_EXITED" = "1" ]; then
      die "the WPILib simulation exited before starting (log: $LOG_DIR/rio.log)"
    fi
    die "the WPILib simulation did not start within 120 s (log: $LOG_DIR/rio.log)"
  fi
  if [ "$HEADLESS" != "1" ]; then
    say "  enable the robot and pick Teleoperated in the sim GUI"
  fi
fi

# --- 2. the node graph ------------------------------------------------------
say "starting node graph (log: $LOG_DIR/launcher.log)"
LAUNCH_ARGS=(--config "$CONFIG" --sim --session-id "$SESSION_ID" --output-dir "$OUT_DIR")
[ -n "$DURATION" ] && LAUNCH_ARGS+=(--duration-s "$DURATION")
# Studio's bridge is a declared node in the launcher's graph ([studio] in the
# robot config): the launcher starts and supervises it, so sim.sh only asks
# for it and forwards the HTTP port override.
if [ "$WANT_STUDIO" = "1" ]; then
  LAUNCH_ARGS+=(--start-studio-bridge)
  [ "$HTTP_PORT_EXPLICIT" = "1" ] && LAUNCH_ARGS+=(--studio-http-port "$HTTP_PORT")
fi
# The lightweight sim gateway stands in for the controller processor only
# in mode 3 (--sim-gateway). In mode 1 WPILib publishes /hw/state; in mode 2
# an existing controller does. Starting the gateway alongside either would
# create two publishers on /hw/state.
if [ "$WANT_GATEWAY" = "1" ]; then
  LAUNCH_ARGS+=(--start-sim-gateway --sim-gateway-arg --physics)
fi
# Bash 3.2 treats an empty array as unset under nounset; the + expansion
# preserves each forwarded argument without expanding an unset array.
./bazel-bin/talOS/launcher/launcher "${LAUNCH_ARGS[@]}" ${GATEWAY_ARGS[@]+"${GATEWAY_ARGS[@]}"} > "$LOG_DIR/launcher.log" 2>&1 &
LAUNCHER_PID=$!
track "$LAUNCHER_PID" "launcher"

GRAPH="$SESSION_DIR/graph.json"
for (( _i = 0; _i < 60; _i++ )); do
  [ -f "$GRAPH" ] && break
  # A launcher that already exited (a refused graph, or another session in
  # this shared-memory namespace) said why in its log: show it now.
  is_alive "$LAUNCHER_PID" || break
  sleep 0.5 &
  SLEEP_PID=$!
  wait "$SLEEP_PID" 2>/dev/null
  SLEEP_PID=""
done
if [ ! -f "$GRAPH" ]; then
  tail -25 "$LOG_DIR/launcher.log" >&2
  die "the launcher never wrote $GRAPH (its reason above)"
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
  for (( _i = 0; _i < 60; _i++ )); do
    if (echo > /dev/tcp/127.0.0.1/"$HTTP_PORT") 2>/dev/null; then
      STUDIO_UP=1
      break
    fi
    is_alive "$LAUNCHER_PID" || break
    sleep 1 &
    SLEEP_PID=$!
    wait "$SLEEP_PID" 2>/dev/null
    SLEEP_PID=""
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
