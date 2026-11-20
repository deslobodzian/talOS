# Pre-match checklist

For the drive team, at the field before each match, with the robot on and
disabled. The Driver Station's green robot code and communications only say
the RoboRIO program runs; whether the main processor is driving it is on the
driver dashboard. This is what every robot on talOS shares; the widgets
beyond these, and how the robot's routine is chosen, are in its own
repository's checklist, beside its `[studio] dashboard` layout.

Open the driver dashboard on the Driver Station laptop:
`http://10.56.87.11:5800/drive` (`http://<main processor>:5800/drive`; what
it draws: [Studio README](../studio/README.md#driver-dashboard)). Full screen
at 1366x768 (or 1280x720) every list fits without scrolling; above a docked
Driver Station the viewport is shorter and lists scroll (the heading shows
how many rows are out of view). The boot
service, or `run.sh`, starts the Studio bridge that serves it
([DEPLOYMENT.md](DEPLOYMENT.md#build-a-bundle)). The full Studio at `/` is
for the pit. A widget dimmed and tagged
STALE is not current, and "no data" means never received: neither passes a
check below.

## Driver Station

- [ ] Robot code and communications green, joysticks in the expected slots.
- [ ] Battery: a freshly charged battery, resting voltage on the Driver
  Station at or above the team's threshold.
- [ ] Message log: no `talos companion link` error since the last check (link
  lost, not heard since robot code start, or another protocol version), and
  no `companion up, but no actuator commands` warning.

## Driver dashboard, top to bottom

- [ ] Status line reads **OK** `Connected: telemetry current` (a session with
  no hardware bridge adds `(RoboRIO link not judged: no hardware bridge)`). Otherwise:

  | Status line | Meaning | Do |
  |---|---|---|
  | **CONNECTING** | the page is opening its feed, or waiting for the first telemetry | wait a few seconds |
  | **LAUNCH REFUSED** `<reason>` | the main processor is powered and on the network, but the launcher refused to start the robot (a bad config, a graph error, `simulation_only = true`, a held session lock, or a full node registry); no robot node is running and the RoboRIO holds every output neutral. Distinct from **BRIDGE LINK**, which means power, network, or the bridge itself | read the refusal reason shown on `/drive` (also saved in `<prefix>/logs/last_refusal.json`); with the robot disabled, fix the config or bundle, install a passing bundle (or roll back with `tools/deploy.sh rollback`), and restart the service ([DEPLOYMENT.md](DEPLOYMENT.md#on-the-main-processor)) |
  | **BRIDGE LINK** `Dashboard to bridge link down` or `nothing heard` | the laptop does not reach the main processor's port 5800: main processor off, still booting, off the robot network, or started with `--no-studio`; or the Studio bridge that serves the page died. The launcher restarts a dead bridge within about a second, a few times ([Studio bridge](../talOS/launcher/README.md#studio-bridge)), and the page reconnects by itself; one that keeps dying, or hangs, stays down for the rest of the session while the robot drives on | wait a few seconds; if it persists, [main processor not reachable](#main-processor-not-reachable) tells which; when only the dashboard is down, [dashboard down for the session](#dashboard-down-for-the-session) |
  | **TELEMETRY** `Telemetry stale` | the bridge is up but the telemetry node publishes nothing new: every value is stale | check the telemetry node in Studio at `/` (the pit); the robot can drive, but the dashboard shows nothing current. With `the page missed time (laptop asleep)`: the laptop slept, wait a second for a new tick |
  | **TELEMETRY** `Telemetry feed could not be decoded: reload the page` | the page could not decode what the bridge sent (a page built for another bridge version, or a corrupt message); it reconnects by itself, every 5 s after 3 failures in a row | reload the page; if it persists, the page and the bridge come from different builds: redeploy the bundle |
  | **ROBORIO LINK** `RoboRIO link lost` | the main processor accepts no hardware state from the RoboRIO: RoboRIO off, its program not running, a wrong address, or another protocol version; every output is neutral | read the Driver Station message log; on a protocol error redeploy both sides from one checkout ([DEPLOYMENT.md](DEPLOYMENT.md#roborio)) |
  | **HARDWARE BRIDGE** `Hardware bridge not publishing` | no hardware status: the hardware bridge (`hardware_node`) on the main processor is not publishing, so the RoboRIO link cannot be judged | check Subsystems below |
  | `/drive` shows `Studio's pages are missing from the installed bundle` (or `are not built`) instead of the dashboard | the bridge runs, but the installed bundle has no dashboard pages | with the robot disabled, install a bundle built from the current checkout and restart the service ([DEPLOYMENT.md](DEPLOYMENT.md#on-the-main-processor)); installing the same bundle again changes nothing |
  | **REPLACED** | another `/drive` page took this page's slot (the layout's `drive_clients` are held) | close the other page, then press Reconnect, which takes the oldest page's slot |
  | **DRAWING ERROR** `The dashboard is not updating; reload the page` | a widget threw while drawing; the top status line stays live while the grid keeps its last drawn values, dimmed and outlined in red | reload the page |

- [ ] Match: `DISABLED`; the alliance and station are the assigned ones; at
  the field, the match number and `FMS`. `NO DRIVER STATION`: the main
  processor gets no Driver Station packets through the RoboRIO. `E-STOP`: the
  robot is e-stopped.
- [ ] Battery: a voltage, with no `LOW` and no `BROWNOUT` tag.
- [ ] Auto routine: the intended routine by name. `routine #N` means an
  index outside the routine channel's labels, the list `telemetry.toml`'s
  `[[subsystem.labels]]` names in the deployed config
  ([automatic channels](../talOS/telemetry/README.md#automatic-channels-and-units)).
- [ ] Alerts: Active reads `none`. Each alert names its owner and its
  condition (`hw link lost`, `hw not configured`, `hw uncommissioned`,
  `hw motor fault`, `hw owner timed out`, `hw brownout`, `hw can bus
  unread`; what each means:
  [bridge_status.fbs](../talOS/bridge/bridge_status.fbs)). `owner timed out`: an actuator node is
  not commanding (down or crash-looping; see Subsystems). `can bus unread`:
  a CAN bus's status read is failing or stopped, so its health is unknown
  (Studio's Hardware panel shows which bus by its slot, `rio` first; the
  RoboRIO console names it). The Raised list
  keeps every alert seen this match with its count: read it after the match.
- [ ] Field: the robot drawn at its starting position, with no `POSE NOT
  VALID` (the layout's `valid` channel false) or `NOT LOCALIZED` (its
  `localized` channel false) tag.
- [ ] Subsystems: every node `UP`, none `FAILING`, and `restarts` not
  rising between checks (hover a name whose row is too narrow to show them).
  `DOWN`: not running; in red, a critical node. `FAILING`: the node failed
  too often and the launcher paused its restarts (quarantined); it is not
  running. `unknown`: the bridge could not read the live registry, so the
  rows say nothing.
- [ ] Pre-match: every item `OK`. The robot's layout's `checks` widget sets
  the items and their bounds; these read the hardware bridge's status, and
  a robot adds its own mechanisms' and estimator's:

  | Item | Reads | FAIL means |
  |---|---|---|
  | Battery | `/hw/status` `battery_v` at the layout's bound or above | swap the battery |
  | Hardware configured | `/hw/status` `configured`: the RoboRIO's current boot acknowledged the main processor's hardware configuration, so it hears the main processor | the RoboRIO has no configuration from it yet and holds every output neutral |
  | Commissioned | `/hw/status` `commissioned`: the configuration is marked commissioned | the RoboRIO holds every output neutral; a person commissions the robot ([BRINGUP.md](BRINGUP.md#human-commissioning)) |

## Main processor not reachable

When the dashboard does not load or stays on **BRIDGE LINK** (as distinct from
**LAUNCH REFUSED**, where the dashboard loads and names why the launcher
refused to start), the main
processor's own view is unavailable, and only the RoboRIO's remains: the
`SmartDashboard/companion/` values it publishes through NetworkTables (what
each means: [RoboRIO](../talOS/hardware/README.md#roborio)), read in any
NetworkTables dashboard (Shuffleboard, Elastic, Glass) on the Driver Station
laptop. They tell whether the RoboRIO hears the main processor at all:

- `companion/connected` true and `companion/frame_age_s` below 0.5: the main
  processor's hardware bridge reaches the RoboRIO, so only the dashboard's
  link is down: [dashboard down for the session](#dashboard-down-for-the-session)
  (or the session runs with `--no-studio`); check power and cables only when
  this is false. False: the main processor is off, still booting, or not on
  the robot network; the RoboRIO drives nothing without it.
- `companion/command_age_s` above 0.5, or -1, with `connected` true: the
  main processor is up but its actuator nodes are not commanding (down or
  crash-looping).
- `companion/protocol_ok` false: the RoboRIO program and the main processor
  bundle come from different builds (`companion/protocol` and
  `companion/peer_protocol` show both versions); redeploy both from one
  checkout ([DEPLOYMENT.md](DEPLOYMENT.md#roborio)).
- `companion/configured` or `companion/commissioned` false: the RoboRIO has
  no hardware configuration (neither its saved copy nor a push from the main
  processor) or an uncommissioned one, and holds every output neutral.

## Dashboard down for the session

The robot drives on without the dashboard, but the dashboard will not come
back by itself in this session when the launcher has left the Studio bridge
after it kept dying (the launcher does not restart it again), or when the
bridge hangs, which the launcher does not notice. Play the match without it;
never restart anything during a match. Between matches, with the robot
disabled, restart the service (`sudo systemctl restart talos` on Linux:
[DEPLOYMENT.md](DEPLOYMENT.md#on-the-main-processor)) and reopen `/drive`.
That restarts every node, so go through this checklist again.

To find out why, in the pit: the session's `talos.log` on the main processor
(`<prefix>/logs/<session-id>/`; `journalctl -u talos` on Linux) has a
`studio_bridge exited` line for each death and, for a left bridge, one
saying it `will not be restarted again`; `studio_bridge.log` beside it says
why the bridge ended, and the session manifest's `studio_bridge` entry and
`session_report` count its restarts. With no such line, the bridge hung.
