# Proposal: robots as TOML plus node folders

Status: a proposal from a design discussion on 2026-11-13. Nothing here is
built or approved, except where a section says the position is decided.
Commands and file names marked "proposed" do not exist yet.

## Goal

Students and mentors design subsystems and coordinators as nodes that are easy
to add to a robot, and building one is fast. Easy means a short path from
nothing to a running, tested node the student owns and can change, not a
mechanism configured without code.

## Recommendation

1. **talOS is a stable framework, released rarely and consumed prebuilt.** It
   ships the runtime, standard messages, libraries (geometry, kinematics,
   controllers, trajectories, vision, the optional mechanism helpers), the
   framework nodes a robot configures in TOML (`teleop`, `telemetry`) and the
   scaffold that starts a new node.
2. **Each season's robot is one repo of its own**, made of TOML files plus one
   folder per custom node.
3. **A node moves to its own repo only on its second use** (another robot or
   another team needs it).

A repo per node from the start is workable but not recommended; see
[Alternative](#alternative-a-repo-per-node).

## Structure

**talOS** (one repo, tagged releases):

- runtime, devices, launcher, simulator, Studio
- standard messages that all nodes speak
- framework nodes configured in TOML: `teleop`, `telemetry`
- libraries robot nodes are built on: `talOS/geometry` (and its kinematics),
  `talOS/vision`, `talOS/trajectory`, controllers, and `talOS/mechanisms`
  (position, velocity and roller-with-sensor helpers a node may call)

talOS ships no mechanism, drivetrain, camera or pose-estimator node. Those are
robot code (decided).

**Robot repo** (one per season, created from a template):

```
robot-2026/
  MODULE.bazel           pins talOS by release tag
  robot.toml             the list of nodes and robot-wide settings
  subsystems/*.toml      per node: which node, parameters, devices, CAN ids
  trajectories/
  dashboard.toml
  nodes/
    drivetrain/          swerve node built on talOS geometry and kinematics
    camera/              built on talOS/vision
    pose_estimator/
    intake/              node code, default TOML, its own test
    shooter/
    superstructure/      a coordinator: same shape as a subsystem
    autonomous/
```

A subsystem reads targets and publishes state. A coordinator reads states and
writes targets to other subsystems. Both are nodes, so there is one pattern to
learn.

## Rules

1. A node folder depends only on talOS and standard messages, never on another
   node's code.
2. Python is the default language. C++ is for nodes that need the performance.
3. The robot pins talOS by release tag. A local checkout of talOS is used only
   while developing talOS itself.
4. Every node has a test that runs it alone against simulated hardware.
5. Season-specific code stays in the robot repo until a second robot needs it.
6. The launcher keeps checking the whole graph before anything starts. This is
   the one place the design deliberately differs from ROS's runtime discovery.

## Three levels of effort

Every mechanism is a node the student writes and owns (decided). The levels
differ in how much of it is generated or borrowed, not in whether there is code.

| What you need | What you write |
|---|---|
| Driver controls, telemetry, target lanes | TOML for the framework nodes (`teleop`, `telemetry`) and the target policy |
| A common mechanism | a node from the scaffold (`--type velocity`, `position` or `estimator`), edited to fit; it may call `talOS/mechanisms` |
| Something unusual | a Python or C++ node written from nothing, with or without the mechanism helpers |
| A fast control loop | a C++ node: a Python node that owns a motor runs at 20 ms or slower (REQ-RT-23) |

## Day-to-day workflows

**Add a subsystem:**

1. `bazel run //tools:new_subsystem -- intake --lang py --type velocity`
   creates the node package, its test, its subsystem file and its
   `robot.toml` entry.
2. Edit the node and run its test (`bazel test //<robot>/main_processor/intake:all`)
   until it behaves.
3. Run `./sim.sh`.

In a robot repo outside talOS the same steps become proposed `talos new`,
`talos test` and `talos sim` commands.

**Change a robot node such as swerve:**

| Change | What to do |
|---|---|
| A gain, limit or geometry value | edit the robot's TOML |
| The node's behavior | edit the node in `nodes/` |
| A library the node uses (kinematics, vision, mechanism helpers) | change talOS against a local override, release it, bump the robot's pin |

## Build speed

Build time depends on the build cache and on what a node depends on. Repo
boundaries do not change it.

| Measure | Effect |
|---|---|
| Shared remote cache filled by CI | a fresh clone downloads the framework instead of compiling it |
| talOS shipped prebuilt | nobody compiles the framework at all |
| Python nodes | no compile step |
| C++ nodes depend only on the node API and messages | a build is a few files and a link |
| Per-node tests | no need to launch the whole robot to iterate |

## C++ node layout

The entry `.cc` stays one line, `TALOS_NODE(pkg::Class)`. It only defines
`main()`. The node class lives in a library so its test can construct it.

| Node size | Layout |
|---|---|
| Small | one header plus the entry `.cc` (today's style) |
| Larger | declarations in `.h`, bodies in `.cc`, split by concern, built as one `cc_library` that both the node and its test depend on |

Today's larger robot nodes are header-only, which compiles each header twice
(node and test) and rebuilds widely on any edit. Moving bodies into `.cc`
files fixes both.

## How ROS does it

- The unit of reuse is the **package**, not the repo. Most teams keep all of a
  robot's packages in one repo.
- Messages live in their own packages, separate from the nodes that use them.
- Generic nodes (`ros2_control`, `nav2`) know nothing about the robot and are
  configured by parameters.
- A **bringup** package holds only launch and parameter files. This is the
  "config files that point to nodes" idea.
- Robots still carry a package of their own code for robot-specific behavior.

This proposal copies all of that except runtime discovery.

## Alternative: a repo per node

Each subsystem is its own repo, pulled into the robot repo as a git submodule.

**What it gives:** separate history, reviews and permissions per subsystem
group.

**What it costs:**

- two commits per change (the node, then the robot's submodule pointer)
- submodule confusion for students (detached HEAD, unpushed commits)
- every node repo must agree on talOS and message versions
- a change spanning two nodes becomes two reviews

It does not make builds faster. It becomes reasonable once talOS is stable and
prebuilt, and a `CODEOWNERS` file per folder gives most of the ownership
benefit inside one repo.

## Hosting on GitHub

One organization holds talOS, each season's robot repo and any promoted node
repos. GitHub has no folders or nested groups inside an organization, so
grouping is done with:

- **teams** for permissions per student group
- **topics** and a naming prefix (`node-`, `robot-`) for filtering
- **template repositories** for the robot and node templates

## Where talOS is today

**Already in place:**

- a node package is self-contained: code, messages, its own test
- teleop bindings live in each subsystem's own TOML
- a node in its own repo with default TOML works (`examples/wrist_repo`)
- Python and shared-library C++ nodes go through one prebuilt-friendly library
  (`libtalos_node.so`)
- scaffolding for a new robot and a new subsystem
- the graph check before launch

**Gaps:**

- a robot can only live inside the talOS checkout
- talOS is only consumable from a sibling checkout, not a pinned release
- `sim.sh` refuses a config outside the repository
- a scaffolded robot's simulator points into `2026-robot`
- no remote build cache
- the node list needs a manual regenerate step after a TOML change
- each subsystem defines its own messages

## Work, in order

| # | Work | Size |
|---|---|---|
| 1 | Remote build cache filled by CI | Medium |
| 2 | Robot repo outside talOS: tagged releases, standalone robot scaffold, sim entry point, generic sim gateway in talOS, Studio and bundle from an external repo, a home for the RoboRIO program | Large |
| 3 | Generate the node list from `robot.toml` during the build | Medium |
| 4 | Standard messages, one message family at a time, starting with roller and position | Large |
| 5 | Migrate `2026-robot` into the template | Medium |
| 6 | Prebuilt talOS release per platform, if the cache alone is not fast enough | Large |

Step 2 is proven when a throwaway robot repo builds, passes its graph test and
runs in simulation.

## Decisions needed before any code

- **Requirements.** The catalogue covers a standalone subsystem repo
  (`REQ-DEV-02`) and a robot beside `2026-robot/` (`REQ-DEV-23`). A standalone
  robot repo, standard messages and a remote cache each need a new requirement.
- **Design notes** for steps 2 and 4: how talOS is distributed, and what the
  standard target and state messages are.
- **Cache hosting.** It is an outside service the team runs or pays for.
- **Generic messages.** Shared message types let dashboards and
  coordinators treat mechanisms alike, so
  `IntakeTarget.roller_velocity_rps` becomes something like a shared
  `VelocityTarget`. Logs and dashboards lose the per-mechanism field names.

## Not yet checked

- how robot-specific the RoboRIO program is
- whether `talos_bundle` and `tools/deploy.sh` work from an external robot repo
- a `.cc`-backed node library built in this repo (expected to work; it is
  plain `cc_library`)
