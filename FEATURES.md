# Future features

Features that have been thought through but are not being built. Nothing
here is scheduled, and none has a requirement yet: before one is built it
gets a `REQ-<ID>` in [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md) (RULES.md
15) and leaves this file in the same change. Open work on what already
exists (defects, unmeasured behaviour, follow-ups) is in [TODO.md](TODO.md),
not here.

Entry form: `## ID — the feature, in one line`, then what it gives a team,
what exists today, the approach and what it was chosen over, the first step,
and what is undecided.

## VISION-SIM-1 — AprilTag cameras that see the MuJoCo world's occlusion and tilt

**What it gives.** Under the MuJoCo gateway
([talOS/simulation/mujoco](talOS/simulation/mujoco/README.md)), a simulated
camera stops seeing a tag that a HUB, a TOWER, another robot or a pile of
FUEL hides, and its view tilts when the robot pitches over a BUMP. Pose
estimation is then tested against the dropouts and the tilted views a real
match produces.

**What exists today.** A simulated camera
(`talOS/vision/field_sim/sim_camera.h`, `tag_rasterizer.h`) draws the
field's tags from the robot's true pose on `/sim/state` and runs the real
detector on the image. The MuJoCo gateway publishes `/sim/state` from the
MuJoCo chassis, so a camera node follows the MuJoCo robot without any
change. Two limits, both stated in `sim_camera.h`:

- nothing occludes a tag: the renderer knows the tag layout, not the field;
- the pose is flat: `/sim/state` is x, y and yaw, and the camera is placed
  with `T_field_robot = Trans(x, y, 0) * Rz(yaw)`.

No camera node has been run against the MuJoCo gateway yet.

**Approach.** Keep the tag renderer and give it more truth from MuJoCo:

- the gateway publishes each configured camera's full pose in the field
  frame (position and orientation of the mount on the MuJoCo chassis), on a
  topic the simulated camera reads when it is present;
- for each tag, the gateway casts rays from the camera to the tag's corners
  and centre with MuJoCo's ray query (`mj_ray`) and publishes which tags are
  blocked; the camera leaves a blocked tag out of its image.

Ray casts run on the CPU and give the same answer on every machine, so a
session stays reproducible.

**Chosen over: rendering the images in MuJoCo.** MuJoCo's offscreen
renderer, with tags as textured surfaces, would give occlusion and tilt
directly. It needs an OpenGL context inside the gateway on Linux, macOS and
CI, and the build has no renderer; rendered pixels differ between GPUs, so
vision would no longer be reproducible; and four camera images per frame
are heavy to draw and to carry from the gateway to the camera nodes (the
simulated cameras' CPU cost on CI is already open, TODO.md CAM-1).

**First step.** Before any of the above: a test that runs a camera node
against the MuJoCo gateway as it is, and checks its pose estimate against
`/sim/state`. That proves the path that should already work.

**Undecided.**

- Whether partial occlusion (some corners blocked) drops the tag or draws
  it clipped. The renderer today reports a tag only when all four corners
  are in the image.
- Whether the camera pose and the blocked-tag list are one topic per camera
  or one for all, and their rate against the camera's frame rate.
- Whether the analytic simulator (`sim_gateway --physics`) should publish
  the same topics with a flat pose and nothing blocked, so camera nodes read
  one interface under both.
