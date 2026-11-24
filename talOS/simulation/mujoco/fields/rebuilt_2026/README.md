# 2026 field (REBUILT) for MuJoCo

`:rebuilt_2026_xml` writes `rebuilt_2026.xml`, a MuJoCo model (MJCF) of the
2026 FRC field: perimeter, both HUBs, four BUMPs, four TRENCHes, both
TOWERs, both DEPOTs, both OUTPOSTs, the 32 AprilTag panels and all 504 FUEL
(REQ-DEV-38). `generate_field.py` (stdlib only) builds it at build time from
the dimension table at its top and the official tag poses
(`//talOS/vision/testdata:2026-rebuilt-welded.json`); nothing generated is
checked in. Packages under `//talOS/simulation` take the file as `data`
through `:rebuilt_2026`. `:rebuilt_2026_glb` writes the same field for
viewers ([Viewing the field](#viewing-the-field)).

```sh
bazel build //talOS/simulation/mujoco/fields/rebuilt_2026:rebuilt_2026_xml
bazel test //talOS/simulation/mujoco/fields/rebuilt_2026:all
```

Every field structure is a static geom of the world body; the 504 fuel are
the only moving bodies. Of the model's `<option>`, the CG solver, the 5 ms
step, 20 iterations and sleeping off are settings measured on 2026 field
scenes for step cost (REQ-DEV-37); the elliptic cone, `implicitfast`, the
1e-8 tolerance and every contact value below are chosen, not measured.
`<size memory="64M"/>`.

## Viewing the field

`:rebuilt_2026_glb` writes `rebuilt_2026.glb`, a self-contained glTF binary
of the field (REQ-DEV-42), from the same element list as the MJCF model:
every geom but the fuel and the two carpet planes is one node named as its
geom, with the geom's pose, size and colour. The carpet is left out because
the viewport draws its own floor over the field's area (16.541 x 8.069 m,
top at z = 0), and a coplanar carpet would flicker against it. The 504 FUEL
are not in the model. Colours are the MJCF's rgba, which MuJoCo shows as
sRGB, converted to glTF's linear `baseColorFactor` so a viewer shows the
same colours. Every primitive is closed and faces outward, and materials are
single-sided. The scene's one root node, `field`, turns the field frame into
glTF's Y-up frame, so field (x, y, z) is glTF (x, z, -y), metres; a viewer
that turns the scene +90 degrees about x, as Studio's 3D viewport does,
sees the field in the field frame below. To load it in Studio:

```sh
bazel build //talOS/simulation/mujoco/fields/rebuilt_2026:rebuilt_2026_glb
```

then, in the 3D Viewport panel, choose "Load field GLB" and pick
`bazel-bin/talOS/simulation/mujoco/fields/rebuilt_2026/rebuilt_2026.glb` in
the workspace. `:field_gltf_test` checks the file is a valid,
self-contained GLB of closed, outward-facing primitives and compares every
node with its geom in the compiled model; `//studio:field_glb_test` loads
it through the viewport's own loader and checks where it lands;
`:generate_field_test` checks that two generations in one process give
identical bytes for both files.

## Frame

The WPILib field frame, which is the drawings' frame: origin at the blue
alliance wall on the scoring-table side, +x toward the red wall, +y away from
the scoring table, z up, metres. The field is 651.22 x 317.69 in
(16.541 x 8.069 m); its centre, site `field_centre`, is (8.2705, 4.0347) m.
The red half is the blue half rotated 180 degrees about that centre: the
generator builds each red geom and site from its blue one and holds no red
dimension. Each alliance's elements are named by its own OUTPOST and DEPOT
sides (blue's outpost is on the scoring-table side, red's on the far side),
which the rotation keeps true.

## Collision contract

MuJoCo collides two geoms when `(contype1 & conaffinity2) ||
(contype2 & conaffinity1)`. A robot model added to this field follows the
robot row.

| What | contype | conaffinity |
|---|---|---|
| field structure (walls, hub, bump, trench, tower, depot bars, outpost, chute door) | 1 | 0 |
| fuel | 2 | 15 |
| robot geoms (not in this model) | 4 | 21 |
| `carpet_fuel` (plane, condim 6, rolling friction 0.01 m) | 8 | 0 |
| `carpet_robot` (plane, condim 3, group 3) | 16 | 0 |
| visual only (`tag_*`) | 0 | 0 |

Fuel: sphere of radius 2.955 in (0.07506 m), 0.215 kg, sliding friction 0.7,
`solref="0.02 1"`, `solimp="0.9 0.95 0.001"`. Carpet friction 0.8.
Structure friction: HDPE (bumps, hub exit floors, chute floor and door) 0.6;
polycarbonate, aluminium and steel 0.4.

Structure geoms carry `solmix="1000"`, so their `solref` (below) governs
every contact with the structure, a robot geom's included: against the
field, a robot geom's own `solref` and `solimp` are ignored unless it sets
`priority`. A robot geom against `carpet_robot` mixes equally with the
carpet's default contact parameters. Friction and condim take the larger of
the two geoms' values, as MuJoCo does by default.

## Robustness rule

MuJoCo has no continuous collision detection, and a ball at 10 m/s moves
5 cm in one 5 ms step. So:

- structure contacts are as stiff as the step allows: `solref` is twice the
  timestep (`0.01 1`, computed from the timestep by the generator), the
  shortest time constant MuJoCo keeps stable, and governs a contact with
  fuel through `solmix`. A 10 m/s ball's centre then stops in front of the
  surface it hits, whatever the phase of its first contact; fuel's own,
  softer contact lets it cross a wall's plane;
- every static collision geom has material behind the surface a ball
  touches wherever nothing else is there: guardrails 0.2 m and alliance
  walls 0.25 m outward, the net 0.2 m away from the hub, funnel panels
  0.05 m (normal to the panel) and roof panels 0.07 m (vertical) away from
  the side a ball touches, the hub's lower walls 0.06 m inward, floor-level
  parts 0.1 m into the carpet. Around
  the outpost's openings the wall is 0.05 m thick, because the chute and
  corral are behind it.

`field_test`'s `FastFuelDoesNotTunnel` checks the first point at 10 m/s.

## Names

| Name | What |
|---|---|
| `fuel_000` ... `fuel_503` | fuel bodies, each with free joint `fuel_NNN_joint` and sphere geom `fuel_NNN_geom`: the model's only free bodies, all of one radius, so they are its game pieces, in index order ([the MuJoCo gateway](../../README.md#the-mujoco-gateway)) |
| `carpet_fuel`, `carpet_robot` | the two coincident carpet planes |
| `guardrail_table`, `guardrail_far` | guardrails along y = 0 and y = 8.069 m |
| `blue_wall_*` | alliance wall segments: `stations_outpost`, `tower`, `stations_depot`, `outpost_guardrail`, `outpost_depot`, `corral_sill`, `outpost_middle`, `chute_jamb_guardrail`, `chute_jamb_depot`, `outpost_top` |
| `blue_hub_*` | `wall_alliance`, `wall_outpost`, `wall_depot`, `wall_neutral` (above the exits), `pocket_back`, `pocket_roof`, `pocket_floor`, `pocket_divider_0..2`, `roof_0..5`, `funnel_0..5`, `net` |
| `blue_bump_outpost`, `blue_bump_depot` | bumps, on the hub's outpost and depot sides |
| `blue_trench_outpost_*`, `blue_trench_depot_*` | `block`, `arm`, `post`, `panel` |
| `blue_tower_*` | `base`, `upright_outpost`, `upright_depot`, `arm_outpost`, `arm_depot`, `rung_low`, `rung_mid`, `rung_high` |
| `blue_depot_bar_*` | `field`, `outpost` (the side bar toward the outpost), `guardrail` |
| `blue_outpost_*` | `chute_floor`, `chute_side_guardrail`, `chute_side_depot`, `chute_back`, `corral_side_guardrail`, `corral_side_depot`, `corral_back`, `corral_pipe` |
| `blue_chute_door` | closes the chute opening; a referee opens it by clearing its contype |
| `tag_01` ... `tag_32`, `tag_NN_code` | visual tag panels at the official poses |

Every `blue_` geom and site has a `red_` twin. Sites (group 4, for a
referee):

| Site | Where |
|---|---|
| `blue_hub_score` | cylinder on the throat's axis: radius the throat's inscribed radius (11.66 in), half-height one ball radius, centred one ball radius below the throat plane |
| `blue_hub_exit_0..3` | release points, one in each exit pocket (blue's in order of increasing y): a fuel's centre resting on the pocket's floor, the site's +x axis horizontal and out of the hub |
| `blue_tower` | the centre of the tower's base plate, on the carpet |
| `field_centre` | the field centre, on the carpet |

The field model alone keeps what a hub scores: the lower box is a closed
bin whose floor is the fuel carpet. It holds scored fuel only until it
fills; the model does not define what a full hub does, so a simulator that
scores fuel takes it out of the bin. Returning fuel to the field is a
release at an exit, and a release follows this contract:

- the pocket is empty: no fuel centre is within one ball diameter of the
  `*_hub_exit_N` site;
- the fuel's centre is placed at the site, with 3 to 5 m/s within 20
  degrees of the site's +x axis in the horizontal plane, varied from
  release to release (the manual's hubs distribute fuel at random);
- a fuel left at rest in a pocket rolls out by itself, which keeps a pocket
  from holding fuel; it is not a way to return fuel at a rate.

`ContractReleasesClearTheirExit` holds one exit to this contract.

Fuel index ranges, also `<custom><numeric>` entries
`fuel_<range>_first` and `fuel_<range>_count`:

| Range | Indices | Staged |
|---|---|---|
| `neutral` | 000-359 | 12 along x by 30 along y, centred on the field centre, ball edges at +-35.95 in in x and +-90.95 in in y |
| `blue_depot` | 360-383 | 4 along x by 6 along y in the depot |
| `red_depot` | 384-407 | rotated from blue |
| `blue_chute` | 408-431 | rows of five up the chute floor, behind the door (the last row four) |
| `red_chute` | 432-455 | rotated from blue |
| `reserve` | 456-503 | a robot's preloads: parked on the carpet outside the field at y = -2.0 m, x = 1.0 m + 0.2 m per ball |

Staged fuel starts clear of the carpet by 0.5 mm, with no two balls
touching.

## Elements

Blue side; inches, as the sources give them. GM is the 2026 Game Manual
(TU22); DWG the field dimension drawings FE-2026 rev B; GS the game-specific
field drawings (PDF page). QUOTED is a printed number or a sum of printed
numbers; ESTIMATED is chosen or scaled, with the reason.

| Element | Dimension | in | Tag | Source or reason |
|---|---|---|---|---|
| field | length x width | 651.22 x 317.69 | QUOTED | DWG s3 |
| guardrail | height | 20.0 | QUOTED | GM 5.2 |
| alliance wall | height (36.8 plate + 42 plastic); at the tower and the outpost | 78.8; 78.25 | QUOTED | GM 5.9.1; GM 5.8, GS p164 |
| hub | centre x, y | 182.11, 158.84 | QUOTED | DWG s3 |
| hub | lower box footprint | 47.00 square | QUOTED | GM 5.4, GS p89 |
| hub | lower box top | 49.75 | QUOTED | GS p89 |
| hub | rim height; rim hexagon flat to flat (flats facing +-y) | 72.00; 41.73 | QUOTED | GM 5.4, DWG s4 |
| hub | throat flat to flat | 23.32 | QUOTED | GS p111 |
| hub | funnel panel slant; throat height 72.00 - 15.35 | 17.90; 56.65 | QUOTED | GS p133 |
| hub | roof: from the box edge at 49.75 up to the throat, in six pieces | - | ESTIMATED | GS p89 shows a roof of about 5 degrees under the funnel; a hip roof to the throat keeps every piece convex |
| hub | lower wall thickness | 2.36 | ESTIMATED | 0.06 m, the robustness rule inside a 47 in box |
| hub | interior below the throat: a closed bin, the fuel carpet its floor | - | ESTIMATED | the interior between the scoring site and the exits is not modelled: no source dimensions it |
| hub | exits: four pockets in the base of the neutral face, 8.9 high, 7.0 deep, in four lanes 9.82 wide between three 1.0 dividers at y offsets 0 and +-10.82; each closed against the bin by a back wall and a roof, its floor falling 15 degrees down and outward to the carpet at the face | 8.9; 7.0; 9.82; 1.0; 15 degrees | ESTIMATED | GM 5.4 puts four exits at the base of the neutral-zone face; GS p100 and p132 give lower panels 30.00 and 38.88 tall with no ball openings, which fits an opening of 38.88 - 30.00 = 8.9 below a 30.00 panel (the drawings' "30.13" is a tag offset). A pocket holds one fuel with 1.09 to spare, and a 5.91 ball on the 15 degree floor clears the 8.9 roof; the slope rolls a fuel left in the pocket out by itself |
| hub | net: 58.41 wide, 65.6 to 120.36 high, 10.26 beyond the neutral-zone face | 58.41; 65.6; 120.36; 10.26 | QUOTED (bottom ESTIMATED) | GS p89; on the neutral-zone side: GM 5.4 puts the net at the back of the hub, against fuel launched from areas where shooting is not allowed, and the rim's front edge faces the alliance |
| bump | depth (x) x length (y), centred on the hub's x, 23.5 to 96.5 from the centre line | 44.40 x 73.00 | QUOTED | GM 5.5, DWG s4 chain |
| bump | lip at each long edge; apex | 0.61; 6.513 | QUOTED | GS p27 |
| trench | block: 12.00 (y) x 47.00 (x), 96.5 to 108.5 from the centre line | - | QUOTED | GS p43, DWG s4 |
| trench | block height | 26.25 | ESTIMATED | GS p43 section, to the arm's top |
| trench | arm: bottom 22.25, 6.00 along x, from the block to the guardrail | - | QUOTED | GM 5.6, GS p45. The field's pivoting arm (the trench away from the scoring table) is 4.00 along x (GS p43); every arm here is 6.00 so red stays blue rotated |
| trench | arm thickness | 4.00 | ESTIMATED | GS p43 |
| trench | opening, block to guardrail | 50.34 (outpost side), 50.35 (depot side) | QUOTED | DWG s4, s8 |
| trench | tag panel: 12.0 (y) x 2.8 (x), arm top to 40.25, centred on the opening | - | ESTIMATED (top QUOTED) | tags 17 and 28 are 2.95 apart in x |
| trench | post at the guardrail: 6.0 (x) x 2.0 (y), carpet to the arm, behind the guardrail's plane | - | ESTIMATED | footprint not drawn; behind the rail so the opening stays 50.35 |
| tower | centre y; width of the tower and its wall plate | 147.47; 49.25 | ESTIMATED; QUOTED | the centred tag 31 (GM 5.11), DWG s3 (11.38); GM 5.8 |
| tower | uprights: 1.50 (y) x 3.51 (x) x 72.1, x 40.00 to 43.51, 32.25 apart face to face | - | QUOTED | GM 5.8, GS p164 |
| tower | rungs: 1.66 OD, 47.0 long along y, centre x 41.755, centres 27, 45, 63 high | - | QUOTED | GM 5.8, GS p164 |
| tower | support arms, upright to wall, 35.125 to 43.375 high | - | ESTIMATED (heights QUOTED) | GS p164; one plate each stands in for the drawn bar and gusset |
| tower | base plate 39.0 (y) x 45.18 (x) x 0.25 | 39.0 x 45.18; 0.25 | QUOTED; ESTIMATED | GM 5.8; edges 0.2 to 0.3 |
| depot | centre y | 234.85 | QUOTED | DWG s3, 317.69 - 82.84 |
| depot | U of bars 3.0 wide and 1.125 tall, 42 (y) x 27 (x), open toward the wall | - | QUOTED | GM 5.7, GS p183 |
| outpost | openings centred on y | 26.22 | ESTIMATED | tag 29 |
| outpost | width along the wall | 49.84 | QUOTED | DWG s3 |
| outpost | chute opening 31.8 x 7.0, bottom 28.13 | - | QUOTED | GM 5.9.2, GS p2 |
| outpost | chute floor rising 15 degrees away from the field | 15 degrees | QUOTED | GM 5.9.2 |
| outpost | chute floor length along its slope; side walls above its high end | 36.0; 12.0 | ESTIMATED | holds 24 fuel in one layer |
| outpost | corral opening 32.0 x 7.0, bottom 1.88, split by a 1.66 OD pipe | - | QUOTED | GM 5.9.2, GS p2 |
| outpost | corral 35.8 (y) x 37.6 (x), walls 8.13 high | - | QUOTED | GM 5.9.2 |
| fuel | diameter 5.91; mass 0.203 to 0.227 kg (0.215 used) | - | QUOTED | GM 5.10.1 |
| AprilTag | panel 10.5 square, printed tag 8.125, at the WPILib welded poses | - | QUOTED | GM 5.11 |

Not modelled: hub lights and electronics, the trench's electronics and pivot,
guardrail gates (closed in a match), driver-station shelves and sponsor
panels, the outpost's structure other than its chute and corral, field
tape, human players, and the hub's interior between its scoring site and its
exits (not dimensioned in any source; the field model keeps scored fuel in
the bin, and fuel returns to the field by a release at an exit), its fuel
counter and its exit randomness.

## Rules

`:rules` (`rules.h`, `MakeRebuilt2026Rules`) is the 2026 game on this model:
a `GameRules` ([`game.h`](../../game.h)), the [world
rule](../../world_rules.h) a `MujocoPlant` runs after every MuJoCo step
(`MujocoPlant::AddRules`), REQ-DEV-40. The MuJoCo gateway adds it to a config
whose `field` is `rebuilt_2026` ([the
match](../../README.md#the-mujoco-gateway)). It reads only this model's
contract: the FUEL range numerics above, `*_hub_score`, `*_hub_exit_0..3`,
and a robot model's sites `preload_0` to `preload_7`; `Attach` refuses a
model without one of them, naming it. It sees the plant's `MatchInput`
(Driver Station mode and enable, and the schedule clock's time once it has
started) when the plant is given it (`ObserveMatch`, whether or not a step
follows) and again with each step, the plant's step count, and nothing else:
no clock, and no allocation after `Attach`. A FUEL is in play, held (scored
and parked, waiting for its HUB to return it) or reserve (in the reserve
row, never placed); `InPlay` answers false for a held or reserve FUEL, so
Studio does not draw it (REQ-DEV-43).

- **Scoring.** A FUEL that is not held, whose centre is inside a
  `*_hub_score` cylinder with any downward velocity at the end of a step,
  scores for that HUB's alliance: it counts once in `<alliance>_scored`,
  and earns a point if the HUB counts then (below). In the same step it is
  held: parked with no velocity at slot k, its index, at
  (0.5 + 0.2 (k mod 80), -2.5 - 0.2 floor(k / 80), radius + 0.0005) m,
  beyond the guardrail, clear of the reserve row and of every other slot.
  The rule judges the position and velocity a rule before it in the step
  left (a FUEL the scripted shooter launched that step is judged at its
  exit). FUEL are checked in index order.
- **Returning.** Each HUB keeps its held FUEL in score order. 200 steps
  (1.0 s) after a FUEL was scored, the HUB releases it, as soon as one of
  its four exit pockets is empty (no FUEL centre within a ball diameter of
  `*_hub_exit_N`): at that site, unrotated, at a speed and
  heading drawn as the release contract above says, with no spin, from an
  exit drawn among its empty ones. At most one FUEL leaves an exit in a
  step; with no empty pocket the HUB waits. The delay counts plant steps,
  not schedule time, so a HUB returns FUEL before the schedule clock
  starts, and when the gateway drops time (its physics falls behind the
  schedule clock) the delay lasts longer than 1.0 s of schedule time.
- **Preloads.** At `Attach`, the reserve FUEL in index order go to the
  robot model's `preload_0` to `preload_7` that exist, in that order, at
  rest and in play; the rest of the reserve stays in its row. A robot with
  no such site has no preloads.
- **Periods**, on the schedule clock, judged at each match input the rules
  see (a "step" in this paragraph: one given to the plant, as at every
  gateway tick, and each step's), so a period begins on the tick whose input
  starts it even when that tick runs no plant step. The match starts at the
  first step that is enabled, on the schedule clock and in autonomous (AUTO)
  or teleop (TELEOP without AUTO, AUTO's points 0-0); test mode starts
  nothing. AUTO ends at the first step out of autonomous, or 20 s after it
  started; a disable or e-stop does not end it. TELEOP starts at the first
  enabled teleop step after that and lasts 140 s whatever the mode:
  TRANSITION 10 s, SHIFTS 1 to 4 of 25 s, END GAME 30 s. The match has then
  Ended, for the rest of the run: nothing restarts it. Every period and
  stage starts at its first step at or after its start time and ends before
  its end time: a step exactly 10 s after TELEOP starts is in SHIFT 1,
  AUTO's last step is the one before 20 s, and one exactly 140 s into TELEOP
  is Ended.
- **When a HUB counts.** Both HUBs count in AUTO, TRANSITION and END GAME.
  In SHIFTS 1 to 4 the alliance with more AUTO points is inactive in 1 and
  3 and active in 2 and 4, the other alliance the reverse; on a tie one
  draw at TELEOP start decides (below 0.5: blue inactive first). A HUB also
  counts for the 3 s after it deactivates (when AUTO ends, when its
  inactive shift begins, when TELEOP ends): a FUEL scored 3 s or later
  after that earns nothing. Points scored in AUTO, or in its 3 s before
  TELEOP starts, are AUTO points. A real match's delay between AUTO and
  TELEOP is longer than 3 s; a schedule with a shorter gap ends AUTO's
  assessment at TELEOP start, so a FUEL landing after that is a TELEOP
  point and does not count toward the shift order.
  Before the match and between AUTO's 3 s and TELEOP, no HUB counts.
- **Draws**, from one splitmix64 stream seeded by `[simulation.mujoco]
  seed` (`SplitMix64`, a uniform double the top 53 bits times 2^-53), in
  the order the steps make them: the tie at TELEOP start; per release,
  blue's HUB before red's, the exit (the empty pockets in exit order,
  uniform), the speed (uniform in 3 to 5 m/s), the heading (uniform within
  20 degrees either side of the exit's +x, in the horizontal plane).
  Changing this order changes every seeded run.

`Report` fills `sim.MatchState`: the period (None before the match and
between AUTO and TELEOP), the time left in AUTO or TELEOP (0 otherwise),
the stage (0 TRANSITION, 1 to 4 SHIFT, 5 END GAME; 0 outside TELEOP), the
points, AUTO points and scored count per alliance, whether each HUB counts
now (`*_scoring_active`, its 3 s included), the shift order and whether it
was drawn (false before TELEOP), and the FUEL in play and held.
`shots_count` is the scripted shooter's, not the game's: `Report` writes 0.
`Fms` is the Driver Station stamp: the match time is the same time left,
and from TELEOP start the game data is `B` or `R`, the alliance inactive
in SHIFT 1.

| Rule | Number | Tag | Source or reason |
|---|---|---|---|
| AUTO; TELEOP: TRANSITION, SHIFTS 1 to 4, END GAME | 20 s; 10 s, 4 x 25 s, 30 s | QUOTED | GM 6.4 Table 6-2 |
| HUB states by shift, tie drawn | - | QUOTED | GM 6.4 Table 6-3 |
| a HUB counts on after it deactivates | 3 s | QUOTED | GM 6.5 |
| points per FUEL in a HUB that counts | 1 | QUOTED | GM Table 6-4 |
| preloads at most | 8 | QUOTED | GM G303 |
| a HUB's delay from scoring to release | 1.0 s | ESTIMATED | no source gives a HUB's rate |
| release speed and heading | 3 to 5 m/s, +-20 degrees | ESTIMATED | the release contract above |
| game data | `B` or `R` | ESTIMATED | GM 6.4 has the FMS send the alliance inactive first at TELEOP start; its format is not in the manual |
| parking slots | 0.2 m pitch, 80 a row, from (0.5, -2.5) m | ESTIMATED | out of the field and clear of the reserve row at y = -2.0 m; parked FUEL keep their collisions and rest on `carpet_fuel` |

Not refereed: the chute doors (closed), the corral, human players, TOWER
climbs and their points, fouls, pins, opponents, ranking points and HUB
lights. A FUEL that leaves the field stays where it lands.
