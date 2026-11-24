"""Writes the MuJoCo model (MJCF) of the 2026 FRC field, REBUILT, and the
same field as a glTF binary for viewers.

    generate_field.py --tags <2026-rebuilt-welded.json>
                      [--out <rebuilt_2026.xml>] [--glb <rebuilt_2026.glb>]

Stdlib only; the build runs it (BUILD, `:rebuilt_2026_xml`,
`:rebuilt_2026_glb`) and nothing it writes is checked in. Both files come
from one element list (`field`). README.md gives the frame, the collision
contract, the names and the source of every dimension. REQ-DEV-38,
REQ-DEV-42.

Frame: the WPILib field frame. Origin at the blue alliance wall on the
scoring-table side, +x toward the red wall, +y away from the scoring table,
z up, metres. Every red element is its blue element rotated 180 degrees
about the field centre (`_red`); no red dimension exists.
"""

import argparse
import json
import math
import struct

# Every dimension, in inches (the unit of the sources). Sources: GM = 2026
# Game Manual TU22, section; DWG = 2026 Field Dimension Drawings FE-2026
# rev B, sheet; GS = 2026 game-specific field drawings, PDF page. QUOTED is
# a printed number (or a sum of printed numbers); ESTIMATED is chosen here,
# with the reason in README.md.
_DIMENSIONS_IN = {
    # Field
    "field_length": 651.22,  # DWG s3, wall to wall; QUOTED
    "field_width": 317.69,  # DWG s3, guardrail to guardrail; QUOTED
    "guardrail_height": 20.0,  # GM 5.2; QUOTED
    "alliance_wall_height": 78.8,  # GM 5.9.1, 36.8 plate + 42 plastic; QUOTED
    "tower_outpost_wall_height": 78.25,  # GM 5.8, GS p164; QUOTED
    # HUB (blue; GS p89 = GE-26300 sheet 4)
    "hub_x": 182.11,  # DWG s3, centre from the blue wall; QUOTED
    "hub_y": 158.84,  # DWG s3, centre from the table guardrail; QUOTED
    "hub_side": 47.00,  # GM 5.4, GS p89, lower box footprint; QUOTED
    "hub_box_top": 49.75,  # GS p89, top of the vertical walls; QUOTED
    "hub_rim_z": 72.00,  # GM 5.4, GS p89, funnel rim; QUOTED
    "hub_rim_flats": 41.73,  # DWG s4, rim hexagon flat to flat; QUOTED
    "hub_throat_flats": 23.32,  # GS p111 GE-26315-01, throat flats; QUOTED
    "hub_funnel_slant": 17.90,  # GS p133 GE-26329, panel slant; QUOTED
    "hub_wall": 2.36,  # lower box wall, thickened inward (0.06 m); ESTIMATED
    "hub_pocket_height": 8.9,  # GS p100, p132: 38.88 - 30.00; ESTIMATED
    "hub_pocket_depth": 7.0,  # one fuel and 1.09 clearance; ESTIMATED
    "hub_divider": 1.0,  # between exit pockets; ESTIMATED
    "hub_net_offset": 10.26,  # GS p89, net plane beyond the box face; QUOTED
    "hub_net_width": 58.41,  # GS p89, DWG s4, net frame width; QUOTED
    "hub_net_bottom": 65.6,  # GS p89, loop pole, scaled; ESTIMATED
    "hub_net_top": 120.36,  # GS p89, DWG s4, top pole; QUOTED
    # BUMP (blue; GS p26-27 GE-26100)
    "bump_depth": 44.40,  # GM 5.5, GS p27, along x; QUOTED
    "bump_length": 73.00,  # GM 5.5, DWG s4, along y; QUOTED
    "bump_lip": 0.61,  # GS p27, vertical lip at each long edge; QUOTED
    "bump_apex": 6.513,  # GM 5.5, GS p27; QUOTED
    # TRENCH (blue; GS p43, p45 GE-26201/26202, DWG s4, s8)
    "trench_block_width": 12.00,  # GS p43, block along y; QUOTED
    "trench_block_depth": 47.00,  # GM 5.6, GS p43, along x; QUOTED
    "trench_block_height": 26.25,  # GS p43 section, to the arm top; ESTIMATED
    "trench_arm_bottom": 22.25,  # GM 5.6, DWG s8, clearance; QUOTED
    "trench_arm_thickness": 4.00,  # GS p43, arm height; ESTIMATED
    "trench_arm_depth": 6.00,  # GS p45, fixed arm along x; QUOTED
    "trench_panel_top": 40.25,  # GM 5.6, trench height; QUOTED
    "trench_panel_width": 12.0,  # tag panel along y; ESTIMATED
    "trench_panel_thickness": 2.8,  # along x, tags 2.95 apart; ESTIMATED
    "trench_post_depth": 6.0,  # rail post along x, as the arm; ESTIMATED
    "trench_post_width": 2.0,  # rail post along y, behind the rail; ESTIMATED
    # TOWER (blue; GM 5.8, GS p163-164 GE-26500)
    "tower_y": 147.47,  # centred tag 31 (GM 5.11), DWG s3 (11.38); ESTIMATED
    "tower_upright_near": 40.00,  # GS p164, from the wall face; QUOTED
    "tower_upright_far": 43.51,  # GS p164, from the wall face; QUOTED
    "tower_upright_thickness": 1.50,  # GM 5.8, along y; QUOTED
    "tower_upright_gap": 32.25,  # GM 5.8, GS p164, face to face; QUOTED
    "tower_upright_height": 72.1,  # GM 5.8; QUOTED
    "tower_rung_od": 1.66,  # GM 5.8, 1-1/4 in Sch 40 pipe; QUOTED
    "tower_rung_length": 47.0,  # GS p164; QUOTED
    "tower_rung_low": 27.0,  # GM 5.8, centre height; QUOTED
    "tower_rung_mid": 45.0,  # GM 5.8; QUOTED
    "tower_rung_high": 63.0,  # GM 5.8; QUOTED
    "tower_arm_bottom": 35.125,  # GS p164, support arm; QUOTED
    "tower_arm_top": 43.375,  # GS p164, support arm; QUOTED
    "tower_width": 49.25,  # GM 5.8, tower and its wall plate along y; QUOTED
    "tower_base_width": 39.0,  # GM 5.8, base plate along y; QUOTED
    "tower_base_depth": 45.18,  # GM 5.8, base plate along x; QUOTED
    "tower_base_thickness": 0.25,  # GM 5.8, edges 0.2 to 0.3; ESTIMATED
    # DEPOT (blue; GM 5.7, GS p183-184 GE-26600, DWG s3)
    "depot_y": 234.85,  # DWG s3, 317.69 - 82.84; QUOTED
    "depot_width": 42.00,  # GM 5.7, along y; QUOTED
    "depot_depth": 27.00,  # GM 5.7, along x; QUOTED
    "depot_bar_width": 3.00,  # GM 5.7; QUOTED
    "depot_bar_height": 1.125,  # GM 5.7, with hook fastener; QUOTED
    # OUTPOST (blue; GM 5.9.2, GS p2 GE-26000)
    "outpost_y": 26.22,  # tag 29, centre of both openings; ESTIMATED
    "outpost_width": 49.84,  # DWG s3, guardrail to outpost edge; QUOTED
    "chute_opening_width": 31.8,  # GM 5.9.2; QUOTED
    "chute_opening_height": 7.0,  # GM 5.9.2; QUOTED
    "chute_opening_bottom": 28.13,  # GS p2; QUOTED
    "chute_length": 36.0,  # floor along its slope; ESTIMATED
    "chute_side_height": 12.0,  # side walls above the floor's top; ESTIMATED
    "corral_opening_width": 32.0,  # GM 5.9.2, GS p2; QUOTED
    "corral_opening_height": 7.0,  # GM 5.9.2; QUOTED
    "corral_opening_bottom": 1.88,  # GM 5.9.2, GS p2; QUOTED
    "corral_pipe_od": 1.66,  # GM 5.9.2; QUOTED
    "corral_width": 35.8,  # GM 5.9.2, along y; QUOTED
    "corral_depth": 37.6,  # GM 5.9.2, along x; QUOTED
    "corral_wall_height": 8.13,  # GM 5.9.2; QUOTED
    # FUEL (GM 5.10, 6.3.4, DWG s3)
    "fuel_radius": 2.955,  # GM 5.10.1, 5.91 diameter; QUOTED
    "neutral_half_x": 35.95,  # DWG s3, block edge from the centre; QUOTED
    "neutral_half_y": 90.95,  # DWG s3, block edge from the centre; QUOTED
    # AprilTags (GM 5.11)
    "tag_panel": 10.5,  # polycarbonate panel; QUOTED
    "tag_code": 8.125,  # printed tag; QUOTED
}

# The only inch-to-metre conversion.
METRES_PER_INCH = 0.0254
D = {name: inches * METRES_PER_INCH for name, inches in _DIMENSIONS_IN.items()}

# Angles, in degrees.
HUB_POCKET_FLOOR_DEG = 15.0  # exit floor, down and outward; ESTIMATED
CHUTE_SLOPE_DEG = 15.0  # GM 5.9.2; QUOTED

# Solver settings measured on 2026 field scenes when MuJoCo was added
# (REQ-DEV-37, recorded in that commit): CG, a 5 ms step, 20 iterations,
# sleeping off (the MuJoCo gateway's option table overrides it with the
# config's [simulation.mujoco] sleeping). Chosen, not measured: the elliptic
# cone, implicitfast, the tolerance and every contact value below.
TIMESTEP_S = 0.005
FUEL_MASS_KG = 0.215  # GM 5.10.1, the middle of 0.203 to 0.227 kg
FUEL_FRICTION = 0.7  # foam, sliding
FUEL_SOLREF = "0.02 1"  # soft, critically damped
FUEL_SOLIMP = "0.9 0.95 0.001"
CARPET_FRICTION = 0.8
CARPET_ROLLING_M = 0.01  # rolling friction for fuel
FRICTION_HDPE = 0.6
FRICTION_HARD = 0.4  # polycarbonate, aluminium, steel
# Robustness rule (README.md). Structure contacts are as stiff as the step
# allows (a time constant of two steps) and govern every contact with the
# structure (solmix), so a 10 m/s ball stops within its radius; behind every
# surface a ball can hit there is this much material.
STRUCTURE_SOLREF = f"{2 * TIMESTEP_S!r} 1"
STRUCTURE_SOLMIX = 1000
BACKING_M = 0.25  # alliance walls, beyond the 0.2 m minimum
GUARDRAIL_BACKING_M = 0.2
NET_BACKING_M = 0.2
FUNNEL_PANEL_M = 0.05  # normal to the panel
ROOF_PANEL_M = 0.07  # vertical; >= 0.05 m normal to the steepest roof
BELOW_CARPET_M = 0.1  # floor-level structure extends under the carpet
OUTPOST_WALL_M = 0.05  # the chute and corral are behind it
OUTPOST_BACKING_M = 0.1  # chute and corral walls, outward
STAGING_GAP_M = 0.0005  # between staged fuel and anything else
# In the chute every row leans on the one below: smaller gaps there keep the
# rows' settling (gaps plus contact compression) well under 5 mm.
CHUTE_GAP_M = 0.0001
RESERVE_Y_M = -2.0  # reserve fuel, outside the field
RESERVE_X0_M = 1.0
RESERVE_PITCH_M = 0.2
CHUTE_FLOOR_M = 0.05  # thickness, normal to the floor
CHUTE_FLOOR_OVERLAP_M = 0.02  # floor into each side wall
CHUTE_WALL_BELOW_M = 0.05  # chute walls below the opening's bottom edge
CARPET_SIZE_M = (11.0, 7.0, 0.1)  # rendering only: planes collide everywhere
LIGHT_HEIGHT_M = 12.0

FIELD_L = D["field_length"]
FIELD_W = D["field_width"]
CENTRE = (FIELD_L / 2, FIELD_W / 2)
R = D["fuel_radius"]

COLOURS = {
    "blue": "0.15 0.3 0.85 1",
    "red": "0.85 0.15 0.12 1",
    "blue_clear": "0.15 0.3 0.85 0.35",
    "red_clear": "0.85 0.15 0.12 0.35",
    "rail": "0.75 0.78 0.8 0.35",
    "fuel": "0.98 0.85 0.1 1",
    "carpet": "0.42 0.42 0.42 1",
}

# Fuel index ranges, in body order. Each is also a <custom><numeric> pair.
FUEL_RANGES = [
    ("neutral", 360),
    ("blue_depot", 24),
    ("red_depot", 24),
    ("blue_chute", 24),
    ("red_chute", 24),
    ("reserve", 48),
]


# --- quaternions (w, x, y, z) ------------------------------------------------


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def qaxis(axis, degrees):
    h = math.radians(degrees) / 2
    n = math.sqrt(sum(c * c for c in axis))
    return (math.cos(h),) + tuple(math.sin(h) * c / n for c in axis)


QZ180 = (0.0, 0.0, 0.0, 1.0)
IDENTITY = (1.0, 0.0, 0.0, 0.0)


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(a, s):
    return tuple(x * s for x in a)


def unit(a):
    n = math.sqrt(sum(c * c for c in a))
    return scale(a, 1 / n)


# --- model items ---------------------------------------------------------------


class Item:
    """A static geom or a site, at a pose in the field frame."""

    def __init__(self, tag, name, kind, pos, quat=IDENTITY, size=(),
                 mesh=None, cls="structure", rgba=None, friction=None,
                 extra=""):
        self.extra = extra  # further MJCF attributes, written as given
        self.tag = tag  # "geom" or "site"
        self.name = name
        self.kind = kind
        self.pos = tuple(pos)
        self.quat = tuple(quat)
        self.size = tuple(size)
        self.mesh = mesh
        self.cls = cls
        self.rgba = rgba
        self.friction = friction


def box(name, x0, x1, y0, y1, z0, z1, **kw):
    """An axis-aligned box from its bounds."""
    return Item("geom", name, "box",
                ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
                size=((x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2), **kw)


def _red(item):
    """The red counterpart of a blue item: 180 degrees about the centre."""
    x, y, z = item.pos
    name = "red_" + item.name[len("blue_"):]
    rgba = item.rgba
    if rgba == COLOURS["blue"]:
        rgba = COLOURS["red"]
    elif rgba == COLOURS["blue_clear"]:
        rgba = COLOURS["red_clear"]
    return Item(item.tag, name, item.kind,
                (2 * CENTRE[0] - x, 2 * CENTRE[1] - y, z),
                qmul(QZ180, item.quat), item.size, item.mesh, item.cls, rgba,
                item.friction)


def _red_point(p):
    return (2 * CENTRE[0] - p[0], 2 * CENTRE[1] - p[1], p[2])


# --- the blue half -----------------------------------------------------------


def hub(meshes):
    """The blue HUB: bin, exit pockets, roof, funnel, net and sites."""
    hx, hy = D["hub_x"], D["hub_y"]
    half = D["hub_side"] / 2
    w = D["hub_wall"]
    top = D["hub_box_top"]
    blue, clear = COLOURS["blue"], COLOURS["blue_clear"]
    items = []
    hard = {"friction": FRICTION_HARD}

    # Lower box: a closed bin with the fuel carpet as its floor, so fuel
    # that scores stays inside. What happens between the scoring site and an
    # exit is game logic (README.md), not geometry.
    items.append(box("blue_hub_wall_alliance", hx - half, hx - half + w,
                     hy - half, hy + half, 0, top, rgba=clear, **hard))
    items.append(box("blue_hub_wall_outpost", hx - half, hx + half,
                     hy - half, hy - half + w, 0, top, rgba=blue, **hard))
    items.append(box("blue_hub_wall_depot", hx - half, hx + half,
                     hy + half - w, hy + half, 0, top, rgba=blue, **hard))
    pocket_top = D["hub_pocket_height"]
    items.append(box("blue_hub_wall_neutral", hx + half - w, hx + half,
                     hy - half, hy + half, pocket_top, top, rgba=clear,
                     **hard))

    # Exits: four pockets in the base of the neutral face, each holding one
    # fuel, closed against the bin by a back wall and a roof, split by three
    # dividers into four equal lanes. Each pocket's floor falls
    # HUB_POCKET_FLOOR_DEG down and outward to the carpet at the face, so a
    # fuel left in it rolls out by itself.
    depth = D["hub_pocket_depth"]
    back = half - depth
    inner = half - w
    items.append(box("blue_hub_pocket_back", hx + back - w, hx + back,
                     hy - half, hy + half, 0, pocket_top + w, rgba=clear,
                     **hard))
    items.append(box("blue_hub_pocket_roof", hx + back - w, hx + half - w,
                     hy - half, hy + half, pocket_top, pocket_top + w,
                     rgba=clear, **hard))
    fall = math.tan(math.radians(HUB_POCKET_FLOOR_DEG))

    def floor_z(u):
        return (half - u) * fall

    meshes["hub_pocket_floor"] = [
        (u, v, z) for u in (back, half) for v in (-half, half)
        for z in (floor_z(u), -BELOW_CARPET_M)]
    items.append(Item("geom", "blue_hub_pocket_floor", "mesh", (hx, hy, 0),
                      mesh="hub_pocket_floor", rgba=blue,
                      friction=FRICTION_HDPE))
    div = D["hub_divider"]
    lane = (2 * inner - 3 * div) / 4
    pitch = lane + div
    for i, v in enumerate((-pitch, 0.0, pitch)):
        items.append(box(f"blue_hub_pocket_divider_{i}", hx + back, hx + half,
                         hy + v - div / 2, hy + v + div / 2, 0, pocket_top,
                         rgba=clear, **hard))
    # Release sites: where a released fuel's centre is placed, resting on
    # the pocket's floor at its middle; +x points out of the hub.
    u_release = half - depth / 2
    z_release = (floor_z(u_release) +
                 R / math.cos(math.radians(HUB_POCKET_FLOOR_DEG)) +
                 STAGING_GAP_M)
    for i in range(4):
        v = -inner + lane / 2 + i * pitch
        items.append(Item("site", f"blue_hub_exit_{i}", "sphere",
                          (hx + u_release, hy + v, z_release), size=(R,),
                          cls="marker"))

    # Funnel and roof. Hexagons have vertices along +-x and flats along +-y.
    a_rim = D["hub_rim_flats"] / 2
    a_throat = D["hub_throat_flats"] / 2
    r_rim = a_rim / math.cos(math.radians(30))
    r_throat = a_throat / math.cos(math.radians(30))
    rise = math.sqrt(D["hub_funnel_slant"] ** 2 - (a_rim - a_throat) ** 2)
    z_rim = D["hub_rim_z"]
    z_throat = z_rim - rise

    def hexagon(radius, k, z):
        a = math.radians(60 * k)
        return (radius * math.cos(a), radius * math.sin(a), z)

    def on_square(k):
        """Where the ray through hexagon vertex k meets the box's edge."""
        a = math.radians(60 * k)
        c, s = math.cos(a), math.sin(a)
        t = half / max(abs(c), abs(s))
        return (t * c, t * s)

    for k in range(6):
        # Funnel panel k: the trapezoid between throat edge k and rim edge k,
        # extruded FUNNEL_PANEL_M away from the side a ball touches.
        t0, t1 = hexagon(r_throat, k, z_throat), hexagon(r_throat, k + 1,
                                                          z_throat)
        r0, r1 = hexagon(r_rim, k, z_rim), hexagon(r_rim, k + 1, z_rim)
        n = unit(cross(sub(t1, t0), sub(r0, t0)))
        mid = scale(add(t0, t1), 0.5)
        if n[0] * mid[0] + n[1] * mid[1] < 0:
            n = scale(n, -1)
        face = [t0, t1, r1, r0]
        meshes[f"hub_funnel_{k}"] = face + [
            add(p, scale(n, FUNNEL_PANEL_M)) for p in face]
        items.append(Item("geom", f"blue_hub_funnel_{k}", "mesh",
                          (hx, hy, 0), mesh=f"hub_funnel_{k}", rgba=clear,
                          **hard))
        # Roof piece k: from throat edge k down to the box's edge between the
        # rays through its two vertices (with the corner between them), and
        # ROOF_PANEL_M thick below.
        a0, a1 = 60 * k, 60 * (k + 1)
        rim_path = [on_square(k)]
        for corner in (45, 135, 225, 315):
            if a0 < corner < a1:
                rim_path.append(
                    (math.copysign(half, math.cos(math.radians(corner))),
                     math.copysign(half, math.sin(math.radians(corner)))))
        rim_path.append(on_square(k + 1))
        surface = [t0, t1] + [(u, v, top) for u, v in rim_path]
        meshes[f"hub_roof_{k}"] = surface + [
            (u, v, z - ROOF_PANEL_M) for u, v, z in surface]
        items.append(Item("geom", f"blue_hub_roof_{k}", "mesh", (hx, hy, 0),
                          mesh=f"hub_roof_{k}", rgba=blue, **hard))

    # Scoring site: the throat's inscribed cylinder, one ball radius deep,
    # just below the throat plane.
    items.append(Item("site", "blue_hub_score", "cylinder",
                      (hx, hy, z_throat - R), size=(a_throat, R),
                      cls="marker"))

    # Net: on the neutral-zone side, the hub's back (GM 5.4); the face toward
    # the hub is the drawn plane, thick away from the hub.
    net_x = hx + half + D["hub_net_offset"]
    items.append(box("blue_hub_net", net_x, net_x + NET_BACKING_M,
                     hy - D["hub_net_width"] / 2, hy + D["hub_net_width"] / 2,
                     D["hub_net_bottom"], D["hub_net_top"],
                     rgba=COLOURS["rail"], **hard))
    return items


def bumps(meshes):
    """The two blue BUMPs: pentagonal prisms, lip and apex as drawn.

    Sides are named by the alliance's own OUTPOST and DEPOT, which stay true
    under the rotation to red (blue's outpost is on the scoring-table side).
    """
    hx, hy = D["hub_x"], D["hub_y"]
    dx, dy = D["bump_depth"] / 2, D["bump_length"] / 2
    section = [(-dx, -BELOW_CARPET_M), (-dx, D["bump_lip"]),
               (0.0, D["bump_apex"]), (dx, D["bump_lip"]),
               (dx, -BELOW_CARPET_M)]
    meshes["bump"] = [(u, v, z) for v in (-dy, dy) for u, z in section]
    inner = D["hub_side"] / 2 + dy
    return [
        Item("geom", f"blue_bump_{side}", "mesh", (hx, hy + sign * inner, 0),
             mesh="bump", rgba=COLOURS["blue"], friction=FRICTION_HDPE)
        for side, sign in (("outpost", -1), ("depot", 1))
    ]


def trenches():
    """The two blue TRENCHes: block, arm to the guardrail, post, tag panel."""
    hx, hy = D["hub_x"], D["hub_y"]
    blue = COLOURS["blue"]
    hard = {"friction": FRICTION_HARD}
    items = []
    block_near = D["hub_side"] / 2 + D["bump_length"]  # from the centre line
    block_far = block_near + D["trench_block_width"]
    bx = D["trench_block_depth"] / 2
    ax = D["trench_arm_depth"] / 2
    px = D["trench_post_depth"] / 2
    tx = D["trench_panel_thickness"] / 2
    arm_z0 = D["trench_arm_bottom"]
    arm_z1 = arm_z0 + D["trench_arm_thickness"]
    post_w = D["trench_post_width"]
    for side in ("outpost", "depot"):
        if side == "outpost":
            block = (hy - block_far, hy - block_near)
            rail = 0.0
            arm = (rail - post_w, block[0])
            post = (rail - post_w, rail)
        else:
            block = (hy + block_near, hy + block_far)
            rail = FIELD_W
            arm = (block[1], rail + post_w)
            post = (rail, rail + post_w)
        opening_mid = (rail + (block[0] if side == "outpost" else block[1])) / 2
        pw = D["trench_panel_width"] / 2
        p = f"blue_trench_{side}_"
        items += [
            box(p + "block", hx - bx, hx + bx, block[0], block[1], 0,
                D["trench_block_height"], rgba=blue, **hard),
            box(p + "arm", hx - ax, hx + ax, arm[0], arm[1], arm_z0, arm_z1,
                rgba=blue, **hard),
            box(p + "post", hx - px, hx + px, post[0], post[1], 0, arm_z0,
                rgba=blue, **hard),
            box(p + "panel", hx - tx, hx + tx, opening_mid - pw,
                opening_mid + pw, arm_z1, D["trench_panel_top"], rgba=blue,
                **hard),
        ]
    return items


def tower():
    """The blue TOWER: base plate, uprights, rungs, support arms, site."""
    ty = D["tower_y"]
    blue = COLOURS["blue"]
    hard = {"friction": FRICTION_HARD}
    x0, x1 = D["tower_upright_near"], D["tower_upright_far"]
    t = D["tower_upright_thickness"]
    centre_offset = (D["tower_upright_gap"] + t) / 2
    # The support arms are plates standing in for the drawn bar and gusset.
    items = [
        box("blue_tower_base", 0, D["tower_base_depth"],
            ty - D["tower_base_width"] / 2, ty + D["tower_base_width"] / 2,
            -BELOW_CARPET_M, D["tower_base_thickness"], rgba=blue, **hard),
    ]
    for side, sign in (("outpost", -1), ("depot", 1)):
        y = ty + sign * centre_offset
        items.append(box(f"blue_tower_upright_{side}", x0, x1, y - t / 2,
                         y + t / 2, 0, D["tower_upright_height"], rgba=blue,
                         **hard))
        items.append(box(f"blue_tower_arm_{side}", 0, x0, y - t / 2,
                         y + t / 2, D["tower_arm_bottom"], D["tower_arm_top"],
                         rgba=blue, **hard))
    rung_quat = qaxis((1, 0, 0), 90)  # cylinder axis along y
    for level in ("low", "mid", "high"):
        items.append(Item("geom", f"blue_tower_rung_{level}", "cylinder",
                          ((x0 + x1) / 2, ty, D[f"tower_rung_{level}"]),
                          rung_quat,
                          (D["tower_rung_od"] / 2,
                           D["tower_rung_length"] / 2),
                          rgba=COLOURS["rail"], friction=FRICTION_HARD))
    items.append(Item("site", "blue_tower", "sphere",
                      (D["tower_base_depth"] / 2, ty, 0), size=(0.05,),
                      cls="marker"))
    return items


def depot():
    """The blue DEPOT: a U of bars open toward the alliance wall."""
    dy = D["depot_y"]
    b = D["depot_bar_width"]
    hw = D["depot_width"] / 2
    depth = D["depot_depth"]
    z1 = D["depot_bar_height"]
    kw = {"rgba": COLOURS["blue"], "friction": FRICTION_HARD}
    return [
        box("blue_depot_bar_field", depth - b, depth, dy - hw, dy + hw,
            -BELOW_CARPET_M, z1, **kw),
        box("blue_depot_bar_outpost", 0, depth - b, dy - hw, dy - hw + b,
            -BELOW_CARPET_M, z1, **kw),
        box("blue_depot_bar_guardrail", 0, depth - b, dy + hw - b, dy + hw,
            -BELOW_CARPET_M, z1, **kw),
    ]


def _outpost_spans():
    oy = D["outpost_y"]
    chute = (oy - D["chute_opening_width"] / 2,
             oy + D["chute_opening_width"] / 2)
    corral = (oy - D["corral_opening_width"] / 2,
              oy + D["corral_opening_width"] / 2)
    return chute, corral


def alliance_wall():
    """The blue alliance wall, in segments around the outpost's openings.

    It is 78.8 in high, 78.25 in at the outpost and the tower (GM 5.8).
    """
    h = D["alliance_wall_height"]
    h_low = D["tower_outpost_wall_height"]
    ow = D["outpost_width"]
    tower = (D["tower_y"] - D["tower_width"] / 2,
             D["tower_y"] + D["tower_width"] / 2)
    chute, corral = _outpost_spans()
    span = (min(chute[0], corral[0]), max(chute[1], corral[1]))
    c0 = D["chute_opening_bottom"]
    c1 = c0 + D["chute_opening_height"]
    k0 = D["corral_opening_bottom"]
    k1 = k0 + D["corral_opening_height"]
    t = OUTPOST_WALL_M
    kw = {"rgba": COLOURS["blue_clear"], "friction": FRICTION_HARD}
    return [
        box("blue_wall_stations_outpost", -BACKING_M, 0, ow, tower[0], 0, h,
            **kw),
        box("blue_wall_tower", -BACKING_M, 0, tower[0], tower[1], 0, h_low,
            **kw),
        box("blue_wall_stations_depot", -BACKING_M, 0, tower[1], FIELD_W, 0,
            h, **kw),
        # Beside the openings nothing is behind the wall: full backing.
        box("blue_wall_outpost_guardrail", -BACKING_M, 0, 0, span[0], 0,
            h_low, **kw),
        box("blue_wall_outpost_depot", -BACKING_M, 0, span[1], ow, 0, h_low,
            **kw),
        # Around the openings the chute and corral are behind it.
        box("blue_wall_corral_sill", -t, 0, span[0], span[1], 0, k0, **kw),
        box("blue_wall_outpost_middle", -t, 0, span[0], span[1], k1, c0,
            **kw),
        box("blue_wall_chute_jamb_guardrail", -t, 0, span[0], chute[0], c0,
            c1, **kw),
        box("blue_wall_chute_jamb_depot", -t, 0, chute[1], span[1], c0, c1,
            **kw),
        box("blue_wall_outpost_top", -t, 0, span[0], span[1], c1, h_low,
            **kw),
    ]


def _chute_frame():
    """The chute floor's unit vectors: up its slope, and its normal."""
    s = math.radians(CHUTE_SLOPE_DEG)
    up_slope = (-math.cos(s), 0.0, math.sin(s))  # away from the field
    normal = (math.sin(s), 0.0, math.cos(s))
    return up_slope, normal


def outpost():
    """The blue OUTPOST behind the wall: chute (with its door) and corral."""
    oy = D["outpost_y"]
    chute, _ = _outpost_spans()
    blue = COLOURS["blue"]
    hard = {"friction": FRICTION_HARD}
    hdpe = {"friction": FRICTION_HDPE}
    items = []
    t = OUTPOST_WALL_M
    c0 = D["chute_opening_bottom"]
    c1 = c0 + D["chute_opening_height"]
    up_slope, normal = _chute_frame()
    length = D["chute_length"]
    # Floor: a tilted box whose top face starts at the opening's bottom edge.
    top_centre = (up_slope[0] * length / 2, oy, c0 + up_slope[2] * length / 2)
    half_t = CHUTE_FLOOR_M / 2
    items.append(Item(
        "geom", "blue_outpost_chute_floor", "box",
        sub(top_centre, scale(normal, half_t)),
        qaxis((0, 1, 0), CHUTE_SLOPE_DEG),
        (length / 2, (chute[1] - chute[0]) / 2 + CHUTE_FLOOR_OVERLAP_M,
         half_t),
        rgba=blue, **hdpe))
    back_x = up_slope[0] * length
    high_z = c0 + up_slope[2] * length
    side_top = high_z + D["chute_side_height"]
    wall_bottom = c0 - CHUTE_WALL_BELOW_M
    items += [
        box("blue_outpost_chute_side_guardrail", back_x - OUTPOST_BACKING_M,
            0, chute[0] - OUTPOST_BACKING_M, chute[0], wall_bottom, side_top,
            rgba=blue, **hard),
        box("blue_outpost_chute_side_depot", back_x - OUTPOST_BACKING_M, 0,
            chute[1], chute[1] + OUTPOST_BACKING_M, wall_bottom, side_top,
            rgba=blue, **hard),
        box("blue_outpost_chute_back", back_x - OUTPOST_BACKING_M, back_x,
            chute[0], chute[1], wall_bottom, side_top, rgba=blue, **hard),
    ]
    items.append(box("blue_chute_door", -t, 0, chute[0], chute[1], c0, c1,
                     rgba=COLOURS["blue_clear"], **hdpe))
    # Corral: walls on the carpet behind the wall; the pipe splits its opening.
    cw = D["corral_width"] / 2
    back = -t - D["corral_depth"]
    z1 = D["corral_wall_height"]
    items += [
        box("blue_outpost_corral_side_guardrail", back - OUTPOST_BACKING_M, -t,
            oy - cw - OUTPOST_BACKING_M, oy - cw, 0, z1, rgba=blue, **hard),
        box("blue_outpost_corral_side_depot", back - OUTPOST_BACKING_M, -t,
            oy + cw, oy + cw + OUTPOST_BACKING_M, 0, z1, rgba=blue, **hard),
        box("blue_outpost_corral_back", back - OUTPOST_BACKING_M, back,
            oy - cw, oy + cw, 0, z1, rgba=blue, **hard),
    ]
    k0 = D["corral_opening_bottom"]
    k1 = k0 + D["corral_opening_height"]
    items.append(Item("geom", "blue_outpost_corral_pipe", "cylinder",
                      (-t / 2, oy, (k0 + k1) / 2), IDENTITY,
                      (D["corral_pipe_od"] / 2, (k1 - k0) / 2),
                      rgba=COLOURS["rail"], **hard))
    return items


def blue_fuel():
    """Blue depot and chute fuel centres (red: rotated)."""
    # Depot: 4 along x by 6 along y, filling the 24 x 36 in interior.
    inside_x = D["depot_depth"] - D["depot_bar_width"]
    inside_y = D["depot_width"] - 2 * D["depot_bar_width"]
    gap_x = (inside_x - 8 * R) / 5  # equal gaps: wall, 4 fuel, bar
    gap_y = (inside_y - 12 * R) / 7
    y0 = D["depot_y"] - inside_y / 2
    depot = [(gap_x + R + i * (2 * R + gap_x),
              y0 + gap_y + R + j * (2 * R + gap_y), R + STAGING_GAP_M)
             for i in range(4) for j in range(6)]
    # Chute: rows up the slope from the door, five across, the last row four.
    chute, _ = _outpost_spans()
    up_slope, normal = _chute_frame()
    across = (chute[1] - chute[0] - 10 * R) / 6
    gap = CHUTE_GAP_M
    s0 = (OUTPOST_WALL_M + R + gap + R * normal[0]) / -up_slope[0]
    chute_fuel = []
    for row in range(5):
        for col in range(5):
            if len(chute_fuel) == 24:
                break
            s = s0 + row * (2 * R + gap)
            surface = (up_slope[0] * s, 0.0,
                       D["chute_opening_bottom"] + up_slope[2] * s)
            centre = add(surface, scale(normal, R + gap))
            y = chute[0] + across + R + col * (2 * R + across)
            chute_fuel.append((centre[0], y, centre[2]))
    return depot, chute_fuel


def all_fuel():
    """Every fuel centre, in index order (FUEL_RANGES)."""
    cx, cy = CENTRE
    span_x = D["neutral_half_x"] - R
    span_y = D["neutral_half_y"] - R
    neutral = [(cx - span_x + i * 2 * span_x / 11,
                cy - span_y + j * 2 * span_y / 29, R + STAGING_GAP_M)
               for i in range(12) for j in range(30)]
    depot, chute = blue_fuel()
    reserve = [(RESERVE_X0_M + i * RESERVE_PITCH_M, RESERVE_Y_M, R)
               for i in range(48)]
    groups = {
        "neutral": neutral,
        "blue_depot": depot,
        "red_depot": [_red_point(p) for p in depot],
        "blue_chute": chute,
        "red_chute": [_red_point(p) for p in chute],
        "reserve": reserve,
    }
    fuel = []
    for group, count in FUEL_RANGES:
        assert len(groups[group]) == count, (group, len(groups[group]))
        fuel += groups[group]
    return fuel


def tags(path):
    """Visual AprilTag panels at the official poses."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    items = []
    for tag in data.get("field-tags", data.get("tags", [])):
        t = tag["pose"]["translation"]
        q = tag["pose"]["rotation"]["quaternion"]
        quat = (q["W"], q["X"], q["Y"], q["Z"])
        pos = (t["x"], t["y"], t["z"])
        name = f"tag_{tag['ID']:02d}"
        panel = D["tag_panel"] / 2
        code = D["tag_code"] / 2
        normal = (1 - 2 * (quat[2] ** 2 + quat[3] ** 2),
                  2 * (quat[1] * quat[2] + quat[0] * quat[3]),
                  2 * (quat[1] * quat[3] - quat[0] * quat[2]))
        items.append(Item("geom", name, "box", pos, quat,
                          (0.001, panel, panel), cls="visual",
                          rgba="1 1 1 1"))
        items.append(Item("geom", name + "_code", "box",
                          add(pos, scale(normal, 0.0015)), quat,
                          (0.0005, code, code), cls="visual",
                          rgba="0.05 0.05 0.05 1"))
    return items


# --- output ------------------------------------------------------------------


def fmt(values):
    """Exact (round-trip) decimals: red is checked against blue to 1e-9 m."""
    return " ".join(repr(float(v)) if abs(v) > 1e-15 else "0" for v in values)


def write_item(out, item):
    attrs = [f'name="{item.name}"', f'type="{item.kind}"',
             f'pos="{fmt(item.pos)}"']
    if item.quat != IDENTITY:
        attrs.append(f'quat="{fmt(item.quat)}"')
    if item.mesh:
        attrs.append(f'mesh="{item.mesh}"')
    else:
        attrs.append(f'size="{fmt(item.size)}"')
    if item.cls:
        attrs.append(f'class="{item.cls}"')
    if item.extra:
        attrs.append(item.extra)
    if item.rgba:
        attrs.append(f'rgba="{item.rgba}"')
    if item.friction is not None:
        attrs.append(f'friction="{item.friction} 0.005 0.0001"')
    out.append(f"    <{item.tag} {' '.join(attrs)}/>")


def field(tags_path):
    """Every mesh asset and every geom and site of the field but the fuel,
    in the order the model lists them: the one description both files are
    written from."""
    meshes = {}
    blue = (hub(meshes) + bumps(meshes) + trenches() + tower() + depot() +
            alliance_wall() + outpost())
    carpet = {"pos": (CENTRE[0], CENTRE[1], 0), "size": CARPET_SIZE_M,
              "cls": None, "rgba": COLOURS["carpet"]}
    rail = {"rgba": COLOURS["rail"], "friction": FRICTION_HARD}
    items = [
        # Two coincident carpets: one for fuel (rolling resistance), one for
        # robots. Planes collide over their whole extent; size is rendering.
        Item("geom", "carpet_fuel", "plane",
             extra=f'contype="8" conaffinity="0" condim="6" '
             f'friction="{CARPET_FRICTION} 0.001 {CARPET_ROLLING_M}"',
             **carpet),
        Item("geom", "carpet_robot", "plane",
             extra=f'contype="16" conaffinity="0" condim="3" '
             f'friction="{CARPET_FRICTION} 0.005 0.0001" group="3"',
             **carpet),
        box("guardrail_table", -BACKING_M, FIELD_L + BACKING_M,
            -GUARDRAIL_BACKING_M, 0, 0, D["guardrail_height"], **rail),
        box("guardrail_far", -BACKING_M, FIELD_L + BACKING_M, FIELD_W,
            FIELD_W + GUARDRAIL_BACKING_M, 0, D["guardrail_height"], **rail),
    ]
    items += blue + [_red(item) for item in blue]
    items.append(Item("site", "field_centre", "sphere",
                      (CENTRE[0], CENTRE[1], 0), size=(0.05,), cls="marker"))
    items += tags(tags_path)
    return meshes, items


def build(tags_path):
    """The MJCF model."""
    meshes, items = field(tags_path)
    fuel = all_fuel()
    out = [
        "<!-- Generated by generate_field.py (REQ-DEV-38); do not edit. -->",
        '<mujoco model="rebuilt_2026">',
        '  <compiler angle="degree" autolimits="true"/>',
        f'  <option timestep="{TIMESTEP_S!r}" solver="CG" iterations="20" '
        'tolerance="1e-8" cone="elliptic" integrator="implicitfast">',
        '    <flag sleep="disable"/>',
        "  </option>",
        '  <size memory="64M"/>',
        "  <custom>",
    ]
    first = 0
    for group, count in FUEL_RANGES:
        out.append(f'    <numeric name="fuel_{group}_first" data="{first}"/>')
        out.append(f'    <numeric name="fuel_{group}_count" data="{count}"/>')
        first += count
    out += [
        "  </custom>",
        "  <default>",
        '    <default class="structure">',
        f'      <geom contype="1" conaffinity="0" condim="3" '
        f'solref="{STRUCTURE_SOLREF}" solmix="{STRUCTURE_SOLMIX}"/>',
        "    </default>",
        '    <default class="fuel">',
        f'      <geom type="sphere" size="{R:.9g}" mass="{FUEL_MASS_KG}" '
        f'contype="2" conaffinity="15" condim="3" '
        f'friction="{FUEL_FRICTION} 0 0" solref="{FUEL_SOLREF}" '
        f'solimp="{FUEL_SOLIMP}" rgba="{COLOURS["fuel"]}"/>',
        "    </default>",
        '    <default class="visual">',
        '      <geom contype="0" conaffinity="0" group="1"/>',
        "    </default>",
        '    <default class="marker">',
        '      <site group="4" rgba="0.2 0.9 0.2 0.3"/>',
        "    </default>",
        "  </default>",
        "  <asset>",
    ]
    for name, verts in meshes.items():
        flat = [c for v in verts for c in v]
        out.append(f'    <mesh name="{name}" vertex="{fmt(flat)}"/>')
    out += [
        "  </asset>",
        "  <worldbody>",
        f'    <light name="sun" directional="true" '
        f'pos="{fmt((CENTRE[0], CENTRE[1], LIGHT_HEIGHT_M))}" dir="0 0 -1"/>',
    ]
    for item in items:
        write_item(out, item)
    for i, p in enumerate(fuel):
        out += [
            f'    <body name="fuel_{i:03d}" pos="{fmt(p)}">',
            f'      <freejoint name="fuel_{i:03d}_joint"/>',
            f'      <geom name="fuel_{i:03d}_geom" class="fuel"/>',
            "    </body>",
        ]
    out += ["  </worldbody>", "</mujoco>", ""]
    return "\n".join(out)


# --- glTF --------------------------------------------------------------------

# glTF is Y-up. The scene's one root node turns the field frame (z up) into
# it: a rotation of -90 degrees about x, field (x, y, z) -> glTF (x, z, -y).
# Studio's viewport (Z up) turns a loaded scene back by +90 degrees about x,
# which puts every element where the MJCF has it.
GLTF_ROOT_ROTATION = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))  # x y z w
CYLINDER_SEGMENTS = 32


def _unit_box():
    """The box from -1 to 1 on each axis, flat faces with outward normals."""
    faces = []
    for axis in range(3):
        for sign in (-1.0, 1.0):
            u, v = (axis + 1) % 3, (axis + 2) % 3
            corners = []
            for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                p = [0.0, 0.0, 0.0]
                p[axis], p[u], p[v] = sign, a, b
                corners.append(tuple(p))
            if sign < 0:
                corners.reverse()
            normal = tuple(sign if i == axis else 0.0 for i in range(3))
            faces.append((corners, normal))
    return _from_polygons(faces)


def _unit_cylinder():
    """Radius 1, from z = -1 to 1 (MuJoCo's cylinder axis is its local z)."""
    n = CYLINDER_SEGMENTS
    ring = [(math.cos(2 * math.pi * k / n), math.sin(2 * math.pi * k / n))
            for k in range(n)]
    positions, normals, indices = [], [], []
    for x, y in ring:  # side, smooth
        positions += [(x, y, -1.0), (x, y, 1.0)]
        normals += [(x, y, 0.0), (x, y, 0.0)]
    for k in range(n):
        a, b = 2 * k, 2 * ((k + 1) % n)
        indices += [a, b, a + 1, b, b + 1, a + 1]
    for z in (-1.0, 1.0):  # caps
        centre = len(positions)
        positions.append((0.0, 0.0, z))
        normals.append((0.0, 0.0, z))
        for x, y in ring:
            positions.append((x, y, z))
            normals.append((0.0, 0.0, z))
        for k in range(n):
            a, b = centre + 1 + k, centre + 1 + (k + 1) % n
            indices += [centre, a, b] if z > 0 else [centre, b, a]
    return positions, normals, indices


def _from_polygons(faces):
    """Flat-shaded triangles from convex polygons wound counterclockwise
    seen from outside."""
    positions, normals, indices = [], [], []
    for corners, normal in faces:
        first = len(positions)
        positions += corners
        normals += [normal] * len(corners)
        for k in range(1, len(corners) - 1):
            indices += [first, first + k, first + k + 1]
    return positions, normals, indices


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _hull(points):
    """The convex hull of a few points as outward-wound polygons (what
    MuJoCo builds from a vertex-only mesh asset). Brute force, O(n^4) in the
    number of points: every triple whose plane has all points on one side
    (within 1e-9 m) gives a face, and the points within 1e-7 m of it are its
    corners. Fine for the field's assets (at most 20 points each)."""
    eps = 1e-9
    planes = []
    count = len(points)
    for i in range(count):
        for j in range(i + 1, count):
            for k in range(j + 1, count):
                n = cross(sub(points[j], points[i]), sub(points[k], points[i]))
                length = math.sqrt(_dot(n, n))
                if length < eps:
                    continue
                n = scale(n, 1 / length)
                d = _dot(n, points[i])
                side = [_dot(n, p) - d for p in points]
                if max(side) <= eps:
                    pass
                elif min(side) >= -eps:
                    n, d = scale(n, -1), -d
                else:
                    continue
                if not any(_dot(n, m) > 1 - 1e-9 and abs(d - e) < 1e-9
                           for m, e in planes):
                    planes.append((n, d))
    faces = []
    for n, d in planes:
        on = [p for p in points if abs(_dot(n, p) - d) < 1e-7]
        # A basis in the face's plane, (u, v, n) right-handed.
        helper = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
        u = unit(cross(helper, n))
        v = cross(n, u)
        flat = sorted({(_dot(p, u), _dot(p, v)): p for p in on}.items())
        polygon = _convex_ring([f for f, _ in flat])
        lookup = dict(flat)
        faces.append(([lookup[f] for f in polygon], n))
    return _from_polygons(faces)


def _convex_ring(points):
    """Andrew's monotone chain: the convex polygon, counterclockwise,
    without collinear points."""
    def turn(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and turn(lower[-2], lower[-1], p) <= 1e-12:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and turn(upper[-2], upper[-1], p) <= 1e-12:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _linear_rgba(text):
    """An MJCF rgba (sRGB, as MuJoCo shows it) as glTF's linear
    baseColorFactor; alpha unchanged."""
    r, g, b, a = (float(c) for c in text.split())

    def linear(c):
        return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4

    return [linear(r), linear(g), linear(b), a]


def build_glb(tags_path):
    """The field as a self-contained glTF binary: one node per geom but the
    carpet planes (a viewer draws its own floor), under one root node that
    turns the field frame into glTF's Y-up."""
    meshes, items = field(tags_path)
    geoms = [item for item in items
             if item.tag == "geom" and item.kind != "plane"]
    shapes = {"box": _unit_box(), "cylinder": _unit_cylinder()}
    for name, verts in meshes.items():
        shapes["mesh:" + name] = _hull([tuple(v) for v in verts])
    binary = bytearray()
    views, accessors, gltf_meshes, materials = [], [], [], []
    material_of = {}
    coloured = {}  # (shape, material) -> glTF mesh

    def view(data, target):
        while len(binary) % 4:
            binary.append(0)
        views.append({"buffer": 0, "byteOffset": len(binary),
                      "byteLength": len(data), "target": target})
        binary.extend(data)
        return len(views) - 1

    mesh_of = {}
    for key, (positions, normals, indices) in shapes.items():
        if key.startswith("mesh:") and not any(
                g.mesh == key[5:] for g in geoms):
            continue
        flat = [c for p in positions for c in p]
        pos_view = view(struct.pack(f"<{len(flat)}f", *flat), 34962)
        lows = [min(p[i] for p in positions) for i in range(3)]
        highs = [max(p[i] for p in positions) for i in range(3)]
        # Bounds as the float32 values stored, so a reader's check is exact.
        lows = list(struct.unpack("<3f", struct.pack("<3f", *lows)))
        highs = list(struct.unpack("<3f", struct.pack("<3f", *highs)))
        accessors.append({"bufferView": pos_view, "componentType": 5126,
                          "count": len(positions), "type": "VEC3",
                          "min": lows, "max": highs})
        flat = [c for n in normals for c in n]
        normal_view = view(struct.pack(f"<{len(flat)}f", *flat), 34962)
        accessors.append({"bufferView": normal_view, "componentType": 5126,
                          "count": len(normals), "type": "VEC3"})
        index_view = view(struct.pack(f"<{len(indices)}H", *indices), 34963)
        accessors.append({"bufferView": index_view, "componentType": 5123,
                          "count": len(indices), "type": "SCALAR",
                          "min": [min(indices)], "max": [max(indices)]})
        mesh_of[key] = (len(accessors) - 3, len(accessors) - 2,
                        len(accessors) - 1)
    nodes = [{"name": "field", "rotation": list(GLTF_ROOT_ROTATION),
              "children": list(range(1, len(geoms) + 1))}]
    for item in geoms:
        if item.rgba not in material_of:
            colour = _linear_rgba(item.rgba)
            material = {"pbrMetallicRoughness": {
                "baseColorFactor": colour, "metallicFactor": 0.0,
                "roughnessFactor": 0.9}}
            if colour[3] < 1:
                material["alphaMode"] = "BLEND"
            materials.append(material)
            material_of[item.rgba] = len(materials) - 1
        key = "mesh:" + item.mesh if item.kind == "mesh" else item.kind
        pair = (key, material_of[item.rgba])
        if pair not in coloured:
            position, normal, index = mesh_of[key]
            gltf_meshes.append({"name": key, "primitives": [{
                "attributes": {"POSITION": position, "NORMAL": normal},
                "indices": index, "material": pair[1]}]})
            coloured[pair] = len(gltf_meshes) - 1
        w, x, y, z = item.quat
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if item.kind == "box":
            size = list(item.size)
        elif item.kind == "cylinder":
            size = [item.size[0], item.size[0], item.size[1]]
        else:
            size = [1.0, 1.0, 1.0]
        nodes.append({"name": item.name, "mesh": coloured[pair],
                      "translation": list(item.pos),
                      "rotation": [x / norm, y / norm, z / norm, w / norm],
                      "scale": size})
    while len(binary) % 4:
        binary.append(0)
    gltf = {
        "asset": {"version": "2.0", "generator": "generate_field.py"},
        "scene": 0,
        "scenes": [{"name": "rebuilt_2026", "nodes": [0]}],
        "nodes": nodes,
        "meshes": gltf_meshes,
        "materials": materials,
        "accessors": accessors,
        "bufferViews": views,
        "buffers": [{"byteLength": len(binary)}],
    }
    text = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    text += b" " * (-len(text) % 4)
    total = 12 + 8 + len(text) + 8 + len(binary)
    return (struct.pack("<4sII", b"glTF", 2, total) +
            struct.pack("<I4s", len(text), b"JSON") + text +
            struct.pack("<I4s", len(binary), b"BIN\0") + bytes(binary))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tags", required=True,
                        help="WPILib 2026-rebuilt-welded.json")
    parser.add_argument("--out", help="MJCF file to write")
    parser.add_argument("--glb", help="glTF binary to write")
    args = parser.parse_args()
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(build(args.tags))
    if args.glb:
        with open(args.glb, "wb") as f:
            f.write(build_glb(args.tags))


if __name__ == "__main__":
    main()
