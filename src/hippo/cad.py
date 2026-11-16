"""Parametric 3D model of the mechanism (build123d).

Each moving part is modelled in its own frame with its pivot at the origin and
in the neutral pose (tail hanging straight down), so the same solids are used
for printing (STL/STEP) and for animation (rotate about the local origin).

Z layers, back to front: frame plate | gears | coupler | rocker arm + tail | chute.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from bd_warehouse.gear import SpurGear
from build123d import (
    Align,
    Box,
    Circle,
    Color,
    Compound,
    Cylinder,
    Location,
    Part,
    Plane,
    Pos,
    RectangleRounded,
    Rot,
    SlotCenterToCenter,
    Spline,
    export_step,
    export_stl,
    extrude,
    make_face,
    sweep,
)

from hippo.design import Design, Pose, heart_outline

GAP = 0.5
"""Axial clearance between Z layers."""

STL_TOLERANCE = 0.02
"""Chordal deviation for STL export, in mm; well below FDM/SLA resolution."""

BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass(frozen=True)
class Layers:
    frame: tuple[float, float]
    gear: tuple[float, float]
    coupler: tuple[float, float]
    rocker: tuple[float, float]

    @classmethod
    def from_design(cls, d: Design) -> "Layers":
        f, g, t = d.params.frame, d.params.gear, d.params.tail
        gear = (0.0, g.thickness)
        coupler = (gear[1] + GAP, gear[1] + GAP + f.link_thickness)
        rocker = (coupler[1] + GAP, coupler[1] + GAP + t.thickness)
        frame = (-GAP - f.plate_thickness, -GAP)
        return cls(frame=frame, gear=gear, coupler=coupler, rocker=rocker)


def _slab(face, z0: float, z1: float) -> Part:
    return Pos(0, 0, z0) * extrude(face, z1 - z0)


def crank_gear(d: Design) -> Part:
    g, f = d.params.gear, d.params.frame
    lay = Layers.from_design(d)
    gear = SpurGear(g.module, g.crank_teeth, g.pressure_angle, g.thickness, align=BOTTOM)
    gear -= Cylinder(g.bore / 2, 3 * g.thickness)
    # Crank pin at the neutral crank angle, long enough to pass through the coupler.
    a = d.linkage.r2 * np.array([np.cos(d.theta0), np.sin(d.theta0)])
    gear += Pos(a[0], a[1], lay.gear[1]) * Cylinder(
        f.pin_diameter / 2, lay.coupler[1] - lay.gear[1], align=BOTTOM
    )
    return gear


def pinion(d: Design) -> Part:
    g = d.params.gear
    # Offset half a tooth so a tooth of the pinion faces a gap in the crank gear.
    gear = SpurGear(
        g.module,
        g.pinion_teeth,
        g.pressure_angle,
        g.thickness,
        rotation=(0, 0, 180 / g.pinion_teeth),
        align=BOTTOM,
    )
    return gear - Cylinder(g.bore / 2, 3 * g.thickness)


def coupler(d: Design) -> Part:
    f = d.params.frame
    lay = Layers.from_design(d)
    s0 = d.linkage.solve(d.theta0)
    ab = s0["b"][0] - s0["a"][0]
    mid, angle = ab / 2, np.degrees(np.arctan2(ab[1], ab[0]))
    face = Pos(mid[0], mid[1]) * Rot(0, 0, angle) * SlotCenterToCenter(d.linkage.r3, f.link_width)
    face -= Circle(f.pin_diameter / 2)
    face -= Pos(ab[0], ab[1]) * Circle(f.pin_diameter / 2)
    return _slab(face, *lay.coupler)


def heart_face(length: float, width: float, overhang: float):
    """Heart outline with the lobes at the pivot and the point at (0, -length)."""
    return make_face(Spline(*zip(*heart_outline(length, width, overhang)), periodic=True))


def rocker_tail(d: Design) -> Part:
    """Rocker arm, shaft hub and heart-shaped tail as one rigid body."""
    f, g, t = d.params.frame, d.params.gear, d.params.tail
    lay = Layers.from_design(d)
    r4 = d.linkage.r4
    hub_r = g.bore / 2 + 4
    arm = Pos(0, r4 / 2) * Rot(0, 0, 90) * SlotCenterToCenter(r4, f.link_width)
    body = _slab(arm, lay.rocker[0], lay.rocker[0] + f.link_thickness)
    body += _slab(heart_face(t.length, t.width, t.overhang), *lay.rocker)
    body += Pos(0, 0, lay.gear[0]) * Cylinder(hub_r, lay.rocker[0] - lay.gear[0], align=BOTTOM)
    # Rocker pin reaches back through the coupler.
    body += Pos(0, r4, lay.coupler[0]) * Cylinder(
        f.pin_diameter / 2, lay.rocker[0] - lay.coupler[0], align=BOTTOM
    )
    return body - Cylinder(g.bore / 2, 4 * (lay.rocker[1] - lay.gear[0]))


def _tube(points: np.ndarray, diameter: float) -> Part:
    path = Spline(*[tuple(p) for p in points])
    start, direction = path @ 0, path % 0
    return sweep(Plane(origin=start, z_dir=direction) * Circle(diameter / 2), path=path)


def hairs(d: Design) -> Compound:
    """Tubes along the split-hair centrelines from `Design.hairs`."""
    h = d.params.hair
    lay = Layers.from_design(d)
    z = np.array([0.0, 0.0, (lay.rocker[0] + lay.rocker[1]) / 2])
    solids = []
    for hair in d.hairs:
        solids.append(_tube(hair.trunk + z, h.trunk_diameter))
        solids.extend(_tube(s + z, h.strand_diameter) for s in hair.strands)
    return Compound(solids)


def frame(d: Design) -> Part:
    f, g = d.params.frame, d.params.gear
    lay = Layers.from_design(d)
    o2, o4, pc = d.linkage.o2, d.linkage.o4, d.pinion_center
    margin = g.module * (g.pinion_teeth + 2) / 2 + 6
    x0, x1 = pc[0] - margin, o4[0] + 20
    y0, y1 = o4[1] - 20, f.chute_top_y + 8
    face = Pos((x0 + x1) / 2, (y0 + y1) / 2) * RectangleRounded(x1 - x0, y1 - y0, 8)
    for c in (o2, o4, pc):
        face -= Pos(c[0], c[1]) * Circle(g.bore / 2)
    plate = _slab(face, *lay.frame)

    # Motor behind the plate, driving the pinion.
    motor_len = 40
    plate += Pos(pc[0], pc[1], lay.frame[0] - motor_len) * Cylinder(14, motor_len, align=BOTTOM)
    return plate


def chute(d: Design) -> Part:
    """Slurry feed tube ending in an outlet below the pivot, in front of the tail,
    with a bracket back to the frame."""
    f = d.params.frame
    lay = Layers.from_design(d)
    length = f.chute_top_y - f.chute_outlet_y
    tube = Pos(0, f.chute_outlet_y, f.chute_outlet_z) * Rot(-90, 0, 0) * Cylinder(
        f.chute_outer_diameter / 2, length, align=BOTTOM
    )
    tube -= Pos(0, f.chute_outlet_y - 1, f.chute_outlet_z) * Rot(-90, 0, 0) * Cylinder(
        f.chute_inner_diameter / 2, length + 2, align=BOTTOM
    )
    depth = f.chute_outlet_z - lay.frame[1]
    bracket = Pos(0, f.chute_top_y, lay.frame[1]) * Box(
        f.chute_outer_diameter, 8, depth, align=BOTTOM
    )
    bracket -= Pos(0, f.chute_top_y - 10, f.chute_outlet_z) * Rot(-90, 0, 0) * Cylinder(
        f.chute_inner_diameter / 2, 20, align=BOTTOM
    )
    return tube + bracket


COLORS = {
    "frame": Color(0.75, 0.75, 0.78),
    "chute": Color(0.55, 0.7, 0.85, 0.6),
    "crank_gear": Color(0.85, 0.55, 0.2),
    "pinion": Color(0.35, 0.35, 0.4),
    "coupler": Color(0.25, 0.5, 0.8),
    "rocker_tail": Color(0.45, 0.35, 0.4),
    "hairs": Color(0.93, 0.88, 0.75),
}


@dataclass
class Parts:
    frame: Part
    chute: Part
    crank_gear: Part
    pinion: Part
    coupler: Part
    rocker_tail: Part
    hairs: Compound

    @classmethod
    def build(cls, d: Design) -> "Parts":
        return cls(
            frame=frame(d),
            chute=chute(d),
            crank_gear=crank_gear(d),
            pinion=pinion(d),
            coupler=coupler(d),
            rocker_tail=rocker_tail(d),
            hairs=hairs(d),
        )

    def items(self) -> dict[str, Part | Compound]:
        return dict(vars(self))


def _place(shape, pose: Pose):
    return shape.moved(Location((pose.x, pose.y, 0), (0, 0, pose.angle_deg)))


def assembly(d: Design, parts: Parts, theta: float | None = None) -> Compound:
    """Full mechanism at crank angle theta (radians); neutral pose if None.

    The tail and its hairs are grouped as one sub-assembly so they move together.
    """
    theta = d.theta0 if theta is None else theta
    poses = {k: v[0] for k, v in d.poses(np.array([theta])).items()}

    def labelled(name: str, shape):
        shape = shape.moved(Location())
        shape.label, shape.color = name, COLORS[name]
        return shape

    tail = Compound(
        children=[labelled("rocker_tail", parts.rocker_tail), labelled("hairs", parts.hairs)],
        label="tail",
    )
    children = [
        labelled("frame", parts.frame),
        labelled("chute", parts.chute),
        _place(labelled("crank_gear", parts.crank_gear), poses["crank_gear"]),
        _place(labelled("pinion", parts.pinion), poses["pinion"]),
        _place(labelled("coupler", parts.coupler), poses["coupler"]),
        _place(tail, poses["rocker_tail"]),
    ]
    return Compound(children=children, label="hippo")


def export(d: Design, parts: Parts, out_dir: Path) -> list[Path]:
    """Write each part (in its local frame) as STL and STEP, plus the assembly."""
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for name, shape in parts.items().items():
        stl, step = out_dir / f"{name}.stl", out_dir / f"{name}.step"
        export_stl(shape, str(stl), tolerance=STL_TOLERANCE, angular_tolerance=0.2)
        export_step(shape, str(step))
        written += [stl, step]
    path = out_dir / "assembly.step"
    export_step(assembly(d, parts), str(path))
    written.append(path)
    return written
