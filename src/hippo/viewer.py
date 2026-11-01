"""Interactive 3D view and animation in the OCP CAD Viewer (VS Code extension)."""

import numpy as np
from ocp_vscode import Animation, show

from hippo.cad import Parts, assembly
from hippo.design import Design


def show_pose(d: Design, parts: Parts, theta_deg: float | None = None) -> None:
    theta = None if theta_deg is None else np.radians(theta_deg)
    show(assembly(d, parts, theta))


def animate(d: Design, parts: Parts, cycles: int = 2, slowdown: float = 4.0, steps: int = 72) -> None:
    """Show the neutral pose, then drive every moving part through `cycles`
    crank turns. `slowdown` stretches real time so the motion is readable."""
    show(assembly(d, parts))

    theta = d.theta0 + np.linspace(0, 2 * np.pi * cycles, steps * cycles + 1)
    period = 60 / d.params.linkage.crank_rpm * slowdown
    times = (np.linspace(0, cycles, len(theta)) * period).tolist()
    poses = d.poses(theta)

    anim = Animation()
    # Parts start at their neutral pose with no rotation, so tracks are deltas
    # from there: a rotation about the part's own pivot plus, for the coupler,
    # the crank pin's displacement.
    tracks = {"crank_gear": "crank_gear", "pinion": "pinion", "coupler": "coupler", "tail": "rocker_tail"}
    for label, key in tracks.items():
        path = f"/hippo/{label}"
        anim.add_track(path, "rz", times, [p.angle_deg for p in poses[key]])
    p0 = poses["coupler"][0]
    anim.add_track(
        "/hippo/coupler", "t", times, [[p.x - p0.x, p.y - p0.y, 0.0] for p in poses["coupler"]]
    )
    anim.animate(speed=1)
