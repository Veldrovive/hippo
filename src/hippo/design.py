"""Single source of truth tying the parameters to the synthesised linkage.

Plots, CAD and animation all go through `Design` so they cannot drift apart.
"""

from dataclasses import dataclass

import numpy as np

from hippo.kinematics import FourBar, synthesize
from hippo.params import DesignParams


def heart_outline(length: float, width: float, overhang: float, n: int = 120) -> np.ndarray:
    """Heart outline (2, n) with the lobes at the pivot and the point at (0, -length)."""
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    x = 16 * np.sin(t) ** 3
    y = 13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)
    y_top, y_bot = y.max(), y.min()
    xs = x / (x.max() - x.min()) * width
    ys = overhang - (y_top - y) / (y_top - y_bot) * (length + overhang)
    return np.stack([xs, ys])


@dataclass(frozen=True)
class Pose:
    """Rigid placement of one part relative to its neutral (as-modelled) pose."""

    x: float
    y: float
    angle_deg: float


@dataclass(frozen=True)
class Design:
    params: DesignParams
    linkage: FourBar
    theta0: float
    """Crank angle (radians) at which the tail hangs straight down. Parts are
    modelled in this pose."""

    @classmethod
    def from_params(cls, params: DesignParams | None = None) -> "Design":
        params = params or DesignParams()
        fb = synthesize(params.linkage)
        return cls(params=params, linkage=fb, theta0=fb.theta_at_rocker(np.pi / 2))

    @property
    def hair_reach(self) -> float:
        """In-plane distance (mm) from the rocker pivot to the centre hair tip,
        the furthest point on the tail. The curl is along Z, parallel to the
        pivot axis, so it does not add to this radius."""
        h = self.params.hair
        curl = np.radians(h.curl)
        return self.params.tail.length + (h.length / curl) * np.sin(curl)

    @property
    def gear_ratio(self) -> float:
        """Pinion turns per crank turn."""
        return self.params.gear.crank_teeth / self.params.gear.pinion_teeth

    @property
    def center_distance(self) -> float:
        g = self.params.gear
        return g.module * (g.crank_teeth + g.pinion_teeth) / 2

    @property
    def pinion_center(self) -> np.ndarray:
        # Motor sits on the far side of the crank, away from the tail.
        return self.linkage.o2 + np.array([-self.center_distance, 0.0])

    def poses(self, theta: np.ndarray) -> dict[str, list[Pose]]:
        """Per-part poses for each crank angle, relative to the neutral pose."""
        theta = np.atleast_1d(theta)
        s = self.linkage.solve(theta)
        s0 = self.linkage.solve(self.theta0)
        dtheta = np.degrees(theta - self.theta0)
        dphi3 = np.degrees(s["phi3"] - s0["phi3"][0])
        dphi4 = np.degrees(s["phi4"] - np.pi / 2)
        o2, o4, pc = self.linkage.o2, self.linkage.o4, self.pinion_center
        return {
            "crank_gear": [Pose(o2[0], o2[1], d) for d in dtheta],
            "pinion": [Pose(pc[0], pc[1], -self.gear_ratio * d) for d in dtheta],
            "coupler": [Pose(a[0], a[1], d) for a, d in zip(s["a"], dphi3)],
            "rocker_tail": [Pose(o4[0], o4[1], d) for d in dphi4],
        }
