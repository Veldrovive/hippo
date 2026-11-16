"""Single source of truth tying the parameters to the synthesised linkage.

Plots, CAD and animation all go through `Design` so they cannot drift apart.
"""

from dataclasses import dataclass
from functools import cached_property

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


def _arc(start, tangent, normal, length: float, curl: float, n: int = 8) -> np.ndarray:
    """Points (n, 3) along a circular arc of given length that turns by `curl`
    radians from `tangent` towards `normal`."""
    s = np.linspace(0, 1, n)[:, None]
    if abs(curl) < 1e-6:
        return start + s * length * tangent
    radius = length / curl
    return start + radius * np.sin(curl * s) * tangent + radius * (1 - np.cos(curl * s)) * normal


@dataclass(frozen=True)
class Hair:
    """Centrelines (n, 3) of one split hair in the rocker frame, neutral pose,
    z = 0 on the tail mid-plane."""

    trunk: np.ndarray
    strands: list[np.ndarray]


HAIR_EMBED = 4.0
"""Length (mm) of each hair root buried in the tail tip."""


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

    @cached_property
    def hairs(self) -> list[Hair]:
        """Rigid split hairs fringing the tail tip and curling towards +Z.

        Each hair is a trunk that splits into `strands` unequal strands at
        `split_at` of its length, like the whisk-shaped pygmy hippo hairs.
        Plots and CAD both use these centrelines.
        """
        h, t = self.params.hair, self.params.tail
        rng = np.random.default_rng(h.seed)
        tip = np.array([0.0, -t.length, 0.0])
        up = np.array([0.0, 0.0, 1.0])
        curl = np.radians(h.curl)
        trunk_len, strand_len = h.length * h.split_at, h.length * (1 - h.split_at)
        # Curvature is uniform along the hair, so the curl splits by length.
        k = curl / h.length
        spread = np.radians(h.strand_spread)

        result = []
        for a in np.radians(np.linspace(-h.fan / 2, h.fan / 2, h.count)):
            out = np.array([np.sin(a), -np.cos(a), 0.0])
            trunk = _arc(tip - HAIR_EMBED * out, out, up, HAIR_EMBED + trunk_len, k * (HAIR_EMBED + trunk_len))

            turned = k * (HAIR_EMBED + trunk_len)
            tangent = np.cos(turned) * out + np.sin(turned) * up
            normal = -np.sin(turned) * out + np.cos(turned) * up
            lateral = np.cross(tangent, normal)
            strands = []
            for j in range(h.strands):
                psi = 2 * np.pi * j / h.strands + rng.uniform(-0.3, 0.3)
                radial = np.cos(psi) * normal + np.sin(psi) * lateral
                t0 = np.cos(spread) * tangent + np.sin(spread) * radial
                n0 = normal - np.dot(normal, t0) * t0
                n0 /= np.linalg.norm(n0)
                length = strand_len * rng.uniform(0.6, 1.0)
                strands.append(_arc(trunk[-1], t0, n0, length, k * length))
            result.append(Hair(trunk, strands))
        return result

    @cached_property
    def hair_reach(self) -> float:
        """Largest in-plane distance (mm) from the rocker pivot to any hair
        point. This is the radius that sets the hair tip speed."""
        pts = np.concatenate([np.concatenate([hr.trunk, *hr.strands]) for hr in self.hairs])
        return float(np.hypot(pts[:, 0], pts[:, 1]).max())

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
