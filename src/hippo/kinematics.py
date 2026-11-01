"""Four-bar (crank-rocker) kinematics and synthesis.

Notation: O2 is the crank pivot, O4 the rocker pivot, A the crank pin and B the
coupler-rocker pin. r1..r4 are ground, crank, coupler and rocker lengths.
Angles are in radians internally; public helpers that say "deg" use degrees.
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq

from hippo.params import LinkageParams


@dataclass(frozen=True)
class FourBar:
    o2: np.ndarray
    o4: np.ndarray
    r2: float
    r3: float
    r4: float
    branch: float = 1.0
    """Which of the two assembly modes B sits on (sign of (O4-A) x (B-A))."""

    @property
    def r1(self) -> float:
        return float(np.linalg.norm(self.o4 - self.o2))

    @property
    def is_grashof_crank_rocker(self) -> bool:
        lengths = sorted([self.r1, self.r2, self.r3, self.r4])
        return self.r2 == lengths[0] and lengths[0] + lengths[3] <= lengths[1] + lengths[2]

    def solve(self, theta: np.ndarray | float) -> dict[str, np.ndarray]:
        """Positions and angles for crank angle(s) theta (radians, world frame)."""
        theta = np.atleast_1d(np.asarray(theta, dtype=float))
        a = self.o2 + self.r2 * np.stack([np.cos(theta), np.sin(theta)], axis=-1)

        # B is the intersection of circle(A, r3) and circle(O4, r4).
        d_vec = self.o4 - a
        d = np.linalg.norm(d_vec, axis=-1)
        u = d_vec / d[:, None]
        along = (self.r3**2 - self.r4**2 + d**2) / (2 * d)
        h = np.sqrt(np.clip(self.r3**2 - along**2, 0.0, None))
        perp = np.stack([-u[:, 1], u[:, 0]], axis=-1)
        b = a + along[:, None] * u + self.branch * h[:, None] * perp

        phi3 = np.arctan2(b[:, 1] - a[:, 1], b[:, 0] - a[:, 0])
        phi4 = np.arctan2(b[:, 1] - self.o4[1], b[:, 0] - self.o4[0])
        cos_mu = (self.r3**2 + self.r4**2 - d**2) / (2 * self.r3 * self.r4)
        mu = np.arccos(np.clip(cos_mu, -1.0, 1.0))
        return {"theta": theta, "a": a, "b": b, "phi3": phi3, "phi4": phi4, "mu": mu}

    def theta_at_rocker(self, phi4: float) -> float:
        """Crank angle on the forward stroke (rocker angle increasing) at which
        the rocker reaches phi4."""
        theta = np.linspace(0, 2 * np.pi, 3601)
        p = np.unwrap(self.solve(theta)["phi4"])
        dp = np.gradient(p)
        err = p - phi4
        for i in range(len(theta) - 1):
            if dp[i] > 0 and err[i] * err[i + 1] <= 0:
                return float(brentq(lambda t: self.solve(t)["phi4"][0] - phi4, theta[i], theta[i + 1]))
        raise ValueError(f"rocker never reaches {np.degrees(phi4):.1f} deg")


def worst_transmission(fb: FourBar, n: int = 1441) -> float:
    """Worst-case transmission angle over a full crank turn, in radians (0..pi/2)."""
    mu = fb.solve(np.linspace(0, 2 * np.pi, n))["mu"]
    return float(np.min(np.minimum(mu, np.pi - mu)))


def centered_crank_rocker(rocker_length: float, swing: float, offset: float) -> FourBar:
    """Crank-rocker with a 1:1 time ratio (equal speed both ways).

    The rocker pivot is at the origin and the rocker arm points up, swinging
    +-swing/2 about +Y; the tail is mounted opposite the arm so it hangs down.
    For a time ratio of 1 the crank pivot must lie on the line through the two
    extreme positions of B. That fixes r2 = r4 sin(swing/2) and r3 = offset,
    where offset is the distance from O2 to the midpoint of B1B2.
    """
    half = swing / 2
    r2 = float(rocker_length * np.sin(half))
    chord_y = rocker_length * np.cos(half)
    o2 = np.array([-offset, chord_y])
    o4 = np.array([0.0, 0.0])
    fb = FourBar(o2=o2, o4=o4, r2=r2, r3=offset, r4=rocker_length)
    # Pick the assembly mode where B stays above the rocker pivot.
    if fb.solve(0.0)["b"][0, 1] < 0:
        fb = FourBar(o2=o2, o4=o4, r2=r2, r3=offset, r4=rocker_length, branch=-1.0)
    return fb


def synthesize(params: LinkageParams) -> FourBar:
    """Shortest centred crank-rocker that meets the transmission-angle target.

    Worst transmission angle improves monotonically as the crank moves further
    out (it tends to 90 - swing/2), so we solve for the smallest offset that
    meets the target to keep the mechanism compact.
    """
    swing = np.radians(params.swing)
    target = np.radians(params.min_transmission)
    limit = np.pi / 2 - swing / 2
    if target >= limit:
        raise ValueError(
            f"min_transmission must be below {np.degrees(limit):.1f} deg for a {params.swing:.0f} deg swing"
        )

    r2 = params.rocker_length * np.sin(swing / 2)
    lo, hi = 1.05 * r2, 50 * params.rocker_length

    def gap(offset: float) -> float:
        return worst_transmission(centered_crank_rocker(params.rocker_length, swing, offset)) - target

    offset = float(brentq(gap, lo, hi, xtol=1e-3)) if gap(lo) < 0 else lo
    fb = centered_crank_rocker(params.rocker_length, swing, offset)
    assert fb.is_grashof_crank_rocker, "synthesised linkage is not a Grashof crank-rocker"
    return fb


@dataclass(frozen=True)
class Report:
    r1: float
    r2: float
    r3: float
    r4: float
    swing_deg: float
    time_ratio: float
    min_transmission_deg: float
    neutral_theta_deg: float
    peak_tip_speed_ms: float
    """Peak tangential speed at `tip_radius`, in m/s."""
    peak_tail_accel_dps2: float


def analyze(fb: FourBar, crank_rpm: float, tip_radius: float, n: int = 3600) -> Report:
    """Motion summary; `tip_radius` (mm from the rocker pivot) sets where the
    linear tip speed is measured."""
    theta = np.linspace(0, 2 * np.pi, n, endpoint=False)
    phi4 = np.unwrap(fb.solve(theta)["phi4"])
    i_max, i_min = int(np.argmax(phi4)), int(np.argmin(phi4))
    forward = (theta[i_max] - theta[i_min]) % (2 * np.pi)

    omega = crank_rpm * 2 * np.pi / 60
    dt = (theta[1] - theta[0]) / omega
    # Periodic central differences.
    vel = (np.roll(phi4, -1) - np.roll(phi4, 1)) / (2 * dt)
    acc = (np.roll(phi4, -1) - 2 * phi4 + np.roll(phi4, 1)) / dt**2

    return Report(
        r1=fb.r1,
        r2=fb.r2,
        r3=fb.r3,
        r4=fb.r4,
        swing_deg=float(np.degrees(phi4.max() - phi4.min())),
        time_ratio=float(forward / (2 * np.pi - forward)),
        min_transmission_deg=float(np.degrees(worst_transmission(fb))),
        neutral_theta_deg=float(np.degrees(fb.theta_at_rocker(np.pi / 2))),
        peak_tip_speed_ms=float(np.abs(vel).max() * tip_radius / 1000),
        peak_tail_accel_dps2=float(np.degrees(np.abs(acc).max())),
    )
