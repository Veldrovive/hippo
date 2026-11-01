"""2D linkage paths, motion curves and a GIF animation for slides (matplotlib)."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from hippo.design import Design, heart_outline
from hippo.kinematics import analyze

INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#a3a29c"
GRID = "#e4e3df"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"

plt.rcParams.update(
    {
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "axes.edgecolor": MUTED,
        "axes.labelcolor": INK_2,
        "axes.titlecolor": INK,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.8,
        "xtick.color": INK_2,
        "ytick.color": INK_2,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "lines.linewidth": 2,
        "font.size": 10,
    }
)


def _rot(points: np.ndarray, angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s], [s, c]]) @ points


def _tail_geometry(d: Design, phi4: float) -> tuple[np.ndarray, list[np.ndarray]]:
    """Heart outline and hair segments (each (2, n)) for rocker angle phi4."""
    t, h = d.params.tail, d.params.hair
    rot = phi4 - np.pi / 2
    heart = _rot(heart_outline(t.length, t.width, t.overhang), rot)
    hair = d.hair_reach - t.length
    hairs = [
        _rot(np.array([[0.0, hair * np.sin(f)], [-t.length, -t.length - hair * np.cos(f)]]), rot)
        for f in np.radians(np.linspace(-h.fan / 2, h.fan / 2, h.count))
    ]
    return heart, hairs


def _extent(d: Design, margin: float = 8.0) -> tuple[tuple[float, float], tuple[float, float]]:
    """Axis limits that contain the whole mechanism over a full crank turn."""
    fb = d.linkage
    s = fb.solve(np.linspace(0, 2 * np.pi, 145))
    pts = [s["a"].T, s["b"].T, fb.o2[:, None], fb.o4[:, None]]
    for phi4 in s["phi4"]:
        heart, hairs = _tail_geometry(d, phi4)
        pts += [heart, *hairs]
    xy = np.concatenate(pts, axis=1)
    lo, hi = xy.min(axis=1) - margin, xy.max(axis=1) + margin
    return (lo[0], hi[0]), (lo[1], hi[1])


def _draw_linkage(ax, d: Design, theta: float, alpha: float = 1.0, tail: bool = True):
    fb = d.linkage
    s = fb.solve(theta)
    a, b = s["a"][0], s["b"][0]
    lines = [
        ax.plot(*np.stack([fb.o2, a]).T, color=ORANGE, alpha=alpha, solid_capstyle="round")[0],
        ax.plot(*np.stack([a, b]).T, color=BLUE, alpha=alpha, solid_capstyle="round")[0],
        ax.plot(*np.stack([fb.o4, b]).T, color=INK_2, alpha=alpha, solid_capstyle="round")[0],
    ]
    if tail:
        heart, hairs = _tail_geometry(d, s["phi4"][0])
        if alpha < 1:
            closed = np.concatenate([heart, heart[:, :1]], axis=1)
            lines.append(ax.plot(*closed, color=MUTED, lw=1, alpha=alpha)[0])
        else:
            lines.append(ax.fill(*heart, color=INK_2, alpha=0.15, lw=0)[0])
        for seg in hairs:
            lines.append(ax.plot(*seg, color=MUTED, lw=1, alpha=alpha)[0])
    return lines


def _setup_mechanism_axes(ax, d: Design):
    fb, t = d.linkage, d.params.tail
    theta = np.linspace(0, 2 * np.pi, 361)
    s = fb.solve(theta)
    ax.plot(*s["a"].T, color=ORANGE, lw=1, ls="--", alpha=0.6)
    ax.plot(*s["b"].T, color=BLUE, lw=1, ls="--", alpha=0.6)
    swing = s["phi4"] - np.pi / 2
    for r, label in ((t.length, "tail tip"), (d.hair_reach, "hair tips")):
        ax.plot(r * np.sin(swing), -r * np.cos(swing), color=AQUA, lw=2)
        ax.annotate(
            label, (r * np.sin(swing.max()), -r * np.cos(swing.max())), xytext=(6, -4),
            textcoords="offset points", color=INK_2, fontsize=9,
        )
    for p, name in ((fb.o2, "O2 crank"), (fb.o4, "O4 rocker")):
        ax.plot(*p, "o", ms=8, color="white", mec=INK, mew=1.5, zorder=5)
        ax.annotate(name, p, xytext=(-8, 8), textcoords="offset points", color=INK_2, fontsize=9, ha="right")
    # Fixed limits, so animation frames don't rescale as the hairs swing.
    xlim, ylim = _extent(d)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_autoscale_on(False)
    ax.set_aspect("equal")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")


def plot_paths(d: Design, path: Path) -> Path:
    fb, rpm = d.linkage, d.params.linkage.crank_rpm
    rep = analyze(fb, rpm, d.hair_reach)

    fig = plt.figure(figsize=(12, 6.5), layout="constrained")
    grid = fig.add_gridspec(3, 2, width_ratios=[1.25, 1])
    ax = fig.add_subplot(grid[:, 0])
    _setup_mechanism_axes(ax, d)
    for th in (0.0, np.pi):
        _draw_linkage(ax, d, th, alpha=0.35)
    _draw_linkage(ax, d, d.theta0)
    ax.set_title(
        f"Centred crank-rocker  |  r1={rep.r1:.1f}  r2={rep.r2:.1f}  r3={rep.r3:.1f}  r4={rep.r4:.1f} mm",
        loc="left",
    )

    theta = np.linspace(0, 2 * np.pi, 721)
    s = fb.solve(theta)
    deg = np.degrees(theta)
    tail = np.degrees(s["phi4"] - np.pi / 2)
    dt = (theta[1] - theta[0]) / (rpm * 2 * np.pi / 60)
    tip_speed = np.gradient(np.unwrap(s["phi4"]), dt) * d.hair_reach / 1000
    mu = np.degrees(s["mu"])

    panels = [
        (tail, "Tail angle [deg]", f"Swing {rep.swing_deg:.1f}°, time ratio {rep.time_ratio:.2f}"),
        (
            tip_speed,
            "Hair tip speed [m/s]",
            f"Peak {rep.peak_tip_speed_ms:.2f} m/s at hair tips (r = {d.hair_reach:.0f} mm), {rpm:.0f} crank rpm",
        ),
        (mu, "Transmission [deg]", f"Worst-case {rep.min_transmission_deg:.1f}° (90° ideal)"),
    ]
    axes = []
    for i, (y, ylabel, title) in enumerate(panels):
        a = fig.add_subplot(grid[i, 1], sharex=axes[0] if axes else None)
        a.plot(deg, y, color=BLUE)
        a.set_ylabel(ylabel)
        a.set_title(title, loc="left", fontsize=10)
        axes.append(a)
    axes[2].axhspan(0, d.params.linkage.min_transmission, color=GRID, lw=0)
    axes[2].axhline(90, color=MUTED, lw=1, ls=":")
    axes[2].set_ylim(0, 180)
    for a in axes:
        a.axvline(np.degrees(d.theta0), color=MUTED, lw=1, ls=":")
    axes[-1].set_xlabel("Crank angle [deg]")
    axes[-1].set_xticks(range(0, 361, 90))

    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def animate_gif(d: Design, path: Path, frames: int = 72, cycles_per_second: float = 0.75) -> Path:
    fig, ax = plt.subplots(figsize=(6, 6.5), layout="constrained")
    _setup_mechanism_axes(ax, d)
    ax.set_title("Hippo tail crank-rocker", loc="left")
    # Lay out once with a representative frame, then freeze it so the axes
    # don't jitter as the moving artists change from frame to frame.
    probe = _draw_linkage(ax, d, d.theta0)
    fig.canvas.draw()
    fig.set_layout_engine("none")
    for art in probe:
        art.remove()
    artists: list = []

    def frame(i: int):
        for art in artists:
            art.remove()
        artists.clear()
        theta = d.theta0 + 2 * np.pi * i / frames
        artists.extend(_draw_linkage(ax, d, theta))
        return artists

    anim = FuncAnimation(fig, frame, frames=frames, blit=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    anim.save(path, writer=PillowWriter(fps=round(frames * cycles_per_second)))
    plt.close(fig)
    return path
