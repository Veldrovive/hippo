"""Command line entry point: `uv run hippo <command>`."""

import argparse
from dataclasses import asdict
from pathlib import Path

from hippo.design import Design
from hippo.kinematics import analyze


def main() -> None:
    parser = argparse.ArgumentParser(prog="hippo", description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("out"), help="output directory")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("report", help="print synthesised link lengths and motion summary")
    sub.add_parser("plot", help="write the linkage path figure and a GIF animation")
    sub.add_parser("export", help="write STL/STEP for every part and the assembly")
    show = sub.add_parser("show", help="show the 3D model in OCP CAD Viewer")
    show.add_argument("--theta", type=float, help="crank angle in degrees (default: tail hanging down)")
    anim = sub.add_parser("animate", help="animate the 3D model in OCP CAD Viewer")
    anim.add_argument("--cycles", type=int, default=2)
    anim.add_argument("--slowdown", type=float, default=4.0, help="playback slower than real time")
    args = parser.parse_args()

    d = Design.from_params()

    if args.command == "report":
        for k, v in asdict(analyze(d.linkage, d.params.linkage.crank_rpm, d.hair_reach)).items():
            print(f"{k:24s} {v:10.2f}")
        print(f"{'o2':24s} {d.linkage.o2.round(2)}")
        print(f"{'pinion_center':24s} {d.pinion_center.round(2)}")
        print(f"{'gear_ratio':24s} {d.gear_ratio:10.2f}")
    elif args.command == "plot":
        from hippo.plots import animate_gif, plot_paths

        print(plot_paths(d, args.out / "paths.png"))
        print(animate_gif(d, args.out / "linkage.gif"))
    elif args.command == "export":
        from hippo.cad import Parts, export

        for path in export(d, Parts.build(d), args.out / "cad"):
            print(path)
    else:
        from hippo.cad import Parts
        from hippo import viewer

        parts = Parts.build(d)
        if args.command == "show":
            viewer.show_pose(d, parts, args.theta)
        else:
            viewer.animate(d, parts, cycles=args.cycles, slowdown=args.slowdown)
