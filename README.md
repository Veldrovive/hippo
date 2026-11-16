# hippo

A crank-rocker mechanism that swings a pygmy-hippo-style tail through 60°
to spread slurry. See [docs/design.md](docs/design.md) for the concept.

## Usage

```sh
uv run hippo report    # synthesised link lengths, swing, time ratio, tail speeds
uv run hippo plot      # out/paths.png (paths + motion curves) and out/linkage.gif
uv run hippo export    # out/cad/*.stl and *.step per part, plus assembly.step
uv run hippo show [--theta DEG]                 # 3D model in OCP CAD Viewer
uv run hippo animate [--cycles N] [--slowdown X]  # animated 3D model
```

`show` and `animate` need the **OCP CAD Viewer** VS Code extension with its
viewer panel open.

## How it fits together

All parameters live in [`params.py`](src/hippo/params.py). Change them there.
Everything else is derived from them through `Design`, so plots, CAD and
animation always agree.

| Module | Role |
|---|---|
| `kinematics.py` | Four-bar position solution and synthesis (numpy/scipy) |
| `design.py` | Parameters + synthesised linkage, split-hair centrelines, per-part poses |
| `cad.py` | build123d parts, gears via bd_warehouse, STL/STEP export |
| `plots.py` | matplotlib figure and GIF for slides |
| `viewer.py` | OCP CAD Viewer display and animation |

### Linkage synthesis

The tail should swing at the same speed in both directions, which means a
time ratio of 1. For a crank-rocker, that requires the crank pivot to lie on
the line through the two extreme positions of the coupler-rocker pin. With the
swing angle fixed, this sets the crank length to `r4·sin(swing/2)`. The solver
then picks the shortest coupler that keeps the worst-case transmission angle at
or above `min_transmission`. Raising that target gives a smoother, larger
mechanism; lowering it gives a more compact one.

### Frames and layers

The mechanism moves in the XY plane, with gravity along −Y and the rocker
pivot at the origin. Parts are stacked along +Z: frame plate, gears, coupler,
rocker arm and tail, then the slurry chute in front. The hairs curl toward +Z
so their tips pass under the chute outlet. Each moving part is modelled
around its own pivot in the "tail hanging down" pose. That's the frame the
exported files use.

### Printing notes

Pins and holes are modelled at the same nominal diameter, so add clearance in
the slicer or in `FrameParams.pin_diameter`. The hairs export as a separate
body (`hairs.stl`). They need to be bonded or fused to the tail tip.
