# Hippo-Tail Slurry Spreader: CAD Specification

This document gives the position and size of every part, so the mechanism can
be modelled in any CAD package. For the concept and biology, see
[design.md](design.md).

All values come from the default parameters in
[`params.py`](../src/hippo/params.py) and the synthesised linkage
(`uv run hippo report`). If a parameter changes, regenerate the numbers rather
than editing this file by hand. Units are **mm** and **degrees**.

---

## 1. Coordinate system and reference pose

| Axis | Direction |
|---|---|
| Origin | **Rocker pivot O4** (the tail's pivot) |
| +X | Horizontal; the crank gear and motor sit on the −X side of the tail pivot |
| +Y | Up (gravity is −Y) |
| +Z | Out of the frame plate, toward the slurry chute (front of the machine) |

- The mechanism moves in the **XY plane**. All pivot shafts are parallel to **Z**.
- **Reference (neutral) pose:** tail hanging straight down, rocker arm pointing
  straight up (+Y). Every part below is drawn in this pose, and the exported
  STEP/STL files use this frame.
- In the neutral pose the crank is at **θ₀ = 82.76°**, measured from +X about O2.

---

## 2. Key points (neutral pose, XY plane)

| Point | Description | X | Y |
|---|---|---|---|
| O4 | Rocker / tail pivot (shaft) | 0.00 | 0.00 |
| O2 | Crank-gear pivot (shaft) | −42.88 | 34.64 |
| P | Pinion / motor shaft | −81.88 | 34.64 |
| A | Crank pin (on crank gear) | −40.36 | 54.48 |
| B | Coupler–rocker pin (on rocker arm) | 0.00 | 40.00 |
| T | Tail tip (point of the heart) | 0.00 | −130.00 |
| C | Chute outlet centre (Z = 42) | 0.00 | −110.00 |

O2 and P lie on the same horizontal line (Y = 34.64).

### Linkage lengths

| Link | Symbol | Length | Between |
|---|---|---|---|
| Ground | r1 | 55.12 | O2 – O4 |
| Crank | r2 | 20.00 | O2 – A |
| Coupler | r3 | 42.88 | A – B |
| Rocker | r4 | 40.00 | O4 – B |

Resulting motion: 60.0° total swing (±30° about straight down), time ratio
1.00 (same speed both ways), worst transmission angle 50.0°.

---

## 3. Z layer stack (back to front)

Parts are stacked along +Z. There is a **0.5 mm axial gap** between moving layers.

| Layer | Z from | Z to | Thickness | Contents |
|---|---|---|---|---|
| Motor (placeholder) | −45.5 | −5.5 | 40.0 | Motor body behind plate |
| Frame plate | −5.5 | −0.5 | 5.0 | Fixed back plate |
| *gap* | −0.5 | 0.0 | 0.5 | |
| Gear layer | 0.0 | 6.0 | 6.0 | Crank gear, pinion, rocker hub |
| *gap* | 6.0 | 6.5 | 0.5 | |
| Coupler layer | 6.5 | 11.5 | 5.0 | Coupler link |
| *gap* | 11.5 | 12.0 | 0.5 | |
| Rocker / tail layer | 12.0 | 24.0 | 12.0 | Rocker arm (12–17) + heart tail (12–24) |
| Hairs | 18.0 | ≈45.9 | — | Rooted on tail mid-plane Z = 18, curl to +Z |
| Chute tube | 34.0 | 50.0 | 16.0 | Tube centred on Z = 42 |

Clearance between the tail's front face (Z = 24) and the chute's back surface (Z = 34) is **10 mm**.

---

## 4. Parts

### 4.1 Frame plate (fixed)

| Feature | Value |
|---|---|
| Outline | Rounded rectangle, corner radius 8 |
| Extents | X −98.38 → +20.00 (width **118.38**), Y −20.00 → +68.00 (height **88.00**) |
| Thickness | 5.0 (Z −5.5 → −0.5) |
| Bores | Ø5.0 through, at **O4 (0, 0)**, **O2 (−42.88, 34.64)**, **P (−81.88, 34.64)** |
| Motor mount | Motor on the back face, centred on P. Placeholder envelope Ø28 × 40 long (Z −45.5 → −5.5) |

Extents are set by: pinion tip radius + 6 mm margin on the left, O4 + 20 on
the right, O4 − 20 at the bottom, chute bracket top + 8 at the top.

> Mounting holes for the motor and for fixing the plate to a stand are **not yet
> defined**. Add them to suit the chosen motor.

### 4.2 Crank gear (rotates about O2)

| Feature | Value |
|---|---|
| Type | Spur gear, module 1.5, 20° pressure angle |
| Teeth | 40 |
| Pitch Ø | 60.0 |
| Tip (outside) Ø | 63.0 |
| Root Ø | 56.25 |
| Face width | 6.0 (Z 0 → 6) |
| Bore | Ø5.0, centred on O2 |
| Crank pin | Ø4.0, at **radius 20.0** from gear centre, angle **82.76°** from +X (local offset X +2.52, Y +19.84). Stands up from the gear face Z 6.0 → 11.5 (5.5 long) so it passes through the coupler. |

### 4.3 Pinion (motor shaft at P)

| Feature | Value |
|---|---|
| Type | Spur gear, module 1.5, 20° pressure angle |
| Teeth | 12 |
| Pitch Ø | 18.0 |
| Tip Ø | 21.0 |
| Root Ø | 14.25 |
| Face width | 6.0 (Z 0 → 6) |
| Bore | Ø5.0 (motor shaft) |
| Mesh | Centre distance 39.0 to O2. Rotate the pinion by half a tooth (15°) so it meshes in the neutral pose. |

Gear ratio is 3.33:1 (pinion : crank). At the nominal 180 rpm crank speed, the
pinion turns at **600 rpm**.

### 4.4 Coupler (link A–B)

| Feature | Value |
|---|---|
| Shape | Obround slot (stadium), full-round ends |
| Centre-to-centre | **42.88** |
| Width | 10.0 (end radius 5.0) |
| Thickness | 5.0 (Z 6.5 → 11.5) |
| Holes | Ø4.0 at each end centre (fit over crank pin at A and rocker pin at B) |
| Neutral orientation | From A to B at **−19.74°** from +X. A = (−40.36, 54.48), B = (0, 40) |

### 4.5 Rocker arm + tail (one rigid body, rotates about O4)

The rocker arm, shaft hub, rocker pin and heart-shaped tail are a single part.

| Feature | Value |
|---|---|
| **Hub** | Ø13.0 cylinder on O4, Z 0 → 12 (runs through the gear layer and coupler layer) |
| Hub bore | Ø5.0 through the entire part, on O4 |
| **Rocker arm** | Obround slot, 40.0 centre-to-centre, 10.0 wide, from O4 (0, 0) straight up to B (0, 40). Thickness 5.0 (Z 12 → 17) |
| **Rocker pin** | Ø4.0 at B (0, 40), projecting **backward** from the arm, Z 6.5 → 12.0 (5.5 long), through the coupler |
| **Tail** | Heart outline, extruded 12.0 (Z 12 → 24) |
| Tail width | 64.0 (X −32 → +32) |
| Tail lobes top | Y +10.0 (10 mm above the pivot) |
| Tail point | Y −130.0 (130 mm below the pivot) |
| Tail mid-plane | Z = 18.0 (hair attachment plane) |

**Heart outline.** The tail uses the classic parametric heart, scaled to the box above:

```
x(t) = 16 sin³t
y(t) = 13 cos t − 5 cos 2t − 2 cos 3t − cos 4t        t ∈ [0, 2π)
```

Scale x so total width = 64. Map y linearly so the top of the lobes sits at
Y = +10 and the point at Y = −130. Fit a closed spline through about 120 points.

### 4.6 Split hairs (bonded to the tail tip)

There are seven rigid hairs, fanned in the swing plane and curling toward the chute (+Z).

| Feature | Value |
|---|---|
| Count | 7 primary hairs |
| Fan | Evenly spaced from −25° to +25° about straight down (every 8.33°), in the XY plane |
| Root | Starts **4 mm inside** the tail tip at T (0, −130), on the mid-plane Z = 18 |
| Total length (exposed) | 50.0 |
| Trunk | Ø3.0, from the root to the split at 75 % of length (37.5 mm exposed + 4 mm embedded) |
| Split | Each trunk splits into **6 strands**, Ø1.2, each deflected **14°** from the trunk axis and spaced roughly evenly (±17° jitter) around it |
| Strand length | Unequal, 7.5 – 12.5 (60 – 100 % of the remaining 12.5 mm) |
| Curl | Constant curvature toward +Z, 60° over 50 mm (**bend radius ≈ 47.7 mm**). Strands continue the same curvature. |
| Envelope (neutral pose) | X −18.2 → +18.0, Y −171.0 → −126.0, Z 18.0 → 45.9 |
| Max reach from O4 | 171.0 (sets the peak hair-tip speed of ≈ 2.0 m/s at 180 rpm) |

Exact centrelines (seeded random strand lengths) are generated by
`Design.hairs`. Use `out/cad/hairs.step` if you need the exact geometry.

### 4.7 Slurry chute (fixed)

| Feature | Value |
|---|---|
| Tube | Straight, vertical (along Y), OD **16.0**, ID **12.0** (2.0 wall) |
| Axis | X = 0, Z = 42.0 |
| Extent | Outlet (open, bottom) at Y −110.0, top at Y +60.0 (length 170.0) |
| Bracket | Block 16.0 (X) × 8.0 (Y) centred at X = 0, Y = 60 (Y 56 → 64), spanning Z −0.5 → 42.0 (42.5 deep). It ties the tube top back to the front face of the frame plate. |
| Inlet | Ø12 bore continues through the bracket at Y 50 → 70, so slurry can be fed from above |

The outlet sits 20 mm above the tail tip and 18 mm in front of the tail's
mid-plane. The curled hairs pass underneath it.

---

## 5. Shafts, pins and fits

| Item | Location | Nominal Ø | Notes |
|---|---|---|---|
| Crank shaft | O2 | 5.0 | Plate → crank gear. Gear rotates freely (or the shaft rotates in the plate) |
| Rocker shaft | O4 | 5.0 | Plate → hub (Z −5.5 → 24). Tail rotates on it |
| Motor shaft | P | 5.0 | Pinion fixed to the shaft (D-flat or set screw) |
| Crank pin | A | 4.0 | Integral with crank gear, through coupler |
| Rocker pin | B | 4.0 | Integral with rocker arm, through coupler |

The model uses the **same nominal diameter for pins and holes**. Before
printing or machining, add running clearance to the holes:

- FDM: +0.3 to +0.4 mm on hole Ø
- SLA / machined: +0.1 to +0.2 mm on hole Ø

Axial retention (circlips, e-clips or printed caps) on the pins and shafts is
**not modelled**. Allow about 2 mm of extra shaft or pin length if you use clips.

---

## 6. Swept envelopes and clearances

These are checked over a full crank revolution.

| Check | Result |
|---|---|
| Crank pin A path | Circle R20 about O2: X −62.9 → −22.9, Y 14.6 → 54.6 |
| Rocker pin B path | Arc R40 about O4: X −20 → +20, Y 34.6 → 40 |
| Coupler centreline to hub (O4) | ≥ 30.6 (hub R6.5 + coupler half-width 5 = 11.5). **OK** |
| Crank gear tip to chute bracket | Gear tip reaches X −11.4; bracket starts at X −8. **3.4 mm gap** |
| Crank gear tip to hub | 55.12 − 31.5 − 6.5 = **17.1 mm gap** |
| Tail tip at ±30° | (±65.0, −112.6) |
| Hair tips at ±30° | Reach R171 from O4, so X ≈ ±85 |
| Tail front face to chute | 10 mm in Z, so the tail and chute never meet |

### Overall envelope (including swing)

| Axis | Min | Max | Size |
|---|---|---|---|
| X | −98.4 (plate) | ≈ +85 (hair tips at +30°) | ≈ 183 |
| Y | −171.0 (hair tips) | +68.0 (plate top) | ≈ 239 |
| Z | −45.5 (motor) | +50.0 (chute front) | ≈ 96 |

---

## 7. Part list

| # | Part | Qty | Motion | Export file |
|---|---|---|---|---|
| 1 | Frame plate (+ motor placeholder) | 1 | Fixed | `frame.step` |
| 2 | Slurry chute + bracket | 1 | Fixed | `chute.step` |
| 3 | Crank gear (with crank pin) | 1 | Rotates about O2 | `crank_gear.step` |
| 4 | Pinion | 1 | Rotates about P (motor) | `pinion.step` |
| 5 | Coupler | 1 | Planar | `coupler.step` |
| 6 | Rocker arm + tail | 1 | Rocks ±30° about O4 | `rocker_tail.step` |
| 7 | Split hairs | 1 set (7) | Bonded to part 6 | `hairs.step` |
| — | Ø5 shafts | 3 | — | Purchased |
| — | DC gear motor, ≈ 600 rpm output (placeholder envelope Ø28 × 40) | 1 | — | Purchased |

Regenerate all exports with `uv run hippo export` (writes `out/cad/`).
