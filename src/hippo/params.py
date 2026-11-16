"""Design parameters for the hippo-tail slurry spreader.

All lengths are in millimetres, all angles in degrees. The mechanism moves in
the XY plane with gravity along -Y; +Z points out of the frame towards the
slurry chute (the "body" side of the tail).
"""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class LinkageParams:
    rocker_length: float = 40.0
    """Rocker pivot (O4) to coupler pin (B)."""
    swing: float = 60.0
    """Total rocker/tail swing, centred on straight down."""
    min_transmission: float = 50.0
    """Synthesis target for the worst-case transmission angle."""
    crank_rpm: float = 180.0
    """Nominal crank speed, used for the velocity report and animation."""


@dataclass(frozen=True)
class GearParams:
    module: float = 1.5
    crank_teeth: int = 40
    pinion_teeth: int = 12
    pressure_angle: float = 20.0
    thickness: float = 6.0
    bore: float = 5.0
    """Shaft diameter for the crank gear and pinion."""


@dataclass(frozen=True)
class TailParams:
    length: float = 130.0
    """Rocker pivot to the point of the heart (tail tip)."""
    width: float = 64.0
    thickness: float = 12.0
    overhang: float = 10.0
    """How far the heart lobes extend above the pivot."""


@dataclass(frozen=True)
class HairParams:
    count: int = 7
    """Primary hairs fringing the tail tip."""
    fan: float = 50.0
    """Total angle the primary hairs fan out over in the swing plane."""
    length: float = 50.0
    split_at: float = 0.75
    """Fraction of the hair length, from the root, at which it splits into strands."""
    strands: int = 6
    """Strands each hair splits into at `split_at`."""
    strand_spread: float = 14.0
    """Angle each strand deflects from its trunk at the split."""
    curl: float = 60.0
    """Total inward curl (towards +Z / the slurry outlet) along the hair."""
    trunk_diameter: float = 3.0
    strand_diameter: float = 1.2
    seed: int = 3
    """Seed for the unequal strand lengths."""


@dataclass(frozen=True)
class FrameParams:
    plate_thickness: float = 5.0
    pin_diameter: float = 4.0
    link_width: float = 10.0
    link_thickness: float = 5.0
    chute_outer_diameter: float = 16.0
    chute_inner_diameter: float = 12.0
    chute_outlet_y: float = -110.0
    chute_outlet_z: float = 42.0
    chute_top_y: float = 60.0


@dataclass(frozen=True)
class DesignParams:
    linkage: LinkageParams = field(default_factory=LinkageParams)
    gear: GearParams = field(default_factory=GearParams)
    tail: TailParams = field(default_factory=TailParams)
    hair: HairParams = field(default_factory=HairParams)
    frame: FrameParams = field(default_factory=FrameParams)
