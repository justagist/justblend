"""justblend: closed-form N-dimensional trajectory generator with smooth corner blends."""

from ._core import (
    BlendShape,
    CornerHandling,
    CornerType,
    GenerationOptions,
    Limits,
    Sample,
    SamplesResult,
    SCurveTrajectoryGenerator,
    SegmentInfo,
    SegmentType,
    Trajectory,
    TrajectoryGenerator,
    TrapezoidalTrajectoryGenerator,
    ValidationError,
)

__all__ = [
    "BlendShape",
    "CornerHandling",
    "CornerType",
    "GenerationOptions",
    "Limits",
    "Sample",
    "SamplesResult",
    "SCurveTrajectoryGenerator",
    "SegmentInfo",
    "SegmentType",
    "Trajectory",
    "TrajectoryGenerator",
    "TrapezoidalTrajectoryGenerator",
    "ValidationError",
]

__version__ = "0.1.0"
