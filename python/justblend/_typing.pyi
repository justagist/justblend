from __future__ import annotations

from enum import Enum
from typing import overload

import numpy as np
import numpy.typing as npt

ArrayLike = npt.NDArray[np.float64]

class ValidationError(ValueError): ...

class BlendShape(Enum):
    PARABOLIC: BlendShape
    HERMITE: BlendShape

class CornerHandling(Enum):
    STRICT_CORNERS: CornerHandling
    USE_BLENDING: CornerHandling
    HYBRID: CornerHandling

class CornerType(Enum):
    ENDPOINT: CornerType
    PASS: CornerType
    STOP: CornerType
    BLEND: CornerType

class SegmentType(Enum):
    LINEAR: SegmentType
    BLEND: SegmentType

class Limits:
    v_max: ArrayLike
    a_max: ArrayLike
    j_max: ArrayLike | None
    def __init__(
        self,
        v_max: ArrayLike,
        a_max: ArrayLike,
        j_max: ArrayLike | None = ...,
    ) -> None: ...

class GenerationOptions:
    blend_radius: float
    corner_handling: CornerHandling
    blend_shape: BlendShape | None
    def __init__(
        self,
        blend_radius: float = ...,
        corner_handling: CornerHandling = ...,
        blend_shape: BlendShape | None = ...,
    ) -> None: ...

class Sample:
    t: float
    q: ArrayLike
    qd: ArrayLike
    qdd: ArrayLike

class SamplesResult:
    t: ArrayLike
    q: ArrayLike
    qd: ArrayLike
    qdd: ArrayLike

class SegmentInfo:
    type: SegmentType
    duration: float
    v0: float
    length: float
    u_dir: ArrayLike
    q0: ArrayLike
    blend_shape: BlendShape
    V: float
    r: float
    d_in: ArrayLike
    d_out: ArrayLike
    q_start: ArrayLike

class Trajectory:
    def sample(self, t: float) -> Sample: ...
    @overload
    def samples(self, dt: float) -> SamplesResult: ...
    @overload
    def samples(self, times: ArrayLike) -> SamplesResult: ...
    def duration(self) -> float: ...
    def dim(self) -> int: ...
    def junction_speeds(self) -> ArrayLike: ...
    def blend_radii(self) -> list[float]: ...
    def corner_types(self) -> list[CornerType]: ...
    def blend_shape(self) -> BlendShape: ...
    def corner_handling(self) -> CornerHandling: ...
    def num_segments(self) -> int: ...
    def segments(self) -> list[SegmentInfo]: ...
    def empty(self) -> bool: ...

class TrajectoryGenerator:
    def set_limits(self, limits: Limits) -> None: ...
    def set_options(self, options: GenerationOptions) -> None: ...
    def generate(self, waypoints: ArrayLike) -> Trajectory: ...
    def dim(self) -> int: ...
    def limits_set(self) -> bool: ...
    def limits(self) -> Limits: ...
    def options(self) -> GenerationOptions: ...

class TrapezoidalTrajectoryGenerator(TrajectoryGenerator):
    def __init__(self, dim: int) -> None: ...

class SCurveTrajectoryGenerator(TrajectoryGenerator):
    def __init__(self, dim: int) -> None: ...
