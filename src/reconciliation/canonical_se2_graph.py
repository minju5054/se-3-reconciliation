"""Frozen canonical L/R/A algebra; all original future nodes editable, no clock.

Delegate the historical Local-SE2 implementation without changing its defaults.
The audit deliberately exposes neither weights nor an opt-out of the R factor.
"""
from dataclasses import dataclass, field
import numpy as np
from .local_se2_reconciliation import LocalSE2Problem


@dataclass(frozen=True)
class CanonicalSE2Problem:
    A: np.ndarray
    B: np.ndarray
    fresh: np.ndarray
    algebra: LocalSE2Problem = field(init=False, repr=False)

    def __post_init__(self):
        p = LocalSE2Problem(self.A, self.B, self.fresh)
        object.__setattr__(self, 'algebra', p)
        for key in ('A', 'B', 'fresh'):
            object.__setattr__(self, key, getattr(p, key))

    @property
    def target(self): return self.algebra.target
    @property
    def progress(self): return self.algebra.progress
    @property
    def arc(self): return self.algebra.arc
    @property
    def scales(self): return self.algebra.scales
    def raw_residuals(self, state): return self.algebra.raw_residuals(state)
    def residual_blocks(self, state): return self.algebra.residual_blocks(state)
    def residual_vector(self, state): return self.algebra.residual_vector(state)
    def costs(self, state): return self.algebra.costs(state)
    def original_observation_local(self, state): return self.algebra.original_observation_local(state)
