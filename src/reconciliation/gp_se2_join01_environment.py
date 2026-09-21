"""Add only the recorded runtime primitive to the frozen Hospital oracle.

Hospital bilinear/error-bound semantics are unchanged. Primitive unsigned exact
distance is combined with min; independent checking uses the GEOS union.
"""
import numpy as np
from .gp_se2_environment import HospitalEnvironment
from .gp_se2_diag02_environment import EnvironmentDerivatives
from .gp_se2_join01 import box_polygon


class RevealEnvironment(HospitalEnvironment):
    def __init__(self, base, obstacle):
        self.base, self.obstacle = base, obstacle
        self.primitive = box_polygon(obstacle)
        super().__init__(base.obstacles.union(self.primitive), base.workspace,
                         parts=[*base.parts, self.primitive], grid=base.grid, metadata=base.metadata.copy())
        self.gates = list(base.gates)

    def box_distance_gradient(self, xy):
        a = np.asarray(xy, float)
        p = np.asarray(self.obstacle['pose_world']); size = np.asarray(self.obstacle['dimensions_m'])
        c, s = np.cos(p[2]), np.sin(p[2]); rot = np.array([[c,-s],[s,c]])
        q = (a-p[:2])@rot
        residual = q-np.clip(q, -size[:2]/2, size[:2]/2)
        distance = np.linalg.norm(residual, axis=-1)
        gradient = residual/np.where(distance>0,distance,1.)[...,None]@rot.T
        return distance, gradient

    def optimizer_distance(self, xy, *, conservative=True):
        base = self.base.optimizer_distance(xy, conservative=conservative)
        distance, _ = self.box_distance_gradient(xy)
        return np.minimum(base, distance)  # NaN remains NaN, never free space.


class RevealDerivatives:
    def __init__(self, environment, radius=.2):
        self.environment, self.radius = environment, radius
        self.base = EnvironmentDerivatives(environment.base, radius)

    def workspace(self, xy):
        return self.base.workspace(xy)

    def obstacle(self, xy):
        values, gradients, metadata = self.base.obstacle(xy)
        distance, dg = self.environment.box_distance_gradient(xy)
        box = distance-self.radius
        # Preserve invalid original grid penalty, including its zero derivative.
        active = box < values
        result = np.where(active, box, values)
        grad = np.where(active[...,None], dg, gradients)
        records = []
        for i, prior in enumerate(metadata['points']):
            tie = bool(abs(float(box[i]-values[i]))<=1e-12)
            records.append(dict(prior) if not active[i] and not tie else dict(
                kind='box_exact_unsigned' if active[i] else 'union_distance_tie_base_branch',
                smooth=bool(distance[i]>0 and not tie), invalid_query=False,
                inside_constant_zero=bool(distance[i]==0), minimum_tie=tie))
        return result, grad, dict(metadata, points=records,
                policy='min(original conservative grid, exact box); strict tie chooses original; inside box distance zero')
