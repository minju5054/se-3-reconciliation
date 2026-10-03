"""Planning-only 2x2 bridge formulation diagnostic; historical code is unchanged."""
import numpy as np
from scipy.optimize import brentq

from .b_to_entry_bridge import BridgeProblem, hermite_curve, unit, bridge_geometry
from .graph_optimizer import SolverConfig
from .se2 import wrap_angle

VARIANTS = {'A2': ('angle', 2), 'A3': ('angle', 3),
            'V2': ('vector', 2), 'V3': ('vector', 3)}
NEW = ['A3', 'V2', 'V3']
FACTORS = ['E_in', 'E_out', 'E_smooth', 'E_space', 'total']
# Decimal 10 * 1e-6; use the same float representation as the frozen YAML threshold.
COLLAPSE_M = 1e-5
PNGS = ['formulation_geometry.png', 'convergence_and_edge_scale.png', 'factor_costs.png']


def sample_hermite(P, B, suffix, M):
    """Sample the unchanged continuous curve at a prescribed spatial resolution."""
    if M not in (2, 3):
        raise ValueError('diagnostic has exactly M=2 or M=3 segments')
    B, suffix = np.asarray(B, float), np.asarray(suffix, float)
    E = suffix[0]
    xy, _, arc, _, _ = hermite_curve(P, B, E, suffix[1])
    length = arc(1.)
    fractions = np.linspace(0., 1., M+1)
    parameters = np.array([0., *[brentq(lambda u: arc(u)-s*length, 0., 1.,
        xtol=1e-14, rtol=4*np.finfo(float).eps) for s in fractions[1:-1]], 1.])
    bridge = np.array([[*xy(t), wrap_angle(B[2]+s*wrap_angle(E[2]-B[2]))]
                       for t, s in zip(parameters, fractions)])
    bridge[0], bridge[-1] = B, E
    return bridge, dict(M=M, parameters=parameters.tolist(), arc_fractions=fractions.tolist(),
                        continuous_arc_m=length)


class DiagnosticProblem(BridgeProblem):
    """Reuse historical boundary assembly, smoothness, spacing and feasibility."""
    def __init__(self, P, B, suffix, d_F, initial_bridge, variant):
        boundary, M = VARIANTS[variant]
        super().__init__(P, B, suffix, d_F, M)
        initial = np.array(initial_bridge, float, copy=True)
        if initial.shape != (M+1, 3) or not np.isfinite(initial).all():
            raise ValueError('finite M+1 Hermite poses required')
        if initial[0].tobytes() != self.B.tobytes() or initial[-1].tobytes() != self.E.tobytes():
            raise ValueError('initial boundary bits differ')
        self.initial = initial
        self.variant, self.boundary = variant, boundary
        self.initial_lengths = np.linalg.norm(np.diff(initial[:, :2], axis=0), axis=1)
        if np.any(self.initial_lengths <= 1e-12):
            raise ValueError('initial Hermite edges must be nondegenerate')
        self.in_target = self.initial_lengths[0]*unit(self.B[:2]-self.P[:2])
        self.out_target = self.initial_lengths[-1]*unit(self.suffix[1,:2]-self.E[:2])
        for x in (self.P, self.B, self.E, self.suffix, self.initial, self.initial_lengths,
                  self.in_target, self.out_target):
            x.flags.writeable = False

    def factors(self, interior):
        factors = super().factors(interior)
        if self.boundary == 'vector':
            x = self.bridge(interior)
            factors['E_in'] = (x[1,:2]-self.B[:2]-self.in_target)/self.d_F
            factors['E_out'] = (self.E[:2]-x[-2,:2]-self.out_target)/self.d_F
        return factors


def describe(problem, interior, solver, trace, safety, wall_s):
    """No linearization or solve. 'Final' gradient/step means last logged linearization."""
    x = problem.bridge(interior)
    g = bridge_geometry(problem, interior, safety)
    lengths = np.asarray(g['segment_lengths_m'])
    rho = lengths/problem.initial_lengths
    phi = np.arctan2(*np.diff(x[:,:2], axis=0).T[::-1])
    last = trace[-1] if trace else {}
    linearized = [t for t in trace if 'gradient_inf' in t]
    grad = linearized[-1].get('gradient_inf') if linearized else None
    steps = [t for t in trace if 'step_norm' in t]
    step = steps[-1]['step_norm'] if steps else None
    gates = dict(converged=bool(solver.get('converged', False)),
        finite=bool(np.isfinite(x).all() and np.isfinite(list(problem.costs(interior).values())).all()),
        B_bit_exact=x[0].tobytes()==problem.B.tobytes(),
        E_bit_exact=x[-1].tobytes()==problem.E.tobytes(),
        suffix_bit_exact=problem.reference(interior)[problem.M+1:].tobytes()==problem.suffix[1:].tobytes(),
        reference_safe=bool(safety['clearance_valid']),
        no_bridge_self_intersection=not g['bridge_self_intersection'],
        numerically_noncollapsed=bool(min(lengths)>COLLAPSE_M))
    return dict(variant=problem.variant, historical=problem.variant=='A2', M=problem.M,
        editable_nodes=problem.M-1, converged=gates['converged'], stable=all(gates.values()),
        stability_gates=gates, termination_reason=solver.get('termination_reason', 'solver_exception'),
        iterations=int(solver.get('iterations', last.get('iteration', 0))),
        accepted_steps=sum(t['decision']=='accepted' for t in trace),
        rejected_steps=sum(t['decision'].startswith('rejected_') for t in trace),
        unsafe_improving_proposals=sum(t['decision']=='rejected_unsafe' for t in trace),
        final_damping=last.get('damping'), final_gradient_inf=grad, final_step_norm=step,
        gradient_step_semantics='last logged linearization; not recomputed at final accepted state',
        wall_s=wall_s, initial_costs=problem.costs(problem.initial[1:-1]),
        final_costs=problem.costs(interior), geometry=g,
        initial_edge_lengths_m=problem.initial_lengths.tolist(), final_edge_lengths_m=lengths.tolist(),
        first_edge_m=float(lengths[0]), outgoing_edge_m=float(lengths[-1]),
        rho=rho.tolist(), rho_min=float(min(rho)), min_edge_over_fd=float(min(lengths)/1e-6),
        numerical_collapse_threshold_m=COLLAPSE_M, numerical_collapse=not gates['numerically_noncollapsed'],
        first_direction_mismatch_rad=float(wrap_angle(phi[0]-problem.phi_in)),
        outgoing_direction_mismatch_rad=float(wrap_angle(phi[-1]-problem.phi_out)),
        Jacobian_column_norm_min=None, Jacobian_column_norm_max=None, normal_condition_number=None,
        optional_linearization_diagnostics='N/A: unchanged solver trace does not retain Jacobian or normal matrix',
        final_state_label=('STABLE PLANNING REFERENCE' if all(gates.values()) else
            ('CONVERGED BUT UNSTABLE DIAGNOSTIC' if gates['converged'] else
             'UNCONVERGED DIAGNOSTIC — NOT A RETURNED REFERENCE')))


def classify(sources, technical_valid=True):
    counts = {v:sum(s[v]['stable'] for s in sources.values()) for v in VARIANTS}
    if not technical_valid:
        label = 'TECHNICAL_BLOCKED'
    elif sum(counts[v]==4 for v in NEW)>=2:
        label = 'MULTIPLE_STABLE_FORMULATIONS'
    elif counts['A3']==4 and counts['V2']!=4 and counts['A2']==0:
        label = 'RESOLUTION_LIMIT_SUPPORTED'
    elif counts['V2']==4 and counts['A3']!=4 and counts['A2']==0:
        label = 'BOUNDARY_DEGENERACY_SUPPORTED'
    elif counts['V3']==4 and counts['A3']!=4 and counts['V2']!=4:
        label = 'BOTH_RESOLUTION_AND_BOUNDARY_NEEDED'
    elif any(counts[v]>=3 for v in NEW):
        label = 'MIXED_FORMULATION_EFFECT'
    else:
        label = 'FORMULATION_STILL_BLOCKED'
    return dict(classification=label, stable_counts=counts,
                technically_viable_all_four=[v for v in NEW if counts[v]==4])
