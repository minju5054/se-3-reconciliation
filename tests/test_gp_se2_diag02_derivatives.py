"""Independent derivative tests; synthetic mathematics, never performance data."""
import copy
import inspect

import numpy as np
import pytest

pytest.importorskip('jax')
import jax
import jax.numpy as jnp

from reconciliation import gp_se2 as original_gp
from reconciliation import se2 as original_se2
from reconciliation import gp_se2_diag02_ad as ad
from reconciliation.gp_se2_diag02_derivatives import DerivativeProvider,DerivativeError
from reconciliation.gp_se2_diag_fixtures import make_fixture_problem
from reconciliation.gp_se2_formulation import GPProblem


class AffineEnvironment:
    def workspace(self,xy):
        return 2.+.3*xy[:,0]-.1*xy[:,1],np.tile([.3,-.1],(len(xy),1)),{'family':'workspace','smooth':True}

    def obstacle(self,xy):
        return 1.5-.2*xy[:,0]+.4*xy[:,1],np.tile([-.2,.4],(len(xy),1)),{'family':'obstacle','smooth':True}


def fixture(name='S1',*,obstacles=True,boundary_shift=None):
    p,f=make_fixture_problem(name)
    env=AffineEnvironment()
    boundary=p.boundary_pose.copy();reference=p.common_reference.copy();goal=p.goal_pose.copy()
    if boundary_shift is not None:
        boundary=original_se2.compose_poses(boundary_shift,boundary)
        reference=original_se2.compose_poses(boundary_shift,reference)
        goal=original_se2.compose_poses(boundary_shift,goal)
    problem=GPProblem(boundary,p.initial_twist,reference,goal,p.config,
                      lambda xy:env.obstacle(xy)[0],lambda xy:env.workspace(xy)[0],[],obstacles)
    poses=np.vstack([boundary,reference])
    velocity=f['support_twists']
    vector=problem.vector(poses,velocity)
    return problem,vector,env


@pytest.fixture(scope='module')
def ready():
    problem,vector,env=fixture()
    provider=DerivativeProvider(problem,env)
    provider.warmup(vector)
    return problem,vector,provider


def central_jacobian(function,z,step=1e-5):
    columns=[]
    for index in range(len(z)):
        delta=np.zeros_like(z);delta[index]=step
        columns.append((np.asarray(function(z+delta))-np.asarray(function(z-delta)))/(2*step))
    return np.stack(columns,axis=-1)


@pytest.mark.parametrize('name',['S0','S1','S2','S3'])
@pytest.mark.parametrize('nonzero_delta',[False,True])
def test_all_original_primal_fields_match(name,nonzero_delta):
    problem,vector,env=fixture(name,boundary_shift=np.array([13.,-7.,1.1]))
    if nonzero_delta:
        vector=vector+np.random.default_rng(812).normal(size=150)*np.tile([.002,.001,.0003,.005,.006],30)
    provider=DerivativeProvider(problem,env)
    result=provider.values(vector);expected=problem.evaluate(vector)
    for key in expected:
        np.testing.assert_allclose(result[key],expected[key],atol=2e-9,rtol=2e-9,err_msg=key)
    assert result['equality'].shape==(30,)
    assert result['inequality'].shape==(903,)


def test_complete_actual_vector_chart_derivatives_and_geometry_chain(ready):
    problem,vector,provider=ready
    vector=vector+np.random.default_rng(833).normal(size=150)*np.tile([.003,.002,.002,.003,.002],30)
    for key,derivative in [('objective',provider.objective_gradient),('equality',provider.equality_jacobian),
                           ('inequality',provider.inequality_jacobian)]:
        supplied=derivative(vector)
        reference=central_jacobian(lambda z:problem.evaluate(z)[key],vector)
        np.testing.assert_allclose(supplied,reference,atol=2e-5,rtol=3e-5,err_msg=key)
    jac=provider.inequality_jacobian(vector)
    assert np.count_nonzero(jac[-180:-90])>0 and np.count_nonzero(jac[-90:])>0
    assert provider.geometry_metadata(vector)['workspace']['family']=='workspace'


def test_all_callbacks_share_cache_and_warmup_is_not_free_solve_work(ready):
    _,vector,provider=ready
    provider.warmup(vector)
    provider.reset_stats(clear_cache=True)
    provider.objective_gradient(vector)
    provider.equality_jacobian(vector.copy())
    provider.inequality_jacobian(vector.copy())
    statistics=provider.stats();runtime=statistics['phases']['runtime']
    assert runtime['cache_misses']==runtime['core_ad_evaluations']==1
    assert runtime['cache_hits']==2
    assert statistics['callback_calls']['objective_gradient']==1
    assert statistics['callback_calls']['equality_jacobian']==1
    assert statistics['callback_calls']['inequality_jacobian']==1
    assert statistics['compile_count']==1
    assert statistics['compile_wall_s']>0 and statistics['warmup_wall_s']>0
    assert statistics['numerical_finite_difference_calls']==0 and not statistics['numerical_fallback_used']


def test_returned_arrays_cannot_corrupt_derivative_cache(ready):
    _,vector,provider=ready
    original=provider.objective_gradient(vector)
    changed=provider.objective_gradient(vector);changed[:]=np.nan
    np.testing.assert_array_equal(provider.objective_gradient(vector),original)
    values=provider.values(vector);values['equality'][:]=1e9
    assert np.max(np.abs(provider.values(vector)['equality']))<1e-10


def test_m2_omits_only_obstacle_rows():
    m2,z,env=fixture(obstacles=False);m3,_,_=fixture(obstacles=True)
    a,b=DerivativeProvider(m2,env),DerivativeProvider(m3,env)
    values2,values3=a.values(z),b.values(z)
    assert a.inequality_count==813 and b.inequality_count==903
    np.testing.assert_array_equal(a.objective_gradient(z),b.objective_gradient(z))
    np.testing.assert_array_equal(a.equality_jacobian(z),b.equality_jacobian(z))
    np.testing.assert_array_equal(a.inequality_jacobian(z),b.inequality_jacobian(z)[:813])
    np.testing.assert_array_equal(values2['inequality'],values3['inequality'][:813])


@pytest.mark.parametrize('angle',[0.,1e-12,-1e-12,.999e-4,1.001e-4,-1.001e-4,.999e-3,1.001e-3,-1.001e-3])
def test_taylor_branches_and_zero_have_finite_correct_local_derivatives(angle):
    xi=np.array([.27,-.06,angle])
    for jax_function,np_function,threshold in [(ad.se2_exp,original_se2.se2_exp,1e-4),(ad.right_jacobian,original_gp.right_jacobian,1e-3)]:
        np.testing.assert_allclose(np.asarray(jax_function(jnp.asarray(xi))),np_function(xi),atol=1e-12,rtol=1e-10)
        derivative=np.asarray(jax.jacfwd(jax_function)(jnp.asarray(xi)))
        assert np.isfinite(derivative).all()
        # Stay on the specified branch. An unnecessarily tiny 1e-8 step at
        # |omega|=.001001 amplifies cancellation in the original non-Taylor
        # Jr evaluation (~1.5e-4 FD error); the independent Fréchet test below
        # separately validates AD without that subtractive difference.
        step=min(1e-7,.1*abs(abs(angle)-threshold))
        reference=central_jacobian(np_function,xi,step=step)
        np.testing.assert_allclose(derivative,reference,atol=2e-5,rtol=2e-4)


@pytest.mark.parametrize('angle',[0.,1e-12,.999e-4,1.001e-4,.999e-3,1.001e-3,-1.001e-3])
def test_right_jacobian_derivative_against_independent_matrix_frechet(angle):
    from scipy.linalg import expm_frechet
    xi=np.array([.27,-.06,angle])
    # exp([[-ad_xi,I],[0,0]]) top-right equals integral_0^1 exp(-s ad_xi)ds.
    # Its Fréchet derivative supplies an independent stable entire-function
    # reference without GP coefficients, branch formulas, or finite differences.
    matrix=np.zeros((6,6));matrix[:3,:3]=-original_gp.adjoint_algebra(xi)
    matrix[:3,3:]=np.eye(3)
    columns=[]
    for axis in np.eye(3):
        direction=np.zeros((6,6));direction[:3,:3]=-original_gp.adjoint_algebra(axis)
        columns.append(expm_frechet(matrix,direction,compute_expm=False)[:3,3:])
    independent=np.stack(columns,axis=-1)
    automatic=np.asarray(jax.jacfwd(ad.right_jacobian)(jnp.asarray(xi)))
    np.testing.assert_allclose(automatic,independent,atol=8e-8,rtol=2e-6)


def test_world_yaw_wrap_and_rotation_only_primal_and_derivatives():
    boundary=np.array([8.,-12.,np.pi-.002]);twist=np.array([0.,0.,.2])
    times=np.linspace(0.,3.,31)
    poses=original_se2.compose_poses(boundary,original_se2.se2_exp(times[:,None]*twist))
    env=AffineEnvironment()
    problem=GPProblem(boundary,twist,poses[1:],poses[-1],{},lambda xy:env.obstacle(xy)[0],lambda xy:env.workspace(xy)[0])
    vector=problem.vector(poses,np.tile(twist,(31,1)))
    vector=vector+np.random.default_rng(5).normal(size=150)*1e-4
    provider=DerivativeProvider(problem,env)
    for key in ('objective','equality','inequality'):
        np.testing.assert_allclose(provider.values(vector)[key],problem.evaluate(vector)[key],atol=2e-9,rtol=2e-9)
    direction=np.random.default_rng(9).normal(size=150);direction/=np.linalg.norm(direction)
    eps=2e-6
    fd=(problem.evaluate(vector+eps*direction)['inequality']-problem.evaluate(vector-eps*direction)['inequality'])/(2*eps)
    np.testing.assert_allclose(provider.inequality_jacobian(vector)@direction,fd,atol=3e-5,rtol=3e-5)


def test_full_acceleration_includes_directional_right_jacobian_derivative():
    z=np.array([.4,-.2,.7,.3,.02,.4,.435,-.171,.744,.34,-.01,.48])
    def function(vector,module):
        _,_,acceleration=module.interpolate_interval(vector[:3],vector[3:6],vector[6:9],vector[9:],.1,.37)
        return acceleration
    analytic=np.asarray(jax.jacfwd(lambda x:function(x,ad))(jnp.asarray(z)))
    numerical=central_jacobian(lambda x:function(x,original_gp),z,step=2e-6)
    np.testing.assert_allclose(function(z,ad),function(z,original_gp),atol=2e-12,rtol=2e-12)
    np.testing.assert_allclose(analytic,numerical,atol=2e-5,rtol=2e-5)
    assert np.max(np.abs(analytic[:,[2,5,8,11]]))>1.


def test_rejects_unsupported_gates_nonfinite_and_environment_failure(ready):
    problem,vector,provider=ready
    saved=copy.copy(problem);saved.gates=[{'unsupported':'gate'}]
    with pytest.raises(DerivativeError,match='gate'):
        DerivativeProvider(saved,AffineEnvironment())
    with pytest.raises(DerivativeError,match='finite'):
        provider.objective_gradient(np.full(150,np.nan))
    with pytest.raises(DerivativeError,match='analytic workspace'):
        DerivativeProvider(problem,None)
    class Broken(AffineEnvironment):
        def workspace(self,xy):
            raise ValueError('unsupported geometry')
    broken=DerivativeProvider(problem,Broken())
    with pytest.raises(DerivativeError,match='unsupported geometry'):
        broken.values(vector)


def test_provider_has_no_finite_difference_fallback_code():
    import reconciliation.gp_se2_diag02_derivatives as provider_module
    source=inspect.getsource(provider_module)
    assert 'approx_derivative(' not in source and 'gp_factor_jacobian(' not in source
    assert 'scipy.optimize' not in source


def test_runtime_branch_counts_locations_and_bounded_unique_records(ready):
    _,_,provider=ready
    provider.reset_stats(clear_cache=True)
    xy=np.column_stack([np.arange(150,dtype=float),np.full(150,2.)])
    metadata={'points':[dict(smooth=False,invalid_query=i%2==0,kind='test_boundary',selected_gradient=[0.,1.])
                        for i in range(150)]}
    provider._record_geometry('runtime','obstacle',xy,metadata)
    provider._record_geometry('runtime','obstacle',xy,metadata)
    report=provider.stats()['geometry_branches']['runtime'];family=report['families']['obstacle']
    assert family['query_calls']==2 and family['point_evaluations']==300
    assert family['nonsmooth_points']==300 and family['invalid_points']==150
    assert family['branch_counts']=={'test_boundary':300}
    assert report['distinct_flagged_locations']==150 and len(report['samples'])==128
    assert report['omitted_distinct_locations']==22
    assert report['samples'][0]['world_xy_m']==[0.,2.]
    assert report['samples'][0]['metadata']['selected_gradient']==[0.,1.]
    provider.reset_stats(clear_cache=True)
    assert provider.stats()['geometry_branches']['runtime']['samples']==[]
    assert provider.stats()['geometry_branches']['runtime']['families']=={}


@pytest.mark.parametrize('family',['fresh_relative_log','adjacent_relative_log','goal_yaw'])
def test_relative_and_goal_log_cuts_are_explicitly_unsupported(family):
    problem,vector,env=fixture('S0')
    if family=='fresh_relative_log':
        vector=vector.copy();vector[12]=np.pi
    elif family=='adjacent_relative_log':
        poses,twists=problem.unpack(vector)
        poses[7,2]=original_se2.wrap_angle(poses[6,2]+np.pi)
        problem.common_reference[6,2]=poses[7,2]
        problem._anchors[7,2]=poses[7,2]
        vector=problem.vector(poses,twists)
    else:
        problem.goal_pose[2]=original_se2.wrap_angle(problem.goal_pose[2]+np.pi)
    provider=DerivativeProvider(problem,env)
    with pytest.raises(DerivativeError,match='unsupported relative/goal yaw wrap cut') as caught:
        provider.warmup(vector)
    assert caught.value.reason_code=='UNSUPPORTED_WRAP_CUT'
    assert family in {row['family'] for row in caught.value.diagnostics['cuts']}
    assert caught.value.diagnostics['cut_tolerance_rad']==1e-12
    assert provider.stats()['compile_count']==0
    assert provider.stats()['phases']['warmup']['core_ad_evaluations']==0
    with pytest.raises(DerivativeError) as repeated:
        provider.values(vector)
    assert repeated.value.reason_code=='UNSUPPORTED_WRAP_CUT'


def test_near_but_supported_relative_log_yaw_is_not_rejected():
    problem,vector,env=fixture('S0');provider=DerivativeProvider(problem,env)
    vector=vector.copy();vector[12]=np.pi-1e-8
    provider._guard_wrap_cuts(vector,'runtime')
    assert not provider.stats()['unsupported_wrap_cut_events']
