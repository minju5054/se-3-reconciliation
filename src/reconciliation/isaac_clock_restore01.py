"""Bounded pre-clock initialization shared by the real Isaac worker and tests."""


def prime_clock(world, B_sim_s, dt):
    """Zero-motion World steps restore saved time without assigning a fake clock."""
    start = float(world.current_time)
    n = int(round((B_sim_s-start)/dt))
    if n < 0 or n > 1000 or abs(start+n*dt-B_sim_s)>1e-10:
        raise ValueError('cannot restore saved World clock by exact physics steps')
    for _ in range(n):
        before = float(world.current_time)
        world.step(render=False)
        if abs(float(world.current_time)-before-dt)>1e-10:
            raise ValueError('Isaac initialization dt mismatch')
    assert abs(float(world.current_time)-B_sim_s)<=1e-10
    return dict(initial_clock_s=start,zero_motion_initialization_steps=n,restored_clock_s=float(world.current_time))
