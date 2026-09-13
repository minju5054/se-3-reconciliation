"""Hospital runtime using the unchanged Stage 0-E follower/execution loop."""

from __future__ import annotations

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading
from isaacsim.robot.experimental.wheeled_robots.controllers import DifferentialController
from isaacsim.storage.native import get_assets_root_path, resolve_asset_path

from closed_loop_execution_runtime import ClosedLoopRuntime
from lightnav_stage0c_runtime import (
    discover_wheels, find_articulation_root, quaternion_from_yaw,
    resolve_jackal_asset, suppress_sensor_viewport_visualization,
)


class HospitalExecutionRuntime(ClosedLoopRuntime):
    def __init__(self, app, source, candidate, first_pose, *, render=False, real_time_factor=None):
        simulation = source["simulation"]
        self.simulation_app = app
        self.physics_dt = float(simulation["physics_dt"])
        self.control_dt = float(simulation["control_dt"])
        self.physics_steps = int(round(self.control_dt / self.physics_dt))
        if not np.isclose(self.physics_steps * self.physics_dt, self.control_dt, atol=1e-12, rtol=0):
            raise ValueError("control interval must span whole physics steps")
        if real_time_factor is not None and (not np.isfinite(real_time_factor) or real_time_factor <= 0):
            raise ValueError("real time factor must be positive and finite")
        self.spawn_height = float(simulation["spawn_height_m"])
        self.render = render
        self.real_time_factor = real_time_factor
        self.parameters = dict(candidate["selected_parameters"])
        self.controller_config = dict(candidate["runtime_controller_config"])
        self.maximum_wheel_target = float(self.controller_config["maximum_abs_wheel_target_rad_s"])
        self.asset = resolve_jackal_asset(source["robot"])
        self.world = World(physics_dt=self.physics_dt, rendering_dt=self.physics_dt,
                           stage_units_in_meters=1.0)
        relative = source["environment"]["asset_relative_path"]
        self.hospital = resolve_asset_path(relative) or resolve_asset_path(
            str(get_assets_root_path()).rstrip("/") + relative)
        if not self.hospital:
            raise RuntimeError("source Hospital asset is unavailable")
        add_reference_to_stage(str(self.hospital), source["environment"]["reference_prim_path"])
        reference = source["robot"]["reference_prim_path"]
        add_reference_to_stage(str(self.asset["resolved_path"]), reference)
        while is_stage_loading():
            app.update()
        self.articulation_root = find_articulation_root(reference)
        self.robot = self.world.scene.add(SingleArticulation(
            self.articulation_root, name="exp02d_execution_jackal",
            position=np.array([first_pose[0], first_pose[1], self.spawn_height]),
            orientation=quaternion_from_yaw(float(first_pose[2])),
        ))
        self.world.reset()
        self.wheels = discover_wheels(self.robot, self.articulation_root)
        self.differential = DifferentialController(wheel_radius=float(self.wheels["radius_m"]),
                                                   wheel_base=float(self.wheels["separation_m"]))
        self.articulation = self.robot.get_articulation_controller()
        self.sensor_display = suppress_sensor_viewport_visualization(reference)


def loop_configs(source, maximum_duration):
    """Adapt explicit source fields to the existing loop without changing values."""
    simulation = source["simulation"]
    loop = {"simulation": {"settling_duration_s": simulation["settling_duration_s"]}}
    follower = {"closed_loop": {**source["follower"], "maximum_duration_s": maximum_duration}}
    return loop, follower
