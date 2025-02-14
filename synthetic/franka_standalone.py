# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse

import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, XFormPrim
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from isaacsim.robot.policy.examples.robots import H1FlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path
# parser = argparse.ArgumentParser(description="Define the number of robots.")
# parser.add_argument("--num-robots", type=int, default=1, help="Number of robots (default: 1)")
# parser.add_argument(
#     "--env-url",
#     default="/Isaac/Environments/Grid/default_environment.usd",
#     required=False,
#     help="Path to the environment url",
# )
# args = parser.parse_args()
# print(f"Number of robots: {args.num_robots}")

first_step = True
reset_needed = False

# initialize robot on first step, run robot advance
def on_physics_step(step_size) -> None:
    pass

# spawn world
#my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=8 / 200)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

# spawn warehouse scene
prim = define_prim("/World/Ground", "Xform")
asset_path = assets_root_path + "/NVIDIA/Assets/ArchVis/Residential/Furniture/FurnitureSets/Appleseed/Appleseed_LivingRoom_Var01.usd"
prim.GetReferences().AddReference(asset_path)
XFormPrim("/World/Ground", name="/World/Ground", scales=np.array([[0.01, 0.01, 0.01]]))
#prim.GetAttribute("xformOp:scale").Set(Gf.Vec3f(0.01, 0.01, 0.01))
#prim.set_local_scale(np.array([0.01, 0.01, 0.01]))

prim = define_prim("/World/Franka", "Xform")
prim.GetReferences().AddReference(assets_root_path + "/Isaac/Robots/Franka/franka.usd")
robot = SingleArticulation(prim_path="/World/Franka", name="Franka")

# robot command
base_command = np.zeros(3)

i = 0
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
