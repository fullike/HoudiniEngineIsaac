
import os, sys
# import Python Libs and add Path for the Dll Files 
# sys.path.append("/opt/hfs20.5.445/houdini/python3.10libs")
sys.path.append("C:\\Program Files\\Side Effects Software\\Houdini 20.5.445\\houdini\\python3.10libs")
os.add_dll_directory("C:\\Program Files\\Side Effects Software\\Houdini 20.5.445\\bin")



import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})





from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from pxr import Gf, Vt, UsdGeom

my_world = World(stage_units_in_meters=1.0)

import hou
import hapi
from hei.he_geometry import HoudiniEngineGeometry
from hei.he_manager import HoudiniEngineManager, SessionType

def launch_hei():
    he_manager = HoudiniEngineManager()
    if not he_manager.startSession(1, he_manager.DEFAULT_NAMED_PIPE, he_manager.DEFAULT_TCP_PORT):
        print("ERROR: Failed to create a Houdini Engine session.")
        return
    if not he_manager.initializeHAPI(True):
        print("ERROR: Failed to initialize HAPI.")
        return
    otl_path = "{}/hda/hexagona_lite.hda".format(os.getcwd())
    node_id = he_manager.loadAsset(otl_path)
    if node_id is None:
        print("Failed to load the default HDA.")
        return
#   parms = he_manager.getParameters(node_id)
    he_manager.setParameters(node_id, {"rounded_tile":False})
    hexagona_cook = he_manager.cookNode(node_id)
#   he_manager.getAttributes(hexagona_node_id, hexagona_part_id)
    plane_mesh = UsdGeom.Mesh.Define(simulation_app.context.get_stage(), "/proc_mesh")
    points, indices = he_manager.readGeometry(node_id)
    points_pxr = Vt.Vec3fArray(len(points) // 3)
    indices_pxr = Vt.IntArray(len(indices))
    for i in range(len(points) // 3):
        points_pxr[i] = Gf.Vec3f(points[i*3+2], points[i*3], points[i*3+1])
    plane_mesh.GetPointsAttr().Set(points_pxr)
    plane_mesh.GetFaceVertexIndicesAttr().Set(indices)
    plane_mesh.GetFaceVertexCountsAttr().Set([3] * (len(indices) // 3))

launch_hei()
cube_1 = my_world.scene.add(
    VisualCuboid(
        prim_path="/new_cube_1",
        name="visual_cube",
        position=np.array([0, 0, 0.5]),
        size=0.3,
        color=np.array([255, 255, 255]),
    )
)

cube_2 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/new_cube_2",
        name="cube_1",
        position=np.array([0, 0, 1.0]),
        scale=np.array([0.6, 0.5, 0.2]),
        size=1.0,
        color=np.array([255, 0, 0]),
    )
)

cube_3 = my_world.scene.add(
    DynamicCuboid(
        prim_path="/new_cube_3",
        name="cube_2",
        position=np.array([0, 0, 3.0]),
        scale=np.array([0.1, 0.1, 0.1]),
        size=1.0,
        color=np.array([0, 0, 255]),
        linear_velocity=np.array([0, 0, 0.4]),
    )
)

my_world.scene.add_default_ground_plane()
my_world.reset()
while simulation_app.is_running():
    simulation_app.update()
simulation_app.close()