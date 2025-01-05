
import os, sys
# import Python Libs and add Path for the Dll Files 
sys.path.append("/opt/hfs20.5.445/houdini/python3.10libs")
#sys.path.append("C:\\Program Files\\Side Effects Software\\Houdini 20.5.445\\houdini\\python3.10libs")
#os.add_dll_directory("C:\\Program Files\\Side Effects Software\\Houdini 20.5.445\\bin")

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from pxr import Gf, Vt, Usd, UsdGeom

import hou
import hapi
from hei.he_manager import HoudiniEngineManager

def main():
    he_manager = HoudiniEngineManager()
    if not he_manager.startSession(1, he_manager.DEFAULT_NAMED_PIPE, he_manager.DEFAULT_TCP_PORT):
        print("ERROR: Failed to create a Houdini Engine session.")
        return
    if not he_manager.initializeHAPI(True):
        print("ERROR: Failed to initialize HAPI.")
        return
    otl_path = "{}/hda/cabinet.hda".format(os.getcwd())
    node_id = he_manager.loadAsset(otl_path)
    if node_id is None:
        print("Failed to load the default HDA.")
        return
    parms = he_manager.getParameters(node_id)
#   he_manager.setParameters(node_id, {"rounded_tile":False})
    hexagona_cook = he_manager.cookNode(node_id)
#   he_manager.getAttributes(hexagona_node_id, hexagona_part_id)




    # Create a tempory stage in memory
    stage = Usd.Stage.CreateInMemory('SampleLayer.usd')

    # Create a transform and add a sphere as mesh data
    plane_mesh = UsdGeom.Mesh.Define(stage, "/proc_mesh")


    points, indices = he_manager.readGeometry(node_id)
    points_pxr = Vt.Vec3fArray(len(points) // 3)
    indices_pxr = Vt.IntArray(len(indices))
    for i in range(len(points) // 3):
        points_pxr[i] = Gf.Vec3f(points[i*3+2], points[i*3], points[i*3+1])
    plane_mesh.GetPointsAttr().Set(points_pxr)
    plane_mesh.GetFaceVertexIndicesAttr().Set(indices)
    plane_mesh.GetFaceVertexCountsAttr().Set([3] * (len(indices) // 3))



    # Save the resulting layer
    stage.GetRootLayer().Export('SampleLayer.usd')

main()
