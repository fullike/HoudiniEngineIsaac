
import os, sys
# import Python Libs and add Path for the Dll Files 
sys.path.append("/opt/hfs20.5.445/houdini/python3.10libs")
#sys.path.append("C:\\Program Files\\Side Effects Software\\Houdini 20.5.445\\houdini\\python3.10libs")
#os.add_dll_directory("C:\\Program Files\\Side Effects Software\\Houdini 20.5.445\\bin")

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from pxr import Gf, Vt, Usd, UsdGeom, UsdPhysics

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
    he_manager.setParameters(node_id, {"Width":1.5})
    outputs = he_manager.cookNode(node_id)

#   he_manager.getAttributes(outputs[0])
#   he_manager.getAttributes(outputs[1])

    bodies = he_manager.readPoints(outputs[0])
    joints = he_manager.readPoints(outputs[1])
    num_bodies = len(bodies["P"])
    num_joints = len(joints["P"])

    # Create a tempory stage in memory
    stage = Usd.Stage.CreateInMemory('SampleLayer.usd')
    stage.DefinePrim('/cabinet', 'Xform')
    for i in range(num_bodies):
        name = bodies["name"][i]
        prim_name = f'{name}_{i}'
        prim = stage.DefinePrim(f'/cabinet/{prim_name}', 'Xform')
        visual = stage.DefinePrim(f'/cabinet/{prim_name}/visuals')
        collision = stage.DefinePrim(f'/cabinet/{prim_name}/collisions')
        visual.GetReferences().AddReference('./sektion_cabinet_visuals.usd', f'/{name}_visuals')
        collision.GetReferences().AddReference('./sektion_cabinet_collisions.usd', f'/{name}_collisions')
    for i in range(num_joints):
        name = joints["name"][i]
        body_index = joints["bodies"][i][0]
        parent_name = f'{bodies["name"][body_index]}_{body_index}'
        joint_name = f'/cabinet/{parent_name}/{name}_{i}'
        if name == "Revolute":
            joint = UsdPhysics.RevoluteJoint.Define(stage, joint_name)
        elif name == "Prismatic":
            joint = UsdPhysics.PrismaticJoint.Define(stage, joint_name)
        elif name == "Fixed":
            joint = UsdPhysics.FixedJoint.Define(stage, joint_name)


    # Save the resulting layer
    stage.GetRootLayer().Export('SampleLayer.usda')

main()
