
import os, sys
# import Python Libs and add Path for the Dll Files 
#sys.path.append("/opt/hfs20.5.445/houdini/python3.10libs")
sys.path.append("C:\\Program Files\\Side Effects Software\\Houdini 20.5.445\\houdini\\python3.10libs")
os.add_dll_directory("C:\\Program Files\\Side Effects Software\\Houdini 20.5.445\\bin")

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
import omni.isaac.core.utils.stage as stage_utils
import omni.ui as ui
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, PhysxSchema
import hou
import hapi
from hei.he_manager import HoudiniEngineManager

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

my_window = ui.Window("My Extension's Window", width=400, height=400, dockPreference=ui.DockPreference.RIGHT_BOTTOM)
my_window.deferred_dock_in("Property", ui.DockPolicy.TARGET_WINDOW_IS_ACTIVE)
num_env = 0
def main():
    he_manager = HoudiniEngineManager()
    if not he_manager.startSession(1, he_manager.DEFAULT_NAMED_PIPE, he_manager.DEFAULT_TCP_PORT):
        print("ERROR: Failed to create a Houdini Engine session.")
        return
    if not he_manager.initializeHAPI(False):
        print("ERROR: Failed to initialize HAPI.")
        return
    otl_path = "{}/hda/cabinet.hda".format(os.getcwd())
    def cookNode():
        global num_env

        node_id = he_manager.loadAsset(otl_path)
        if node_id is None:
            print("Failed to load the default HDA.")
            return
        parms = he_manager.getParameters(node_id)
        print("Ready to set parameters")        
        he_manager.setParameters(node_id, {"Width":1.5})
        print("Ready to cook")
        
        outputs = he_manager.cookNode(node_id)
    #   he_manager.getAttributes(outputs[0])
    #   he_manager.getAttributes(outputs[1])

        bodies = he_manager.readPoints(outputs[0])
        joints = he_manager.readPoints(outputs[1])
        num_bodies = len(bodies["P"])
        num_joints = len(joints["P"])

        # Create a tempory stage in memory
        usd_name = f'cabinet_{num_env}.usda'
        stage = Usd.Stage.CreateInMemory(usd_name)

        # Enable physics
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
        # Set gravity
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        # Set solver settings
        PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
        physxSceneAPI.CreateEnableCCDAttr(True)
        physxSceneAPI.CreateEnableStabilizationAttr(True)
        physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
        physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
        physxSceneAPI.CreateSolverTypeAttr("TGS")


        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        root = stage.DefinePrim('/cabinet', 'Xform')
        UsdPhysics.ArticulationRootAPI.Apply(root)
    #    UsdPhysics.MassAPI.Apply(prim)
        for i in range(num_bodies):
            name = bodies["name"][i]
            pos = bodies["P"][i]
            scale = bodies["scale"][i]
            orient = bodies["orient"][i]
            prim_name = f'{name}_{i}'
            prim = stage.DefinePrim(f'/cabinet/{prim_name}', 'Xform')
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim)
            UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(pos[2], pos[0], pos[1]))
            UsdGeom.Xformable(prim).AddOrientOp().Set(Gf.Quatf(orient[3], orient[2], orient[0], orient[1]))
            UsdGeom.Xformable(prim).AddScaleOp().Set(Gf.Vec3d(scale[2], scale[0], scale[1]))
            matrix = UsdGeom.Xformable(prim).GetLocalTransformation()

        #   xform_api = UsdGeom.XformCommonAPI(prim)
        #   xform_api.SetTranslate(Gf.Vec3d(0, 0, 0))

            visual = stage.DefinePrim(f'/cabinet/{prim_name}/visuals')
            visual.SetInstanceable(True)
            collision = stage.DefinePrim(f'/cabinet/{prim_name}/collisions')
            collision.SetInstanceable(True)
            visual.GetReferences().AddReference('./sektion_cabinet_visuals.usd', f'/{name}_visuals')
            collision.GetReferences().AddReference('./sektion_cabinet_collisions.usd', f'/{name}_collisions')

        for i in range(num_joints):
            name = joints["name"][i]
            pos = joints["P"][i]
            orient = joints["orient"][i]
            quat = Gf.Quatf(orient[3], orient[2], orient[0], orient[1])
            body_index_0 = joints["bodies"][i][0]
            body_index_1 = joints["bodies"][i][1]
            body_name_0 = f'/{bodies["name"][body_index_0]}_{body_index_0}' if body_index_0 >= 0 else ''
            body_name_1 = f'/{bodies["name"][body_index_1]}_{body_index_1}' if body_index_1 >= 0 else ''
            joint_name = f'/cabinet{body_name_0}/{name}_{i}'
            if name == "Revolute":
                joint = UsdPhysics.RevoluteJoint.Define(stage, joint_name)
            elif name == "Prismatic":
                joint = UsdPhysics.PrismaticJoint.Define(stage, joint_name)
            elif name == "Fixed":
                joint = UsdPhysics.FixedJoint.Define(stage, joint_name)
            if body_index_0 >= 0:
                rel_0 = joint.CreateBody0Rel()
                rel_0.AddTarget(f'/cabinet{body_name_0}')
                mat_0 = UsdGeom.Xformable(stage.GetPrimAtPath(f'/cabinet{body_name_0}')).GetLocalTransformation().GetInverse()
                joint.GetLocalPos0Attr().Set(mat_0.Transform(Gf.Vec3d(pos[2], pos[0], pos[1])))
                joint.GetLocalRot0Attr().Set(Gf.Quatf(mat_0.ExtractRotationQuat()) * quat)
            if body_index_1 >= 0:
                rel_1 = joint.CreateBody1Rel()
                rel_1.AddTarget(f'/cabinet{body_name_1}')
                mat_1 = UsdGeom.Xformable(stage.GetPrimAtPath(f'/cabinet{body_name_1}')).GetLocalTransformation().GetInverse()
                joint.GetLocalPos1Attr().Set(mat_1.Transform(Gf.Vec3d(pos[2], pos[0], pos[1])))
                joint.GetLocalRot1Attr().Set(Gf.Quatf(mat_1.ExtractRotationQuat()) * quat)

        # Save the resulting layer
        stage.GetRootLayer().defaultPrim = "cabinet"
        stage.GetRootLayer().Export(usd_name)
        stage_utils.open_stage(usd_name)
    #   stage_utils.add_reference_to_stage(usd_name, f'/World/env_{num_env}')
    #   prim = Articulation(prim_path=f'/World/env_{num_env}', name="env", position=np.array([num_env*2, 0, 0.5]))
    #   num_env +=1

        he_manager.unloadAsset(node_id)

    with my_window.frame:
        ui.Button("Cook", clicked_fn=cookNode)

    while simulation_app.is_running():
        simulation_app.update()
    simulation_app.close()

main()
