import os, sys
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
from pxr import Gf, Vt, Usd, UsdGeom
from typing import List, Type

stage_ref = Usd.Stage.Open('d:/4.2/Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd')
stage_ref.GetRootLayer().Export('scene.usda')

layer_name = stage_ref.GetRootLayer().GetDisplayName().rsplit('.', 1)[0]
os.mkdir("./"+layer_name)

def find_prims_by_type(stage: Usd.Stage, prim_type: Type[Usd.Typed]) -> List[Usd.Prim]:
    found_prims = [prim_type(x) for x in stage.TraverseAll() if x.IsA(prim_type)]
    return found_prims
meshes = find_prims_by_type(stage_ref, UsdGeom.Mesh)

for mesh in meshes:
    points = mesh.GetPointsAttr().Get()
    indices = mesh.GetFaceVertexIndicesAttr().Get()
    with open(f'./{layer_name}/{str(mesh.GetPath().GetParentPath())}.obj', 'w') as file:
        for point in points:
            file.write(f'v {point[1]:.4f} {point[2]:.4f} {point[0]:.4f}\n')
        for i in range(0, len(indices), 3):
            file.write(f'f {indices[i]+1} {indices[i+1]+1} {indices[i+2]+1}\n')
        file.close()




