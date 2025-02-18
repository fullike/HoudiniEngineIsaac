"""This module provides the LightPropertyExtension class for adding a custom light properties widget to the Omniverse Kit."""

import omni.ext
import omni.kit.app
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, PhysxSchema
import omni.ui as ui
from omni.physx.scripts import utils
from omni.kit.property.usd.usd_property_widget import UsdPropertyUiEntry, UsdPropertiesWidget
from omni.kit.property.usd.custom_layout_helper import CustomLayoutFrame, CustomLayoutProperty
from isaacsim.core.utils.stage import add_reference_to_stage, open_stage

import os, sys
# import Python Libs and add Path for the Dll Files 
if sys.platform == 'linux':
    sys.path.append("/opt/hfs20.5.445/houdini/python3.10libs")
elif sys.platform == 'win32':
    sys.path.append("C:\\Program Files\\Side Effects Software\\Houdini 20.5.445\\houdini\\python3.10libs")
    os.add_dll_directory("C:\\Program Files\\Side Effects Software\\Houdini 20.5.445\\bin")
os.environ['HOUDINI_ASYNCIO'] = "0"
import hou
import hapi
from hda_loader.houdini_engine import HoudiniEngineManager

class HDASchemaAttributesWidget(UsdPropertiesWidget):
    def __init__(self, *args, **kwargs):
        UsdPropertiesWidget.__init__(self, *args, **kwargs)
        self.he_manager = HoudiniEngineManager()
        if not self.he_manager.startSession(1, self.he_manager.DEFAULT_NAMED_PIPE, self.he_manager.DEFAULT_TCP_PORT):
            print("ERROR: Failed to create a Houdini Engine session.")
        if not self.he_manager.initializeHAPI(False):
            print("ERROR: Failed to initialize HAPI.")        
    def clean(self):
        self.he_manager.stopSession()
        UsdPropertiesWidget.clean(self)

    def cookNode(self):
        outputs = self.he_manager.cookNode(self.node_id)
    #   he_manager.getAttributes(outputs[0])
    #   he_manager.getAttributes(outputs[1])

        bodies = self.he_manager.readPoints(outputs[0])
        joints = self.he_manager.readPoints(outputs[1])
        num_bodies = len(bodies["P"])
        num_joints = len(joints["P"])

        # Create a tempory stage in memory
        usd_name = 'cabinet.usda'
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
    #   open_stage(usd_name)

        current_prim = self._get_prim(self._payload[-1])
        current_prim.GetReferences().AddReference(usd_name)

    def build_parms_ui(self, frame, parms):
        with frame:
            with ui.VStack(height=0, spacing=5):
                for name, value in parms.items():
                    if isinstance(value, int):
                        with ui.HStack(spacing=8):
                            ui.Label(name,width=24)
                            widget = ui.IntField(width=ui.Fraction(1))
                            widget.model.set_value(value)
                            widget.model.add_end_edit_fn(lambda m, name=name:self.he_manager.setParameters(self.node_id, {name:m.as_int}))
                    elif isinstance(value, float):
                        with ui.HStack(spacing=8):
                            ui.Label(name,width=24)
                            widget = ui.FloatField(width=ui.Fraction(1))
                            widget.model.set_value(value)
                            widget.model.add_end_edit_fn(lambda m, name=name:self.he_manager.setParameters(self.node_id, {name:m.as_float}))
                    elif isinstance(value, str):
                        with ui.HStack(spacing=8):
                            ui.Label(name,width=24)
                            widget = ui.StringField(width=ui.Fraction(1))
                            widget.model.set_value(value)
                            widget.model.add_end_edit_fn(lambda m, name=name:self.he_manager.setParameters(self.node_id, {name:m.as_string}))         
                    elif isinstance(value, dict):
                        self.build_parms_ui(ui.CollapsableFrame(title=name), value)

    def on_file_changed(self, hda_path):
        self.node_id = self.he_manager.loadAsset(hda_path)
        if self.node_id is None:
            print("Failed to load the default HDA.")
            return
        self.file_input.model.set_value(hda_path)
        parms = self.he_manager.getParameters(self.node_id)
        self.build_parms_ui(self.frame, parms)

    def on_pick_file(self):
        from omni.kit.widget.filebrowser import FileBrowserItem
        from omni.kit.window.filepicker import FilePickerDialog
        def on_apply(dialog, dirname, filename):
            self.on_file_changed(dirname + filename)
            dialog.hide()
        def on_filter_hda_files(item: FileBrowserItem) -> bool:
            return not item or os.path.splitext(item.path)[1] == ".hda"
        def on_selection_changed(dialog, items: [FileBrowserItem] = []):
            dialog._widget.file_bar.enable_apply_button(enable=os.path.splitext(items[-1].path)[1] == ".hda")            
        dialog = FilePickerDialog(
            "Select HDA File",
            allow_multi_selection=False,
            apply_button_label="Select",
            item_filter_options=["HDA Files (*.hda)"],
            item_filter_fn=on_filter_hda_files,
            selection_changed_fn=lambda items:on_selection_changed(dialog, items),
            click_apply_handler=lambda filename, dirname:on_apply(dialog, dirname, filename),
            click_cancel_handler=lambda filename, dirname: dialog.hide())
        dialog.set_current_directory(os.getcwd())
        dialog.navigate_to(os.getcwd())
        dialog.refresh_current_directory()
        
    def _customize_props_layout(self, attrs):
        print(attrs)
        frame = CustomLayoutFrame(hide_extra=False)
        with frame:
            with ui.VStack(height=0, spacing=5):
                with ui.HStack(spacing=8):
                    ui.Label("HDA File:",width=24)
                    self.file_input = ui.StringField(width=ui.Fraction(1))
                    self.file_input.model.add_end_edit_fn(lambda text: self.on_file_changed(text.as_string))
                    ui.Button("...", clicked_fn=self.on_pick_file, width=24)
                self.frame = ui.CollapsableFrame(title="HDA Parameters")
                ui.Button("Cook", clicked_fn=self.cookNode)
        return []       
    #   return frame.apply(attrs)

class HDAPropertyExtension(omni.ext.IExt):
    """A class that extends the Omniverse Kit by registering a custom widget for light properties.

    This class is an extension for the Omniverse Kit, designed to enhance the user interface by adding a custom widget that allows for easy manipulation of light properties in a 3D scene. It leverages USD and UsdLux for defining and managing these properties. Upon startup, the class registers the widget and sets up a path for test data. On shutdown, it ensures the widget is properly unregistered.

    It is important to note that the visibility of this widget is contingent on the presence of the property window within the Omniverse Kit. The widget is specifically tailored for light primitives and includes a comprehensive set of light attributes for various light types supported by UsdLux. The class also handles version-specific features of USD, ensuring compatibility across different versions of the software.
    """

    def __init__(self):
        """Initializes the LightPropertyExtension."""
        self._registered = False
        super().__init__()

    def on_startup(self, ext_id):
        """This method is called when the extension is started.

        Args:
            ext_id (str): The ID of the extension being started."""
        self._register_widget()

    def on_shutdown(self):
        """Cleans up resources and unregisters widgets before the extension shuts down."""
        if self._registered:
            self._unregister_widget()

    def _register_widget(self):
        import omni.kit.window.property as p
        w = p.get_window()
        if w:
            w.register_widget("prim", "hda", HDASchemaAttributesWidget(title="HDA Parameters", collapsed=False), True)
            self._registered = True

    def _unregister_widget(self):
        import omni.kit.window.property as p
        w = p.get_window()
        if w:
            w.unregister_widget("prim", "hda")
            self._registered = False
