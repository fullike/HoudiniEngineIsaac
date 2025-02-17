"""This module provides the LightPropertyExtension class for adding a custom light properties widget to the Omniverse Kit."""

import omni.ext
import omni.kit.app
from pxr import Usd, UsdGeom, UsdLux
import omni.ui as ui
from omni.physx.scripts import utils
from omni.kit.property.usd.usd_property_widget import UsdPropertyUiEntry, UsdPropertiesWidget
from omni.kit.property.usd.custom_layout_helper import CustomLayoutFrame, CustomLayoutProperty

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
from hei.he_manager import HoudiniEngineManager

class HDASchemaAttributesWidget(UsdPropertiesWidget):
    def __init__(self, *args, **kwargs):
        UsdPropertiesWidget.__init__(self, *args, **kwargs)

    def build_parms_ui(frame, parms):
        with frame:
            with ui.VStack(height=0, spacing=5):
                for name, value in parms.items():
                    if isinstance(value, int):
                        with ui.HStack(spacing=8):
                            ui.Label(name,width=24)
                            widget = ui.IntField(width=ui.Fraction(1))
                            widget.model.set_value(value)
                            widget.model.add_end_edit_fn(lambda m, name=name:he_manager.setParameters(node_id, {name:m.as_int}))
                    elif isinstance(value, float):
                        with ui.HStack(spacing=8):
                            ui.Label(name,width=24)
                            widget = ui.FloatField(width=ui.Fraction(1))
                            widget.model.set_value(value)
                            widget.model.add_end_edit_fn(lambda m, name=name:he_manager.setParameters(node_id, {name:m.as_float}))
                    elif isinstance(value, str):
                        with ui.HStack(spacing=8):
                            ui.Label(name,width=24)
                            widget = ui.StringField(width=ui.Fraction(1))
                            widget.model.set_value(value)
                            widget.model.add_end_edit_fn(lambda m, name=name:he_manager.setParameters(node_id, {name:m.as_string}))         
                    elif isinstance(value, dict):
                        build_parms_ui(ui.CollapsableFrame(title=name), value)

    def on_file_changed(hda_path):
        global node_id
        node_id = he_manager.loadAsset(hda_path)
        if node_id is None:
            print("Failed to load the default HDA.")
            return
        file_input.model.set_value(hda_path)
        parms = he_manager.getParameters(node_id)
        build_parms_ui(frame, parms)

    def on_pick_file(self):
        from omni.kit.widget.filebrowser import FileBrowserItem
        from omni.kit.window.filepicker import FilePickerDialog
        def on_apply(dialog, dirname, filename):
            on_file_changed(dirname + filename)
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
        
    def _customize_props_layout(self, attrs):
        print(attrs)
        frame = CustomLayoutFrame(hide_extra=False)
        with frame:
            with ui.VStack(height=0, spacing=5):
                with ui.HStack(spacing=8):
                    ui.Label("HDA File:",width=24)
                    file_input = ui.StringField(width=ui.Fraction(1))
                    file_input.model.add_end_edit_fn(lambda text: on_file_changed(text.as_string))
                    ui.Button("...", clicked_fn=self.on_pick_file, width=24)
                frame = ui.CollapsableFrame(title="HDA Parameters")
                ui.Button("Cook")#, clicked_fn=cookNode)
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
