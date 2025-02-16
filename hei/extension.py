"""This module provides the LightPropertyExtension class for adding a custom light properties widget to the Omniverse Kit."""

import omni.ext
import omni.kit.app
from pxr import Usd, UsdGeom, UsdLux
from omni.physx.scripts import utils
from omni.kit.property.usd.usd_property_widget import (  # create_primspec_asset,
    MultiSchemaPropertiesWidget,
    UsdPropertyUiEntry,
    create_primspec_bool,
    create_primspec_float,
    create_primspec_string,
    create_primspec_token,
)
class HDA(UsdGeom.Xformable, Usd.Typed, Usd.SchemaBase):
    pass

class HDASchemaAttributesWidget(MultiSchemaPropertiesWidget):
    def __init__(
        self, title: str, schema, schema_subclasses: list, include_list: list = None, exclude_list: list = None
    ):
        """
        Constructor.

        Args:
            title (str): Title of the widgets on the Collapsable Frame.
            schema: The USD IsA schema or applied API schema to filter attributes.
            schema_subclasses (list): list of subclasses
            include_list (list): list of additional schema named to add
            exclude_list (list): list of additional schema named to remove
        """
        super().__init__(title, schema, schema_subclasses, include_list, exclude_list)

    def _customize_props_layout(self, attrs):
        frame = CustomLayoutFrame(hide_extra=False)
        return frame.apply(attrs)

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

            # https://github.com/PixarAnimationStudios/USD/commit/7540fdf3b2aa6b6faa0fce8e7b4c72b756286f51
            w.register_widget(
                "prim",
                "light",
                HDASchemaAttributesWidget(
                    "Light",
                    # https://github.com/PixarAnimationStudios/USD/commit/7540fdf3b2aa6b6faa0fce8e7b4c72b756286f51
                    UsdLux.LightAPI if hasattr(UsdLux, "LightAPI") else UsdLux.Light,
                    [
                    ]
                ),
            )
            self._registered = True

    def _unregister_widget(self):
        import omni.kit.window.property as p
        w = p.get_window()
        if w:
            w.unregister_widget("prim", "light")
            self._registered = False
