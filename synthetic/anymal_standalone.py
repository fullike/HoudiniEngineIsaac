# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.sensor import Camera
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.quadruped.robots import AnymalFlatTerrainPolicy


class Anymal_runner(object):
    def __init__(self, physics_dt, render_dt) -> None:
        """
        creates the simulation world with preset physics_dt and render_dt and creates an anymal robot inside the warehouse

        Argument:
        physics_dt {float} -- Physics downtime of the scene.
        render_dt {float} -- Render downtime of the scene.

        """
        self._world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)

        assets_root_path = "/media/jay/data/4.2"#get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        # spawn warehouse scene
        prim = define_prim("/World/Warehouse", "Xform")
        asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        prim.GetReferences().AddReference(asset_path)

        self._anymal = AnymalFlatTerrainPolicy(
            prim_path="/World/Anymal",
            name="Anymal",
            usd_path=assets_root_path + "/Isaac/Robots/ANYbotics/anymal_c.usd",
            position=np.array([0, 0, 0.7]),
        )
        self._camera = self._world.scene.add(Camera(
                    prim_path="/World/Anymal/depth_camera_front_camera/Camera",
                    position=np.array([0, 0, 2]),
                    name="camera1",
                    frequency=20,
                    resolution=(256, 256), 
                ))

        render_product = rep.create.render_product("/World/Anymal/depth_camera_front_camera/Camera", (1024, 1024))
        # basic_writer = rep.WriterRegistry.get("BasicWriter")
        # basic_writer.initialize(
        #     output_dir=f"~/replicator_examples/multi_render_product/basic",
        #     rgb=True,
        #     distance_to_camera=True,
        #     colorize_depth=True,
        # )
        # Attach render_product to the writer
        #basic_writer.attach([render_product])

        self._base_command = np.zeros(3)

        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            "NUMPAD_8": [1.0, 0.0, 0.0],
            "UP": [1.0, 0.0, 0.0],
            # back command
            "NUMPAD_2": [-1.0, 0.0, 0.0],
            "DOWN": [-1.0, 0.0, 0.0],
            # left command
            "NUMPAD_6": [0.0, -1.0, 0.0],
            "RIGHT": [0.0, -1.0, 0.0],
            # right command
            "NUMPAD_4": [0.0, 1.0, 0.0],
            "LEFT": [0.0, 1.0, 0.0],
            # yaw command (positive)
            "NUMPAD_7": [0.0, 0.0, 1.0],
            "N": [0.0, 0.0, 1.0],
            # yaw command (negative)
            "NUMPAD_9": [0.0, 0.0, -1.0],
            "M": [0.0, 0.0, -1.0],
        }
        self.needs_reset = False
        self.first_step = True

    def setup(self) -> None:
        """
        Set up keyboard listener and add physics callback

        """
        carb.settings.get_settings().set_bool("/persistent/app/omniverse/gamepadCameraControl", False)
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._gamepad = self._appwindow.get_gamepad(0)
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        self._sub_gamepad = self._input.subscribe_to_gamepad_events(self._gamepad, self._sub_gamepad_event)
        self._world.add_physics_callback("anymal_advance", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        """
        Physics call back, initialize robot (first frame) and call robot advance function to compute and apply joint torque

        """
        if self.first_step:
            self._anymal.initialize()
            self.first_step = False
        elif self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
            self.first_step = True
        else:
            self._anymal.advance(step_size, self._base_command)

    def run(self) -> None:
        """
        Step simulation based on rendering downtime

        """
        # change to sim running
        while simulation_app.is_running():
            self._world.step(render=True)
            if self._world.is_stopped():
                self.needs_reset = True
            self._camera.add_distance_to_camera_to_frame()
            data = self._camera.get_current_frame()
            depth = data["distance_to_camera"]
            pass
        return

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """
        Keyboard subscriber callback to when kit is updated.

        """

        # when a key is pressed for released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # on pressing, the command is incremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(self._input_keyboard_mapping[event.input.name])

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # on release, the command is decremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(self._input_keyboard_mapping[event.input.name])
        return True

    def _sub_gamepad_event(self, event):
        print("{} ({})".format(event.input, event.value))

        # if event.input == carb.input.GamepadInput.RIGHT_TRIGGER:
        #     self.throttle = event.value

        # if event.input == carb.input.GamepadInput.LEFT_TRIGGER:
        #     self.brake = event.value

        # if event.input == carb.input.GamepadInput.LEFT_STICK_LEFT:
        #     self.steerLeft = event.value

        # if event.input == carb.input.GamepadInput.LEFT_STICK_RIGHT:
        #     self.steerRight = event.value

        return True

def main():
    """
    Parse arguments and instantiate the ANYmal runner

    """
    physics_dt = 1 / 200.0
    render_dt = 1 / 60.0

    runner = Anymal_runner(physics_dt=physics_dt, render_dt=render_dt)
    simulation_app.update()
    runner.setup()
    simulation_app.update()
    runner._world.reset()
    simulation_app.update()
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
