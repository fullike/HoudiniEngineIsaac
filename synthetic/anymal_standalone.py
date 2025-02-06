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

import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from isaacsim.robot.policy.examples.robots import AnymalFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path


class Anymal_runner(object):
    def __init__(self, physics_dt, render_dt) -> None:
        """
        creates the simulation world with preset physics_dt and render_dt and creates an anymal robot inside the warehouse

        Argument:
        physics_dt {float} -- Physics downtime of the scene.
        render_dt {float} -- Render downtime of the scene.

        """
        self._world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)

        assets_root_path = get_assets_root_path()
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
        camera = Camera(
                    prim_path="/World/Anymal/depth_camera_front_camera/Camera",
                    translation=np.array([0, 0, 0.1]),
                    name="camera1",
                    frequency=20,
                    resolution=(256, 256), 
                )
        camera.initialize()
        self._camera = self._world.scene.add(camera)
        # Given the OpenCV camera matrix and distortion coefficients (Rational Polynomial model),
        # creates a camera and a sample scene, renders an image and saves it to
        # camera_opencv_fisheye.png file. The asset is also saved to camera_opencv_fisheye.usd file.
        width, height = 1920, 1200
        camera_matrix = [[958.8, 0.0, 957.8], [0.0, 956.7, 589.5], [0.0, 0.0, 1.0]]
        distortion_coefficients = [0.14, -0.03, -0.0002, -0.00003, 0.009, 0.5, -0.07, 0.017]

        # Camera sensor size and optical path parameters. These parameters are not the part of the
        # OpenCV camera model, but they are nessesary to simulate the depth of field effect.
        #
        # To disable the depth of field effect, set the f_stop to 0.0. This is useful for debugging.
        pixel_size = 3  # in microns, 3 microns is common
        f_stop = 1.8  # f-number, the ratio of the lens focal length to the diameter of the entrance pupil
        focus_distance = 0.6  # in meters, the distance from the camera to the object plane
        diagonal_fov = 140  # in degrees, the diagonal field of view to be rendered

        ((fx, _, cx), (_, fy, cy), (_, _, _)) = camera_matrix
        horizontal_aperture = pixel_size * 1e-3 * width
        vertical_aperture = pixel_size * 1e-3 * height
        focal_length_x = fx * pixel_size * 1e-3
        focal_length_y = fy * pixel_size * 1e-3
        focal_length = (focal_length_x + focal_length_y) / 2  # in mm

        # Set the camera parameters, note the unit conversion between Isaac Sim sensor and Kit
        camera.set_focal_length(focal_length / 10.0)
        camera.set_focus_distance(focus_distance)
        camera.set_lens_aperture(f_stop * 100.0)
        camera.set_horizontal_aperture(horizontal_aperture / 10.0)
        camera.set_vertical_aperture(vertical_aperture / 10.0)

        camera.set_clipping_range(0.05, 1.0e5)

        # Set the distortion coefficients
        camera.set_projection_type("fisheyePolynomial")
        camera.set_rational_polynomial_properties(width, height, cx, cy, diagonal_fov, distortion_coefficients)

        render_product = rep.create.render_product("/World/Anymal/depth_camera_front_camera/Camera", (1024, 1024))
        basic_writer = rep.WriterRegistry.get("BasicWriter")
        basic_writer.initialize(
            output_dir=f"./output",
            rgb=True,
            distance_to_camera=True,
            colorize_depth=True,
        )
        #Attach render_product to the writer
        basic_writer.attach([render_product])

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
        self._world.add_physics_callback("anymal_forward", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        """
        Physics call back, initialize robot (first frame) and call controller forward function to compute and apply joint torque

        """
        if self.first_step:
            self._anymal.initialize()
            self.first_step = False
        elif self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
            self.first_step = True
        else:
            self._anymal.forward(step_size, self._base_command)

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
    runner._world.reset()
    simulation_app.update()
    runner.setup()
    simulation_app.update()
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
