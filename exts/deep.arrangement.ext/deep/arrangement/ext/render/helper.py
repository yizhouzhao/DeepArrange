import asyncio
import numpy as np
from PIL import Image


from .config import *
IS_IN_ISAAC_SIM, IS_IN_CREAT = False, False

import omni.usd
from pxr import Gf

if IS_IN_CREAT:
    import omni.syntheticdata as syn
    from omni.kit.viewport.utility import get_active_viewport

if IS_IN_ISAAC_SIM or IS_PYTHON:
    import omni.syntheticdata 
    from omni.isaac.synthetic_utils import SyntheticDataHelper


class RenderHelper():
    def __init__(self, task_type, side_choice, resolution = (1024, 1024)) -> None:
        self.task_type = task_type
        self.side_choice = side_choice
        self.resolution = resolution
        # setup view port   
        

        if IS_IN_CREAT: 
            self.viewport = get_active_viewport()
            self.viewport.resolution = resolution

            syn.sensors.enable_sensors(
                    self.viewport,
                    [
                        syn._syntheticdata.SensorType.Rgb, 
                    ],
                )
        
        if IS_IN_ISAAC_SIM:
            viewport_interface = omni.kit.viewport_legacy.get_viewport_interface()
            viewport_handle = viewport_interface.create_instance()
            self.viewport = viewport_interface.get_viewport_window(viewport_handle)
            self.viewport.set_window_pos(1000, 400)
            self.viewport.set_window_size(420, 420)
            self.viewport.set_texture_resolution(*self.resolution)

            self.sd_helper = SyntheticDataHelper() 
            # self.sd_helper.initialize(sensor_names=["rgb"], viewport=self.viewport)

            # Wait for viewports to be created
            # async def init_helper_async():

            
            # asyncio.ensure_future(init_helper_async())

    def add_camera(self, camera_path = "/World/Camera", position = (0, 0, 0), rotation = (1, 0, 0, 0), **kwargs):
        stage = omni.usd.get_context().get_stage()
        target_path = omni.usd.get_stage_next_free_path(stage, camera_path, True)
        omni.kit.commands.execute("CreatePrimWithDefaultXformCommand", prim_path=target_path, prim_type="Camera", create_default_xform=True)
       
        # move to correct position and rotation
        xform_mat = Gf.Matrix4d().SetScale([1,1,1]) *  \
                Gf.Matrix4d().SetRotate(Gf.Quatf(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3]))) * \
                Gf.Matrix4d().SetTranslate([float(position[0]), float(position[1]), float(position[2])])

        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=target_path,
            new_transform_matrix=xform_mat,
        )
    
    def set_cameras(self):
        """
        Set cameras from task types
        """
        assert IS_PYTHON, "Only works in Python driven environment"
        camera_paths = TASK2CAMERA_PATHS[self.task_type][self.side_choice]
        viewport_interface = omni.kit.viewport_legacy.get_viewport_interface()

        self.viewports = []
        self.handles = []
        self.sd_helpers = []

        for i in range(len(camera_paths)):
            viewport_handle = viewport_interface.create_instance()
            viewport = viewport_interface.get_viewport_window(viewport_handle)
            viewport.set_active_camera(camera_paths[i])
            viewport.set_window_pos(1000, 400)
            viewport.set_window_size(420, 420)
            viewport.set_texture_resolution(*self.resolution)

            sd_helper = SyntheticDataHelper()
            sd_helper.initialize(sensor_names=["rgb"], viewport=viewport)
            self.sd_helpers.append(sd_helper)
            self.viewports.append(viewport)
            self.handles.append(viewport_handle)
    
    def get_images(self): 
        """
        Get images from cameras
        """
        assert IS_PYTHON, "Only works in Python driven environment"
        for i in range(len(self.sd_helpers)):
            sensor_data = self.sd_helpers[i].get_groundtruth(["rgb"], self.viewports[i])
            self.save_rgb(sensor_data["rgb"], f"{DATA_PATH}/{i}")


    def capture_image_debug(self, camera_prim_path = "", image_name = "test"):
        """
        Capture image from camera
        """
        stage = omni.usd.get_context().get_stage()
        camera = stage.GetPrimAtPath(camera_prim_path)

        async def capture_image_async():
            # Render one frame
            await syn.sensors.next_sensor_data_async(self.viewport.get_id())
            data = syn.sensors.get_rgb(self.viewport)
            print("img", data.shape, data.dtype)
        
        async def get_synthetic_data():
            await omni.syntheticdata.sensors.next_sensor_data_async(self.viewport.get_id())
            # Sensor initialization
            await self.sd_helper.initialize_async(["rgb"], self.viewport)
            # Get Sensor data
            await omni.syntheticdata.sensors.next_sensor_data_async(self.viewport.get_id())
            sensor_data = self.sd_helper.get_groundtruth(["rgb"], self.viewport, verify_sensor_init=False)
            print("sensor_data", sensor_data)
            self.save_rgb(sensor_data["rgb"], f"{DATA_PATH}/{image_name}")

        if IS_IN_CREAT:
            asyncio.ensure_future(capture_image_async())
        
        if IS_IN_ISAAC_SIM:
            sensor_data = self.sd_helper.get_groundtruth(["rgb"], self.viewport)
            print("sensor_data", sensor_data)
            self.save_rgb(sensor_data["rgb"], f"{DATA_PATH}/{image_name}")
            # asyncio.ensure_future(get_synthetic_data())
        
    def save_rgb(self, rgb_data, file_name):
        """
        Save image to path
        """
        rgb_image_data = np.frombuffer(rgb_data, dtype=np.uint8).reshape(*rgb_data.shape, -1)
        rgb_img = Image.fromarray(rgb_image_data, "RGBA")
        rgb_img.save(file_name + ".png")