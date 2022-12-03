# python import
import os
import sys
sys.path.append(os.path.dirname(__file__))

import numpy as np
import random
import asyncio



# omniverse import
import carb
import omni.usd
import omni.ext
import omni.ui as ui

# deep arrangement import
from .params import *
from .layout.randomizer import Randomizer
from .task.scene import ArrangeScene
from task.config import ASSET_PATH


class MyExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[deep.arrangement.ext] MyExtension startup")

        self._count = 0

        self._window = ui.Window("Deep Arrangement", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                # with ui.HStack(height = 20):
                #     ui.Button("Add Task Base", clicked_fn = self.add_task_base)
                with ui.HStack(height = 20):
                    ui.Button("Init Uva Env", clicked_fn = self.uva_test)
                with ui.HStack(height = 20):
                    ui.Label("Load nucleus", width = 100)
                    self.load_nucleus_checkbox = omni.ui.CheckBox(width=20, style={"font_size": 16})
                # with ui.HStack(height = 20):
                #     ui.Button("Add Task Base", clicked_fn = self.add_task_base)
                with ui.HStack(height = 20):
                    ui.Button("Add room", clicked_fn=self.add_room)
                    ui.Button("Randomize scene", clicked_fn=self.randomize_scene)
                with ui.HStack(height = 20):
                    self.task_type_ui = ui.ComboBox( 0, *TASK_CHOICES)
                    self.side_choice_ui = ui.ComboBox( 0, *SIDE_CHOICES)
                    self.task_base_id_ui = omni.ui.IntField()   
                with ui.HStack(height = 20):
                    ui.Button("Add Object", clicked_fn = self.add_task_object)
                    ui.Button("Move Object", clicked_fn = self.move_task_object)
                with ui.HStack(height = 20):
                    ui.Button("Get selected camera", clicked_fn = self.get_camera)
                    ui.Button("Set camera", clicked_fn = self.set_camera)
                    ui.Button("Capture image", clicked_fn = self.capture_image)          
                    ui.Button("Uva Play", clicked_fn = self.uva_play)
                with ui.HStack(height = 20):
                    ui.Button("Uva Reset", clicked_fn = self.uva_reset)
                    ui.Button("Uva Clean", clicked_fn = self.uva_clean)
                with ui.HStack(height = 20):
                    ui.Button("Uva Record", clicked_fn = self.uva_record)
                with ui.HStack(height = 20):
                    ui.Button("Debug", clicked_fn = self.debug)

                ui.Spacer(height = 20)
                with ui.HStack(height = 20):
                    ui.Button("YH Debug", clicked_fn = self.yuan_hong_debug)

    ################################ scene #########################################

    def add_room(self):
        """
        Add room background
        """
        self.task_scene.add_room()
 
    def randomize_scene(self, rand = True):
        """
        Randomize house materials
        """
        self.stage = omni.usd.get_context().get_stage()
        if not self.stage.GetPrimAtPath("/World/layout"):
            carb.log_error("Please add /World/layout (load scene) first!")
            self.task_desc_ui.model.set_value(f"Please `Load Scene`")
            return 

        self.randomizer = Randomizer()
        self.randomizer.randomize_house(rand = rand)
        self.task_desc_ui.model.set_value("Added floor/wall material")


    def add_task_base(self):
        """
        Add task asset
        """
        # type and id
        task_index = self.task_type_ui.model.get_item_value_model().get_value_as_int()
        self.task_type = TASK_CHOICES[task_index]
        side_index = self.side_choice_ui.model.get_item_value_model().get_value_as_int()
        self.side_choice = SIDE_CHOICES[side_index]
        asset_id = self.task_base_id_ui.model.get_value_as_int()
        self.load_nucleus = self.load_nucleus_checkbox.model.get_value_as_bool()

        self.task_scene = ArrangeScene(self.task_type, self.side_choice, asset_id, "/World/base", load_nucleus=self.load_nucleus)
        self.task_scene.add_base_asset()
 
    def add_task_object(self, mode = "random"):
        """
        Add task object
        """
        object_type = random.choice(self.task_scene.object_candidates)
        self.task_scene.load_obj_info(object_type, 1)
        print("objects", self.task_scene.objects)

    def move_task_object(self):
        """
        Move task object
        """
        stage = omni.usd.get_context().get_stage()
        object_prim_path = self.task_scene.objects[-1]["prim_path"]  
        # object_prim = stage.GetPrimAtPath(object_prim_path)
        x, y = np.tanh(np.random.randn()), np.tanh(np.random.randn())
        self.task_scene.map_object(object_prim_path, (x, y)) 

    def get_camera(self):
        """
        Get camera position debug
        """

        selection = omni.usd.get_context().get_selection()
        prim_path = selection.get_selected_prim_paths()[0]
        print("selected prim: ", prim_path)

        from .render.helper import RenderHelper
        stage = omni.usd.get_context().get_stage()
        camera_prim = stage.GetPrimAtPath(prim_path) # Camera_Table_Border_0 # Camera_Bookshelf_Border_0

        mat = omni.usd.get_world_transform_matrix(camera_prim)
        pos = mat.ExtractTranslation()
        rot = mat.ExtractRotationQuat()
        print("camera trans: ", pos, rot)

        RenderHelper.add_camera(camera_path = "/World/render/Camera_0", position = pos, rotation=rot)

    def set_camera(self):
        """
        Set up camera
        """
        from .render.helper import RenderHelper

        self.render_helper = RenderHelper("Bookshelf", "Border")
        # pos = (0, 500, 80)
        # rot = [0, 0, -0.7071068, 0.7071068]

        self.render_helper.add_task_cameras()


    def capture_image(self):
        self.render_helper.capture_image_debug()

    def uva_test(self):
        print("uva_test")
        from uva_env import UvaEnv
        self.env = UvaEnv()

        self.add_task_base()
        self.env.register_scene(self.task_scene)
        self.env.add_scene_obj()

        
    def uva_play(self):
        print("uva_play")
        self.env.step()

        last_obj_path = self.env.scene.objects[-1]["prim_path"]
        # reward = self.env.reward_affordance(last_obj_path)
        # print("reward", last_obj_path, reward)

        # asyncio.ensure_future(self.env.reward_basic_async(last_obj_path))
        self.env.reward_perturbation(last_obj_path)
    
    def uva_reset(self):
        print("uva_reset")
        self.env.reset()
    
    def uva_clean(self):
        print("uva_clean")
        self.env.clean()

    def uva_record(self):
        print("uva_record")
        record = self.env.scene.get_scene_data()
        self.env.scene.save_scene_data()
        print("record: ", record)
    
    #####################################################################################################

    def on_shutdown(self):
        print("[deep.arrangement.ext] MyExtension shutdown") 

        

    #############################################################################

    def debug(self):
        from pxr import Gf, UsdPhysics, Sdf
        self.stage = omni.usd.get_context().get_stage()
        object_prim_path = "/World/objects/Book"
        linVelocity = Gf.Vec3f(2.0, 1.0, 2.0)
        angularVelocity = Gf.Vec3f(0, 0, 45)
        physicsAPI = UsdPhysics.RigidBodyAPI.Get(self.stage, Sdf.Path(object_prim_path))
        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)

    def yuan_hong_debug(self):

        """
        debug reward function
        """
        from uva_env import UvaEnv
        self.env = UvaEnv()

        self.add_task_base()

        # self.add_task_object()
        print("hello")
        object_type = self.task_scene.object_candidates[3]
        print(self.task_scene.object_candidates[3])
        self.task_scene.load_obj_info(object_type, 1)
        print("hello")

        object = self.task_scene.objects[-1]
        from uv import reward
        r = reward.Rewarder(self.env.world)
        print(r.reward_perturbation(object["prim_path"], mode="debug"))



