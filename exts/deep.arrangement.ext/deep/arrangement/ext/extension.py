# python import
import os
import random

# omniverse import
import carb
import omni.usd
import omni.ext
import omni.ui as ui

# deep arrangement import
from .params import ASSET_PATH, SIDE_CHOICES, TASK_CHOICES
from .layout.randomizer import Randomizer


class MyExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[deep.arrangement.ext] MyExtension startup")

        self._count = 0

        self._window = ui.Window("Deep Arrangement", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                with ui.HStack(height = 20):
                    ui.Button("Add scene", clicked_fn=self.add_scene)
                    ui.Button("Randomize scene", clicked_fn=self.randomize_scene)
                with ui.HStack(height = 20):
                    self.task_type_ui = ui.ComboBox( 0, *TASK_CHOICES)
                    self.side_choice_ui = ui.ComboBox( 0, *SIDE_CHOICES)
                    self.task_base_id_ui = omni.ui.IntField()   
                with ui.HStack(height = 20):
                    ui.Button("Add Task Base", clicked_fn = self.add_task_base)
                    ui.Button("Add Object", clicked_fn = self.add_task_object)
                with ui.HStack(height = 20):
                    ui.Button("Set camera", clicked_fn = self.set_camera)
                    
                
                    
    ################################ scene #########################################

    def add_scene(self):
        print("Add scene EXTENSION_FOLDER_PATH", ASSET_PATH)

        HOUSE_INFO_PATH = os.path.join(ASSET_PATH, "S")

        print("asset", os.listdir(ASSET_PATH))

        # scene
        self.stage = omni.usd.get_context().get_stage() 
        self.layer = self.stage.GetRootLayer()
        house_prim_path = "/World/layout"
        house_path = os.path.join(HOUSE_INFO_PATH, "0", "layout.usd")  

        from .utils import import_asset_to_stage
        import_asset_to_stage(self.stage, house_prim_path, house_path, position=(0, 456, 0), rotation=(0.7071068, 0.7071068, 0, 0))


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
        from .task.base import BookshelfBase, DeskBase, TableBase, WallBase

        # type and id
        task_index = self.task_type_ui.model.get_item_value_model().get_value_as_int()
        self.task_type = TASK_CHOICES[task_index]
        side_index = self.side_choice_ui.model.get_item_value_model().get_value_as_int()
        self.side_choice = SIDE_CHOICES[side_index]
        asset_id = self.task_base_id_ui.model.get_value_as_int()

        print("side_choice", self.side_choice)
        if self.task_type == "Bookshelf":
            self.task_scene = BookshelfBase(self.side_choice, asset_id)
            self.task_scene.add_base_asset()
        elif self.task_type == "Table":
            self.task_scene = TableBase(self.side_choice, asset_id)
            self.task_scene.add_base_asset()
        elif self.task_type == "Desk":
            self.task_scene = DeskBase(self.side_choice, asset_id)
            self.task_scene.add_base_asset()
        else: # Wall
            self.task_scene = WallBase()

    def add_task_object(self, mode = "random"):
        """
        Add task object
        """
        object_type = random.choice(self.task_scene.object_candidates)
        self.task_scene.load_obj_info(object_type, 1)
        print("objects", self.task_scene.objects)

    def set_camera(self):
        """
        Set up camera
        """
        from .render.helper import RenderHelper

        self.render_helper = RenderHelper()
        pos = (0, 500, 80)
        rot = [0, 0, -0.7071068, 0.7071068]
        # self.render_helper.add_camera(camera_path = "/World/Camera_0", position=pos, rotation=rot)

    def on_shutdown(self):
        print("[deep.arrangement.ext] MyExtension shutdown") 
