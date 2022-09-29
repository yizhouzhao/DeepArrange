# python import
import os

# omniverse import
import carb
import omni.usd
import omni.ext
import omni.ui as ui

# deep arrangement import
from .params import ASSET_PATH
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
                    

    ################################ scene #########################################

    def add_scene(self):
        print("Add scene EXTENSION_FOLDER_PATH", ASSET_PATH)

        HOUSE_INFO_PATH = os.path.join(ASSET_PATH, "Scene")

        print("asset", os.listdir(ASSET_PATH))

        # scene
        self.stage = omni.usd.get_context().get_stage() 
        self.layer = self.stage.GetRootLayer()
        house_path = os.path.join(HOUSE_INFO_PATH, "0", "layout.usd")  

        # move obj to the correct place
        house_prim_path = "/World/layout"
        house_prim = self.stage.GetPrimAtPath(house_prim_path)
        if not house_prim.IsValid():
            house_prim = self.stage.DefinePrim(house_prim_path)

        print("house_path:", house_path)
        success_bool = house_prim.GetReferences().AddReference(house_path)

        assert success_bool, "Scene loading unsuccessful"

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


    def on_shutdown(self):
        print("[deep.arrangement.ext] MyExtension shutdown")
