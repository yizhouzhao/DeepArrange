# uva environment
import omni.kit
from omni.isaac.core import World

from task.scene import ArrangeScene

class UvaEnv():
    def __init__(self) -> None:
        # init world
        self.world = World()
        self.stage = self.world.scene.stage
    
    def reset_scene(self):
        """
        Reset scene to key layout and camera only
        """
        # clean base
        base_prim = self.stage.GetPrimAtPath("/World/base")
        if base_prim:
            omni.kit.commands.execute("DeletePrims", paths=["/World/base"])
        
        # clean object
        base_prim = self.stage.GetPrimAtPath("/World/objects")
        if base_prim:
            omni.kit.commands.execute("DeletePrims", paths=["/World/objects"])
        

