# uva environment
import json
import numpy as np

import omni.kit
from omni.isaac.core import World

from task.scene import ArrangeScene
from params import IS_IN_PYTHON

class UvaEnv():
    def __init__(self) -> None:
        # init world
        self.world = World()
        self.stage = self.world.scene.stage

        # record
        self.scene = None
        self.scene_record = {}

    def register_scene(self, scene):
        self.scene = scene
    
    
    def reset_scene(self):
        """
        Reset scene to key layout and camera only
        """
        # clean base
        base_prim = self.stage.GetPrimAtPath("/World/base")
        if base_prim.IsValid():
            omni.kit.commands.execute("DeletePrims", paths=["/World/base"])
        
        # clean object
        object_prim = self.stage.GetPrimAtPath("/World/objects")
        if object_prim.IsValid():
            omni.kit.commands.execute("DeletePrims", paths=["/World/objects"])


    def step(self, render = False):
        """
        Step env
        """
        self.world.step(render=render)

    

    
        

