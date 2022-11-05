# uva environment
import json
from this import d
import numpy as np
import random

import omni.kit
from omni.isaac.core import World
from omni.isaac.core.prims.xform_prim import XFormPrim

from pxr import UsdGeom, Gf

from task.scene import ArrangeScene
from params import IS_IN_PYTHON

class UvaEnv():
    def __init__(self) -> None:
        # init world
        self.world = World()
        self.stage = self.world.scene.stage
        self.timeline = omni.timeline.get_timeline_interface()
        

        # record
        self.scene: ArrangeScene = None
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

    def add_scene_obj(self, mode = "random"):
        # object
        object_type = random.choice(self.scene.object_candidates)
        self.scene.load_obj_info(object_type, 1)
        # modify scene and object info
        object_info = self.scene.objects[-1]
        object_prim_path = object_info["prim_path"]
        obj_xform_prim = XFormPrim(object_prim_path) 
        self.world.scene.add(obj_xform_prim)
        object_info["xform_name"] = obj_xform_prim.name
    
    def move_object_to(self, object_prim_path, pos):
        self.scene.map_object(object_prim_path, pos) 

    def step(self, render = False):
        """
        Step env
        """
        self.world.step(render=render)

    ## ---------------------------------------- Reward ---------------------------------------------
    def affordance_aware_reward(self, object_prim_path, simulation_step = 10):
        object_prim = self.stage.GetPrimAtPath(object_prim_path)
        xform_cache = UsdGeom.XformCache()
        transform = xform_cache.GetLocalToWorldTransform(object_prim)
        begin_translation = transform.ExtractTranslation() # record beginning translation

        #　timecode = self.timeline.get_current_time() * self.stage.GetTimeCodesPerSecond()
        for _ in range(simulation_step):
            self.step()

        xform_cache = UsdGeom.XformCache(simulation_step)
        transform = xform_cache.GetLocalToWorldTransform(object_prim)
        end_translation = transform.ExtractTranslation() # record beginning translation

        self.world.reset()
        
        return Gf.GetLength(end_translation - begin_translation)



