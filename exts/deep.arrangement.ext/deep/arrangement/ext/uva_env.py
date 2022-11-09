# uva environment
import json
import time
import numpy as np
import random
import asyncio

import omni.usd
import omni.kit

from omni.isaac.core import World
from omni.isaac.core.prims.xform_prim import XFormPrim

from pxr import UsdGeom, Gf

from task.scene import ArrangeScene
from params import IS_IN_PYTHON, IS_IN_ISAAC_SIM

class UvaEnv():
    def __init__(self) -> None:
        # init world
        self.world = World()

        if IS_IN_PYTHON:
            self.world.reset()
            
        self.stage = self.world.scene.stage
        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()

        # record
        self.scene: ArrangeScene = None
        self.scene_record = {}

        # enable forcefield
        manager = omni.kit.app.get_app().get_extension_manager()
        self.forcefields_api_was_enabled = manager.is_extension_enabled("omni.physx.forcefields")
        if not self.forcefields_api_was_enabled:
            manager.set_extension_enabled_immediate("omni.physx.forcefields", True)

    def register_scene(self, scene):
        self.scene = scene
    
    def clean(self):
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

        for object_info in self.scene.objects:
            self.world.scene.remove_object(object_info["xform_name"])
        
        self.world.reset()

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
        if IS_IN_PYTHON:
            self.world.step(render=render)        

    
    def reset(self):
        self.world.reset()
        if not IS_IN_PYTHON:
            self.timeline.stop()

    ## ---------------------------------------- Reward ---------------------------------------------
    def reward_affordance(self, object_prim_path, simulation_step = 10):
        object_prim = self.stage.GetPrimAtPath(object_prim_path)
        # xform_cache = UsdGeom.XformCache()
        # transform = xform_cache.GetLocalToWorldTransform(object_prim)
        transform = omni.usd.get_world_transform_matrix(object_prim)
        begin_translation = transform.ExtractTranslation() # record beginning translation

        
        for _ in range(simulation_step):
            self.step()

        timecode = self.timeline.get_current_time() * self.stage.GetTimeCodesPerSecond()
        # xform_cache = UsdGeom.XformCache(timecode)
        # transform = xform_cache.GetLocalToWorldTransform(object_prim)
        transform = transform = omni.usd.get_world_transform_matrix(object_prim, timecode)
        end_translation = transform.ExtractTranslation() # record beginning translation

        if IS_IN_PYTHON:
            self.reset()

        print(object_prim_path, "begin_translation", begin_translation, "timecode", timecode, "end_translation", end_translation)
        
        return Gf.GetLength(end_translation - begin_translation)

    async def reward_affordance_async(self, object_prim_path):
        object_prim = self.stage.GetPrimAtPath(object_prim_path)
        # xform_cache = UsdGeom.XformCache()
        # transform = xform_cache.GetLocalToWorldTransform(object_prim)
        transform = omni.usd.get_world_transform_matrix(object_prim)
        begin_translation = transform.ExtractTranslation() # record beginning translation
        self.timeline.play()
        await asyncio.sleep(1)
        timecode = self.timeline.get_current_time() * self.stage.GetTimeCodesPerSecond()
        transform = transform = omni.usd.get_world_transform_matrix(object_prim, timecode)
        end_translation = transform.ExtractTranslation() # record end translation
        self.timeline.stop()

        print(object_prim_path, "begin_translation", begin_translation, "timecode", timecode, "end_translation", end_translation)
     




