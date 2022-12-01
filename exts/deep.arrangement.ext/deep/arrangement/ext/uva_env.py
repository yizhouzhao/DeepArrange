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

from pxr import UsdGeom, Sdf, Gf, UsdPhysics
# from pxr import ForceFieldSchema

from task.scene import ArrangeScene
from render.helper import RenderHelper
from params import IS_IN_PYTHON, IS_IN_ISAAC_SIM

class UvaEnv():
    def __init__(self) -> None:
        # init world
        self.world = World()

        # if IS_IN_PYTHON:
        self.world.reset()
            
        self.stage = self.world.scene.stage
        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()

        # record
        self.scene: ArrangeScene = None
        self.scene_record = {}

        # 

        # force
        # self.create_force_field()
        
    def create_force_field(self):
        # enable forcefield
        manager = omni.kit.app.get_app().get_extension_manager()
        forcefields_api_was_enabled = manager.is_extension_enabled("omni.physx.forcefields")
        if not forcefields_api_was_enabled:
            manager.set_extension_enabled_immediate("omni.physx.forcefields", True)

        # disable flatcache
        flatcache_api_was_enabled = manager.is_extension_enabled("omni.physx.flatcache")
        if flatcache_api_was_enabled:
            manager.set_extension_enabled_immediate("omni.physx.flatcache", False)

        self.force_prim = self.stage.GetPrimAtPath("/World/forcefield")
        if not self.force_prim.IsValid():
            from task.utils import add_force_field
            add_force_field()
            self.force_prim = self.stage.GetPrimAtPath("/World/forcefield")

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

        if self.scene:
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
