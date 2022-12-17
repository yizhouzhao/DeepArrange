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
from uv.reward import Rewarder
from render.helper import RenderHelper
from params import IS_IN_PYTHON, IS_IN_ISAAC_SIM

class UvaEnv():
    def __init__(self) -> None:
        # init world
        self.world = World()

        # if IS_IN_PYTHON:
        # self.world.reset()
            
        self.stage = self.world.scene.stage
        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()

        # scene
        self.scene: ArrangeScene = None

        # reward
        self.rewarder: Rewarder = None

        # render
        self.render: RenderHelper = None


        # force # doesn't work in current version
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

    def clean(self, clean_all = True):
        """
        Reset scene to key layout and camera only
        """
        self.world.reset()
        
        # Disable object prim in world
        if self.scene:
            for object_info in self.scene.objects:
                self.world.scene.remove_object(object_info["xform_name"])
            self.scene.objects.clear()

        # clean object
        object_prim = self.stage.GetPrimAtPath("/World/objects")
        if object_prim.IsValid():
            omni.kit.commands.execute("DeletePrims", paths=["/World/objects"])

        
        if clean_all:
            # clean base
            base_prim = self.stage.GetPrimAtPath("/World/base")
            if base_prim.IsValid():
                omni.kit.commands.execute("DeletePrims", paths=["/World/base"])
            
            # clean render
            render_prim = self.stage.GetPrimAtPath("/World/render")
            if render_prim.IsValid():
                omni.kit.commands.execute("DeletePrims", paths=["/World/render"])


        
    def add_scene_obj(self, mode = "random"):
        # object
        object_type = random.choice(self.scene.object_candidates)
        self.scene.load_obj_info(object_type, 1)

    
    def put_last_object(self, pos):
        """
        Put the last added object in to position
        """
        # modify scene and object info
        object_info = self.scene.objects[-1]
        object_prim_path = object_info["prim_path"]
        object_info["action"] = pos

        # move object
        self.scene.map_object(object_prim_path, pos) 

    def register_last_object(self):
        """
        Register last object to scene
        """
        object_info = self.scene.objects[-1]
        object_prim_path = object_info["prim_path"]
        object_name = object_prim_path.split("/")[-1]
        obj_xform_prim = XFormPrim(object_prim_path, name = object_name)  # RigidPrim
        self.world.scene.add(obj_xform_prim)
        # obj_xform_prim.name = object_info["name"]
        object_info["xform_name"] = obj_xform_prim.name

    def step(self, render = False):
        """
        Env world step
        """
        if IS_IN_PYTHON:
            self.world.step(render=render)        

    def reset(self):
        """
        Env world reset
        """
        self.world.reset()
        if not IS_IN_PYTHON:
            self.timeline.stop()

    ##################################################### REWARD ####################################################
    def calculate_last_reward(self, option = "U",  simulation_step = 30):
        """
        Calculate the reward for the object recent added
        ::params:
            option: Utility or Value
        """
        
        # Utility: affordance
        last_object_prim = self.scene.objects[-1]["prim_path"]
        reward_affordance = self.rewarder.reward_basic(last_object_prim, simulation_step=simulation_step)
        self.scene.objects[-1]["reward"]["affordance"] = reward_affordance

        # # Utility: perturbation
        # last_object_prim = self.scene.objects[-1]["prim_path"]
        # reward_affordance = self.rewarder.reward_basic(last_object_prim)
        # self.scene.objects[-1]["reward"]["perturbation"] = reward_affordance




