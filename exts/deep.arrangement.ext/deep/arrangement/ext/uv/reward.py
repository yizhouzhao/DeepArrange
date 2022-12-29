# reward
import asyncio
import os 
import numpy as np

from pxr import Gf, UsdPhysics, Sdf

import omni.usd
import omni.timeline

from params import IS_IN_PYTHON, IS_IN_ISAAC_SIM

if IS_IN_PYTHON or IS_IN_ISAAC_SIM:
    from omni.isaac.core import World

from .utils import is_boxes_overlap

class Rewarder():
    def __init__(self, world) -> None:
        self.stage = omni.usd.get_context().get_stage()
        self.world = world
        self.timeline = omni.timeline.get_timeline_interface()
    
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
    def reward_basic(self, object_prim_path, simulation_step = 30):
        """
        Get reward from affordance: play the timeline for seconds to see whether the object moves
        """
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

        # if IS_IN_PYTHON:
        #     self.reset()

        # print(object_prim_path, "begin_translation", begin_translation, "timecode", timecode, "end_translation", end_translation)
        
        return Gf.GetLength(end_translation - begin_translation)

    async def reward_basic_async(self, object_prim_path):
        """
        Async debug only
        """
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
        # yield Gf.GetLength(end_translation - begin_translation)

    def reward_perturbation(self, object_prim_path, mode = "noise"):
        """
        Get reward from perturbation for object prim at path
        mode: noise, wind
        """
        # if mode == "noise":
        #     forcePrimApi = ForceFieldSchema.PhysxForceFieldNoiseAPI.Apply(self.force_prim, "Shake")

        # forcePrimApi.CreateEnabledAttr(True)

        print("object_prim_path: ", object_prim_path)

        # rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(object_prim_path)
        # rigidBodyAPI.CreateVelocityAttr().Set(linVelocity)
        # rigidBodyAPI.CreateAngularVelocityAttr().Set(angularVelocity)

        # massAPI = UsdPhysics.MassAPI.Apply(object_prim_path)
        # massAPI.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 10.0))


        # Q1: no object mass 
        # linVelocity = Gf.Vec3f(list(8 * np.random.randn(3)))
        # angularVelocity = Gf.Vec3f(list(10 * np.random.randn(3)))
        if mode == "debug":
            linVelocity = Gf.Vec3f(0, 0, 10.0)
            angularVelocity = Gf.Vec3f(0, 0, 0)
        else:
            linVelocity = Gf.Vec3f(list(8 * np.random.randn(3)))
            angularVelocity = Gf.Vec3f(list(10 * np.random.randn(3)))

        def get_size(prim):
            from pxr import Usd, UsdGeom
            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
            bbox_cache.Clear()
            prim_bbox = bbox_cache.ComputeWorldBound(prim)
            prim_range = prim_bbox.ComputeAlignedRange()
            prim_size = prim_range.GetSize()
            return prim_size

        size = get_size(self.stage.GetPrimAtPath(object_prim_path))
        print(f"Size: {size}")

        mass = np.prod(size) / 1e4

        physicsAPI = UsdPhysics.RigidBodyAPI.Get(self.stage, Sdf.Path(object_prim_path))
        
        massAPI = UsdPhysics.MassAPI.Get(self.stage, Sdf.Path(object_prim_path))
        massAPI.CreateMassAttr().Set(mass)

        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)    

        # Q2: revert
        print("revert")
        physicsAPI.CreateVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
        physicsAPI.CreateAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))    
        # linVelocity = 0, angularVelocity = 0

        if IS_IN_PYTHON:
            reward = self.reward_basic(object_prim_path)
            # forcePrimApi.CreateEnabledAttr(False)
            return reward
        else:
            asyncio.ensure_future(self.reward_basic_async(object_prim_path))
        

    def reward_collision(self, objects):
        """
        Calculate the collision information for the last object in objects
        """
        last_object_boxes = objects[-1]["bounding_box"]
        is_overlap = False
        for obj_info in objects[:-1]:
            boxes = obj_info["bounding_box"]
            if is_boxes_overlap(last_object_boxes, boxes):
                is_overlap = True
                break

        return 1.0 if is_overlap else 0.0

