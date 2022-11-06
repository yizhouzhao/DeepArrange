import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import os
import numpy as np
import random

import omni.usd
from pxr import Gf, Sdf
from omni.physx.scripts.utils import setStaticCollider, setRigidBody
from omni.physx.scripts import physicsUtils

# from params import 

from .utils import import_asset_to_stage
from .config import *


class ArrangeScene():
    def __init__(self, task_choice, side_choice, base_asset_id, base_prim_path, load_nucleus = True) -> None:
        # task/side choice
        self.task_choice = task_choice
        self.side_choice = side_choice    

        # base asset path
        self.base_asset_id = base_asset_id
        self.base_prim_path = base_prim_path
        self.base_asset_file_paths = BASE_ASSET_PATHS[self.task_choice][self.side_choice]
        self.base_pos = BASE_TASK_POSITION[self.side_choice]

        # objects
        self.object_candidates = OBJS_CANDIDATES[self.task_choice]
        self.objects = []
        self.object_mean = np.array(OBJS_PLACEMENT_CONFIGS[self.task_choice][self.side_choice]["mean"])
        self.object_sd = np.array(OBJS_PLACEMENT_CONFIGS[self.task_choice][self.side_choice]["sd"])

        # scene
        self.stage = omni.usd.get_context().get_stage() 

        # asset
        self.load_nucleus = load_nucleus
        self.asset_path = ASSET_PATH if not load_nucleus else "omniverse://localhost/Users/yizhou/Asset"

    def add_ground(self):
        """
        Add ground
        """
        ground_path = physicsUtils.add_ground_plane(self.stage, "/World/groundPlane", "Z", 750.0, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.2))
        ground_prim = self.stage.GetPrimAtPath(ground_path)
        ground_prim.GetAttribute('visibility').Set('invisible')

    def add_layout(self):
        """
        Add house layout background
        """
        # scene
        self.stage = omni.usd.get_context().get_stage() 
        self.layer = self.stage.GetRootLayer()
        house_prim_path = "/World/layout"
        house_path = os.path.join(self.asset_path, "S", "0", "layout.usd")  
        import_asset_to_stage(self.stage, house_prim_path, house_path, position=(0, 456, 0), rotation=(0.7071068, 0.7071068, 0, 0))
 

    def add_base_asset(self):
        """
        Add base asset to current scene
        ::params:
            index: base asset index
            lean_on_pos: position to lean on the base asset
        """
        
        # asset 
        self.stage = omni.usd.get_context().get_stage() 
        self.base_file_path = os.path.join(self.asset_path, self.base_asset_file_paths[self.base_asset_id])
        self.base_prim = import_asset_to_stage(self.stage, self.base_prim_path, self.base_file_path)

        # update path
        self.base_prim_path = self.base_prim.GetPath().pathString
    
        # get bounding box
        bboxes = omni.usd.get_context().compute_path_world_bounding_box(self.base_prim_path) # get_prim_bbox(self.stage, bookshelf_prim)
        
        # calculate offset in each direction
        len_x = bboxes[1][0] - bboxes[0][0]
        len_y = bboxes[1][1] - bboxes[0][1]

        if len_x > len_y:
            offset_x = - bboxes[0][0]
            offset_y = - bboxes[0][1]
            rotation = (1, 0, 0, 0) # wxyz
        else:
            offset_y = - bboxes[0][0]
            offset_x = - bboxes[0][1]
            rotation = (0.7071068, 0, 0, 0.7071068) # wxyz
        
        offset_z = -bboxes[0][2]
        

        # position
        if self.side_choice == "Corner":
            position = (self.base_pos[0] + offset_x, self.base_pos[1] + offset_y, offset_z)
        elif self.side_choice == "Center":
            position = (self.base_pos[0], self.base_pos[1], offset_z) 
        else: # "Border"
            position = (self.base_pos[0], self.base_pos[1] + offset_y, offset_z) 


        xform_mat = Gf.Matrix4d().SetScale([1,1,1]) *  \
                Gf.Matrix4d().SetRotate(Gf.Quatf(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3]))) * \
                Gf.Matrix4d().SetTranslate([float(position[0]), float(position[1]), float(position[2])])

        # move to correct position and rotation
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=self.base_prim_path,
            new_transform_matrix=xform_mat,
        )

        # add collider
        setStaticCollider(self.base_prim)

        # re-calculate box for placement
        self.base_bboxes = omni.usd.get_context().compute_path_world_bounding_box(self.base_prim_path) 
        if self.task_choice == "Bookshelf":
            self.object_mean[1] = (self.base_bboxes[1][1] + self.base_bboxes[0][1]) * 0.5
        elif self.task_choice in ["Table", "Desk"]:
            self.object_mean[2] = self.base_bboxes[1][2]
        
        print("object_mean", self.object_mean)
    
    def load_obj_info(self, object_type:str, amount:int = 1):
        """
        Add object to scene
        ::params:
            object_type: string
            amount: int
        """
        assert object_type in self.object_candidates, f"OBJ Type {object_type} not in candidates"

        if self.load_nucleus:
            r = omni.client.list(os.path.join(self.asset_path, "I", object_type))    
            print("loading asset from omni nucleus")
            object_folder = sorted([e.relative_path for e in r[1]])
        else:
            object_folder = [obj for obj in os.listdir(os.path.join(self.asset_path, "I", object_type)) if obj.endswith(".usd")]
        object_name = random.choice(object_folder)

        for i in range(amount):
            object_info = {
                "type": object_type,
                "name": object_name,
                "file_path": os.path.join(self.asset_path, "I", object_type, object_name),
                "image_path": os.path.join(self.asset_path, "I", object_type, ".thumbs/256x256", object_name + ".png"),
                "prim_path": "",
                "xform_name": "",
            }
            
            # modify size 
            if object_type in OBJS_SIZE_MODIFICATION:
                object_scale = OBJS_SIZE_MODIFICATION[object_type]
            else:
                object_scale = 1.0

            # add object to scene
            self.add_object(object_info, scale=object_scale)
            self.objects.append(object_info)


    def add_object(self, object_info, position = (0, 200, 0), rotation = (1, 0, 0, 0), scale = 1.0):
        """
        Add object to scene
        """

        obj_root_prim = self.stage.GetPrimAtPath("/World/objects")
        if not obj_root_prim.IsValid():
            # create an unit xform
            omni.kit.commands.execute(
                    "CreatePrim",
                    prim_path="/World/objects",
                    prim_type="Xform",
                    select_new_prim=False,
                )

        object_type = object_info["type"]
        object_prim_path = f"/World/objects/{object_type}" 

        # import 
        object_prim = import_asset_to_stage(self.stage, object_prim_path, object_info["file_path"])
        object_info["prim_path"] = object_prim.GetPath().pathString

        # xform
        xform_mat = Gf.Matrix4d().SetScale(scale) *  \
                Gf.Matrix4d().SetRotate(Gf.Quatd(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3]))) * \
                Gf.Matrix4d().SetTranslate([float(position[0]), float(position[1]), float(position[2])])

        # move to correct position and rotation
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=object_prim.GetPath(),
            new_transform_matrix=xform_mat,
        )

        # attribute
        object_prim.CreateAttribute("arr:type", Sdf.ValueTypeNames.String, False).Set(object_type)
        setRigidBody(object_prim, "convexHull", False)

    def move_object(self, object_prim, position = (0, 200, 0), rotation = (1, 0, 0, 0)):
        """
        Move object
        """
        # scale
        scale = object_prim.GetAttribute("xformOp:scale").Get()[0]
        # xform
        xform_mat = Gf.Matrix4d().SetScale(scale) *  \
                Gf.Matrix4d().SetRotate(Gf.Quatd(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3]))) * \
            Gf.Matrix4d().SetTranslate([float(position[0]), float(position[1]), float(position[2])])

        # move to correct position and rotation
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=object_prim.GetPath(),
            new_transform_matrix=xform_mat,
        )

    def map_object(self, object_prim_path, pos2d, rot = (1, 0, 0, 0)):
        """
        Map object to position:
        """
        object_prim = self.stage.GetPrimAtPath(object_prim_path)
        assert object_prim.IsValid(), f"Cannot find object at {object_prim_path}"

        # print("pos2d", pos2d)
        if self.task_choice in ["Bookshelf", "Wall"]:
            offset = np.array([pos2d[0], 0, pos2d[1]])
        else:
            offset = np.array([pos2d[0], pos2d[1], 0])

        object_position = self.object_mean + 2 * self.object_sd * offset
        # print("object_position", object_position)
        # move object
        self.move_object(object_prim, object_position, rot)
        
        

        



