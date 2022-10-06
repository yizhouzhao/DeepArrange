import os
import random

import omni.usd
from pxr import Gf, Sdf
from omni.physx.scripts.utils import setStaticCollider

from ..params import ASSET_PATH
from ..utils import import_asset_to_stage

from .config import *


class CommonScene():
    def __init__(self, side_choice, base_asset_id, base_prim_path, base_asset_file_paths) -> None:
        # asset path
        self.base_asset_id = base_asset_id
        self.base_asset_file_paths = base_asset_file_paths
        self.base_prim_path = base_prim_path

        # objects
        self.object_candidates = None
        self.objects = []
        
        # task/side choice
        self.task_choice = ""
        self.side_choice = side_choice

        if self.side_choice == "Corner":
            self.side_pos = (-378, 0)
        elif self.side_choice == "Center":
            self.side_pos = (0, 200)
        else:
            self.side_pos = (0, 0)

    
    def add_base_asset(self):
        """
        Add base asset to current scene
        ::params:
            index: base asset index
            lean_on_pos: position to lean on the base asset
        """
        
        # asset 
        self.stage = omni.usd.get_context().get_stage() 
        file_path = os.path.join(ASSET_PATH, self.base_asset_file_paths[self.base_asset_id])
        self.base_prim = import_asset_to_stage(self.stage, self.base_prim_path, file_path)

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
            position = (self.side_pos[0] + offset_x, self.side_pos[1] + offset_y, offset_z)
        elif self.side_choice == "Center":
            position = (self.side_pos[0], self.side_pos[1], offset_z) 
        else: # "Border"
            position = (self.side_pos[0], self.side_pos[1] + offset_y, offset_z) 


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
    
    def load_obj_info(self, object_type:str, amount:int = 1):
        """
        Add object to scene
        ::params:
            object_type: string
            amount: int
        """
        assert object_type in self.object_candidates, f"OBJ Type {object_type} not in candidates"

        object_folder = os.listdir(os.path.join(ASSET_PATH, "I", object_type))

        object_name = random.choice(object_folder)

        for i in range(amount):
            object_info = {
                "type": object_type,
                "name": object_name,
                "file_path": os.path.join(ASSET_PATH, "I", object_type, object_name),
                "image_path": os.path.join(ASSET_PATH, "I", object_type, ".thumbs/256x256", object_name + ".png")
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
        object_type = object_info["type"]
        object_prim_path = f"/World/{object_type}" 

        # import 
        self.stage = omni.usd.get_context().get_stage() 
        object_prim = import_asset_to_stage(self.stage, object_prim_path, object_info["file_path"])
        object_info["prim_path"] = object_prim.GetPath().pathString

        # xform
        xform_mat = Gf.Matrix4d().SetScale(scale) *  \
                Gf.Matrix4d().SetRotate(Gf.Quatf(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3]))) * \
                Gf.Matrix4d().SetTranslate([float(position[0]), float(position[1]), float(position[2])])

        # move to correct position and rotation
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=object_prim.GetPath(),
            new_transform_matrix=xform_mat,
        )

        # attribute
        object_prim.CreateAttribute("arr:type", Sdf.ValueTypeNames.String, False).Set(object_type)

    def move_object(self, object_prim, position = (0, 200, 0), rotation = (1, 0, 0, 0)):
        """
        Move object
        """
        # scale
        scale = object_prim.GetAttribute("xformOp:scale").Get()
        # xform
        xform_mat = Gf.Matrix4d().SetScale(scale) *  \
                Gf.Matrix4d().SetRotate(Gf.Quatf(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3]))) * \
            Gf.Matrix4d().SetTranslate([float(position[0]), float(position[1]), float(position[2])])

        # move to correct position and rotation
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=object_prim.GetPath(),
            new_transform_matrix=xform_mat,
        )

    def map_object(self, object_prim, pos):
        """
        Map object to position
        """
        
