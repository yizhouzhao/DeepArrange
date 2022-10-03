import os

import omni.usd
from pxr import Gf
from omni.physx.scripts.utils import setStaticCollider

from ..params import ASSET_PATH
from ..utils import import_asset_to_stage


class CommonScene():
    def __init__(self, base_asset_id, base_prim_path, base_asset_file_paths) -> None:
        # asset path
        self.base_asset_id = base_asset_id
        self.base_asset_file_paths = base_asset_file_paths
        self.base_prim_path = base_prim_path

        # side choice
        self.side_choice = "border"
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

        # get bounding box
        bboxes = omni.usd.get_context().compute_path_world_bounding_box(self.base_prim_path) # get_prim_bbox(self.stage, bookshelf_prim)

        if self.side_choice == "corner":
            position = (self.side_pos[0] - bboxes[0][0], self.side_pos[1] - bboxes[0][1], -bboxes[0][2])
        else:
            position = (self.side_pos[0], self.side_pos[1] - bboxes[0][1], -bboxes[0][2]) # position
        rotation = (1, 0, 0, 0)
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
        

