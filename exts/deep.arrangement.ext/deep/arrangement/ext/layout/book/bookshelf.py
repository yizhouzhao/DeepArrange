import os

import omni.usd
from pxr import Gf

from ..params import ASSET_PATH
from .config import BOOK_SHELVES_PATHS

class BookShelfScene():
    def __init__(self) -> None:
        pass
    
    def add_bookshelf(self, index = 0, lean_on_pos = (0, 0)):
        """
        Add bookshelf to current scene
        ::params:
            index: bookshelf index
            lean_on_pos: position to lean on the bookshelf
        """
        bookshelf_path = os.path.join(ASSET_PATH, BOOK_SHELVES_PATHS[index])
        # asset
        self.stage = omni.usd.get_context().get_stage() 

        # move obj to the correct place
        bookshelf_prim_path = "/World/bookshelf"
        bookshelf_prim = self.stage.GetPrimAtPath(bookshelf_prim_path)
        if not bookshelf_prim.IsValid():
            bookshelf_prim = self.stage.DefinePrim(bookshelf_prim_path)

        print("bookshelf_path:", bookshelf_path)
        success_bool = bookshelf_prim.GetReferences().AddReference(bookshelf_path)

        assert success_bool, "Scene loading unsuccessful"

        position = (0,0,0)
        rotation = (1, 0, 0, 0)
        robot_xform_mat = Gf.Matrix4d().SetScale([1,1,1]) *  \
                Gf.Matrix4d().SetRotate(Gf.Quatf(float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3]))) * \
                Gf.Matrix4d().SetTranslate([float(position[0]), float(position[1]), float(position[2])])

        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=robot_prim.GetPath().pathString,
            new_transform_matrix=robot_xform_mat,
        )

