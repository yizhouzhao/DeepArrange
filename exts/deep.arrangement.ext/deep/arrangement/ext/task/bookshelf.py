import os

import omni.usd
from pxr import Gf

from ..params import ASSET_PATH
from .config import BOOK_SHELVES_PATHS
from ..utils import import_asset_to_stage

from .common import CommonScene

class BookShelfScene(CommonScene):
    def __init__(self, side_choice = "border", base_asset_id = 0, base_prim_path = "/World/bookshelf") -> None:
        super().__init__(base_asset_id, base_prim_path, BOOK_SHELVES_PATHS[side_choice])
        self.side_choice = side_choice

        if self.side_choice == "corner":
            self.side_pos = (-378, 0)
        else:
            self.side_pos = (0, 0)
    


        

