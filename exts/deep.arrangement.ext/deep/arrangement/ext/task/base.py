import omni.usd
from pxr import Gf

from .config import *

from .common import CommonScene

class BookshelfBase(CommonScene):
    def __init__(self, side_choice = "Border", base_asset_id = 0, base_prim_path = "/World/bookshelf") -> None:
        super().__init__(side_choice, base_asset_id, base_prim_path, BOOK_SHELVES_PATHS[side_choice])
        self.object_candidates = BOOKSHELVE_OBJS
        self.task_choice = "Bookshelf"

class TableBase(CommonScene):
    def __init__(self, side_choice = "Border", base_asset_id = 0, base_prim_path = "/World/table") -> None:
        super().__init__(side_choice, base_asset_id, base_prim_path, TABLE_PATHS[side_choice])
        self.object_candidates = TABLE_OBJS
        self.task_choice = "Table"
        
class DeskBase(CommonScene):
    def __init__(self, side_choice = "Border", base_asset_id = 0, base_prim_path = "/World/desk") -> None:
        super().__init__(side_choice, base_asset_id, base_prim_path, DESK_PATHS[side_choice])
        self.object_candidates = DESK_OBJS
        self.task_choice = "Desk"


class WallBase(CommonScene):
    def __init__(self, side_choice = "Border", base_asset_id = 0, base_prim_path = "") -> None:
        super().__init__(side_choice, base_asset_id, base_prim_path, None)
        self.object_candidates = WALL_OBJS
        self.task_choice = "Wall"



