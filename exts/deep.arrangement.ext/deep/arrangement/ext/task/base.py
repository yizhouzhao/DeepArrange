import omni.usd
from pxr import Gf

from .config import BOOK_SHELVES_PATHS, TABLE_PATHS, DESK_PATHS

from .common import CommonScene

class BookshelfBase(CommonScene):
    def __init__(self, side_choice = "Border", base_asset_id = 0, base_prim_path = "/World/bookshelf") -> None:
        super().__init__(side_choice, base_asset_id, base_prim_path, BOOK_SHELVES_PATHS[side_choice])

class TableBase(CommonScene):
    def __init__(self, side_choice = "Border", base_asset_id = 0, base_prim_path = "/World/table") -> None:
        super().__init__(side_choice, base_asset_id, base_prim_path, TABLE_PATHS[side_choice])

class DeskBase(CommonScene):
    def __init__(self, side_choice = "Border", base_asset_id = 0, base_prim_path = "/World/desk") -> None:
        super().__init__(side_choice, base_asset_id, base_prim_path, DESK_PATHS[side_choice])

class WallBase(CommonScene):
    def __init__(self, side_choice = "Border", base_asset_id = 0, base_prim_path = "") -> None:
        super().__init__(side_choice, base_asset_id, base_prim_path, None)


