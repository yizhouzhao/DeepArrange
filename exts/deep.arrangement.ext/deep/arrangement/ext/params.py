import omni
import carb
import os
from pathlib import Path

EXTENSION_FOLDER_PATH = Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
)

ASSET_PATH = str(EXTENSION_FOLDER_PATH.parent.parent.resolve()) + "/Asset"

LEAN_ON_POSITIONS = [
    (0,0),
    (1,1),
]