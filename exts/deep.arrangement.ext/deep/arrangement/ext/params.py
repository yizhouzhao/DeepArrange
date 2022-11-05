import os
import sys
sys.path.append(os.path.dirname(__file__))

import omni
import carb
from pathlib import Path

IS_IN_PYTHON =  str(carb.settings.get_settings().get("/app/window/title")).startswith("Isaac Sim Python") 
IS_IN_ISAAC_SIM = str(carb.settings.get_settings().get("/app/window/title")).startswith("Isaac Sim")
IS_IN_CREAT = str(carb.settings.get_settings().get("/app/window/title")).startswith("Create")
APP_VERION = str(carb.settings.get_settings().get("/app/version"))

EXTENSION_FOLDER_PATH = Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
)

TASK_CHOICES = ["Bookshelf", "Table", "Desk", "Wall"]
SIDE_CHOICES = ["Border", "Corner", "Center"]

ASSET_PATH = str(EXTENSION_FOLDER_PATH.parent.parent.resolve()) + "/Asset"
DATA_PATH = str(EXTENSION_FOLDER_PATH.parent.parent.resolve()) + "/Data"
