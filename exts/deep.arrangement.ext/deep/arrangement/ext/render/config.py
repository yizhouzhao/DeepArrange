import carb

from task.config import IS_IN_ISAAC_SIM, IS_IN_CREAT, IS_PYTHON, DATA_PATH

TASK2CAMERA_PATHS = {
    "Bookshelf":{
        "Border":[
            "/World/layout/Camera_Bookshelf_Border_0",
            "/World/layout/Camera_Table_Border_0",
        ],
        "Corner":[
            "/World/layout/Camera_Bookshelf_Corner_0",
        ],
    },
    "Table": {
        "Border":[
            "/World/layout/Camera_Table_Border_0",
        ],
        "Corner":[
            "/World/layout/Camera_Table_Corner_0",
        ],
        "Center":[
        ],
    },
    "Desk":{
        "Border": [
            "/World/layout/Camera_Table_Border_0",
        ]
    },
    "Wall":{
        "Border":[
            "/World/layout/Camera_Bookshelf_Border_0",
        ]
    }
}