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

TASK_CAMERA_INFO = {
    "Bookshelf":{
        "Border":{
            # 0 for main camera
            "main": {
                "position": [0, 260, 100],
                "rotation": (0, 0, 0.7071, 0.7071),
                "focal_length": 14,
            },
            # side camera
            "left": {

            }
        }
    },
    "Table":{
        "Border":{
            # 0 for main camera
            "main": {
                "position": [0, 56, 200],
                "rotation": (0, 0, 0, 1),
                "focal_length": 14,
            },
            # side camera
            "left": {

            }
        }
    },
}
