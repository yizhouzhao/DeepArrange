import carb

IS_PYTHON = str(carb.settings.get_settings().get("/app/window/title")) == "Isaac Sim Python"

DATA_PATH = "/home/yizhou/Research/DeepArrange/Data"


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