BOOK_SHELVES_PATHS = {
    "Border":[
        'B/Bookshelve/Clayton.usd',
        'B/Bookshelve/Dogtown.usd',
        'B/Bookshelve/Fenton.usd',
        'B/Bookshelve/Ferguson.usd',
        'B/Bookshelve/Ladue.usd',
    ],
    "Corner":[
        'B/Shelve/CornerShelf_Square.usd', 
        'B/Shelve/CornerShelf_Vera.usd',
    ],
}


TABLE_PATHS = {
    "Border":[
        'T/Appleseed/Appleseed_CoffeeTable.usd',
        'T/DesPere/DesPeres_Table.usd',
        'T/EastRural/EastRural_Table.usd',
        'T/Jenning/Jennings_Table.usd',
        'T/Roxana/Roxana_DiningBench.usd',
    ],
    "Corner":[
        'T/Appleseed/Appleseed_EndTable.usd',
        'T/EndTable/Ellendale.usd',
        'T/EndTable/Festus01.usd',
        'T/Roxana/Roxana_RoundEndTable.usd',
        'T/Roxana/Rozana_EndTable.usd',
    ],
    "Center":[
        'T/Roxana/Roxana_CoffeeTable.usd',
        'T/Roxana/Roxana_DiningTable.usd',
        'T/Roxana/Roxana_RoundCoffeeTable.usd',
    ],
}



DESK_PATHS = {
    "Border": [
        "D/Desk/Desk_01.usd",
    ]
}

BOOKSHELVE_OBJS = [
    "Book", "Magazine","Plant","Sculpture","Vase",
]

TABLE_OBJS = [
    "Fruit","Dinnerware","Flatware","GadgetsAndTool","Magazine","Tchotchke","Vase",
]

DESK_OBJS = [
    "Book","Clock_Desk","Magazine","DeskDecor","Plant","Sculpture","Supply","TableLamp",
]

WALL_OBJS = [
    "Clock","Picture",
]

OBJS_SIZE_MODIFICATION = {
    "Book": 0.7,
    "Magazine": 0.7,
    "Sculpture": 0.7,
    "Picture":0.7,
    "Vase":0.7,
}


CAMERA_CONFIGS = {
    "Bookshelf": {
        "Border": (0, 60, 200),
        "Corner": (0, 0, 0),
    },
    "Table": {
        "Border": (0, 60, 200),
        "Corner": (0, 0, 0),
        "Center": (0 ,0, 0),
    },
    "Desk": {
        "Border": (0, 60, 200),
    },
    "Wall": {
        "Border": (0, 60, 200),
    },
    
}

# how to place the object
PLACEMENT_CONFIGS = {
    "Bookshelf":{
        "Border": {
            "mean": (0, -1, 100),
            "sd": (50, -1, 50),
        },
        "Corder":{
            "mean":(-378, -1, 100),
            "sd": (50, -1, 50),
        }
    },
    "Table":{
        "Border": {
            "mean": (0, 0, -1),
            "sd": (50, 50, -1),
        },
        "Corder":{
            "mean":(-378, 0, -1),
            "sd": (50, 50, -1),
        },
        "Center":{
            "mean":(0, 150, -1),
            "sd": (50, 50, -1),
        }
    },
    "Desk":{
        "Border": {
            "mean": (0, 0, -1),
            "sd": (50, 50, -1),
        }
    },
    "Wall": {
        "Border": {
            "mean": (0, -1, 100),
            "sd": (50, -1, 50),
        },
    },
}