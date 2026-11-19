RELATIONS = [
    "left_of",
    "right_of",
    "front_of",
    "behind",
    "nearest_to",
    "farthest_from",
    "leftmost",
    "rightmost",
]
RELATION_TO_ID = {name: i for i, name in enumerate(RELATIONS)}

QUERY_KINDS = ["OBJECT_QUERY", "TARGET_QUERY"]
QUERY_KIND_TO_ID = {name: i for i, name in enumerate(QUERY_KINDS)}

PHASES = ["DETECT_PICK", "TARGET_PRECOMPUTE", "DETECT_PLACE"]
PHASE_TO_ID = {name: i for i, name in enumerate(PHASES)}

COLOR_TO_ID = {
    "red_block": 0,
    "green_block": 1,
    "blue_block": 2,
    "basket": 3,
}
ID_TO_COLOR = {v: k for k, v in COLOR_TO_ID.items()}
BLOCK_COLOR_IDS = {0, 1, 2}
