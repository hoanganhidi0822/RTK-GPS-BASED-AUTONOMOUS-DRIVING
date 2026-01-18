import numpy as np
ALL_CLASSES = ['background', 'road', 'car', 'person']

LABEL_COLORS_LIST = [
    (np.uint8(0), np.uint8(0), np.uint8(0)),         # Background
    (np.uint8(31), np.uint8(120), np.uint8(180)),    # Road
    (np.uint8(106), np.uint8(61), np.uint8(154)),    # Car
    (np.uint8(227), np.uint8(26), np.uint8(28)),     # Person
]

VIS_LABEL_MAP = [
    (0, 0, 0),
    (0, 255, 0),
    (255, 0, 0),
    (0, 0, 255),
]