import numpy as np

# Values obtained from https://www.cvlibs.net/datasets/kitti/setup.php
# P0_TF_LIDAR should be obtained from calib.txt for each dataset

KITTI_CALIB_FILENAME: str = "calib.txt"

P0_TF_LIDAR_ROW_LABEL = "Tr:"
# Regex that searches for - 'Tr: followed by 12 space-separated scientific notation floats'
P0_TF_LIDAR_ROW_REGEX: str = "Tr:( [+\-]?(?=.)(0|[1-9]\d*)?(\.\d*)?(?:(\d)[eE][+\-]?\d+)?){12}$"  # noqa: E501, W605

P0_TF_P1: np.ndarray = np.array([[1.,  0.,  0.,  0.54],
                                 [0.,  1.,  0.,  0.],
                                 [0.,  0.,  1.,  0.],
                                 [0.,  0.,  0.,  1.]])

P0_TF_P2: np.ndarray = np.array([[1.,  0.,  0.,  -0.06],
                                 [0.,  1.,  0.,  0.],
                                 [0.,  0.,  1.,  0.],
                                 [0.,  0.,  0.,  1.]])

P0_TF_P3: np.ndarray = np.array([[1.,  0.,  0.,  0.48],
                                 [0.,  1.,  0.,  0.],
                                 [0.,  0.,  1.,  0.],
                                 [0.,  0.,  0.,  1.]])

P0_TF_WFL: np.ndarray = np.array([[0.,  -1.,  0., -0.8],
                                  [0.,   0.,  -1., 1.35],
                                  [1.,   0.,  0.,  1.68],
                                  [0.,   0.,  0.,    1.]])

P0_TF_WFR: np.ndarray = np.array([[0.,  -1.,  0.,  0.8],
                                  [0.,   0.,  -1., 1.35],
                                  [1.,   0.,  0.,  1.68],
                                  [0.,   0.,  0.,    1.]])

P0_TF_WBL: np.ndarray = np.array([[0.,  -1.,  0., -0.8],
                                  [0.,   0.,  -1., 1.35],
                                  [1.,   0.,  0.,  -1.03],
                                  [0.,   0.,  0.,    1.]])

P0_TF_WBR: np.ndarray = np.array([[0.,  -1.,  0.,  0.8],
                                  [0.,   0.,  -1., 1.35],
                                  [1.,   0.,  0.,  -1.03],
                                  [0.,   0.,  0.,    1.]])

WHEEL_RADIUS: float = 0.3
