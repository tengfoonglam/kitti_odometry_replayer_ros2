import numpy as np
import urdfpy

from typing import Dict

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

P0_NAME: str = "p0"
BASE_LINK_NAME: str = P0_NAME
CAMERA_TFS: Dict[str, np.ndarray] = {"p1": P0_TF_P1, "p2": P0_TF_P2, "p3": P0_TF_P3}
LIDAR_LINK_NAME: str = "lidar"

WHEEL_TFS: Dict[str, np.ndarray] = {"wfl": P0_TF_WFL,
                                    "wfr": P0_TF_WFR, "wbl": P0_TF_WBL, "wbr": P0_TF_WBR}
WHEEL_RADIUS: float = 0.3
WHEEL_WIDTH: float = 0.1
WHEEL_GEOMETRY: urdfpy.Geometry = urdfpy.Geometry(
    cylinder=urdfpy.Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH))
WHEEL_MATERIAL: urdfpy.Material = urdfpy.Material(
    name="wheel_material", color=(0.5, 0.5, 0.5, 1.0))
WHEEL_VISUAL: urdfpy.Visual = urdfpy.Visual(geometry=WHEEL_GEOMETRY, material=WHEEL_MATERIAL)
