#!/usr/bin/env python3

import click
import logging
import numpy as np
import re
import ros2_kitti_description.constants as constants
import urdfpy

from pathlib import Path
from typing import Callable, List, Optional


def extract_p0_tf_lidar(calib_file_path: Path) -> Optional[np.ndarray]:
    output = None

    with open(calib_file_path, 'r') as f:
        lines: List[str] = [line.rstrip() for line in f]
        candidate_lines: List[str] = [line for line in lines if re.fullmatch(
            constants.P0_TF_LIDAR_ROW_REGEX, line) is not None]
        if len(candidate_lines) != 1:
            logging.info(
                f"Multiple/no lines in {calib_file_path} contains a row with 'Tr: "
                "followed by 12 space-separated scientific notation floats'. Ignoring file")
        else:
            candidate_line: str = candidate_lines[0]
            mat_str: str = re.sub(f'^{constants.P0_TF_LIDAR_ROW_LABEL} ',
                                  '', string=candidate_line, count=1)
            rot_trans_mat: np.ndarray = np.fromstring(
                mat_str, dtype=float, count=12, sep=' ').reshape((3, 4))
            output = np.vstack((rot_trans_mat, np.array([0., 0., 0., 1.])))
            logging.debug(f"Successfully parsed p0_tf_lidar: \n {output}")

    return output


def create_urdf(p0_tf_lidar: np.ndarray) -> urdfpy.URDF:
    base_link = [urdfpy.Link(name=constants.BASE_LINK_NAME, visuals=None,
                             collisions=None, inertial=None)]
    wheel_links = [urdfpy.Link(name=name, visuals=[constants.WHEEL_VISUAL],
                               collisions=None, inertial=None)
                   for name in constants.WHEEL_TFS.keys()]
    wheel_joints = [urdfpy.Joint(name=f"{constants.BASE_LINK_NAME}_{name}_link",
                                 joint_type="fixed", parent=constants.BASE_LINK_NAME,
                                 child=name, origin=tf)
                    for name, tf in constants.WHEEL_TFS.items()]
    return urdfpy.URDF(name="car", links=base_link + wheel_links, joints=wheel_joints)


def generate_urdf(dataset_path: Path, output_path: Path) -> None:

    calib_file_path: Path = dataset_path / Path(constants.KITTI_CALIB_FILENAME)

    if not calib_file_path.exists():
        logging.error(
            f"{constants.KITTI_CALIB_FILENAME} not found in folder {dataset_path}")
        return

    p0_tf_lidar = extract_p0_tf_lidar(calib_file_path=calib_file_path)

    if p0_tf_lidar is None:
        logging.error(
            f"Unable to extract transformation matrix from p0 to lidar in {calib_file_path}")
        return

    vehicle_urdf = create_urdf(p0_tf_lidar=p0_tf_lidar)
    output_file = output_path / Path(f"{dataset_path.stem}.urdf")
    vehicle_urdf.save(str(output_file))

    logging.info(f"URDF saved to {output_file}")

    pass


@click.command()
@click.option('--data_odometry_calib_dir', '-d', required=True, help="Location of the "
              "KITTI 'data_odometry_calib' folder")
@click.option('--output_dir', '-o', required=True, help="Directory where generated "
              "URDFs will be located")
def generate_urdfs(data_odometry_calib_dir: str, output_dir: str) -> None:
    """Generate URDFs for the KITTI Odometry Dataset"""

    logging.basicConfig(level=logging.INFO)

    # Make sure output directory exists
    output_path: Path = Path(output_dir).resolve()
    if not output_path.is_dir():
        logging.error(
            f"output_path provided ({output_path}) "
            "is not a folder or does not exist")
        return

    # Get all folders
    data_odometry_calib_path: Path = Path(data_odometry_calib_dir).resolve()
    if not data_odometry_calib_path.is_dir():
        logging.error(
            f"data_odometry_calib_dir provided ({data_odometry_calib_path}) "
            "is not a folder or does not exist")
        return

    sequences_path: Path = data_odometry_calib_path / Path("dataset/sequences")

    if not sequences_path.exists():
        logging.error(
            f"sequences folder within data_odometry_calib_dir {sequences_path} does not exist")
        return

    is_dataset_folder: Callable[[str], bool] = lambda stem: stem.isnumeric() and len(stem) >= 2
    dataset_folders: List[Path] = sorted([folder for folder in sequences_path.iterdir(
    ) if is_dataset_folder(folder.stem)], key=lambda path: int(path.stem))

    logging.info(f"Found sequences {[folder.stem for folder in dataset_folders]}")

    # For each folder
    for folder in dataset_folders:
        generate_urdf(dataset_path=folder, output_path=output_path)
