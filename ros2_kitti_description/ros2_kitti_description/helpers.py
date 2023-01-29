import logging
import numpy as np
import re
import ros2_kitti_description.constants as constants
import urdfpy

from pathlib import Path
from typing import List, Optional


def extract_p0_tf_lidar(calib_file_path: Path) -> Optional[np.ndarray]:
    """Extracts the p0_tf_lidar transformation from a KITTI calibration file

    Args:
        calib_file_path (Path): Calibration file

    Returns:
        Optional[np.ndarray]: 4x4 p0_tf_lidar transformation matrix if extraction was successful
    """

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


def create_urdf(urdf_name: str, p0_tf_lidar: np.ndarray) -> urdfpy.URDF:
    """Create vehicle URDF

    Args:
        urdf_name (str): URDF name
        p0_tf_lidar (np.ndarray): 4x4 p0_tf_lidar transformation matrix

    Returns:
        urdfpy.URDF: Resulting URDF
    """

    Links = List[urdfpy.Link]
    Joints = List[urdfpy.Joint]

    base_link: Links = [urdfpy.Link(name=constants.BASE_LINK_NAME,
                                    visuals=[constants.CAM_VISUAL],
                                    collisions=None, inertial=None)]
    lidar_link: Links = [urdfpy.Link(name=constants.LIDAR_LINK_NAME,
                                     visuals=[constants.LIDAR_VISUAL],
                                     collisions=None, inertial=None)]
    wheel_links: Links = [urdfpy.Link(name=name, visuals=[constants.WHEEL_VISUAL],
                                      collisions=None, inertial=None)
                          for name in constants.WHEEL_TFS.keys()]
    camera_links: Links = [urdfpy.Link(name=name, visuals=[constants.CAM_VISUAL],
                                       collisions=None, inertial=None)
                           for name in constants.CAMERA_TFS.keys()]
    vehicle_link: Links = [urdfpy.Link(name=constants.VEHICLE_NAME,
                                       visuals=[constants.VEHICLE_VISUAL],
                                       collisions=None, inertial=None)]

    lidar_joint: Joints = [urdfpy.Joint(name=f"{constants.BASE_LINK_NAME}_"
                                        f"{constants.LIDAR_LINK_NAME}_link",
                                        joint_type="fixed", parent=constants.BASE_LINK_NAME,
                                        child=constants.LIDAR_LINK_NAME, origin=p0_tf_lidar)]
    wheel_joints: Joints = [urdfpy.Joint(name=f"{constants.BASE_LINK_NAME}_{name}_link",
                                         joint_type="fixed", parent=constants.BASE_LINK_NAME,
                                         child=name, origin=tf)
                            for name, tf in constants.WHEEL_TFS.items()]
    camera_joints: Joints = [urdfpy.Joint(name=f"{constants.BASE_LINK_NAME}_{name}_link",
                                          joint_type="fixed", parent=constants.BASE_LINK_NAME,
                                          child=name, origin=tf)
                             for name, tf in constants.CAMERA_TFS.items()]
    vehicle_joint: Joints = [urdfpy.Joint(name=f"{constants.BASE_LINK_NAME}_"
                                               f"{constants.VEHICLE_NAME}_link",
                                          joint_type="fixed", parent=constants.BASE_LINK_NAME,
                                          child=constants.VEHICLE_NAME,
                                          origin=constants.P0_TF_VEHICLE)]

    all_links: Links = base_link + lidar_link + wheel_links + camera_links + vehicle_link
    all_joints: Joints = lidar_joint + wheel_joints + camera_joints + vehicle_joint

    return urdfpy.URDF(name=urdf_name, links=all_links, joints=all_joints)


def generate_urdf(dataset_path: Path, output_path: Path) -> None:
    """Generate a vehicle URDF given a dataset folder with calibration information

    Args:
        dataset_path (Path): Dataset folder
        output_path (Path): Folder where resulting URDF will be saved
    """

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

    vehicle_urdf = create_urdf(urdf_name=dataset_path.stem, p0_tf_lidar=p0_tf_lidar)
    output_file = output_path / Path(f"{dataset_path.stem}.urdf")
    vehicle_urdf.save(str(output_file))

    logging.info(f"Vehicle URDF for sequence {dataset_path.stem} saved to {output_file}")
