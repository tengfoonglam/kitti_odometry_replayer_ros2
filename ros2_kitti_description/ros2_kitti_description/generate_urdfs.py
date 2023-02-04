import click
import logging
import re
import ros2_kitti_description.helpers as helpers
import ros2_kitti_description.constants as constants

from ament_index_python.packages import get_package_share_path
from pathlib import Path
from typing import Callable, List


@click.command()
@click.option('--data_odometry_calib_dir', '-d', required=True, help="Location of the "
              "KITTI 'data_odometry_calib' folder")
@click.option('--output_dir', '-o', default="", help="Directory where generated "
              "URDFs will be located. If unspecified, will attempt to save "
              "to various folders in the following order: "
              f"1) {constants.PACKAGE_NAME} package's shared directory "
              "2) user's home directory")
def generate_urdfs(data_odometry_calib_dir: str, output_dir: str) -> None:
    """ Generate vehicle URDFs from the calibration data provided by the KITTI Vision Benchmark Suite

    Args:
        data_odometry_calib_dir (str): data_odometry_calib folder downloaded from
                                       the KITI Benchmark website
        output_dir (str): Directory where generated URDFS will be saved
    """

    logging.basicConfig(level=logging.INFO)

    # Make sure calib folder exists
    data_odometry_calib_path: Path = Path(data_odometry_calib_dir).resolve()
    if not data_odometry_calib_path.is_dir():
        logging.error(
            f"data_odometry_calib_dir provided ({data_odometry_calib_path}) "
            "is not a folder or does not exist")
        return

    # Resolve output directory
    output_path: Path = Path.home()

    if len(output_dir) > 0:
        user_defined_path: Path = Path(output_dir).resolve()
        if helpers.is_existing_folder(path=user_defined_path):
            logging.error(
                f"User defined output_path ({output_path}) "
                "is not a folder or does not exist. Terminating process")
            return
    else:
        try:
            share_path = get_package_share_path(package_name=constants.PACKAGE_NAME)
            output_path = share_path if helpers.is_existing_folder(
                path=share_path) else output_path
        except Exception:
            logging.exception(
                f"Could not find shared URDF folder for package {constants.PACKAGE_NAME}")

    logging.info(f"Output URDFs will be saved to {output_path}")

    # Make sure sequence folder in calib folder exists
    sequences_path: Path = data_odometry_calib_path / Path("dataset/sequences")
    if not sequences_path.exists():
        logging.error(
            f"sequences folder within data_odometry_calib_dir {sequences_path} does not exist")
        return

    # Identify all dataset folders
    is_dataset_folder: Callable[[str], bool] = lambda stem: re.fullmatch(
        constants.SEQUENCE_FOLDER_REGEX, stem) is not None
    dataset_folders: List[Path] = sorted([folder for folder in sequences_path.iterdir(
    ) if is_dataset_folder(folder.stem)], key=lambda path: int(path.stem))
    logging.info(f"Found sequences {[folder.stem for folder in dataset_folders]}")

    # For each dataset folder generate and save URDF
    for folder in dataset_folders:
        helpers.generate_urdf(dataset_path=folder, output_path=output_path)
