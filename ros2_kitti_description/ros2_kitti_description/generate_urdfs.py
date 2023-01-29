import click
import logging
import ros2_kitti_description.helpers as helpers
import ros2_kitti_description.constants as constants
import re

from pathlib import Path
from typing import Callable, List


@click.command()
@click.option('--data_odometry_calib_dir', '-d', required=True, help="Location of the "
              "KITTI 'data_odometry_calib' folder")
@click.option('--output_dir', '-o', required=True, help="Directory where generated "
              "URDFs will be located")
def generate_urdfs(data_odometry_calib_dir: str, output_dir: str) -> None:
    """ Generate vehicle URDFs from the calibration data provided by the KITTI Vision Benchmark Suite

    Args:
        data_odometry_calib_dir (str): data_odometry_calib folder downloaded from
                                       the KITI Benchmark website
        output_dir (str): Directory where generated URDFS will be saved
    """

    logging.basicConfig(level=logging.INFO)

    # Make sure output directory exists
    output_path: Path = Path(output_dir).resolve()
    if not output_path.is_dir():
        logging.error(
            f"output_path provided ({output_path}) "
            "is not a folder or does not exist")
        return

    # Make sure calib folder exists
    data_odometry_calib_path: Path = Path(data_odometry_calib_dir).resolve()
    if not data_odometry_calib_path.is_dir():
        logging.error(
            f"data_odometry_calib_dir provided ({data_odometry_calib_path}) "
            "is not a folder or does not exist")
        return

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
