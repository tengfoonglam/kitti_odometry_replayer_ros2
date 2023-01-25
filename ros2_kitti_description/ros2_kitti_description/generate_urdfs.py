#!/usr/bin/env python3

# import launch_ros
import click
# import urdfpy
import logging


from pathlib import Path
from typing import Callable, List


@click.command()
@click.option('--data_odometry_calib_dir', '-d', required=True, help="Location of the "
              "KITTI 'data_odometry_calib' folder")
@click.option('--output_dir', '-o', required=True, help="Directory where generated "
              "URDFs will be located")
def generate_urdfs(data_odometry_calib_dir: str, output_dir: str) -> None:
    """Generate URDFs for the KITTI Odometry Dataset"""

    logging.basicConfig(level=logging.INFO)

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
    # Find calib.txt
    # Parse txt
    # Add links
    # Save file
    pass


if __name__ == "__main__":
    pass
