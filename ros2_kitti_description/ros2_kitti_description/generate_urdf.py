#!/usr/bin/env python3

# import launch_ros
import click
# import urdfpy
# import logging


# from pathlib import Path


@click.command(name="Kill me")
@click.option('--data_odometry_calib_dir', '-d', required=True, help="Location of the "
              "KITTI 'data_odometry_calib' folder")
# @click.option('--output_dir', '-o', default = f"{
# get_package_share_directory(package_name='ros2_kitti_description')}/../urdf",
#  show_default=True,help = "Directory where generated URDFs will be located")
def generate_urdf(data_odometry_calib_dir: str, output_dir: str) -> None:
    """Generate URDFs for the KITTI Odometry Dataset"""
    pass


if __name__ == "__main__":
    pass
