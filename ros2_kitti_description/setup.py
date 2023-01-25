
from setuptools import setup

package_name = 'ros2_kitti_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'generate_urdfs = ros2_kitti_description.generate_urdfs:generate_urdfs',
        ]
    },
    install_requires=['setuptools'],
    zip_safe=True,
    author='Lam Teng Foong',
    author_email='tengfoonglam@yahoo.com.sg',
    maintainer='Lam Teng Foong',
    maintainer_email='tengfoonglam@yahoo.com.sg',
    keywords=['ROS2'],
    description='URDF Files and Generation Scripts for the KITTI Dataset',
    license='BSD 3-clause',
)
