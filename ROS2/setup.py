from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_vive_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 1. Install Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # 2. Install Config Files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # 3. Install RViz Files (Ensure you have a 'rviz' folder in your src!)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cdonoso',
    maintainer_email='clemente.donoso@inria.fr',
    description='Vive Controller Node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vive_tracker = ros2_vive_controller.vive_tracker:main',
            'calibration_node = ros2_vive_controller.calibrate_workspace:main',
            'joystick_node = ros2_vive_controller.joystick_node:main',
            'teleop_bridge_node = ros2_vive_controller.teleop_bridge_node:main',
        ],
    },
)