from setuptools import find_packages, setup

package_name = 'ros2_vive_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cdonoso',
    maintainer_email='clemente.donoso@inria.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vive_tracker = ros2_vive_controller.vive_tracker:main',
            'calibrate_workspace = ros2_vive_controller.calibrate_workspace:main',
            'joystick_node = ros2_vive_controller.joystick_node:main',
        ],
    },
)