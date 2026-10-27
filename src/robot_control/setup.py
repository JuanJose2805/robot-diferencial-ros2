from glob import glob
import os

from setuptools import setup

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan José Martínez Medina',
    maintainer_email='eng.juanmartinez@gmail.com',
    description='Control de posicion y teleoperacion del robot diferencial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_controller = robot_control.position_controller:main',
            'teleop_wasd = robot_control.teleop_wasd:main',
            'hardware_validation = robot_control.hardware_validation:main',
        ],
    },
)
