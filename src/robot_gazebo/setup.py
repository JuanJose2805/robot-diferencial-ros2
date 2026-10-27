from glob import glob
import os

from setuptools import setup

package_name = 'robot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan José Martínez Medina',
    maintainer_email='eng.juanmartinez@gmail.com',
    description='Mundos de simulacion y launch de Gazebo Classic del robot diferencial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
