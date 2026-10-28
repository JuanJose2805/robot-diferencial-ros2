from glob import glob
import os

from setuptools import setup

package_name = 'robot_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'),
            glob('web/*.html')),
        (os.path.join('share', package_name, 'web', 'css'),
            glob('web/css/*.css')),
        (os.path.join('share', package_name, 'web', 'js'),
            glob('web/js/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan José Martínez Medina',
    maintainer_email='eng.juanmartinez@gmail.com',
    description='Interfaz web (rosbridge + roslibjs) del robot diferencial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_bridge = robot_web.web_bridge:main',
        ],
    },
)
