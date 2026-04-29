from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'teleop_vision'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

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
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='ud90uhak',
    maintainer_email='jeswanth.kanipakam@fau.de',

    description='Dual arm vision teleoperation package',
    license='Apache-2.0',

    extras_require={
        'test': ['pytest'],
    },

    entry_points={
        'console_scripts': [
            'teleop_mapper = teleop_vision.vision_node:main',
        ],
    },
)
