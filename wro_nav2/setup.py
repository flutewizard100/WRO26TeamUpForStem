import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wro_nav2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'params'),
            glob(os.path.join('params', '*.yaml'))),
        (os.path.join('share', package_name, 'bt'),
            glob(os.path.join('bt', '*.xml'))),
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*.yaml')) +
            glob(os.path.join('maps', '*.pgm'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wroteam',
    maintainer_email='sagnikbiswas712@gmail.com',
    description='Nav2 config, BTs, and launch for the WRO Ackermann robot.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
