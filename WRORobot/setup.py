import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'WRORobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro')) +
            glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wroteam',
    maintainer_email='sagnikbiswas712@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'HighController = WRORobot.RobotController:main',
            'Motor = WRORobot.Motor:main',
            'Servo = WRORobot.Servo:main',
            'otos_node = wroRobot.otos_node:main',
        ],
    },

)
