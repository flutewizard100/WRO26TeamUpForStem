from glob import glob

from setuptools import find_packages, setup

package_name = 'wro_behavior'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wroteam',
    maintainer_email='sagnikbiswas712@gmail.com',
    description='High-level behavior for the WRO robot (sim + real).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_limelight_bridge = wro_behavior.sim_limelight_bridge:main',
            'open_mission = wro_behavior.open_mission:main',
            'obstacle_mission = wro_behavior.obstacle_mission:main',
            'camera_map_augmenter = wro_behavior.camera_map_augmenter:main',
        ],
    },
)
