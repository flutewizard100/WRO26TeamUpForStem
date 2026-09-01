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
            'goto_pose = wro_behavior.goto_pose:main',
            'pillar_detector = wro_behavior.pillar_detector:main',
        ],
    },
)
