from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hand_calibration'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhengyang Kris Weng',
    maintainer_email='wengmister@gmail.com',
    description='Calibration utilities for BiDexHand',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collector = hand_calibration.data_collector:main',
            'motor_calibrator = hand_calibration.motor_calibrator:main',
        ],
    },
)
