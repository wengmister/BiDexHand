from setuptools import find_packages, setup

package_name = 'hand_motion_shadowing'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', ['launch/shadowing.launch.xml']),
        ('share/' + package_name + '/config/', ['config/v1mp.yaml',
                                                'config/v1quest.yaml',
                                                'config/v2quest.yaml',
                                                'config/v2mp.yaml',
                                                'config/v3quest.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhengyang Kris Weng',
    maintainer_email='wengmister@gmail.com',
    description='Node for mirroring usb cam input to hand servos',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_shadowing = hand_motion_shadowing.hand_shadowing_node:main',
        ],
    },
)
