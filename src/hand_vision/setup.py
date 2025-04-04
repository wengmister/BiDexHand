from setuptools import find_packages, setup

package_name = 'hand_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', ['launch/vision.launch.xml',
                                                'launch/wrist_cam.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhengyang Kris Weng',
    maintainer_email='wengmister@gmail.com',
    description='Package for using mediapipe hand recognition',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_angle_node = hand_vision.hand_angle_node:main',
            'hand_joint_gui_node = hand_vision.hand_joint_gui_node:main',
            'hand_angle_quest = hand_vision.hand_angle_quest:main',
            'wrist_camera_node = hand_vision.wrist_camera_node:main',
            'yolo_node = hand_vision.yolo_node:main',
        ],
    },
)
