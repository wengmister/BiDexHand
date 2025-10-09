from setuptools import find_packages, setup

package_name = 'hand_servo_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', ['launch/multi_servo.launch.xml',
                                                'launch/demo.launch.xml',
                                                'launch/calibrated_servo.launch.xml']),
        ('share/' + package_name + '/config/', ['config/servo_calibration.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhengyang Kris Weng',
    maintainer_email='wengmister@gmail.com',
    description='Servo control ROS2 node for BiDexHand',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'single_servo_control = hand_servo_control.single_servo_control:main',
            'multi_servo_control = hand_servo_control.servo_control_node:main',
            'demo_motion = hand_servo_control.demo_motion_node:main',
            'servo_control_gui = hand_servo_control.servo_control_gui:main',
            'calibrated_control_node = hand_servo_control.calibrated_control_node:main',
            'calibrated_control_gui = hand_servo_control.calibrated_control_gui:main',
            'calibrated_ik_verification_node = hand_servo_control.calibrated_ik_verification_node:main',
        ],
    },
)
