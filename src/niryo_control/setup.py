from setuptools import setup

package_name = 'niryo_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/camera_params.yaml'] if package_name=='gesture_recognition' else []),
        ('share/' + package_name + '/config', ['config/gesture_params.yaml'] if package_name=='gesture_recognition' else []),
        ('share/' + package_name + '/config', ['config/gesture_mappings.yaml'] if package_name=='gesture_control_interface' else []),
        ('share/' + package_name + '/launch', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='niryo_control',
    license='MIT',
    entry_points={
        'console_scripts': [
            "robot_controller = niryo_control.robot_controller:RobotController"
        ],
    },
)
