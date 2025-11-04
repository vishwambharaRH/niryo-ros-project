from setuptools import setup
from glob import glob
import os

package_name = 'gesture_recognition'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Gesture recognition package for Niryo Ned2 robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_detector = gesture_recognition.gesture_detector:main',
            'gesture_classifier = gesture_recognition.gesture_classifier:main',
            'camera_handler = gesture_recognition.camera_handler:main',
        ],
    },
)