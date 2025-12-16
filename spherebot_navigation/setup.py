import os
from glob import glob
from setuptools import setup

package_name = 'spherebot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install the package.xml (Standard)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # [CRITICAL] Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # [CRITICAL] Install config files (Nav2 params)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # [CRITICAL] Install map files (.yaml and .pgm)
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Navigation configuration for spherebot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_control = spherebot_navigation.mission_control:main',
            'vision_detector = spherebot_navigation.vision_detector:main',
            'go_to_pen = spherebot_navigation.go_to_pen:main',
        ],
    },
)