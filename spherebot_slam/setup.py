import os
from glob import glob
from setuptools import setup

package_name = 'spherebot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install the package.xml (Standard)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # [CRITICAL] Install all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # [CRITICAL] Install all config files (YAML)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='faseeh',
    maintainer_email='faseeh@todo.todo',
    description='SLAM configuration for spherebot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)