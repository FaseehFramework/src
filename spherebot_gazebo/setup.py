import os
from glob import glob
from setuptools import setup

package_name = 'spherebot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Install config files (Bridge YAML)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # --- NEW: Install world files ---
        # This copies everything from the 'worlds' folder to 'share/spherebot_gazebo/worlds'
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='faseeh',
    maintainer_email='faseeh@faseeh.todo',
    description='Gazebo simulation bringup for spherebot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)