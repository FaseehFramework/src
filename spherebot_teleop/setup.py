from setuptools import setup

package_name = 'spherebot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='faseeh',
    maintainer_email='faseeh@faseeh.todo',
    description='Teleoperation node for spherebot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This creates the executable 'keyboard'
            # syntax: executable_name = package_dir.file_name:function_name
            'keyboard = spherebot_teleop.keyboard_teleop:main',
        ],
    },
)