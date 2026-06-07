import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ur3_moveit_module'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS2bootcamp',
    maintainer_email='psj15641@gmail.com',
    description='MoveIt module driving UR3 + Robotiq 2F-85 from LLM Agent commands.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit_module_node = ur3_moveit_module.moveit_module_node:main',
        ],
    },
)
