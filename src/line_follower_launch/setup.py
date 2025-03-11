from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'line_follower_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # ✅ Important
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soham',
    maintainer_email='soham@todo.todo',
    description='Launch files for line follower robot in Gazebo',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [],
    },
)
