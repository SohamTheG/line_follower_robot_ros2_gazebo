from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'line_follower_pid'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.'),  # Automatically find packages
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soham',
    maintainer_email='your-email@example.com',
    description='PID-based line follower package for ROS 2',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'line_follower = line_follower_pid.line_follower_pid:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    extras_require={'test': ['pytest']},
)
