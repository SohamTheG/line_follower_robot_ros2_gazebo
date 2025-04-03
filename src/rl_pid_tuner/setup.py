from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rl_pid_tuner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.'),
    install_requires=[
        'setuptools', 
        'gym', 
        'stable-baselines3',
        'rclpy',
        'numpy',
        'torch',
        'matplotlib'  # For potential visualization
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Reinforcement Learning for PID tuning in line-following robots using PPO',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'train_ppo = rl_pid_tuner.train_ppo:main',
            'test_ppo = rl_pid_tuner.test_ppo:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    extras_require={'test': ['pytest']},
)