from setuptools import setup

package_name = 'ir_sensor_subscriber'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Soham',
    maintainer_email='user@example.com',
    description='IR Sensor Subscriber Node',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'ir_sensor_subscriber = ir_sensor_subscriber.ir_sensor_subscriber:main'
        ],
    },
)
