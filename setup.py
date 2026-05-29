import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autonomous_driving'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='R.W.',
    maintainer_email='rmwl4213@hochschule-trier.de',
    description='Gazebo simulation worlds and launch files for the Aeris robot.',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ouster_timestamp_relay = autonomous_driving.ouster_timestamp_relay:main',
            'gazebo_odom_tf = autonomous_driving.gazebo_odom_tf:main',
            'imu_world_orientation_relay = autonomous_driving.imu_world_orientation_relay:main',
        ],
    },
)
