import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rusteze_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohammed',
    maintainer_email='mo-wesam-mecha2006@example.com',
    description='Multi-module vehicle kinematics and odometry (Task 11.3)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_node = rusteze_kinematics.kinematics_node:main',
            'wheel_odometry_node = rusteze_kinematics.wheel_odometry_node:main',
        ],
    },
)
