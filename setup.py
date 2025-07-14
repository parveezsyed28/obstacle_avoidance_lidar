from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required to register the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install URDF/Xacro and mesh files
        (os.path.join('share', package_name, 'models/urdf'), glob('models/urdf/*')),
        (os.path.join('share', package_name, 'models/meshes'), glob('models/meshes/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parveez',
    maintainer_email='parveezbanu.s@gmail.com',
    description='ROS 2 Gazebo-based obstacle avoidance robot simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_node = obstacle_avoidance.obstacle_avoidance_node:main',
        ],
    },
)
