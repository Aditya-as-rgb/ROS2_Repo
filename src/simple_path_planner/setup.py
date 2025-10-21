from setuptools import setup
from glob import glob
import os

package_name = 'simple_path_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Simple path planning with ROS 2 Jazzy',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'simple_navigator = scripts.simple_navigator:main',
        ],
    },
)
