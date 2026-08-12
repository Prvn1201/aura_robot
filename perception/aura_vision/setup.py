import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aura_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- Add this line to install all python files in the launch folder ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pravin_workstation',
    maintainer_email='pravin_workstation@todo.todo',
    description='AURA Vision package for perception nodes',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_follower = aura_vision.line_follower_camera:main',
            'traffic_light_detector = aura_vision.traffic_light_detector:main',
        ],
    },
)
