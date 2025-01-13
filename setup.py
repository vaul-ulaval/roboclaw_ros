from setuptools import setup
import os
from glob import glob

package_name = 'roboclaw_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        'roboclaw_ros',               # The main package
        'roboclaw_ros.roboclaw_driver'  # The driver subpackage
    ],
    data_files=[
        # This resource file notifies ROS 2 about your package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Install your package.xml
        (os.path.join('share', package_name), ['package.xml']),

        # If you have launch files:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # If you have config files:
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Roboclaw ROS2 package for controlling motors',
    license='BSD',
    entry_points={
        'console_scripts': [
            # The main node / script (roboclaw_plow.py must have a def main():)
            'roboclaw_plow = roboclaw_ros.roboclaw_plow:main',
        ],
    },
    package_dir={'': '.'},
)
