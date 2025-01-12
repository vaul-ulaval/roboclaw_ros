from setuptools import setup
import os
from glob import glob

package_name = 'roboclaw_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=['roboclaw_ros'],
    data_files=[
        ('share/ament_index/resource_index/packagenames', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('lib', package_name, 'roboclaw_driver'), glob(os.path.join('roboclaw_driver', '*.[py]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Roboclaw node for controlling motors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roboclaw_plow = roboclaw_ros.roboclaw_plow:main',
        ],
    },
)
