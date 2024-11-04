from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'espresso_frame_simple_simulator'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='project@srs',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_move = espresso_frame_simple_simulator.dummy_move:main',
            'dummy_turret = espresso_frame_simple_simulator.dummy_turret:main',
        ],
    },
)
