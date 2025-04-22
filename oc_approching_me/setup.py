from setuptools import setup
import os
from glob import glob

package_name = 'oc_approching_me'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Resource files (like music)
        (os.path.join('share', package_name, 'sounds'), glob('sounds/*.mp3')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='takumi',
    maintainer_email='takumi@example.com',
    description='Approaching person detection and navigation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'approaching_person = oc_approching_me.approachingperson:main',
            'raisehands = oc_approching_me.raisehands:main',
            'raisehandsTest = oc_approching_me.raisehandTest:main',
        ],
    },
)
