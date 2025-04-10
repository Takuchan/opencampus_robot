from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'oc_denger_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'sounds'), glob('sounds/*.wav')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tk',
    maintainer_email='takuchanapp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sound_test = oc_denger_detection.sound_test:main',
            'stopfrontperson = oc_denger_detection.stopfrontperson:main'
        ],
    },
)
