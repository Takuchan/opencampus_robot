from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'oc_recognition_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

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
            'yolov11_publisher = oc_recognition_yolo.yolov11_publisher:main',
            'open3d_sample = oc_recognition_yolo.open3d_sample:main',
            'yolo_publisher = oc_recognition_yolo.yolo_publisher:main',
        ],
    },
)
