from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'oc_photo_apiserver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), [f'{package_name}/fastapi_image_server.py']),  # FastAPIスクリプトを `share` に配置

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
            'photoclient = oc_photo_apiserver.photoclient:main',
            'test_photo_publish = oc_photo_apiserver.test_photo_publish:main'
        ],
    },
)
