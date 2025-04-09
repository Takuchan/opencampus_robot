from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'oc_megarover_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
<<<<<<< HEAD
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
=======
        (os.path.join('share', package_name,'config'), [f'param/navigation_param.yaml']),  
        (os.path.join('share', package_name,'maps'), [f'maps/map.yaml']),  
        (os.path.join('share', package_name,'maps'), [f'maps/map.pgm']),  
>>>>>>> main
        ('share/' + package_name, ['package.xml']),
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
            'bringup = oc_megarover_bringup.bringup:main'
        ],
    },
)
