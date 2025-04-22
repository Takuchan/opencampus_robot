from setuptools import setup
import os
from glob import glob

package_name = 'oc_tts'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # サービス定義ファイルを含める場合
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tk',
    maintainer_email='takuchanapp@gmail.com',
    description='ROS2サービスを用いたPython TTSサーバー',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_server = oc_tts.tts_server:main',
            'tts_client = oc_tts.tts_client:main',
        ],
    },
)
