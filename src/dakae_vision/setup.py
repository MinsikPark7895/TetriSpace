import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dakae_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dakae',
    maintainer_email='dakae@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cube_service_server = dakae_vision.cube_service_server:main',
            'marker_service_server = dakae_vision.marker_service_server:main',
            'camera_node = dakae_vision.camera_node:main',
            'marker_service_server_offset = dakae_vision.marker_service_server_offset:main',
        ],
    },
)
