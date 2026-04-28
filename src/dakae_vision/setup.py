import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dakae_vision'
model_files = [
    (os.path.join('share', package_name, os.path.dirname(path)), [path])
    for path in glob('best/**/*', recursive=True)
    if os.path.isfile(path)
]
if os.path.isfile('best.pt'):
    model_files.append((os.path.join('share', package_name), ['best.pt']))
if os.path.isfile('dakae_v2_best.pt'):
    model_files.append((os.path.join('share', package_name), ['dakae_v2_best.pt']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + model_files,
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
            'object_contour_service_server = dakae_vision.object_contour_service_server:main',
            'collect_dataset = dakae_vision.collect_dataset:main',
            'ppo_robot_controller = dakae_vision.ppo_robot_controller:main',
        ],
    },
)
