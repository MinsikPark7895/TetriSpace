import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'test_first'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, package_name), glob('test_first/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Doosan MoveLine client package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_line = test_first.move_line_client:main',
            'arm_gripper_test = test_first.arm_gripper_test_client:main',
            'test_color = test_first.test_color:main',
            'gripper_server = test_first.gripper_server:main',
            'cube_place = test_first.cube_place_client:main',
            'go_home = test_first.home_client:main',
        ],
    },
)
