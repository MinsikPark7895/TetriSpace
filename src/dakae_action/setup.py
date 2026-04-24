from setuptools import find_packages, setup

package_name = 'dakae_action'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dakae',
    maintainer_email='dakae@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_place_action_server = dakae_action.pick_place_action_server:main',
            'pick_place_client = dakae_action.pick_place_action_client:main',
            'pick_place_client_loop = dakae_action.pick_place_action_client_loop:main',
        ],
    },
)
