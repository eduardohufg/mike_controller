from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mike_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=[package_name, 'lib', 'lib.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardohufg',
    maintainer_email='eduardochavezmartin10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry = mike_controller.odometry:main',
            'main_controller = mike_controller.main_controller:main',
            'path_generator = mike_controller.path_generator:main',
            'odometry_turtle = mike_controller.odometry_turtle:main',
        ],
    },
)
