from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'nav2_apps'

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
    maintainer='user',
    maintainer_email='wrightjake5683@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_shelf_to_ship = nav2_apps.move_shelf_to_ship:main',
            'shelf_handler = nav2_apps.scripts.shelf_handler:main',
            'shelf_handler_real = nav2_apps.scripts.shelf_handler_real:main',
        ],
    },
)

# for console scripts:
# 'script_name = package_name.module_name:function_name',
