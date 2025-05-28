from setuptools import setup
import os
from glob import glob

package_name = 'battery_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssafy',
    maintainer_email='ssafy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_manager_node = battery_system.system_manager_node:main',
            'zone_manager_node = battery_system.zone_manager_node:main',
            'vision_proxy_node = battery_system.vision_proxy_node:main',
        ],
    },
)
