from setuptools import setup
from glob import glob 
import os

package_name = 'jetson_ec2_mqtt_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # ───────────────────────────────────────────────────────────────────
    data_files=[
        # ament index 등록
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # launch 폴더에 있는 모든 .py (launch/*.py)
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        # config 폴더에 있는 모든 yaml
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        # resource/cert 폴더에 있는 모든 crt
        (os.path.join('share', package_name, 'resource', 'cert'),
         glob('resource/cert/*.crt')),
    ],
    # ───────────────────────────────────────────────────────────────────
    install_requires=['setuptools','psutil'],
    zip_safe=True,
    maintainer='hongfa',
    maintainer_email='hongfa@example.com',
    description='ROS2 MQTT Telemetry Node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mqtt_bridge_node = jetson_ec2_mqtt_comm.mqtt_bridge_node:main',
            'telemetry_node = jetson_ec2_mqtt_comm.telemetry_node:main',
            'battery_calc_node = jetson_ec2_mqtt_comm.battery_calc_node:main'
        ],
    },
)
