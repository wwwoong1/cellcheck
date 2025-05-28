from setuptools import setup

package_name = 'serial_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/serial_mapping.yaml']),
        ('share/' + package_name + '/launch', ['launch/serial_reader.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hongfa',
    maintainer_email='hongfa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_reader_node = serial_comm.serial_reader_node:main',
        ],
    },
)
