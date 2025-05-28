from setuptools import setup

package_name = 'image_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/ws_only.launch.py',
        ]),
    ],
    install_requires=[
        'setuptools',
        'websockets',
        ],
    zip_safe=True,
    maintainer='hongfa',
    maintainer_email='bup553@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ws_node      = image_comm.ws_node:main',
            'capture_node = image_comm.capture_node:main',
        ],
    },
)
